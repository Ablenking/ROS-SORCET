#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include <mutex>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/FixedWingOffboard.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/UavState.h>

#include <uav_state/UavStatePull.h>
#include <ref_generation/InitializeSrv.h>
#include <ref_generation/ReferenceSrv.h>
#include <ref_generation/RefStatusSrv.h>
#include <ref_generation/ReferenceTarget.h>


namespace off_mission
{

class TaskManager
{
public:
    TaskManager();

    ~TaskManager();
private:
    std::mutex state_mutex_;
    std::mutex landed_mutex_;
    std::mutex rc_mutex_;

private:
    void mainLoopOnlineOffline(const ros::TimerEvent& event);

    void state_cb(const mavros_msgs::State::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_ = *msg;
    }

    void rcinCallback(const mavros_msgs::RCIn::ConstPtr& msg){
        std::lock_guard<std::mutex> lock(rc_mutex_);
        m_rcin_ = *msg;
    }

    void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
        m_global_pos_ = *msg;
    }

    void onlineTarget_cb(const ref_generation::ReferenceTarget::ConstPtr& msg);

    void initTagetVector(XmlRpc::XmlRpcValue &wp_list);

    void flyThroughWaypoints();
    void sendCurrentStateAsRef();

    bool isSwitchIdle(const mavros_msgs::RCIn& rcin){
        return (rcin.channels.size()>=7 && rcin.channels.at(6)<1300);
    }

    bool isSwitchTakeoff(const mavros_msgs::RCIn& rcin){
        return (rcin.channels.size()>=7 && rcin.channels.at(6)>1300 && rcin.channels.at(6)<1700);
    }

    bool isSwitchPath(const mavros_msgs::RCIn& rcin){
        return (rcin.channels.size()>=7 && rcin.channels.at(6)>1800 && rcin.channels.at(6)<2000);
    }

    bool isAutoOffboardMode(const mavros_msgs::RCIn& rcin){
        return (rcin.channels.size()>=7 &&
                rcin.channels.at(4)>1800 && rcin.channels.at(4)<2000 &&
                rcin.channels.at(5)>1800 && rcin.channels.at(5)<2000);
    }

    bool isAutoMode(const mavros_msgs::RCIn& rcin){
        return (rcin.channels.size()>=7 &&
                rcin.channels.at(4)>1800 && rcin.channels.at(4)<2000);
    }

    bool isOffboardSwitchOn(const mavros_msgs::RCIn& rcin){
        return (rcin.channels.size()>=7 &&
                rcin.channels.at(5)>1800 && rcin.channels.at(5)<2000);
    }

    void checkEnterOffboardByRcSwitch();
    void setOnGroundOrigin();
    void resetOnGroundOrigin();
    void addRefOrigin(ref_generation::ReferenceTarget *ref);
    void resetOffMission();
    void checkArmingState();
    bool checkWaypointRefReachedFlag();
    bool sendTargetReferenceToFlightControl(ref_generation::ReferenceTarget &ref);
    void sendCurrentReferenceAsTarget(ref_generation::ReferenceTarget &ref);
    void setTakeoffTargetAtCurrentState(mavros_msgs::UavState cur_state, double enu_relative_takeoff_hight_);
    void setLandTargetAtCurrentState(mavros_msgs::UavState cur_state);

    void landAtCurrentPoint();
    void getCurrentState();

private:
    ros::NodeHandle node_;

    ros::Subscriber state_sub_;
    ros::Subscriber online_target_sub_;

    ros::ServiceClient pull_state_client_;
    ros::ServiceClient initialize_ref_client_;
    ros::ServiceClient get_ref_client_;
    ros::ServiceClient get_reached_flag_client_;

    ref_generation::InitializeSrv initilize_srv_;
    ref_generation::ReferenceSrv  get_ref_srv_;
    ref_generation::RefStatusSrv  get_ref_status_srv_;

    ros::Subscriber m_rcin_sub_;

    ros::Subscriber m_global_pos_sub_;
    sensor_msgs::NavSatFix m_global_pos_;

    ros::Publisher local_pos_ned_ref_pub_;

    mavros_msgs::State current_state_;

    mavros_msgs::RCIn m_rcin_;
    mavros_msgs::RCIn m_rcin_prev_;

    std::vector<geometry_msgs::PoseStamped> waypoints;

    ros::Timer main_loop_timer_;
    double main_loop_duration_;

    bool task_manager_start_flag_;
    bool pp_initilized_flag_;
    bool waypoint_switched_flag_;
    int  current_index_;

    // UAV_STATE uav_state;
    mavros_msgs::UavState uav_state;

    bool ground_origin_position_initialized_flag_;

    //REFERENCE m_ref_origin_;
    ref_generation::ReferenceTarget m_ref_origin_;
    int origin_counter;

    int m_outdoor_indoor_flag_; // 1/true for indoor, 0/false for outdoor;
    double m_takeoff_acc_;
    double m_takeoff_hight_;
    bool m_takeoff_finished_flag_;
    ref_generation::ReferenceTarget m_takeoff_target_;
    ref_generation::ReferenceTarget m_land_target_;
    ref_generation::ReferenceTarget m_idle_target_;

    ref_generation::ReferenceTarget m_online_target_;

    int m_online_offline_flag_; // online path 1, offline path 0;

    bool reenter_arming_flag_;
    bool reset_off_mission_flag_;

    bool path_flied_flag_; // flag to indicate whether PATH has been flied once
    bool takeoff_flied_flag_; // flag to indicate whether TAKEOFF has been flied once
    bool land_flied_flag_; // flag to indicate whethe LAND has been flied once

    ref_generation::ReferenceTarget current_ref_target_; // save the current referece position as target

    // for display mission state change
    bool first_time_IDLE_flag_;
    bool first_time_TAKEOFF_flag_;
    bool first_time_TAKEOFF_finihsed_flag_;
    bool first_time_PATH_flag_;

    bool first_time_OFFBOARD_flag_;
    bool pp_land_initilized_flag_;
    bool pp_idle_initilized_flag_;
};

} // namespace off_mission


#endif // TASKMANAGER_H
