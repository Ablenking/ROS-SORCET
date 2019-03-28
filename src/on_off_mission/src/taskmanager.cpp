#include "taskmanager.h"

#include <tf/transform_datatypes.h>

namespace off_mission {


TaskManager::TaskManager():
    waypoint_switched_flag_(false),
    main_loop_duration_(0.02),
    ground_origin_position_initialized_flag_(false),
    origin_counter(0),
    reset_off_mission_flag_(false)
{
    /* init ros */
    ros::NodeHandle private_nh("~");

    state_sub_               = node_.subscribe<mavros_msgs::State>("mavros/state", 10, &TaskManager::state_cb, this);
    m_rcin_sub_        = node_.subscribe("mavros/rc/in", 10, &TaskManager::rcinCallback, this);
    m_global_pos_sub_  = node_.subscribe("mavros/global_position/global", 10, &TaskManager::globalPosition_cb, this);
    online_target_sub_ = node_.subscribe("online_target", 10, &TaskManager::onlineTarget_cb, this);

    private_nh.param("tm_outdoor_indoor_flag", m_outdoor_indoor_flag_, 0);
    private_nh.param("tm_online_offline_flag", m_online_offline_flag_, 0);
    private_nh.param("tm_takeoff_acc", m_takeoff_acc_, 1.0);
    private_nh.param("tm_takeoff_hight", m_takeoff_hight_, 3.0);

    local_pos_ned_ref_pub_ = node_.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_position/local_ref", 10);

    pull_state_client_ = node_.serviceClient<uav_state::UavStatePull>("update_uavstate");

    initialize_ref_client_   = node_.serviceClient<ref_generation::InitializeSrv>("initialize_reference_generator");
    get_ref_client_          = node_.serviceClient<ref_generation::ReferenceSrv>("get_ref_to_target");
    get_reached_flag_client_ = node_.serviceClient<ref_generation::RefStatusSrv>("get_waypoint_reached_flag");

    //  main_loop_timer_        = node_.createTimer(ros::Duration(main_loop_duration_), &TaskManager::mainLoopSquare, this);
    main_loop_timer_        = node_.createTimer(ros::Duration(main_loop_duration_), &TaskManager::mainLoopOnlineOffline, this);

    XmlRpc::XmlRpcValue wp_list;
    private_nh.getParam("waypoints",wp_list);
    ROS_INFO("waypoints size %d ", wp_list.size());
    initTagetVector(wp_list);

    resetOffMission();

    m_global_pos_.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
}

TaskManager::~TaskManager()
{
}

void TaskManager::onlineTarget_cb(const ref_generation::ReferenceTarget::ConstPtr& msg)
{
    m_online_target_ = *msg;
}

/**
 * @brief TaskManager::mainLoopOnlineOffline
 * @param event
 */
void TaskManager::mainLoopOnlineOffline(const ros::TimerEvent& event)
{
    getCurrentState();

    checkArmingState();

    if(!ground_origin_position_initialized_flag_ )
    {
        if (m_outdoor_indoor_flag_ == 0 ) // outdoor, wait for gps fix
        {
            if(m_global_pos_.status.status == sensor_msgs::NavSatStatus::STATUS_FIX)
                setOnGroundOrigin();
        }
        else // indoor, just add
        {
            setOnGroundOrigin();
        }
        return;
    }

    //
    checkEnterOffboardByRcSwitch();
    // Check RC ch6 "OFFBOARD" OFF
    // better send something to px4, for easy mode switch.
    // If not sent, mode switch has problem
    if ( !isOffboardSwitchOn(m_rcin_)  || !task_manager_start_flag_)
    {
        sendCurrentStateAsRef();
        if (!first_time_OFFBOARD_flag_)
        {
            ROS_INFO("[0-0] mission ready, wait for OFFBOARD, IDLE, TAKEOFF, PATH command!");
            first_time_OFFBOARD_flag_ = true;
        }
        return;
    }

    // TAKEOFF & HOVER case
    if(isSwitchTakeoff(m_rcin_) )
    {
        if(!takeoff_flied_flag_)
            takeoff_flied_flag_ = true;

        if(!path_flied_flag_ && !land_flied_flag_)
        {
            if(!first_time_TAKEOFF_flag_)
                ROS_INFO("[2-1] sys Takeoff! Takeoff");

            if( !pp_initilized_flag_)
            {
                initilize_srv_.request.initialState = uav_state;
                initilize_srv_.request.acz          = m_takeoff_acc_;

                if(initialize_ref_client_.call(initilize_srv_))
                {
                    setTakeoffTargetAtCurrentState(uav_state, m_takeoff_hight_);
                    pp_initilized_flag_ = true;
                    first_time_TAKEOFF_finihsed_flag_ = false;
                    ROS_INFO("[2-2] Takeoff on ground! With acc to Target  %f %f %f ", m_takeoff_target_.x, m_takeoff_target_.y, m_takeoff_target_.z );
                }
            }

            m_takeoff_finished_flag_ =  sendTargetReferenceToFlightControl(m_takeoff_target_);

            if (m_takeoff_finished_flag_== true && !first_time_TAKEOFF_finihsed_flag_)
            {
                ROS_INFO("[2-4] sys Takeoff! TAKEOF Finished ");
                first_time_TAKEOFF_finihsed_flag_ = true;
            }
        }
        else
        {
            sendCurrentReferenceAsTarget(current_ref_target_);
            if(!first_time_TAKEOFF_flag_)
                ROS_INFO("[2-5] sys Takeoff! HOVER");
        }

        if(!first_time_TAKEOFF_flag_)
        {
            first_time_IDLE_flag_    = false;
            first_time_TAKEOFF_flag_ = true;
            first_time_PATH_flag_    = false;

            pp_land_initilized_flag_ = false;
        }
        return;
    }

    // online or offline waypoint case
    if(isSwitchPath(m_rcin_))
    {
        if(!first_time_PATH_flag_)
            ROS_INFO("entering sys PATH");

        if(!m_takeoff_finished_flag_) // if takeoff not finished, contitue to takeoff
        {
            m_takeoff_finished_flag_ =  sendTargetReferenceToFlightControl(m_takeoff_target_);
            if (m_takeoff_finished_flag_== 1 && !first_time_TAKEOFF_finihsed_flag_)
            {
                ROS_INFO("[3-1] sys Takeoff in PATH finished ");
                first_time_TAKEOFF_finihsed_flag_ = true;
            }
        }
        else
        {
            if (!path_flied_flag_)
                path_flied_flag_ = true;

            // If first cycle into OFFBOARD from other mode, Reset pathplanning
            if(!pp_initilized_flag_)
            {
                initilize_srv_.request.initialState = uav_state;
                initilize_srv_.request.acz          = 0;

                if(initialize_ref_client_.call(initilize_srv_))
                {
                    ROS_INFO("[3-2] sys Path Initialized at current state! with no acc ");
                }
                pp_initilized_flag_ = true;
            }

            if( m_online_offline_flag_ == 1) // online target following
            {
                sendTargetReferenceToFlightControl(m_online_target_);
                if(!first_time_PATH_flag_)
                    ROS_INFO("[3-3] to online target  %f %f %f %f ", m_online_target_.x, m_online_target_.y, m_online_target_.z, m_online_target_.c );
            }
            else
            {
                flyThroughWaypoints(); // offline waypoint flying
                if(!first_time_PATH_flag_)
                    ROS_INFO("[3-4] to offline waypoint");
            }
        }

        if(!first_time_PATH_flag_)
        {
            first_time_IDLE_flag_    = false;
            first_time_TAKEOFF_flag_ = false;
            first_time_PATH_flag_    = true;
            pp_land_initilized_flag_ = false;
        }
    }

    // LAND case
    if(isSwitchIdle(m_rcin_))
    {
        if(!first_time_IDLE_flag_)
            ROS_INFO("[1-2] sys IDLE! LAND at current position");

        if(!land_flied_flag_)
            land_flied_flag_ = true;

        landAtCurrentPoint();

        if(!first_time_IDLE_flag_)
        {
            first_time_IDLE_flag_    = true;
            first_time_TAKEOFF_flag_ = false;
            first_time_PATH_flag_    = false;
        }

        return;
    }
}

/**
 * @brief Initialized Waypoint vector from waypoints.yaml
 * @param wp_list
 */

void TaskManager::initTagetVector(XmlRpc::XmlRpcValue &wp_list)
{
    waypoints.clear();

    geometry_msgs::PoseStamped tempPose;
    for (size_t i = 0; i < wp_list.size(); ++i)
    {
        tempPose.header.seq = i;
        XmlRpc::XmlRpcValue data_list(wp_list[i]);

        // get position
        tempPose.pose.position.x = data_list[0];
        tempPose.pose.position.y = data_list[1];
        tempPose.pose.position.z = data_list[2];

        // get orientation
        tf::Quaternion q = tf::createQuaternionFromYaw(data_list[3]);

        tf::quaternionTFToMsg(q, tempPose.pose.orientation);

        waypoints.push_back(tempPose);

        ROS_INFO("---- %d %f %f %f %f", (int)tempPose.header.seq, (double)tempPose.pose.position.x, (double)tempPose.pose.position.y, (double)tempPose.pose.position.z, tf::getYaw(tempPose.pose.orientation));

    }
    ROS_INFO("Waypoint size %d ", (int)waypoints.size());
}


void TaskManager::flyThroughWaypoints()
{
    if (current_index_ < waypoints.size())
    {
        if(waypoint_switched_flag_) waypoint_switched_flag_ = false;

        mavros_msgs::PositionTarget ref_pose;
        ref_generation::ReferenceTarget ref;
        ref.x = waypoints.at(current_index_).pose.position.x;
        ref.y = waypoints.at(current_index_).pose.position.y;
        ref.z = waypoints.at(current_index_).pose.position.z;
        ref.c = tf::getYaw(waypoints.at(current_index_).pose.orientation);

        // add reference orgin to count for on ground positin drifts.
        addRefOrigin(&ref);

        //        get_ref_srv_.request.target = ref;
        //        get_ref_client_.call(get_ref_srv_);
        //        ref_pose = get_ref_srv_.response.pva_ref;
        //        local_pos_ned_ref_pub_.publish(ref_pose);
        sendTargetReferenceToFlightControl(ref);

        if ( checkWaypointRefReachedFlag() && !waypoint_switched_flag_)
        {
            current_index_++;
            waypoint_switched_flag_ = true;
            ROS_INFO("waypoint switched to %d ", current_index_);
        }
    }
    else // All waypoints reached, donot run taskmanager anymore.
    {
        task_manager_start_flag_ = false;
    }
}

bool TaskManager::sendTargetReferenceToFlightControl(ref_generation::ReferenceTarget &ref)
{
    mavros_msgs::PositionTarget ref_pose;

    get_ref_srv_.request.target = ref;
    get_ref_client_.call(get_ref_srv_);
    ref_pose = get_ref_srv_.response.pva_ref;

    local_pos_ned_ref_pub_.publish(ref_pose);

    // put current reference position as current target for emergency use

    current_ref_target_.x = ref_pose.position.x;
    current_ref_target_.y = ref_pose.position.y;
    current_ref_target_.z = ref_pose.position.z;
    current_ref_target_.c = ref_pose.yaw;
    //   ROS_INFO("Current Target 2! at %f %f %f %f ", current_ref_target_.x ,current_ref_target_.y ,current_ref_target_.z ,current_ref_target_.c );

    return checkWaypointRefReachedFlag();
}


void TaskManager::sendCurrentReferenceAsTarget(ref_generation::ReferenceTarget &ref)
{

    if(!pp_initilized_flag_)
    {
        initilize_srv_.request.initialState = uav_state;
        initilize_srv_.request.acz          = 0;

        if(initialize_ref_client_.call(initilize_srv_))
        {
            ROS_INFO("[off-IDLE/TAKEOFF] sys Path Initialized at current state! with no acc ");
        }
        pp_initilized_flag_ = true;
    }

    mavros_msgs::PositionTarget ref_pose;

    get_ref_srv_.request.target = ref;
    get_ref_client_.call(get_ref_srv_);
    ref_pose = get_ref_srv_.response.pva_ref;

    local_pos_ned_ref_pub_.publish(ref_pose);

    //  ROS_INFO("Current Target stop! at %f %f %f %f ", current_ref_target_.x ,current_ref_target_.y ,current_ref_target_.z ,current_ref_target_.c );
}

void TaskManager::landAtCurrentPoint()
{
    getCurrentState();

    if(!pp_land_initilized_flag_)
    {
        initilize_srv_.request.initialState = uav_state;
        initilize_srv_.request.acz          = 0;

        if(initialize_ref_client_.call(initilize_srv_))
        {
            pp_land_initilized_flag_ = true;
            setLandTargetAtCurrentState(uav_state);
            ROS_INFO("Land to %f %f %f ", m_land_target_.x, m_land_target_.y, m_land_target_.z );
        }
    }

    mavros_msgs::PositionTarget ref_pose;

    get_ref_srv_.request.target = m_land_target_;
    get_ref_client_.call(get_ref_srv_);
    ref_pose = get_ref_srv_.response.pva_ref;

    local_pos_ned_ref_pub_.publish(ref_pose);
    // update current ref, incase switch to takeoff or path again from LAND or TAKEOFF to PATH
    current_ref_target_.x = ref_pose.position.x;
    current_ref_target_.y = ref_pose.position.y;
    current_ref_target_.z = ref_pose.position.z;
    current_ref_target_.c = ref_pose.yaw;
}

bool TaskManager::checkWaypointRefReachedFlag()
{
    return get_reached_flag_client_.call(get_ref_status_srv_);
}

void TaskManager::sendCurrentStateAsRef()
{

    //  uav_state = uav_state_ptr_->getUAVState();
    uav_state::UavStatePull srv;

    if(pull_state_client_.call(srv))
    {
        uav_state = srv.response.uavstate;
    }
    // State in ENU frame.No need to convert
    mavros_msgs::PositionTarget ref_pose;
    ref_pose.position.x = uav_state.x_pos;
    ref_pose.position.y = uav_state.y_pos;
    ref_pose.position.z = uav_state.z_pos;
    ref_pose.yaw        = uav_state.yaw;

    // RC6 -> offboard, set current state as target for RC7 at IDLE or TAKEOFF;
    current_ref_target_.x = ref_pose.position.x;
    current_ref_target_.y = ref_pose.position.y;
    current_ref_target_.z = ref_pose.position.z;
    current_ref_target_.c = ref_pose.yaw;

    // Publise state as reference to flightcontrller in order to let RC enable "OFFBOARD"
    local_pos_ned_ref_pub_.publish(ref_pose);
}

void TaskManager::checkEnterOffboardByRcSwitch()
{
    if ( !isOffboardSwitchOn(m_rcin_prev_) && isOffboardSwitchOn(m_rcin_) ) {
        // Offboard switched OFF-> On
        pp_initilized_flag_ = false; // enter offboard, reset pp flag
        //   path_flied_flag_    = false; // enter offboard, reset flied path flag
        pp_land_initilized_flag_ = false;

        first_time_IDLE_flag_    = false;
        first_time_TAKEOFF_flag_ = false;
        first_time_PATH_flag_    = false;

        ROS_INFO("Entering Offboard by RC!\n");
    }
    m_rcin_prev_ = m_rcin_;
}

void TaskManager::setOnGroundOrigin()
{
    if (origin_counter < 100)
    {
        origin_counter++;
        m_ref_origin_.x += uav_state.x_pos;
        m_ref_origin_.y += uav_state.y_pos;
        m_ref_origin_.z += uav_state.z_pos;

    }

    if (origin_counter == 100 && !ground_origin_position_initialized_flag_)
    {
        m_ref_origin_.x = m_ref_origin_.x/origin_counter;
        m_ref_origin_.y = m_ref_origin_.y/origin_counter;
        m_ref_origin_.z = m_ref_origin_.z/origin_counter;
        ground_origin_position_initialized_flag_ = true;
        ROS_INFO("counter %d gps status %d " , origin_counter, m_global_pos_.status.status);
        ROS_INFO("refernce origin set at current position x  %f y  %f z %f " , m_ref_origin_.x , m_ref_origin_.y , m_ref_origin_.z);
    }
}

void TaskManager::resetOnGroundOrigin()
{
    ground_origin_position_initialized_flag_ = false;
    origin_counter  = 0;
    m_ref_origin_.x = 0;
    m_ref_origin_.y = 0;
    m_ref_origin_.z = 0;
    m_ref_origin_.c = 0;
}

void TaskManager::getCurrentState()
{
    uav_state::UavStatePull srv;

    if(pull_state_client_.call(srv))
    {
        uav_state = srv.response.uavstate;
    }
}

void TaskManager::addRefOrigin(ref_generation::ReferenceTarget  *ref)
{
    ref->x = ref->x + m_ref_origin_.x;
    ref->y = ref->y + m_ref_origin_.y;
    ref->z = ref->z + m_ref_origin_.z;
}

void TaskManager::checkArmingState()
{
    if(!current_state_.armed && !reset_off_mission_flag_)
    {
        reset_off_mission_flag_ = true;
        ROS_INFO("-----------------------------------------------");
        ROS_INFO("Disarmed! Will restart the mission next time armed!\n");
    }
    if(current_state_.armed && reset_off_mission_flag_)
    {
        reset_off_mission_flag_ = false;
        ROS_INFO("First time to Arm");
        resetOffMission();
    }
}

void TaskManager::resetOffMission()
{

    pp_initilized_flag_        = false;
    task_manager_start_flag_   = true;
    // restart the same mission defined by waypoints
    current_index_ = 0;
    m_takeoff_finished_flag_ = false;
    resetOnGroundOrigin();

    path_flied_flag_         = false;
    takeoff_flied_flag_      = false;
    land_flied_flag_         = false;

    first_time_IDLE_flag_    = false;
    first_time_TAKEOFF_flag_ = false;
    first_time_PATH_flag_    = false;
    first_time_TAKEOFF_finihsed_flag_ = false;
    first_time_OFFBOARD_flag_ = false;
    pp_land_initilized_flag_ = false;
    pp_idle_initilized_flag_ = false;

    if (m_outdoor_indoor_flag_ == 0 )
        ROS_INFO("In outdoor mode, need GPS to fly!");
    else
        ROS_INFO("In indoor mode, no need GPS to fly!");
}

void TaskManager::setTakeoffTargetAtCurrentState(mavros_msgs::UavState cur_state, double enu_relative_takeoff_hight_)
{
    m_takeoff_target_.x = cur_state.x_pos;
    m_takeoff_target_.y = cur_state.y_pos;
    m_takeoff_target_.z = cur_state.z_pos + enu_relative_takeoff_hight_;
    m_takeoff_target_.c = cur_state.c_pos;
}

void TaskManager::setLandTargetAtCurrentState(mavros_msgs::UavState cur_state)
{
    m_land_target_.x = cur_state.x_pos;
    m_land_target_.y = cur_state.y_pos;
    m_land_target_.z = cur_state.z_pos - 1000;
    m_land_target_.c = cur_state.c_pos;
}

} // namespace off_mission
