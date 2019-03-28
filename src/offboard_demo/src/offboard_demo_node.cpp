/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

mavros_msgs::RCIn m_rcin_;
void rcinCallback(const mavros_msgs::RCIn::ConstPtr& msg){
        m_rcin_ = *msg;
        
}


bool isSwitchOffBoard(const mavros_msgs::RCIn& rcin){
        return (rcin.channels.size()>=7 && rcin.channels.at(5)>1600 && rcin.channels.at(5)<2100);
    }

bool isSwitchPath(const mavros_msgs::RCIn& rcin){
        return (rcin.channels.size()>=7 && rcin.channels.at(6)>1800 && rcin.channels.at(6)<2000);
    }




int main(int argc, char **argv)
{
    int arm_switch = 0;

    ros::init(argc, argv, "offboard_demo_node");
    ros::NodeHandle nh;

    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>
            ("mavros/rc/in",10, rcinCallback);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    arming_client.call(arm_cmd);//make sound to remind PI3 is connencted
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode,current_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if(ros::Time::now() - last_request >= ros::Duration(0.5))
        {
            if( current_state.mode != "OFFBOARD"&& isSwitchOffBoard(m_rcin_)   )
            {
                current_mode.request.custom_mode = current_state.mode;
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success )
                    ROS_INFO("Offboard enabled");
                else
                    ROS_INFO("Offboard cannot enabled ");
            }
            if( current_state.mode == "OFFBOARD"&& !isSwitchOffBoard(m_rcin_))
            {
                if( set_mode_client.call(current_mode) )
                    ROS_INFO("Offboard disabled");
                else
                ROS_INFO("Offboard cannot disabled ");
            }
            last_request = ros::Time::now();
        }
       // if( current_state.mode == "OFFBOARD")
        local_pos_pub.publish(pose);
            
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
