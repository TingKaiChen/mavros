/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <iostream>

using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
mavros_msgs::RCIn rcin;
void state_rc(const mavros_msgs::RCIn::ConstPtr& msg){
    rcin = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>
            ("mavros/rc/in", 10, state_rc);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    mavros_msgs::OverrideRCIn rc_msg;
    rc_msg.channels = {1600, 0, 1600, 0, 0, 0, 0, 0};

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    ROS_INFO("TTTTEST");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("TTTTEST2");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("TTTTEST3");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "STABILIZE";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_rc_time = ros::Time::now();
    ROS_INFO("TTTTEST4");

    while(ros::ok()){
        if( current_state.mode != "STABILIZE" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        cout<<rcin.channels[2]<<endl;

        if( current_state.armed &&
            (ros::Time::now() - last_rc_time > ros::Duration(5.0))){
            rc_msg.channels = {0, 0, 0, 0, 0, 0, 0, 0};
        }
        rc_pub.publish(rc_msg);
        ros::spinOnce();
        rate.sleep();
        if( current_state.armed &&
            (ros::Time::now() - last_rc_time > ros::Duration(10.0))){
            arm_cmd.request.value = false;
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                break;
            }
        }
    }

    return 0;
}