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
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/Imu.h>
#include <iostream>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>

using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
mavros_msgs::RCIn rcin;
void state_rc(const mavros_msgs::RCIn::ConstPtr& msg){
    rcin = *msg;
}
sensor_msgs::Imu imu_state;
void state_imu(const sensor_msgs::Imu::ConstPtr& msg){
    imu_state = *msg;
}
sensor_msgs::Imu imuraw_state;
void state_imuraw(const sensor_msgs::Imu::ConstPtr& msg){
    imuraw_state = *msg;
}
geometry_msgs::PoseStamped pose_state;
void state_pose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_state = *msg;
}
mavros_msgs::AttitudeTarget attraw_state;
void state_attraw(const mavros_msgs::AttitudeTarget::ConstPtr& msg){
    attraw_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient TOL_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, state_imu);
    ros::Subscriber imuraw_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data_raw", 10, state_imuraw);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, state_pose);
    ros::Publisher local_att_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher attraw_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/target_attitude", 10);
    ros::Publisher rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 10);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>
            ("mavros/rc/in", 10, state_rc);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    ROS_INFO("TTTTEST");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("TTTTEST2");

    mavros_msgs::OverrideRCIn rc_msg;
    // rc_msg.channels = {1800, 0, 0, 0, 0, 0, 0, 0};

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;

    geometry_msgs::PoseStamped pose_mv;
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 3.1415926*30/180, 0);
    q.normalize();
    quaternionTFToMsg(q, pose_mv.pose.orientation);
    // pose_mv.pose.position.z = 5;
    // local_att_pub.publish(pose_mv);

    mavros_msgs::AttitudeTarget pose_attraw;
    quaternionTFToMsg(q, pose_attraw.orientation);
    pose_attraw.type_mask = 1+2+4+64;

    // //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    mavros_msgs::CommandTOL tol_cmd;
    tol_cmd.request.altitude = 5.0;
    tol_cmd.request.min_pitch = 0.0;
    tol_cmd.request.yaw = 0.0;
    tol_cmd.request.latitude = 0.0;
    tol_cmd.request.longitude = 0.0;

    ROS_INFO("TTTTEST3");
    mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "CMODE(20)";
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ROS_INFO("TTTTEST4");

    bool to_success = false;
    bool altitude_success = false;
    bool set_att = false;
    bool setmode_twice = false;

    while(ros::ok()){
        if( current_state.mode != "GUIDED" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) 
            && !to_success){
            // ){
            cout<<current_state.mode<<endl;
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        if(!current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        if(current_state.armed && !to_success &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
            if(TOL_client.call(tol_cmd) &&
                tol_cmd.response.success &&
                !to_success){
                ROS_INFO("Takeoff success");
                to_success = true;
            }
        }
        // if(to_success && pose_state.pose.position.z >= 4.9 &&
        //     !setmode_twice){
        //     altitude_success = true;
        //     cout<<current_state.mode<<endl;
        //     offb_set_mode.request.custom_mode = "STABILIZE";
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.success){
        //         ROS_INFO("STABILIZE enabled");
        //     }
        //     last_request = ros::Time::now();
        //     setmode_twice = true;
        // }

        // // if( current_state.mode != "CMODE(20)" &&
        // if( current_state.mode != "GUIDED" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0)) 
        //     && !to_success){
        //     // ){
        //     cout<<current_state.mode<<endl;
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.success){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        //     else if(current_state.armed &&
        //             !to_success &&
        //             (ros::Time::now() - last_request > ros::Duration(2.0))){
        //         if(TOL_client.call(tol_cmd) &&
        //             tol_cmd.response.success &&
        //             !to_success){
        //             ROS_INFO("Takeoff success");
        //             to_success = true;
        //         }
        //     }
        // }


        // if(to_success){
        //     cout<<pose_state.pose.position.z<<endl;
        // }

        // if(to_success && pose_state.pose.position.z >= 4.9){
        //     altitude_success = true;
        // }

        if(altitude_success){
            // rc_msg.channels[0] = rcin.channels[0]+100;
            // rc_msg.channels[1] = rcin.channels[1];
            // rc_msg.channels[2] = rcin.channels[2];
            // rc_msg.channels[3] = rcin.channels[3];
            // rc_msg.channels[4] = rcin.channels[4];
            // rc_msg.channels[5] = rcin.channels[5];
            // rc_msg.channels[6] = rcin.channels[6];
            // rc_msg.channels[7] = rcin.channels[7];
            // rc_pub.publish(rc_msg);
            // local_att_pub.publish(pose_mv);
            // attraw_pub.publish(pose_attraw);
            set_att = true;
            ROS_INFO("pitch success");
        }
        // else{
        //     local_pos_pub.publish(pose);
        // }

        ros::spinOnce();
        rate.sleep();
        // if( current_state.armed &&
        //     (ros::Time::now() - last_request > ros::Duration(15.0))){
        //     arm_cmd.request.value = false;
        //     if( arming_client.call(arm_cmd) &&
        //         arm_cmd.response.success){
        //         break;
        //     }
        // }
    }

    return 0;
}