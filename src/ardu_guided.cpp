/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>

////////////////////
#include <iostream>
#include "std_msgs/Float64.h"
#include <cmath>
#include <math.h>
#include <ros/duration.h>

using namespace std;

// Set global variables
mavros_msgs::State current_state;
// nav_msgs::Odometry current_pose;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped pose;
std_msgs::Float64 current_heading;
float GYM_OFFSET;

// get armed state
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
    bool connected = current_state.connected;
    bool armed = current_state.armed;
}

// get current position of drone
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    ROS_INFO("x: %f y: %f z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    // ROS_INFO("y: %f", current_pose.pose.position.y);
    // ROS_INFO("z: %f", current_pose.pose.position.z);
}

// get compass heading
void heading_cb(const std_msgs::Float64::ConstPtr &msg)
{
    current_heading = *msg;
    // ROS_INFO("current heading: %f", current_heading.data);
}

// set orientation of the drone (drone should always be level)
void setHeading(float heading)
{
    heading = -heading + 90 - GYM_OFFSET;
    float yaw = heading * (M_PI / 180);
    float pitch = 0;
    float roll = 0;

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);

    float qw = cy * cr * cp + sy * sr * sp;
    float qx = cy * sr * cp - sy * cr * sp;
    float qy = cy * cr * sp + sy * sr * cp;
    float qz = sy * cr * cp - cy * sr * sp;

    pose.pose.orientation.w = qw;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
}

// set position to fly to in the gym frame
void setDestination(float x, float y, float z)
{
    float deg2rad = (M_PI / 180);
    float X = x * cos(-GYM_OFFSET * deg2rad) - y * sin(-GYM_OFFSET * deg2rad);
    float Y = x * sin(-GYM_OFFSET * deg2rad) + y * cos(-GYM_OFFSET * deg2rad);
    float Z = z;
    pose.pose.position.x = X;
    pose.pose.position.y = Y;
    pose.pose.position.z = Z;
    ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // state_cb is a call back function
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher set_gp_origin_pub = nh.advertise<geographic_msgs::GeoPointStamped>("mavros/global_position/set_gp_origin", 10);
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    // ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/global_position/pose", 10, );
    ros::Subscriber currentHeading = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, heading_cb);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // allow the subscribers to initialize
    ROS_INFO("INITILIZING...");
    for (int i = 0; i < 100; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // set the orientation of the gym
    GYM_OFFSET = 0;
    for (int i = 1; i <= 30; ++i)
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        GYM_OFFSET += current_heading.data;
        ROS_INFO("current heading%d: %f", i, GYM_OFFSET / i);
    }
    GYM_OFFSET /= 30;
    ROS_INFO("the N' axis is facing: %f", GYM_OFFSET);
    cout << GYM_OFFSET << "\n"
         << endl;

    // set global position origin
    ROS_INFO("set GP origin");
    geographic_msgs::GeoPointStamped geo;
    geo.position.latitude = 33.595270;
    geo.position.longitude = 130.215496;
    set_gp_origin_pub.publish(geo);

    // geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5;
    // send a few setpoints before starting
    ROS_INFO("send a few setpoints");
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("finished sending a few setpoints");

    // mavros_msgs::SetMode offb_set_mode;
    // // for PX4
    // // offb_set_mode.request.custom_mode = "OFFBOARD";
    // // for Ardupilot
    // offb_set_mode.request.custom_mode = "GUIDED";
    // if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    // {
    //     ROS_INFO("GUIDED enabled");
    // }

    ROS_INFO("Change to GUIDED Mode");
    while (current_state.mode != "GUIDED")
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // arming
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("ARM sent %d", arm_cmd.response.success);
    }
    else
    {
        ROS_ERROR("Failed arming");
        return -1;
    }

    // request takeoff
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 1.5;
    if (takeoff_client.call(srv_takeoff))
    {
        ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
    }
    else
    {
        ROS_ERROR("Failed Takeoff");
        return -1;
    }

    sleep(10);

    // move foreward
    setHeading(0);
    setDestination(1.0, 0, 1.0);
    float tollorance = .35;
    local_pos_pub.publish(pose);

    sleep(10);
    ros::Time last_request = ros::Time::now();

    if (local_pos_pub)
    {
        for (int i = 10000; ros::ok() && i > 0; --i)
        {
            // ros::Time now = ros::Time::now();
            // double t = (now - last_request).toSec();
            // setDestination(cos(0.1 * t), sin(0.1 * t), 1.5);
            local_pos_pub.publish(pose);

            // float deltaX = abs(pose.pose.position.x - current_pose.pose.position.x);
            // float deltaY = abs(pose.pose.position.y - current_pose.pose.position.y);
            // float deltaZ = abs(pose.pose.position.z - current_pose.pose.position.z);
            // // cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
            // float dMag = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));
            // cout << dMag << endl;
            // if (dMag < tollorance)
            // {
            //     break;
            // }
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            // if (i == 1)
            // {
            //     ROS_INFO("Failed to reach destination. Stepping to next task.");
            // }
        }
        ROS_INFO("Done moving foreward.");
    }

    // land
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    if (land_client.call(srv_land) && srv_land.response.success)
        ROS_INFO("land sent %d", srv_land.response.success);
    else
    {
        ROS_ERROR("Landing failed");
        ros::shutdown();
        return -1;
    }

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    // ros::Time last_request = ros::Time::now();

    return 0;
}
