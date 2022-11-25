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

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
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
    ros::serviceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cm/takeoff");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // set global position origin
    ROS_INFO("set GP origin");
    geographic_msgs::GeoPointStamped geo;
    set_gp_origin_pub.publish(geo);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    // send a few setpoints before starting
    ROS_INFO("send a few setpoints");
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("finished sending a few setpoints");

    mavros_msgs::SetMode offb_set_mode;
    // for PX4
    // offb_set_mode.request.custom_mode = "OFFBOARD";
    // for Ardupilot
    offb_set_mode.request.custom_mode = "GUIDED";

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

    ros::Time last_request = ros::Time::now();

    ROS_INFO("Moving start");
    while (ros::ok())
    {
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
