/**
 * @file ardu_guided.cpp
 * @brief ArduPilot Guided Mode control example node, written with MAVROS version 0.19.x, ArduPilot Flight
 * Stack and tested in Gazebo SITL
 * ホバリング、円軌道、上下運動、8の字飛行するプログラム
 * string MODEで飛行モードを切り替え
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/RCOut.h>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Global variables
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

mavros_msgs::RCOut current_rcout;
void rcout_cb(const mavros_msgs::RCOut::ConstPtr& msg) {
    current_rcout = *msg;
    // channels1, 2, 3, 4の値を表示
    // ROS_INFO("RC Channels 1-4: [%d, %d, %d, %d]",
    // current_rcout.channels[0],
    // current_rcout.channels[1],
    // current_rcout.channels[2],
    // current_rcout.channels[3]);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher set_gp_origin_pub = nh.advertise<geographic_msgs::GeoPointStamped>("mavros/global_position/set_gp_origin", 10);
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    ros::Subscriber rcout_sub = nh.subscribe<mavros_msgs::RCOut>("mavros/rc/out", 10, rcout_cb);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1000, imuCallback);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    ROS_INFO("INITILIZING...");

    // wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // set global position origin
    // 緯度、経度を指定
    ROS_INFO("set GP origin");
    geographic_msgs::GeoPointStamped geo;
    geo.position.latitude = 33.595270;
    geo.position.longitude = 130.215496;
    set_gp_origin_pub.publish(geo);


    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 1;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5; // Start at 2 meters above ground

    // Send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // Set mode to GUIDED
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";
    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("Guided enabled");
    }

    // ROS_INFO("Change to GUIDED Mode");
    // // プロポでモードをGuidedモードに変更する
    // while (current_state.mode != "GUIDED") {
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // Arm the drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }

    // request takeoff
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 1.5;
    if (takeoff_client.call(srv_takeoff)) {
        ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
    } else {
        ROS_ERROR("Failed Takeoff");
        return -1;
    }
    
    sleep(5);

    // send a few setpoints before starting
    ROS_INFO("send a few setpoints");
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("finished sending a few setpoints");
    
    sleep(5);

    ros::Time last_request = ros::Time::now();
    ros::Time start_time = ros::Time::now();
    ROS_INFO("moving start");

    if (local_pos_pub)
    {
        while (ros::ok() && (ros::Time::now() - start_time) < ros::Duration(120.0))
        {
            double time = (ros::Time::now() - last_request).toSec();
            // Circle
            pose.pose.position.x = 1.0 * cos(time);
            pose.pose.position.y = 1.0 * sin(time);
            pose.pose.position.z = 1.5;
            
            // Uncomment for up and down movement
            // pose.pose.position.z = 2 + 1 * sin(time * 2);
            
            // Uncomment for left and right movement
            // pose.pose.position.x = 5 * sin(time);

            local_pos_pub.publish(pose);

            ros::spinOnce();
            // rate.sleep();
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("Done moving foreward.");
    }
    // land
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    if (land_client.call(srv_land) && srv_land.response.success){
        ROS_INFO("land sent %d", srv_land.response.success);
    } else {
        ROS_ERROR("Landing failed");
        ros::shutdown();
        return -1;
    }

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
