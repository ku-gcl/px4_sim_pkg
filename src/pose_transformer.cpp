#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// Declare the publisher globally
ros::Publisher pub;

// Function to transform data from mocap_optitrack (FLU) to local coordinate system (ENU)
// and publish it on /mavros/vision_pose/pose
// This code is meant to be launched via experiment.launch

void callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
    geometry_msgs::PoseStamped transformed_pose;

    // Copy the header
    transformed_pose.header = data->header;

    // Apply coordinate transformation
    transformed_pose.pose.position.x = -data->pose.position.y;
    transformed_pose.pose.position.y = data->pose.position.x;
    transformed_pose.pose.position.z = data->pose.position.z;

    transformed_pose.pose.orientation.x = -data->pose.orientation.y;
    transformed_pose.pose.orientation.y = data->pose.orientation.x;
    transformed_pose.pose.orientation.z = data->pose.orientation.z;
    transformed_pose.pose.orientation.w = data->pose.orientation.w;

    // Publish the transformed pose
    pub.publish(transformed_pose);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_transformer_node");
    ros::NodeHandle nh;

    // Initialize the publisher
    pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    // Subscribe to the mocap_node topic
    ros::Subscriber sub = nh.subscribe("/mocap_node/Robot_1/pose", 10, callback);

    ros::spin();
    return 0;
}
