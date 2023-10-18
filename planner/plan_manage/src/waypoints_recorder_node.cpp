#include "plan_manage/yaml_util.hpp"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <cmath>


Eigen::Vector4d waypoint;


struct Quaternion {
    double w, x, y, z;
};

struct EulerAngle {
    double roll, pitch, yaw;
};

EulerAngle quaternionToEuler(const Quaternion& q) {
    EulerAngle euler;

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    euler.roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        euler.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler.pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    euler.yaw = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}




void odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    waypoint(0) = msg->pose.pose.position.x;
    waypoint(1) = msg->pose.pose.position.y;
    waypoint(2) = msg->pose.pose.position.z;

//odom_acc_ = estimateAcc( msg );
    Quaternion quaternion;
    quaternion.w = msg->pose.pose.orientation.w;
    quaternion.x = msg->pose.pose.orientation.x;
    quaternion.y = msg->pose.pose.orientation.y;
    quaternion.z = msg->pose.pose.orientation.z;
    EulerAngle euler = quaternionToEuler(quaternion);
    // std::cout << "Yaw: " << euler.yaw << std::endl;
    waypoint(3) = euler.yaw;
}
  
int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoints_recorder_node");
    ros::NodeHandle nh("~");
    ros::Rate r(20);  // 20hz
    WaypointRecorder waypoint_recorder;
    ros::Subscriber odom_sub = nh.subscribe( "/odom", 1,  odometryCallback); 

    std::string file_path;
    if (!nh.getParam("file_path", file_path)) {
        ROS_ERROR("Failed to retrieve parameter 'file_path'");
        return 1;
    }
    char key;
    while(ros::ok())
    {
        std::cout << "press any key to record one waypoint. Press \"q\" to exit." << std::endl;
        std::cin >> key;
        if(key == 'q')
        {
           waypoint_recorder.writeWaypointsToFile(file_path);
           return 0;
        }
        else
        {
            waypoint_recorder.recordWaypoint(waypoint);
        }
        ros::spinOnce();
        r.sleep();
    }

}