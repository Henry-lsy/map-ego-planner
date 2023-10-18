#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Visualizer for the planner
class Visualizer
{
private:
    // config contains the scale for some markers
    ros::NodeHandle nh;

    // These are publishers for path, waypoints on the trajectory,
    // the entire trajectory, the mesh of free-space polytopes,
    // the edge of free-space polytopes, and spheres for safety radius
    ros::Publisher routePub;
    ros::Publisher staywaypointPub;
    // ros::Publisher wayPointsPub;

public:
    Visualizer(ros::NodeHandle &nh_)
        : nh(nh_)
    {
        routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
        staywaypointPub = nh.advertise<visualization_msgs::Marker>("/visualizer/staywaypoints", 10);
    }

    // Visualize the trajectory and its front-end path
    inline void visualize(const std::vector<Eigen::Vector3d> &route)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "world";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.1;


        if (route.size() > 0)
        {
            bool first = true;
            Eigen::Vector3d last;
            for (auto it : route)
            {
                if (first)
                {
                    first = false;
                    last = it;
                    continue;
                }
                geometry_msgs::Point point;

                point.x = last(0);
                point.y = last(1);
                point.z = last(2);
                routeMarker.points.push_back(point);
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                routeMarker.points.push_back(point);
                last = it;
            }

            routePub.publish(routeMarker);
        }
    }

    // inline void visualizeStartGoal(const Eigen::Vector3d &center,
    //                                const double &radius,
    //                                const int sg)
    // {
    //     visualization_msgs::Marker sphereMarkers, sphereDeleter;

    //     sphereMarkers.id = sg;
    //     sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
    //     sphereMarkers.header.stamp = ros::Time::now();
    //     sphereMarkers.header.frame_id = "odom";
    //     sphereMarkers.pose.orientation.w = 1.00;
    //     sphereMarkers.action = visualization_msgs::Marker::ADD;
    //     sphereMarkers.ns = "StartGoal";
    //     sphereMarkers.color.r = 1.00;
    //     sphereMarkers.color.g = 0.00;
    //     sphereMarkers.color.b = 0.00;
    //     sphereMarkers.color.a = 1.00;
    //     sphereMarkers.scale.x = radius * 2.0;
    //     sphereMarkers.scale.y = radius * 2.0;
    //     sphereMarkers.scale.z = radius * 2.0;

    //     sphereDeleter = sphereMarkers;
    //     sphereDeleter.action = visualization_msgs::Marker::DELETEALL;

    //     geometry_msgs::Point point;
    //     point.x = center(0);
    //     point.y = center(1);
    //     point.z = center(2);
    //     sphereMarkers.points.push_back(point);

    //     if (sg == 0)
    //     {
    //         spherePub.publish(sphereDeleter);
    //         ros::Duration(1.0e-9).sleep();
    //         sphereMarkers.header.stamp = ros::Time::now();
    //     }
    //     spherePub.publish(sphereMarkers);
    // }
};

#endif