#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/ego_replan_fsm.h>
#include "quadrotor_msgs/PositionCommand.h"

using namespace ego_planner;

std::vector<Eigen::VectorXd> readWaypointsFromFile(const std::string & path)
{
    std::vector<Eigen::VectorXd> waypoints;
    Eigen::VectorXd waypoint(5);

    YAML::Node node = YAML::LoadFile(path);
    waypoints.reserve(node.size());
    for(int i = 0; i < node.size(); i++)
    {
      waypoint(0) = node[i]["x"].as<double>();
      waypoint(1) = node[i]["y"].as<double>();
      waypoint(2) = node[i]["z"].as<double>();
      waypoint(3) = node[i]["yaw"].as<double>();
      waypoint(4) = node[i]["stay"].as<double>();
      waypoints.push_back(waypoint);
    }
    return waypoints;
}

ros::Publisher waypoints_pub;
ros::Publisher ref_yaw_pub;


void visualize(const std::vector<Eigen::Vector4d> &waypoints, double scale=0.1)
{
    visualization_msgs::Marker wayPointsMarker;

    wayPointsMarker.id = 0;
    wayPointsMarker.type = visualization_msgs::Marker::LINE_LIST;
    wayPointsMarker.header.stamp = ros::Time::now();
    wayPointsMarker.header.frame_id = "world";
    wayPointsMarker.pose.orientation.w = 1.00;
    wayPointsMarker.action = visualization_msgs::Marker::ADD;
    wayPointsMarker.ns = "wayPointsMarker";
    wayPointsMarker.color.r = 1.00;
    wayPointsMarker.color.g = 0.00;
    wayPointsMarker.color.b = 0.00;
    wayPointsMarker.color.a = 1.00;
    wayPointsMarker.scale.x = scale;


    if (waypoints.size() > 0)
    {
        bool first = true;
        Eigen::Vector4d last;
        for (auto it : waypoints)
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
            wayPointsMarker.points.push_back(point);
            point.x = it(0);
            point.y = it(1);
            point.z = it(2);
            wayPointsMarker.points.push_back(point);
            last = it;
        }

        waypoints_pub.publish(wayPointsMarker);
    }
}

void ref_yaw_publish(const double & ref_yaw)
{
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";

  cmd.yaw = ref_yaw;

  ref_yaw_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");
  ros::Rate r(20);  // 20hz

  std::string waypoint_file;
  if (!nh.getParam("fsm/waypoint_file", waypoint_file)) {
      ROS_ERROR("Failed to retrieve parameter 'file_path'");
      return 1;
  }

  waypoints_pub = nh.advertise<visualization_msgs::Marker>("/visualizer/stay_waypoints", 10);
  ref_yaw_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/yaw", 10);

  std::vector<Eigen::VectorXd> all_waypoints;
  all_waypoints = readWaypointsFromFile(waypoint_file);

  if(all_waypoints.empty())
  {
    std::cout << "No waypoints in the file. Something goes wrong!" << std::endl;
    return 0;
  }

  //extract stay waypoint
  std::vector<Eigen::Vector4d> stay_waypoints;
  std::vector<Eigen::Vector4d> move_waypoints;

  for(auto waypoint: all_waypoints)
  {
    if(waypoint(4) == 1)
    {
      stay_waypoints.push_back(waypoint.head<4>());
    }
    else
    {
      move_waypoints.push_back(waypoint.head<4>());
    }
  }

  // ego planner start
  EGOReplanFSM rebo_replan;
  rebo_replan.init(nh);
  auto it =  all_waypoints.begin();
  bool if_move = false;
  while(ros::ok())
  {
    visualize(stay_waypoints, 0.3);
    // visualize(move_waypoints);

    if(it == all_waypoints.end())
    {
      it =  all_waypoints.begin();
    }

    if(if_move && rebo_replan.ifCloseToTarget(0.5))
    {
      
        Eigen::Vector3d target_pos(it->head<3>());
        ref_yaw_publish((*it)(3));
        if(rebo_replan.trigger_by_one_waypoint(target_pos))
        {
          if_move = (*(it))(4) == 1 ? false : true;
          it++;
        }
    }
    else if(!rebo_replan.if_have_target() && it != all_waypoints.end())
    {
      // wait for 2 seconds

      // take a photo in here

      // wait for 1 second

      // trigger another waypoint
      // Eigen::Vector3d target_pos;
        ros::Duration(2.0).sleep();
        Eigen::Vector3d target_pos(it->head<3>());
        ref_yaw_publish((*it)(3));
        if(rebo_replan.trigger_by_one_waypoint(target_pos))
        {
          if_move = (*(it))(4) == 1 ? false : true;
          it++;
        }
    }

    ros::spinOnce();
    r.sleep();
  }
  // rebo_replan.wp_record.writeWaypointsToFile();

  return 0;
}
