#include "global_planner.hpp"

#include <vector>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh_ = ros::NodeHandle("~");

    global_planner::GlobalPlanner global_planner(global_planner::Config(nh_), nh_);
    ros::Rate r(20);  // 20hz

    Eigen::Vector3d start_wp; 
    start_wp << 0, 0, 1;
    Eigen::Vector3d goal_wp;
    goal_wp << -12.387, -13.155, 1;
    Eigen::Vector3d lb;
    lb << -20, -20, 0;
    Eigen::Vector3d ub;
    ub << 20, 20, 3;
    std::vector<Eigen::Vector3d> route;
    bool success;

    while(ros::ok())
    {
        if(global_planner.is_mapInitialized())
        {
            ros::Duration(0.1).sleep();
            success = global_planner.planPath(start_wp, goal_wp, 0.1, route);
            // ros::Duration(5.0).sleep();
            global_planner.visualizer.visualize(route);
            // if(success)
            // {
            //     std::cout << "problem start to solve" << std::endl;
            //     break;
            // }
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
};