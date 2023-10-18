#ifndef GLOBALPLANNER_HPP
#define GLOBALPLANNER_HPP

#include <stdlib.h>

#include <ompl/util/Console.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>

#include <deque>
#include <memory>
#include <Eigen/Eigen>

#include "voxel_map.hpp"
#include "visualizer.hpp"

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>

namespace global_planner
{
struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
    }

};

class GlobalPlanner
{
private:
    global_planner::Config config;

    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    // ros::Subscriber targetSub;

    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;

public:
    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;

    typedef std::unique_ptr<GlobalPlanner> Ptr;

    GlobalPlanner(const global_planner::Config &conf,
                  ros::NodeHandle &nh_);
        // : config(conf),
        //   nh(nh_),
        //   mapInitialized(false),
        //   visualizer(nh);
    
    bool is_mapInitialized();

    void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);

    bool planPath(const Eigen::Vector3d &s,
                  const Eigen::Vector3d &g,
                  const double &timeout,
                  std::vector<Eigen::Vector3d> &p);
    
    inline bool if_collision(const Eigen::Vector3d& pos)
    {
        return voxelMap.query(pos);
    }
}; 
}

#endif