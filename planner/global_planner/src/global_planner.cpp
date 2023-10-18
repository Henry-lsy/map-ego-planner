#include "global_planner.hpp"


global_planner::GlobalPlanner::GlobalPlanner(const global_planner::Config &conf,
                                                  ros::NodeHandle &nh_)
    : config(conf),
        nh(nh_),
        mapInitialized(false),
        visualizer(nh)
{
    const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

    const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

    voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

    mapSub = nh.subscribe(config.mapTopic, 1, &GlobalPlanner::mapCallBack, this,
                            ros::TransportHints().tcpNoDelay());
}
    
bool global_planner::GlobalPlanner::is_mapInitialized()
{
    return mapInitialized;
}

void global_planner::GlobalPlanner::mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (!mapInitialized)
    {
        std::cout << "Map is create" << std::endl;
        size_t cur = 0;
        const size_t total = msg->data.size() / msg->point_step;
        float *fdata = (float *)(&msg->data[0]);
        for (size_t i = 0; i < total; i++)
        {
            cur = msg->point_step / sizeof(float) * i;

            if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
            {
                continue;
            }
            voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                    fdata[cur + 1],
                                                    fdata[cur + 2]));
        }

        voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

        mapInitialized = true;
    }
}

bool global_planner::GlobalPlanner::planPath(const Eigen::Vector3d &s,
                        const Eigen::Vector3d &g,
                        const double &timeout,
                        std::vector<Eigen::Vector3d> &p)
{
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));
    Eigen::Vector3d lb, hb;
    lb << config.mapBound[0], config.mapBound[2], config.mapBound[4];
    hb << config.mapBound[1], config.mapBound[3], config.mapBound[5];

    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, 0.0);
    bounds.setHigh(0, hb(0) - lb(0));
    bounds.setLow(1, 0.0);
    bounds.setHigh(1, hb(1) - lb(1));
    bounds.setLow(2, 0.0);
    bounds.setHigh(2, hb(2) - lb(2));
    space->setBounds(bounds);

    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    si->setStateValidityChecker(
        [&](const ompl::base::State *state)
        {
            const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
            const Eigen::Vector3d position(lb(0) + (*pos)[0],
                                            lb(1) + (*pos)[1],
                                            lb(2) + (*pos)[2]);
            return voxelMap.query(position) == 0;
        });
    si->setup();

    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

    ompl::base::ScopedState<> start(space), goal(space);
    start[0] = s(0) - lb(0);
    start[1] = s(1) - lb(1);
    start[2] = s(2) - lb(2);
    goal[0] = g(0) - lb(0);
    goal[1] = g(1) - lb(1);
    goal[2] = g(2) - lb(2);
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);
    pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
    auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    ompl::base::PlannerStatus solved;
    solved = planner->ompl::base::Planner::solve(timeout);

    double cost = INFINITY;
    if (solved && pdef->hasExactSolution())
    {
        p.clear();
        const ompl::geometric::PathGeometric path_ =
            ompl::geometric::PathGeometric(
                dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
        for (size_t i = 0; i < path_.getStateCount(); i++)
        {
            const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            p.emplace_back(lb(0) + state[0], lb(1) + state[1], lb(2) + state[2]);
        }
        cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
        return true;
    }

    return false;
}

