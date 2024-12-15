#include <cirs_boids/RRT.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/ScopedState.h>

namespace ob = ompl::base;

RRT::RRT()
{
    std::cout << "loading RRT planner\n\n";
    // create a state space
    navSpace_ = std::make_shared<ob::RealVectorStateSpace>(3);
    navSpace_->setName("Position");
    navSpace_->setDimensionName(0, "X");
    navSpace_->setDimensionName(1, "Y");
    navSpace_->setDimensionName(2, "Z");
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-50);
    bounds.setHigh(50);
    bounds.setLow(2, 1);
    bounds.setHigh(2, 15);
    navSpace_->setBounds(bounds);
    navSpace_->setName("navSpace");
    // percentage of the space maximun extent ??
    navSpace_->setLongestValidSegmentFraction(0.03);
    // std::cout << navSpace_->getMaximumExtent() << "\n";

    std::cout << "setting simple setup\n";
    ss_ = std::make_shared<og::SimpleSetup>(navSpace_);
    auto si = ss_->getSpaceInformation();
    std::cout << "setting validity checker setup\n";

    std::cout << "setting planner \n";
    planner_ = (std::make_shared<og::RRT>(si));
    planner_->params().setParam("range", "0.5");
    planner_->params().setParam("intermediate_states", "1");
    planner_->getSpecs();
    ss_->setPlanner(planner_);

    std::cout << "printing setup\n";
    ss_->setup();
    ss_->print();
    std::cout << "done printing setup\n";
    std::cout << "loaded RRT planner\n\n";
}

std::vector<Eigen::Vector3f> RRT::doPlan(Eigen::Vector3f start, Eigen::Vector3f goal)
{
    std::cout << "actually planning\n";
    this->ss_->clear();

    ob::ScopedState<> start_stt(navSpace_);
    start_stt[0] = start[0];
    start_stt[1] = start[1];
    start_stt[2] = start[2];

    ob::ScopedState<> goal_stt(navSpace_);
    goal_stt[0] = goal[0];
    goal_stt[1] = goal[1];
    goal_stt[2] = goal[2];
    std::cout << "requesting solution to:\n\n";
    ss_->setStartAndGoalStates(start_stt, goal_stt);
    ss_->print();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    // attempt to solve the problem
    ob::PlannerStatus status = ss_->solve(10.0);
    std::cout << status << "\n";

    if (ss_->haveExactSolutionPath())
    {
        std::cout << "Found solution:\n";
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

        og::PathGeometric pathres = ss_->getSolutionPath();
        std::cout << pathres.getStateCount() << "\n";
        pathres.interpolate(pathres.getStateCount() * 5);

        // start
        path_.push_back(start);
        Eigen::Vector3f point;
        for (auto stt : pathres.getStates())
        {
            ob::ScopedState<ob::CompoundStateSpace> sstt(navSpace_, stt);
            point.x() = sstt.reals().at(0);
            point.y() = sstt.reals().at(1);
            point.z() = sstt.reals().at(2);
            path_.push_back(point);
        }

        // goal
        path_.at(path_.size() - 1) = goal;
    }
    else
    {
        std::cout << "Solution not found\n";
    }
    std::cout << "done planning\n";

    return path_;
}
