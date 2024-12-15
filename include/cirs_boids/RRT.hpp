#ifndef BOIDSRRT_
#define BOIDSRRT_

// ompl stuff
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <Eigen/Core>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class RRT
{
private:
public:
    RRT();
    // ~RRT();

    std::shared_ptr<ob::RealVectorStateSpace> navSpace_;
    ompl::base::ProblemDefinitionPtr pdef_;
    ob::PlannerPtr planner_;
    og::PathSimplifierPtr simply;
    og::SimpleSetupPtr ss_;
    std::vector<Eigen::Vector3f> path_;

    std::vector<Eigen::Vector3f> doPlan(Eigen::Vector3f start, Eigen::Vector3f goal);
};

#endif