#ifndef CAR_
#define CAR_

#include <iauv_motion_planner/Planner.h>

// ompl stuff
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

namespace iauv_motion_planner
{
    IAUVPLANNER_CLASS_FORWARD(CarPlanner);
    class CarPlanner : public Planner
    {
    private:
    public:
        std::shared_ptr<ob::RealVectorStateSpace> navSpace_;
        ompl::base::ProblemDefinitionPtr pdef_;
        ob::PlannerPtr planner_;
        og::PathSimplifierPtr simply;
        og::SimpleSetupPtr ss_;

        CarPlanner(ros::NodeHandle &nh);

        void CarPlanner::propagate(const oc::SpaceInformation *si, const ob::State *start, const oc::Control *control, double duration, ob::State *result);

        nav_msgs::Path doPlan(std::vector<double> start, std::vector<double> goal);

        double width_;
        double length_;
    };

} // namespace iauv_motion_planner

#endif