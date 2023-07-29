#ifndef ASAR_PLANNER_H
#define ASAR_PLANNER_H

//* C

//* C++

//* External
// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
// ROS
#include <ros/ros.h>
#include <ros/package.h>
// Eigen
#include <Eigen/Dense>
// Actions
#include <actionlib/server/simple_action_server.h>

//* Internal
// CODCS
#include <codcs_ik/codcs_ik.hpp>
// Actions
#include <asar_control/SolveIKAction.h>

using namespace Eigen;
using namespace actionlib;
using namespace codcs_ik;

namespace pin = pinocchio;

namespace asar_ns
{
    class AsarPlanner
    {
        enum
        {
            ACT_RESET = -1,
            ACT_NONE = 0,
            ACT_SOLVEIK,
        };

    private:
        ros::NodeHandle nh_;
        bool *kill_this_node_;

        int n_joints_;
        VectorXd act_joints_state_;

        // CODCS
        std::unique_ptr<CODCS_IK> ik_solver_;

        // Action Server
        std::shared_ptr<SimpleActionServer<asar_control::SolveIKAction>> actSolveIK_;

        // Mutex
        boost::mutex mtx_Act_;
        int m_curAct;

    public:
        AsarPlanner(ros::NodeHandle &node_handle, bool *kill_this_node);
        ~AsarPlanner();

        // Callbacks
        void actSolveIKCb(const asar_control::SolveIKGoalConstPtr &goal);
        void actCancelCb();
        bool solveIK(const pin::SE3 &Xd, VectorXd q_sol);

        void solveFK();
    };

    // AsarPlanner::AsarPlanner(ros::NodeHandle &node_handle, bool *kill_this_node) {}

    // AsarPlanner::~AsarPlanner() {}

} // namespace asar_ns

#endif