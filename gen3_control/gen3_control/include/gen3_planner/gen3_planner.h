/******************************************************************************
# gen3_planner.h:  GEN3 Arm Control                               #
# Copyright (c) 2021                                                          #
# Hasegawa Laboratory at Nagoya University                                    #
#                                                                             #
# Redistribution and use in source and binary forms, with or without          #
# modification, are permitted provided that the following conditions are met: #
#                                                                             #
#     - Redistributions of source code must retain the above copyright        #
#       notice, this list of conditions and the following disclaimer.         #
#     - Redistributions in binary form must reproduce the above copyright     #
#       notice, this list of conditions and the following disclaimer in the   #
#       documentation and/or other materials provided with the distribution.  #
#     - Neither the name of the Hasegawa Laboratory nor the                   #
#       names of its contributors may be used to endorse or promote products  #
#       derived from this software without specific prior written permission. #
#                                                                             #
# This program is free software: you can redistribute it and/or modify        #
# it under the terms of the GNU Lesser General Public License LGPL as         #
# published by the Free Software Foundation, either version 3 of the          #
# License, or (at your option) any later version.                             #
#                                                                             #
# This program is distributed in the hope that it will be useful,             #
# but WITHOUT ANY WARRANTY; without even the implied warranty of              #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                #
# GNU Lesser General Public License LGPL for more details.                    #
#                                                                             #
# You should have received a copy of the GNU Lesser General Public            #
# License LGPL along with this program.                                       #
# If not, see <http://www.gnu.org/licenses/>.                                 #
#                                                                             #
# #############################################################################
#                                                                             #
#   Author: Jacinto Colan, email: colan@robo.mein.nagoya-u.ac.jp           #
#                                                                             #
# ###########################################################################*/

#ifndef GEN3_PLANNER_H
#define GEN3_PLANNER_H

// C++
#include <mutex>
#include <functional>
#include <memory>

// Ros related
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

// Eigen
#include <Eigen/Dense>

// Orocos KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

// KDL Parser
#include <kdl_parser/kdl_parser.hpp>

// Boost
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

// Track IK
#include <trac_ik/trac_ik.hpp>

// Trajectory Generator
#include <gen3_planner/MotionPlanner.h>

// RT

// Dynamic reconfigure
#include <gen3_control/DynVarConfig.h>
#include <dynamic_reconfigure/server.h>

// Eigen conversions
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

// Actions
#include <actionlib/server/simple_action_server.h>
#include <gen3_control/FollowJointCmdAction.h>
#include <gen3_control/FollowToolCmdAction.h>

// Package headers
#include <gen3_control/gen3_status_code.h>

using namespace Eigen;
using namespace KDL;
using namespace rt_motion_planner;
using namespace actionlib;

namespace gen3_nu
{

  class Gen3Planner
  {

    // const int NSEC2SEC = 1000000000;
    // const int NSEC2SEC_D = 1000000000.0;
    const double kDEG2RAD = (M_PI) / 180;
    const double kRAD2DEG = 180 / (M_PI);

    // Joint limits for planning
    const double kMaxJointVel = 0.87; //[rad/s]=50 deg/s (real:150 deg/s)
    const double kMaxJointAcc = 0.44; //[rad/s2]=25deg/s2

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor
    Gen3Planner(ros::NodeHandle &node_handle, bool *kill_this_node);
    ~Gen3Planner();

    // Callbacks
    void actFollowJointCmdCb(const gen3_control::FollowJointCmdGoalConstPtr &goal);
    void actFollowToolCmdCb(const gen3_control::FollowToolCmdGoalConstPtr &goal);
    void actCancelCb();
    void updateJointStateCb(const sensor_msgs::JointState::ConstPtr &msg);
    void updateArmStateCb(const std_msgs::Int32MultiArray::ConstPtr &msg);
    void setDynVarCb(gen3_control::DynVarConfig &config, uint32_t level);

    // Trajectory generation
    bool start_joint_traj(VectorXd des_joint_pos);
    int generate_joint_traj(VectorXd &joint_traj_step);

    // Kinematics
    int solveFK(VectorXd joint_val, Vector3d &B_p_TOOL, Matrix3d &R_B_TOOL);
    int solveIK(const Vector3d pd, const Matrix3d Rd, VectorXd &joint_cmd);

    void update_act_vel();
    void update_act_acc();

    void control_loop();

    int followJointCmd(VectorXd joint_cmd);
    int followToolCmd(Vector3d pd, Matrix3d Rd);
    void sendCmdController(VectorXd joint_cmd_step);

  private:
    bool *kill_this_node_;

    // ROS
    ros::NodeHandle nh_;

    // Suscribers
    ros::Subscriber sub_joint_state_;
    ros::Subscriber sub_arm_state_;

    // Publishers
    ros::Publisher pub_joint_cmd_;
    ros::Publisher pub_tool_pose_;
    std_msgs::Float64MultiArray pub_joint_cmd_msg_;
    geometry_msgs::PoseStamped pub_tool_pose_msg_;

    // Action Server
    std::shared_ptr<SimpleActionServer<gen3_control::FollowJointCmdAction>>
        actFollowJointCmd;
    std::shared_ptr<SimpleActionServer<gen3_control::FollowToolCmdAction>> actFollowToolCmd;

    // Robot Variables
    int robot_id;
    std::string prefix_;
    int n_dof_;

    // Time variables
    double cycle_t_;
    int cycle_t_us_;
    double cycle_t_us_d_;

    // Trajectory Generation
    OnlineTrajGen *joint_traj_gen_;

    // Joint variables
    VectorXd act_joint_pos_;
    VectorXd act_joint_vel_;
    VectorXd act_joint_acc_;

    // Dynamic Reconfigure
    dynamic_reconfigure::Server<gen3_control::DynVarConfig> dyn_srv_;
    dynamic_reconfigure::Server<gen3_control::DynVarConfig>::CallbackType dyn_cb_;

    double joint_vel_limit_;
    double joint_acc_limit_;

    // FK KDL
    boost::scoped_ptr<KDL::ChainFkSolverPos> fk_solver_;

    // TracIK
    boost::scoped_ptr<TRAC_IK::TRAC_IK> tracik_solver_;

    // Limits
    // double max_joint_vel_;
    // double max_joint_acc_;

    // Temporary variables for KDL
    KDL::Chain kdl_chain_;
    KDL::JntArray nominal_;
    KDL::JntArray qtmp_;
    KDL::JntArray kdl_ll_, kdl_ul_;
    KDL::Frame xtmp_;
    KDL::Twist xdot_temp_;
    KDL::JntArray qdot_tmp_;

    std::string urdf_param_;

    // Mutex
    boost::mutex mtx_Act_;
    int m_curAct;

    // Current state of the arm
    int state_;

    // Flags
    bool flag_joint_pos_initialized_;
  };

} // namespace gen3_nu

#endif