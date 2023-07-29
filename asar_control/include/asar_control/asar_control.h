/******************************************************************************
# asar_planner.h:  GEN3 Arm Control                               #
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

#ifndef ASAR_CONTROL_H
#define ASAR_CONTROL_H

//* C

//* C++
#include <functional>
#include <memory>
#include <mutex>

//* External
// Pinocchio
#include <pinocchio/algorithm/frames.hpp>

// Ros related
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>

// Eigen
#include <Eigen/Dense>

// CODCS
#include <codcs_ik/codcs_ik.hpp>

// Boost
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>

// Eigen conversions
#include <eigen_conversions/eigen_msg.h>

// Actions
#include <actionlib/server/simple_action_server.h>

// RT
#include <rt_utils/rt_clock.h>

//* Internal
#include <asar_control/FollowCartesianTargetAction.h>
#include <asar_control/FollowJointTargetAction.h>
#include <asar_control/SolveIKAction.h>
// Trajectory Generator
#include <asar_control/MotionPlanner.h>
#include <asar_control/DynVarConfig.h>

using namespace Eigen;
using namespace actionlib;
using namespace realtime_utils;
using namespace codcs_ik;
using namespace rt_motion_planner;

namespace pin = pinocchio;
namespace asar_ns
{
  enum
  {
    ARM_UNINITIALIZED = 0,
    ARM_CONNECTED,
    ARM_READY,
    ARM_SERVO_MODE,
  };

  enum
  {
    FORCEPS_UNINITIALIZED = 0,
    FORCEPS_CONNECTED,
    FORCEPS_READY,
    FORCEPS_SERVO_MODE,
  };

  enum
  {
    ASAR_UNINITIALIZED = 0,
    ASAR_NOT_READY,
    ASAR_SERVO_MODE,
  };

  void poseSE3ToMsg(const pin::SE3 &e, geometry_msgs::Pose &m);
  void poseMsgToSE3(const geometry_msgs::Pose &m, pin::SE3 &e);
  void transformSE3ToMsg(const pin::SE3 &e, geometry_msgs::Transform &m);
  void transformMsgToSE3(const geometry_msgs::Transform &m, pin::SE3 &e);

  class AsarControl
  {
    typedef Matrix<double, 6, 1> Vector6d;
    typedef Matrix<double, 6, 1> Twist;

    enum
    {
      ACT_RESET = -1,
      ACT_NONE = 0,
      ACT_FOLLOWJOINT,
      ACT_FOLLOWCARTESIAN,
      ACT_SOLVEIK,
    };
    // const int NSEC2SEC = 1000000000;
    // const int NSEC2SEC_D = 1000000000.0;

    const double kDEG2RAD = (M_PI) / 180;
    const double kRAD2DEG = 180 / (M_PI);

    const double kMaxCartVel = 25.0;  //[mm/s]
    const double kMaxCartAcc = 100.0; //[mm/s2]

    const double kMaxSpeedLargeJoint = 1.39;         //[rad/s] Gen3Manual
    const double kMaxSpeedSmallJoint = 1.22;         //[rad/s] Gen3Manual
    const double kMaxSpeedForcepsJoint = 2.0;        //[rad/s]
    const double kMaxAccelerationLargeJoint = 5.2;   //[rad/s2] Gen3Manual
    const double kMaxAccelerationSmallJoint = 10.0;  //[rad/s2] Gen3Manual
    const double kMaxAccelerationForcepsJoint = 5.0; //[rad/s2]

    const double kMaxJointVelLower = 0.5; // Limit to 50% Max Speed
    const double kMaxJointVelUpper = 0.8; // Limit to 80% Max Speed
    const double kMaxJointAccLower = 0.5; // Limit to 80% Max Acc
    const double kMaxJointAccUpper = 0.8; // Limit to 80% Max Acc

    // const double kMaxJointVel = 0.87; //[rad/s]=50 deg/s (real:150 deg/s)
    // const double kMaxJointAcc = 0.44; //[rad/s2]=25deg/s2

    VectorXd kJointVelLimits;
    VectorXd kJointAccLimits;

    // const VectorXd kJointAccLimits{
    //     {0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 1.0, 1.0}};
    // const VectorXd kJointAccLimits{
    //     {1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 4.0, 4.0}};
    const int kNumberDofArm = 7;
    const int kNumberDofForceps = 3;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor
    AsarControl(ros::NodeHandle &node_handle, bool *kill_this_node);
    // Destructor
    ~AsarControl();

    // Callbacks
    void actFollowCartesianTargetCb(
        const asar_control::FollowCartesianTargetGoalConstPtr &goal);
    void actFollowJointTargetCb(
        const asar_control::FollowJointTargetGoalConstPtr &goal);
    void actSolveIKCb(const asar_control::SolveIKGoalConstPtr &goal);
    void actCancelCb();
    void actCancelPlanCb();
    void subUpdateArmJointStateCb(const sensor_msgs::JointState::ConstPtr &msg);
    void
         subUpdateForcepsJointStateCb(const sensor_msgs::JointState::ConstPtr &msg);
    void subUpdateArmStateCb(const std_msgs::Int32MultiArray::ConstPtr &msg);
    void
         subUpdateForcepsStateCb(const std_msgs::Int32MultiArray::ConstPtr &msg);
    void subUpdateForcepsGripAngleCb(const std_msgs::Float32::ConstPtr &msg);
    void setDynVarCb(asar_control::DynVarConfig &config, uint32_t level);

    bool startNewJointTraj(VectorXd des_joint_pos);
    int  generateJointTrajStep(VectorXd &joint_traj_step);

    // Accesors
    VectorXd get_joint_state() { return act_joints_state_; };
    pin::SE3 get_ee_pose() { return ee_pose_act_; }

    // Mutators
    void update_arm_act_vel();
    void update_arm_act_acc();
    void update_forceps_act_vel();
    void update_forceps_act_acc();
    void updateAsarState();

    bool solveIK(const pin::SE3 &Xd, VectorXd &q_sol);
    void updateFK();

    void control_loop();

    int  followCartesianTarget(const pin::SE3 &Xd);
    int  followJointTarget(const sensor_msgs::JointState q_target);
    void sendCmdToControllers(VectorXd joint_cmd_step);
    void publishStates();

private:
    bool *kill_this_node_;

    std::string group_name_;

    // Trajectory Generation
    OnlineTrajGen *joint_traj_gen_;

    // CODCS
    std::unique_ptr<CODCS_IK> ik_solver_;

    // ROS
    ros::NodeHandle nh_;

    // Suscribers
    ros::Subscriber sub_arm_state_;
    ros::Subscriber sub_arm_joint_state_;
    ros::Subscriber sub_forceps_state_;
    ros::Subscriber sub_forceps_joint_state_;
    ros::Subscriber sub_forceps_grip_angle_;

    // Publishers
    ros::Publisher              pub_arm_joint_cmd_;
    std_msgs::Float64MultiArray pub_arm_joint_cmd_msg_;
    ros::Publisher              pub_forceps_joint_cmd_;
    std_msgs::Float64MultiArray pub_forceps_joint_cmd_msg_;
    ros::Publisher              pub_asar_ee_pose_;
    geometry_msgs::PoseStamped  pub_asar_ee_pose_msg_;
    ros::Publisher              pub_asar_flange_pose_;
    geometry_msgs::PoseStamped  pub_asar_flange_pose_msg_;
    ros::Publisher              pub_asar_joint_states_;
    sensor_msgs::JointState     pub_asar_joint_states_msg_;
    ros::Publisher              pub_asar_state_;
    std_msgs::Int32             pub_asar_state_msg_;

    // Action Server
    std::shared_ptr<
        SimpleActionServer<asar_control::FollowCartesianTargetAction>>
        actFollowCartesianTarget_;
    std::shared_ptr<SimpleActionServer<asar_control::FollowJointTargetAction>>
        actFollowJointTarget_;
    std::shared_ptr<SimpleActionServer<asar_control::SolveIKAction>>
        actSolveIK_;

    // Robot Variables
    //   int robot_id;
    //   std::string prefix_;
    int n_dof_arm_;
    int n_dof_forceps_;
    int n_dof_total_;

    // State variables
    int arm_state_;
    int forceps_state_;
    int asar_state_;

    // Time variables
    double  cycle_t_;
    int     cycle_t_us_;
    double  cycle_t_us_d_;
    RTClock rt_clock_;

    // Joint variables
    VectorXd act_arm_joint_pos_;
    VectorXd act_arm_joint_vel_;
    VectorXd act_arm_joint_acc_;
    VectorXd prev_arm_joint_pos_;
    VectorXd prev_arm_joint_vel_;

    VectorXd act_forceps_joint_pos_;
    VectorXd act_forceps_joint_vel_;
    VectorXd act_forceps_joint_acc_;
    VectorXd prev_forceps_joint_pos_;
    VectorXd prev_forceps_joint_vel_;

    VectorXd act_joints_state_;
    VectorXd act_joints_vel_;
    VectorXd act_joints_acc_;

    VectorXd joint_cmd_step_;
    double   gripper_angle_;

    // Pose Variables
    pin::SE3 ee_pose_act_;
    pin::SE3 flange_pose_act_;

    // Frames Index
    pin::FrameIndex flange_idx;
    pin::FrameIndex ee_idx;

    // Dynamic Reconfigure
    dynamic_reconfigure::Server<asar_control::DynVarConfig> dyn_srv_;
    dynamic_reconfigure::Server<asar_control::DynVarConfig>::CallbackType
        dyn_cb_;

    std::string urdf_param_;

    // Mutex
    boost::mutex mtx_Act_track_;
    boost::mutex mtx_Act_plan_;
    int          m_curAct_track_;
    int          m_curAct_plan_;

    // Limits
    double joint_vel_limit_;
    double joint_acc_limit_;

    // Flags
    bool flag_arm_joint_state_initialized_;
    bool flag_forceps_joint_state_initialized_;
    bool flag_asar_joint_state_initialized_;
  };

} // namespace asar_ns

#endif