/******************************************************************************
# asar_control.cpp:  Autonomous Surgical Assistant Robot Controller           #
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
#   Author: Jacinto Colan, email: colan@robo.mein.nagoya-u.ac.jp              #
#                                                                             #
# ###########################################################################*/

#include <asar_control/asar_control.h>

namespace asar_ns
{

  bool add_noise, debug_info;
  double noiseMean, noiseVar;
  

  std::default_random_engine generator;

  void poseSE3ToMsg(const pin::SE3 &e, geometry_msgs::Pose &m)
  {
    m.position.x = e.translation()[0];
    m.position.y = e.translation()[1];
    m.position.z = e.translation()[2];
    Eigen::Quaterniond q = (Eigen::Quaterniond)e.rotation();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    m.orientation.w = q.w();
    if (m.orientation.w < 0)
    {
      m.orientation.x *= -1;
      m.orientation.y *= -1;
      m.orientation.z *= -1;
      m.orientation.w *= -1;
    }
  }

  void transformSE3ToMsg(const pin::SE3 &e, geometry_msgs::Transform &m)
  {
    m.translation.x = e.translation()[0];
    m.translation.y = e.translation()[1];
    m.translation.z = e.translation()[2];
    Eigen::Quaterniond q = (Eigen::Quaterniond)e.rotation();
    m.rotation.x = q.x();
    m.rotation.y = q.y();
    m.rotation.z = q.z();
    m.rotation.w = q.w();
    if (m.rotation.w < 0)
    {
      m.rotation.x *= -1;
      m.rotation.y *= -1;
      m.rotation.z *= -1;
      m.rotation.w *= -1;
    }
  }

  void poseMsgToSE3(const geometry_msgs::Pose &m, pin::SE3 &e)
  {
    e = pin::SE3(pin::SE3::Quaternion(m.orientation.w, m.orientation.x,
                                      m.orientation.y, m.orientation.z),
                 pin::SE3::Vector3(m.position.x, m.position.y, m.position.z));
  }

  void transformMsgToSE3(const geometry_msgs::Transform &m, pin::SE3 &e)
  {
    e = pin::SE3(
        pin::SE3::Quaternion(m.rotation.w, m.rotation.x, m.rotation.y,
                             m.rotation.z),
        pin::SE3::Vector3(m.translation.x, m.translation.y, m.translation.z));
  }

  // Constructor
  AsarControl::AsarControl(ros::NodeHandle &node_handle, bool *kill_this_node)
      : nh_(node_handle), n_dof_arm_(kNumberDofArm),
        n_dof_forceps_(kNumberDofForceps)
  {
    kill_this_node_ = kill_this_node;
    n_dof_total_ = n_dof_arm_ + n_dof_forceps_ - 1; // Remove gripper angle

    // Defining Joint Limits
    kJointVelLimits = VectorXd::Zero(9);
    kJointAccLimits = VectorXd::Zero(9);
    kJointVelLimits << kMaxJointVelLower * kMaxSpeedLargeJoint,
        kMaxJointVelLower * kMaxSpeedLargeJoint,
        kMaxJointVelLower * kMaxSpeedLargeJoint,
        kMaxJointVelUpper * kMaxSpeedLargeJoint,
        kMaxJointVelUpper * kMaxSpeedSmallJoint,
        kMaxJointVelUpper * kMaxSpeedSmallJoint,
        kMaxJointVelUpper * kMaxSpeedSmallJoint, kMaxSpeedForcepsJoint,
        kMaxSpeedForcepsJoint;

    kJointAccLimits << kMaxJointAccLower * kMaxAccelerationLargeJoint,
        kMaxJointAccLower * kMaxAccelerationLargeJoint,
        kMaxJointAccLower * kMaxAccelerationLargeJoint,
        kMaxJointAccUpper * kMaxAccelerationLargeJoint,
        kMaxJointAccUpper * kMaxAccelerationSmallJoint,
        kMaxJointAccUpper * kMaxAccelerationSmallJoint,
        kMaxJointAccUpper * kMaxAccelerationSmallJoint,
        kMaxAccelerationForcepsJoint, kMaxAccelerationForcepsJoint;

    //* ROS PARAMETER SERVER
    // Thread Sampling Time
    if (nh_.getParam("asar_cyclic_time_usec", cycle_t_us_))
    {
      ROS_INFO_STREAM(
          "Thread sampling time obtained from ROS Parameter server as: "
          << cycle_t_us_ << " [us] ");
    }
    else
    {
      ROS_INFO_STREAM("No sampling time information in parameter server, using "
                      "default 2ms");
      cycle_t_us_ = 2000;
    }

    if (!nh_.getParam("group_name", group_name_))
    {
      group_name_ = "/unit0";
    }

    if (!nh_.getParam("add_noise", add_noise))
    {
      add_noise = false;
    }

    if (!nh_.getParam("noise_mean", noiseMean))
    {
      noiseMean = 0;
    }

    if (!nh_.getParam("noise_var", noiseVar))
    {
      noiseVar = 0;
    }

    //* Subscribers
    sub_arm_joint_state_ = nh_.subscribe(
        "arm/joint_states", 1, &AsarControl::subUpdateArmJointStateCb, this);
    sub_forceps_joint_state_ =
        nh_.subscribe("forceps/joint_states", 1,
                      &AsarControl::subUpdateForcepsJointStateCb, this);
    sub_arm_state_ = nh_.subscribe("arm/arm_state", 1,
                                   &AsarControl::subUpdateArmStateCb, this);
    sub_forceps_state_ =
        nh_.subscribe("forceps/forceps_state", 1,
                      &AsarControl::subUpdateForcepsStateCb, this);
    sub_forceps_grip_angle_ = nh_.subscribe(
        "asar/gripper/cmd", 1, &AsarControl::subUpdateForcepsGripAngleCb, this);

    //* Publishers
    pub_arm_joint_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(
        "arm/position_controller/command", 1);
    pub_forceps_joint_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(
        "forceps/effort_controller/command", 1);
    pub_asar_ee_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("asar/ee", 1);
    pub_asar_flange_pose_ =
        nh_.advertise<geometry_msgs::PoseStamped>("asar/flange", 1);
    pub_asar_joint_states_ =
        nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    pub_asar_state_ = nh_.advertise<std_msgs::Int32>("asar/state", 1);

    ROS_INFO_STREAM("Starting actions");

    //* Action Server
    actFollowCartesianTarget_ = std::make_shared<
        SimpleActionServer<asar_control::FollowCartesianTargetAction>>(
        nh_, "FollowCartesianTarget",
        boost::bind(&AsarControl::actFollowCartesianTargetCb, this, _1), false);

    actFollowCartesianTarget_->registerPreemptCallback(
        boost::bind(&AsarControl::actCancelCb, this));

    actFollowCartesianTarget_->start();

    actFollowJointTarget_ = std::make_shared<
        SimpleActionServer<asar_control::FollowJointTargetAction>>(
        nh_, "FollowJointTarget",
        boost::bind(&AsarControl::actFollowJointTargetCb, this, _1), false);

    actFollowJointTarget_->registerPreemptCallback(
        boost::bind(&AsarControl::actCancelCb, this));

    actFollowJointTarget_->start();

    actSolveIK_ =
        std::make_shared<SimpleActionServer<asar_control::SolveIKAction>>(
            nh_, "SolveIK", boost::bind(&AsarControl::actSolveIKCb, this, _1),
            false);

    actSolveIK_->registerPreemptCallback(
        boost::bind(&AsarControl::actCancelPlanCb, this));

    actSolveIK_->start();

    // CODCS config
    std::string ik_solver;
    if (!nh_.getParam("ik_solver", ik_solver))
    {
      ik_solver = "codcs_tp";
    }

    int max_iter;
    if (!nh_.getParam("max_iter", max_iter))
    {
      max_iter = 10000;
    }

    double max_time;
    if (!nh_.getParam("max_time", max_time))
    {
      max_time = 5e-3;
    }

    double max_error;
    if (!nh_.getParam("max_error", max_error))
    {
      max_error = 1e-5;
    }

    double dt;
    if (!nh_.getParam("delta_integration", dt))
    {
      dt = 1.0;
    }

    std::string nlp_linear_solver;
    if (!nh_.getParam("nlp_linear_solver", nlp_linear_solver))
    {
      nlp_linear_solver = "ma57";
    }

    std::string error_method;
    if (!nh_.getParam("error_method", error_method))
    {
      error_method = "log6";
    }

    double mu0;
    if (!nh_.getParam("mu0", mu0))
    {
      mu0 = 1.0;
    }

    double mu1;
    if (!nh_.getParam("mu1", mu1))
    {
      mu1 = 0.005;
    }

    double mu2;
    if (!nh_.getParam("mu2", mu2))
    {
      mu2 = 0.001;
    }

    double mu3;
    if (!nh_.getParam("mu3", mu3))
    {
      mu3 = 1000.0;
    }

    double rcm_error_max;
    if (!nh_.getParam("rcm_error_max", rcm_error_max))
    {
      rcm_error_max = 2.5e-5;
    }

    bool constrained_control;
    if (!nh_.getParam("constrained_control", constrained_control))
    {
      constrained_control = false;
    }

    bool rcm_is_cost;
    if (!nh_.getParam("rcm_is_cost", rcm_is_cost))
    {
      rcm_is_cost = true;
    }

    int verb_level;
    if (!nh_.getParam("solv_verb_level", verb_level))
    {
      verb_level = 0;
    }

    std::string time_stats;
    if (!nh_.getParam("solv_time_stats", time_stats))
    {
      time_stats = "no";
    }

    Vector3d trocar_pos;
    double trocar_x;
    double trocar_y;
    double trocar_z;
    if (!nh_.getParam("trocar_x", trocar_x))
    {
      trocar_x = 0.0;
    }
    if (!nh_.getParam("trocar_y", trocar_y))
    {
      trocar_y = 0.0;
    }
    if (!nh_.getParam("trocar_z", trocar_z))
    {
      trocar_z = 0.0;
    }
    trocar_pos[0] = trocar_x;
    trocar_pos[1] = trocar_y;
    trocar_pos[2] = trocar_z;

    //* CODCS *//
    ROS_INFO("Starting CODCS-IK");
    // urdf_param_ = "robot_description";
    // ROS_INFO_STREAM("Extracting robot model from " << urdf_param_);

    std::string pkg_path = ros::package::getPath("asar_description");
    // std::string urdf_path =
    //     pkg_path + std::string("/urdf/") + "gen3_4dof_v4.urdf";
    std::string urdf_path =
        pkg_path + std::string("/urdf/") + "asar_v2_ik_solver.urdf";

    // Creating solver options class
    SolverOptions so;
    // so.prercm_joint_name_ = "joint_interface";
    // so.postrcm_joint_name_ = "joint_f0_pitch";
    so.time_stats_ = time_stats;
    so.verb_level_ = verb_level;
    so.err_method_ = error_method;
    so.nlp_linear_solver_ = nlp_linear_solver;
    so.rcm_error_max_ = rcm_error_max;
    so.rcm_is_cost_ = rcm_is_cost;
    so.constrained_control_ = constrained_control;
    so.cost_coeff_.push_back(mu0);
    so.cost_coeff_.push_back(mu1);
    so.cost_coeff_.push_back(mu2);
    so.cost_coeff_.push_back(mu3);
    // so.trocar_pos_ = Vector3d(0.5458, 0.0471, 0.4112);
    so.trocar_pos_ = trocar_pos;

    ik_solver_.reset(new CODCS_IK(urdf_path, "base_link", "ee_link", ik_solver,
                                  so, max_time, max_error, max_iter, dt));

    // Control loop
    cycle_t_us_d_ = (double)cycle_t_us_;
    cycle_t_ = cycle_t_us_d_ / 1000000.0;

    // Real time clock intialization
    rt_clock_ = RTClock(cycle_t_us_);

    // Resize publishing messages
    pub_arm_joint_cmd_msg_.data.resize(n_dof_arm_);
    pub_forceps_joint_cmd_msg_.data.resize(n_dof_forceps_);

    m_curAct_track_ = ACT_NONE;
    m_curAct_plan_ = ACT_NONE;

    flag_arm_joint_state_initialized_ = false;
    flag_forceps_joint_state_initialized_ = false;
    flag_asar_joint_state_initialized_ = false;

    //* Dynamic Reconfigure Server *//
    dyn_cb_ = boost::bind(&AsarControl::setDynVarCb, this, _1, _2);
    dyn_srv_.setCallback(dyn_cb_);

    act_arm_joint_pos_.resize(n_dof_arm_);
    act_arm_joint_vel_.resize(n_dof_arm_);
    act_arm_joint_acc_.resize(n_dof_arm_);
    act_forceps_joint_pos_.resize(n_dof_forceps_);
    act_forceps_joint_vel_.resize(n_dof_forceps_);
    act_forceps_joint_acc_.resize(n_dof_forceps_);

    prev_arm_joint_pos_ = VectorXd::Zero(n_dof_arm_);
    prev_arm_joint_vel_ = VectorXd::Zero(n_dof_arm_);

    prev_forceps_joint_pos_ = VectorXd::Zero(n_dof_forceps_);
    prev_forceps_joint_vel_ = VectorXd::Zero(n_dof_forceps_);

    act_joints_state_.resize(n_dof_total_);

    act_joints_state_ = VectorXd::Zero(n_dof_total_);
    act_joints_vel_ = VectorXd::Zero(n_dof_total_);
    act_joints_acc_ = VectorXd::Zero(n_dof_total_);

    //* Trajectory Generator *//
    joint_traj_gen_ = new OnlineTrajGen(n_dof_total_, cycle_t_);

    joint_vel_limit_ = 0.5; // 30% of max velocity
    joint_acc_limit_ = 0.5; // 30% of max acceleration

    arm_state_ = ARM_UNINITIALIZED;
    forceps_state_ = FORCEPS_UNINITIALIZED;
    asar_state_ = ASAR_UNINITIALIZED;

    gripper_angle_ = 0.0;

    // ASAR Joint State message
    pub_asar_joint_states_msg_.name.resize(n_dof_arm_ + n_dof_forceps_);
    pub_asar_joint_states_msg_.position.resize(n_dof_arm_ + n_dof_forceps_);
    pub_asar_joint_states_msg_.velocity.resize(n_dof_arm_ + n_dof_forceps_);
    pub_asar_joint_states_msg_.effort.resize(n_dof_arm_ + n_dof_forceps_);

    pub_asar_joint_states_msg_.name = {"joint_1",
                                       "joint_2",
                                       "joint_3",
                                       "joint_4",
                                       "joint_5",
                                       "joint_6",
                                       "joint_7",
                                       "joint_f0_pitch",
                                       "joint_f1_finger_left",
                                       "joint_f2_finger_right"};

    ROS_INFO_STREAM("ASAR initialized");
  }

  AsarControl::~AsarControl()
  {
    // spinner->stop();
    delete joint_traj_gen_;
  }

  //* Callbacks

  void AsarControl::actFollowCartesianTargetCb(
      const asar_control::FollowCartesianTargetGoalConstPtr &goal)
  {
    int hr;
    asar_control::FollowCartesianTargetResult res;
    pin::SE3 Xd;

    // ROS_INFO_STREAM(" FOLLOW CARTESIAN TARGET action received ");

    // Set current action
    boost::mutex::scoped_lock lockAct(mtx_Act_track_);
    if (m_curAct_track_ != ACT_NONE)
    {
      if (m_curAct_track_ != ACT_RESET)
      {
        res.succeed = false;
        actFollowCartesianTarget_->setAborted(res);
        if (debug_info)
        ROS_INFO_STREAM("Aborting Action");
      }
      return;
    }

    m_curAct_track_ = ACT_FOLLOWCARTESIAN;
    lockAct.unlock();
    if (asar_state_ >= ASAR_SERVO_MODE)
    {
      poseMsgToSE3(goal->tool_pose, Xd);
      hr = followCartesianTarget(Xd);
    }
    else
    {
      if (debug_info)
      ROS_WARN_STREAM("ASAR robot is not connected. Skipping action");
      hr = EXIT_FAILURE;
    }

    // Reset current action
    mtx_Act_track_.lock();
    if (m_curAct_track_ == ACT_FOLLOWCARTESIAN)
    {
      if (hr == EXIT_SUCCESS)
      {
        res.succeed = true;
        actFollowCartesianTarget_->setSucceeded(res);
      }
      else
      {
        res.succeed = false;
        actFollowCartesianTarget_->setAborted(res);
      }
      m_curAct_track_ = ACT_NONE;
    }
    mtx_Act_track_.unlock();
  }

  void AsarControl::actFollowJointTargetCb(
      const asar_control::FollowJointTargetGoalConstPtr &goal)
  {
    int hr;
    asar_control::FollowJointTargetResult res;
    pin::SE3 Xd;
    if (debug_info)
    ROS_INFO_STREAM(" FOLLOW JOINT TARGET action received ");

    // Set current action
    boost::mutex::scoped_lock lockAct(mtx_Act_track_);
    if (m_curAct_track_ != ACT_NONE)
    {
      if (m_curAct_track_ != ACT_RESET)
      {
        res.succeed = false;
        actFollowJointTarget_->setAborted(res);
        if (debug_info)
        ROS_INFO_STREAM("Aborting Action");
      }
      return;
    }

    m_curAct_track_ = ACT_FOLLOWJOINT;
    lockAct.unlock();
    if (asar_state_ >= ASAR_SERVO_MODE)
    {
      // poseMsgToSE3(goal->joint_target, Xd);
      if (debug_info)
      ROS_INFO("Folow joint target");
      hr = followJointTarget(goal->joint_target);
    }
    else
    {
      if (debug_info)
      ROS_WARN_STREAM("ASAR robot is not in SERVO MODE. Skipping action");
      hr = EXIT_FAILURE;
    }

    // Reset current action
    mtx_Act_track_.lock();
    if (m_curAct_track_ == ACT_FOLLOWJOINT)
    {
      if (hr == EXIT_SUCCESS)
      {
        res.succeed = true;
        actFollowJointTarget_->setSucceeded(res);
      }
      else
      {
        res.succeed = false;
        actFollowJointTarget_->setAborted(res);
      }
      m_curAct_track_ = ACT_NONE;
      if (debug_info)
      ROS_INFO("m_curAct_track_ reset");
    }
    mtx_Act_track_.unlock();
  }

  void AsarControl::actSolveIKCb(const asar_control::SolveIKGoalConstPtr &goal)
  {
    int hr;
    asar_control::SolveIKResult res;
    res.joint_sol.position.resize(n_dof_total_);
    pin::SE3 Xd;
    VectorXd q_sol;

    // ROS_INFO_STREAM(" SOLVE IK action received ");

    // Set current action
    boost::mutex::scoped_lock lockAct(mtx_Act_plan_);
    if (m_curAct_plan_ != ACT_NONE)
    {
      if (m_curAct_plan_ != ACT_RESET)
      {
        res.succeed = false;
        actSolveIK_->setAborted(res);
        if (debug_info)
        ROS_INFO_STREAM("Aborting Action");
      }
      return;
    }

    m_curAct_plan_ = ACT_SOLVEIK;
    lockAct.unlock();
    // if (arm_state_ >= ARM_READY && forceps_state_ >= FORCEPS_READY)
    // {
    poseMsgToSE3(goal->target_pose, Xd);
    hr = solveIK(Xd, q_sol);
    // }
    // else
    // {
    // ROS_WARN_STREAM("ASAR robot is not connected. Skipping action");
    // hr = EXIT_FAILURE;
    // }

    // Reset current action
    mtx_Act_plan_.lock();
    if (m_curAct_plan_ == ACT_SOLVEIK)
    {
      if (hr == EXIT_SUCCESS)
      {
        res.succeed = true;
        // ROS_INFO_STREAM("Solve IK action succeded. Mapping q_sol: " <<
        // q_sol.transpose());
        VectorXd::Map(&res.joint_sol.position[0], n_dof_total_) = q_sol;
        actSolveIK_->setSucceeded(res);
      }
      else
      {
        res.succeed = false;
        actSolveIK_->setAborted(res);
      }
      m_curAct_plan_ = ACT_NONE;
    }
    mtx_Act_plan_.unlock();
  }

  void AsarControl::actCancelCb()
  {
    boost::mutex::scoped_lock lockAct(mtx_Act_track_);

    if (m_curAct_track_ > ACT_NONE)
    {
      // ROS_WARN_STREAM("Preempting Controller Actions from state "
      //                 << m_curAct_track_);
      switch (m_curAct_track_)
      {
      case ACT_FOLLOWCARTESIAN:
      {
        asar_control::FollowCartesianTargetResult res_tool;
        res_tool.succeed = false;
        actFollowCartesianTarget_->setPreempted(res_tool);
        break;
      }
      case ACT_FOLLOWJOINT:
      {
        asar_control::FollowJointTargetResult res_tool;
        res_tool.succeed = false;
        actFollowJointTarget_->setPreempted(res_tool);
        break;
      }
      // case ACT_SOLVEIK:
      // {
      //   asar_control::SolveIKResult res;
      //   res.succeed = false;
      //   actSolveIK_->setPreempted(res);
      //   break;
      // }
      }
      m_curAct_track_ = ACT_NONE;

      // ROS_INFO_STREAM("Preempted" << m_curAct_track_);
    }
  }

  void AsarControl::actCancelPlanCb()
  {
    boost::mutex::scoped_lock lockAct(mtx_Act_plan_);

    if (m_curAct_plan_ > ACT_NONE)
    {
      // ROS_WARN_STREAM("Preempting Controller Plan Actions from state "
      //                 << m_curAct_plan_);
      switch (m_curAct_plan_)
      {
      case ACT_SOLVEIK:
      {
        asar_control::SolveIKResult res;
        res.succeed = false;
        actSolveIK_->setPreempted(res);
        break;
      }
      }
      m_curAct_plan_ = ACT_NONE;

      // ROS_INFO_STREAM("Preempted");
    }
  }

  void AsarControl::subUpdateArmJointStateCb(
      const sensor_msgs::JointState::ConstPtr &msg)
  {
    act_arm_joint_pos_ = (VectorXd::Map(&msg->position[0], n_dof_arm_)); //[deg]

    if (!flag_arm_joint_state_initialized_)
    {
      flag_arm_joint_state_initialized_ = true;
      prev_arm_joint_pos_ = act_arm_joint_pos_;
    }

    act_joints_state_.head(n_dof_arm_) = act_arm_joint_pos_;
    VectorXd::Map(&pub_asar_joint_states_msg_.position[0], n_dof_arm_) =
        act_arm_joint_pos_;

    updateFK();
    update_arm_act_vel();
    update_arm_act_acc();
    VectorXd::Map(&pub_asar_joint_states_msg_.velocity[0], n_dof_arm_) =
        act_arm_joint_vel_;
    VectorXd::Map(&pub_asar_joint_states_msg_.effort[0], n_dof_arm_) =
        act_arm_joint_acc_;

    if (!flag_asar_joint_state_initialized_ &&
        flag_forceps_joint_state_initialized_)
    {
      flag_asar_joint_state_initialized_ = true;

      joint_traj_gen_->setCurrIn(act_joints_state_, act_joints_vel_,
                                 act_joints_acc_);

      // joint_traj_gen_->setLimit(joint_vel_limit_ *
      // VectorXd::Ones(n_dof_total_),
      //                           joint_acc_limit_ *
      //                           VectorXd::Ones(n_dof_total_), 200 *
      //                           VectorXd::Ones(n_dof_total_));
      joint_traj_gen_->setLimit(joint_vel_limit_ * kJointVelLimits,
                                joint_acc_limit_ * kJointAccLimits,
                                200 * VectorXd::Ones(n_dof_total_));
      joint_traj_gen_->printLimits();
      joint_traj_gen_->setTarget(act_joints_state_,
                                 VectorXd::Zero(n_dof_total_));
      joint_traj_gen_->setStateReset();
      joint_cmd_step_ = act_joints_state_;

      // startNewJointTraj(act_joints_state_);
    }
  }

  void AsarControl::subUpdateForcepsJointStateCb(
      const sensor_msgs::JointState::ConstPtr &msg)
  {
    act_forceps_joint_pos_ =
        (VectorXd::Map(&msg->position[0], n_dof_forceps_)); //[deg]
    if (!flag_forceps_joint_state_initialized_)
    {
      flag_forceps_joint_state_initialized_ = true;
      prev_forceps_joint_pos_ = act_forceps_joint_pos_;
    }

    act_joints_state_.tail(n_dof_forceps_ - 1) =
        act_forceps_joint_pos_.head(n_dof_forceps_ - 1);
    VectorXd::Map(&pub_asar_joint_states_msg_.position[n_dof_arm_],
                  n_dof_forceps_) = act_forceps_joint_pos_;
    update_forceps_act_vel();
    update_forceps_act_acc();

    if (!flag_asar_joint_state_initialized_ &&
        flag_arm_joint_state_initialized_)
    {
      flag_asar_joint_state_initialized_ = true;
      joint_traj_gen_->setCurrIn(act_joints_state_, act_joints_vel_,
                                 act_joints_acc_);

      joint_traj_gen_->setLimit(joint_vel_limit_ * kJointVelLimits,
                                joint_acc_limit_ * kJointAccLimits,
                                200 * VectorXd::Ones(n_dof_total_));
      joint_traj_gen_->printLimits();
      joint_traj_gen_->setTarget(act_joints_state_,
                                 VectorXd::Zero(n_dof_total_));
      joint_traj_gen_->setStateReset();
      joint_cmd_step_ = act_joints_state_;
      // startNewJointTraj(act_joints_state_);
    }
  }

  void AsarControl::updateAsarState()
  {
    if (arm_state_ == ARM_SERVO_MODE && forceps_state_ == FORCEPS_SERVO_MODE)
      asar_state_ = ASAR_SERVO_MODE;
    else
      asar_state_ = ASAR_NOT_READY;
  }

  void AsarControl::subUpdateArmStateCb(
      const std_msgs::Int32MultiArray::ConstPtr &msg)
  {
    arm_state_ = msg->data[0];
    updateAsarState();
  }

  void AsarControl::subUpdateForcepsStateCb(
      const std_msgs::Int32MultiArray::ConstPtr &msg)
  {
    forceps_state_ = msg->data[0];
    updateAsarState();
  }

  void AsarControl::subUpdateForcepsGripAngleCb(
      const std_msgs::Float32::ConstPtr &msg)
  {
    gripper_angle_ = msg->data;
    // Limiting gripper angle
    if (gripper_angle_ >= 3.14)
      gripper_angle_ = 3.14;
    else if (gripper_angle_ <= -1.0)
      gripper_angle_ = -1.0;
  }

  void AsarControl::setDynVarCb(asar_control::DynVarConfig &config,
                                uint32_t level)
  {
    joint_vel_limit_ = (double)(config.vel_lim_coeff) / 100;
    joint_acc_limit_ = (double)(config.acc_lim_coeff) / 100;
    if (debug_info)
    ROS_WARN("Reconfigure Request velLimCoeff: %f accLimCoeff: %f",
             config.vel_lim_coeff, config.acc_lim_coeff);
    if (flag_asar_joint_state_initialized_)
    {
      joint_traj_gen_->setLimit(joint_vel_limit_ * kJointVelLimits,
                                joint_acc_limit_ * kJointAccLimits,
                                200 * VectorXd::Ones(n_dof_total_));
      joint_traj_gen_->printLimits();
    }
    return;
  }

  bool AsarControl::startNewJointTraj(VectorXd des_joint_pos)
  {
    // joint_traj_gen_->setCurrIn(act_joints_state_, act_joints_vel_,
    //                            act_joints_acc_);

    // joint_traj_gen_->setLimit(joint_vel_limit_ *
    // VectorXd::Ones(n_dof_total_),
    //                           joint_acc_limit_ *
    //                           VectorXd::Ones(n_dof_total_), 200 *
    //                           VectorXd::Ones(n_dof_total_));
    // joint_traj_gen_->printLimits();
    joint_traj_gen_->setTarget(des_joint_pos, VectorXd::Zero(n_dof_total_));
    joint_traj_gen_->setStateReset();
    return true;
  }

  //* Kinematics functions

  bool AsarControl::solveIK(const pin::SE3 &Xd, VectorXd &q_sol)
  {
    VectorXd sol = q_sol;

    int res_ik = 1;
    // ROS_INFO_STREAM("Solving with seed act_joints_state_ "
    //                 << act_joints_state_.transpose());
    // ROS_INFO_STREAM(
    //     "Solving IK with Target p:" << Xd.translation().transpose());
    // ROS_INFO_STREAM("Target R:" << Xd.rotation());

    // res_ik = ik_solver_->IkSolve(act_joints_state_, Xd, q_sol); //No comments
    res_ik = ik_solver_->IkSolve(act_joints_state_, Xd,
                                 sol); // With time performance comments

    if (res_ik == EXIT_SUCCESS)
    {
      q_sol = sol;
      // ROS_INFO_STREAM("Solution found " << q_sol.transpose());
      // if (group_name_ == "/unit1")
      // {
      //   if (q_sol[0] < 0.0)
      //   {
      //     q_sol[0] = 2 * M_PI + q_sol[0];
      //   }
      //   if (q_sol[2] < 0.0)
      //   {
      //     q_sol[2] = 2 * M_PI + q_sol[2];
      //   }
      //   if (q_sol[4] < 0.0)
      //   {
      //     q_sol[4] = 2 * M_PI + q_sol[4];
      //   }
      // }
      // ROS_INFO("IK solver succeed");
      return EXIT_SUCCESS;
    }
    else
    {
      // ROS_WARN("IK solver failed");
      return EXIT_FAILURE;
    }
  }

  void AsarControl::updateFK()
  {
    ik_solver_->solveFk(act_joints_state_, ee_pose_act_);
  }

  //* Generates a new trajectory step
  int AsarControl::generateJointTrajStep(VectorXd &joint_traj_step)
  {
    if (joint_traj_gen_->getState() != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
      int ResultValue = joint_traj_gen_->initRml();
      if (ResultValue < 0)
      {
        ROS_ERROR_STREAM(
            "Traj. Generation failed. Check the following error code: "
            << ResultValue); // Inside ReflexxesApi.h
        joint_traj_gen_->printTarget();
        joint_traj_gen_->printActual();
        return -1;
      }
      else
      {
        RMLPositionOutputParameters *OP;
        OP = joint_traj_gen_->get_output();
        joint_traj_gen_->updateCurrIn(OP);
        joint_traj_step =
            VectorXd::Map(&OP->NewPositionVector->VecData[0], n_dof_total_);

        return 0;
      }
    }
    else
      return 1;
  }

  int AsarControl::followCartesianTarget(const pin::SE3 &Xd)
  {
    VectorXd joint_target(n_dof_total_);
    // VectorXd joint_cmd_step(n_dof_total_);
    int traj_finished;
    // int res_ik = 1;

    // ROS_WARN_STREAM("Goal received: " << Xd.translation().transpose());
    // ROS_WARN_STREAM("Goal received: " << Xd.rotation());
    rt_clock_.Init();

    // ros::Rate loop_rate(1 / cycle_t_);

    // joint_target.resize(n_dof_total_);
    // VectorXd q_init(9);

    // q_init << 263.8 * kDEG2RAD, -56.4 * kDEG2RAD, -125.4 * kDEG2RAD,
    //     -97.9 * kDEG2RAD, 42.0 * kDEG2RAD, -58.3 * kDEG2RAD, -175.7 *
    //     kDEG2RAD, 0 * kDEG2RAD, 0 * kDEG2RAD;

    // TODO  Solve IK
    // res_ik = ik_solver_->IkSolve(act_joints_state_, Xd, joint_target);

    //
    // res_ik = ik_solver_->IkSolve(q_init, Xd, joint_target);

    // joint_cmd_step = act_joints_state_;
    // joint_target = VectorXd::Zero(n_dof_total_);
    traj_finished = 0;
    // joint_cmd_step.resize(n_dof_total_);
    // start_joint_traj(joint_target);

    // if (!res_ik)
    // {
    // ROS_INFO_STREAM("Solution found: " << joint_target.transpose());
    // for (int i = 0; i < n_dof_arm_; i++)
    // {
    //   if (joint_target[i] > 6.283185307)
    //   {
    //     joint_target[i] -= 6.283185307;
    //   }
    //   else if (joint_target[i] < 0.0)
    //   {
    //     joint_target[i] += 6.283185307;
    //   }
    // }
    startNewJointTraj(joint_target);
    // ROS_INFO_STREAM("Joint Traj initialized");

    while (actFollowCartesianTarget_->isActive() and traj_finished != 1)
    {
      if (flag_asar_joint_state_initialized_)
      {
        // joint_traj_gen_->printActual();

        VectorXd cur_joint_pos = joint_cmd_step_;
        traj_finished = generateJointTrajStep(joint_cmd_step_);
        if (((joint_cmd_step_ - cur_joint_pos).cwiseAbs() / cycle_t_ -
             kJointVelLimits)
                .maxCoeff() > 0.0)
        {
          ROS_ERROR_STREAM("MAX SPEED EXCEEDED. VEL: "
                           << (joint_cmd_step_ - cur_joint_pos).transpose() *
                                  1000);
        }
        if (traj_finished == 1)
        {
          return EXIT_SUCCESS;
        }
        if (traj_finished != -1)
        {
          sendCmdToControllers(joint_cmd_step_);
        }
      }
      rt_clock_.SleepToCompleteCycle();
    }
    // }
    // else
    // {
    // ROS_ERROR("No solution found");
    // return -1;
    // }

    return EXIT_FAILURE;
  }

  int AsarControl::followJointTarget(const sensor_msgs::JointState q_target)
  {
    VectorXd joint_target(n_dof_total_);
    joint_target =
        VectorXd::Map(&q_target.position[0], q_target.position.size());
    // VectorXd joint_cmd_step(n_dof_total_);
    int traj_finished;

    if ((joint_target - act_joints_state_).cwiseAbs().maxCoeff() < 0.001)
    {
      if (debug_info)
      ROS_WARN("Joint target within tolerance. Skipping request.");
      return EXIT_SUCCESS;
    }
    // int res_ik = 1;

    rt_clock_.Init();

    // ros::Rate loop_rate(1 / cycle_t_);

    // joint_target.resize(n_dof_total_);
    // VectorXd q_init(9);

    // q_init << 263.8 * kDEG2RAD, -56.4 * kDEG2RAD, -125.4 * kDEG2RAD,
    //     -97.9 * kDEG2RAD, 42.0 * kDEG2RAD, -58.3 * kDEG2RAD, -175.7 *
    //     kDEG2RAD, 0 * kDEG2RAD, 0 * kDEG2RAD;

    // res_ik = ik_solver_->IkSolve(act_joints_state_, Xd, joint_target);

    //
    // res_ik = ik_solver_->IkSolve(q_init, Xd, joint_target);

    // joint_cmd_step = act_joints_state_;
    // joint_target = VectorXd::Zero(n_dof_total_);
    traj_finished = 0;
    // joint_cmd_step.resize(n_dof_total_);
    // start_joint_traj(joint_target);

    // if (!res_ik)
    // {
    // ROS_INFO_STREAM("Solution found: " << joint_target.transpose());
    // for (int i = 0; i < n_dof_arm_; i++)
    // {
    //   if (joint_target[i] > 6.283185307)
    //   {
    //     joint_target[i] -= 6.283185307;
    //   }
    //   else if (joint_target[i] < 0.0)
    //   {
    //     joint_target[i] += 6.283185307;
    //   }
    // }

    startNewJointTraj(joint_target);
    if (debug_info)
      ROS_INFO_STREAM("Joint Traj initialized");

    VectorXd cur_joint_pos = joint_cmd_step_;

    // For direct comand to robot (No Reflexxes)
    joint_cmd_step_ = joint_target;
    sendCmdToControllers(joint_cmd_step_);
    return EXIT_SUCCESS;
    // if (add_noise)
    //       {
    //         ROS_ERROR("add noise !");
    //         std::normal_distribution<double> distribution(noiseMean, noiseVar);
            
    //         for (int i = 0; i < joint_cmd_step_.size(); ++i)
    //         {
    //           double number = distribution(generator);
    //           joint_cmd_step_(i) = joint_cmd_step_(i) + number; 
    //         }
    //         sendCmdToControllers(joint_cmd_step_);
    //       }
    // else 
    // sendCmdToControllers(joint_cmd_step_);

    // For Smooth trajectory generation (Reflexxes)
    // while (actFollowJointTarget_->isActive() and traj_finished != 1)
    // {
    //   if (flag_asar_joint_state_initialized_)
    //   {
    //     // joint_traj_gen_->printActual();
    //     cur_joint_pos = joint_cmd_step_;
    //     ROS_INFO_STREAM("Tracking joint_cmd_step_"
    //                     << joint_cmd_step_.transpose());

    //     traj_finished = generateJointTrajStep(joint_cmd_step_);
    //     if (((joint_cmd_step_ - cur_joint_pos).cwiseAbs() / cycle_t_ -
    //          kJointVelLimits)
    //             .maxCoeff() > 0.0)
    //     {
    //       ROS_ERROR_STREAM("MAX SPEED EXCEEDED. VEL: "
    //                        << (joint_cmd_step_ - cur_joint_pos).transpose() *
    //                               1000);
    //     }
    //     if (traj_finished == 1)
    //     {
    //       ROS_INFO("Tracking finished");

    //       return EXIT_SUCCESS;
    //     }
    //     if (traj_finished != -1)
    //     {
    //       sendCmdToControllers(joint_cmd_step_);
    //     }
    //   }
    //   rt_clock_.SleepToCompleteCycle();
    // }
    if (debug_info)
      ROS_INFO("Tracking cancel or preempted");
    return EXIT_FAILURE;
    
  }

  //* Output functions
  void AsarControl::sendCmdToControllers(VectorXd joint_cmd_step)
  {
    // ROS_INFO_STREAM("PUB: " << joint_cmd_step.transpose());

    VectorXd::Map(&pub_arm_joint_cmd_msg_.data[0], n_dof_arm_) =
        joint_cmd_step.head(n_dof_arm_);

    // VectorXd::Map(&pub_forceps_joint_cmd_msg_.data[0], n_dof_forceps_ - 1) =
    //     joint_cmd_step.tail(n_dof_forceps_ - 1);

    pub_forceps_joint_cmd_msg_.data[0] = joint_cmd_step[n_dof_total_ - 2];
    // pub_forceps_joint_cmd_msg_.data[1] = joint_cmd_step[n_dof_total_ - 1];

    // if (group_name_ == "/unit0")
    // {
    //   pub_forceps_joint_cmd_msg_.data[1] = joint_cmd_step[n_dof_total_ - 1];
    // }

    //  For center grasper
    pub_forceps_joint_cmd_msg_.data[1] =
        joint_cmd_step[n_dof_total_ - 1] - gripper_angle_ / 2;

    pub_forceps_joint_cmd_msg_.data[2] =
        -joint_cmd_step[n_dof_total_ - 1] - gripper_angle_ / 2;

    // For grasper in the left
    // pub_forceps_joint_cmd_msg_.data[1] = joint_cmd_step[n_dof_total_ - 1];

    // pub_forceps_joint_cmd_msg_.data[2] =
    //     -joint_cmd_step[n_dof_total_ - 1] - gripper_angle_;

    // For Omega
    // pub_forceps_joint_cmd_msg_.data[2] =
    //     0.15 - joint_cmd_step[n_dof_total_ - 1] - 2.5 * gripper_angle_;

    // pub_forceps_joint_cmd_msg_.data[n_dof_forceps_ - 1] =
    //     -joint_cmd_step[n_dof_total_ - 1];
    // pub_forceps_joint_cmd_msg_.data[n_dof_forceps_ - 1] = 0.0;

    pub_arm_joint_cmd_.publish(pub_arm_joint_cmd_msg_);
    pub_forceps_joint_cmd_.publish(pub_forceps_joint_cmd_msg_);
  }

  void AsarControl::publishStates()
  {
    // Publish ASAR state
    pub_asar_state_msg_.data = asar_state_;
    pub_asar_state_.publish(pub_asar_state_msg_);

    // Publish ASAR end-effector pose
    pub_asar_ee_pose_msg_.header.stamp = ros::Time::now();
    poseSE3ToMsg(ee_pose_act_, pub_asar_ee_pose_msg_.pose);
    pub_asar_ee_pose_.publish(pub_asar_ee_pose_msg_);

    // Publish ASAR joint_states
    pub_asar_joint_states_msg_.header.stamp = ros::Time::now();
    pub_asar_joint_states_.publish(pub_asar_joint_states_msg_);
  }

  //* Mutators

  void AsarControl::update_arm_act_vel()
  {
    // act_arm_joint_vel_ = (act_arm_joint_pos_ - prev_arm_joint_pos_) /
    // (cycle_t_); Based on /arm/joint_state rate. Check the config file.
    act_arm_joint_vel_ = (act_arm_joint_pos_ - prev_arm_joint_pos_) / 0.002;
    prev_arm_joint_pos_ = act_arm_joint_pos_;
    act_joints_vel_.head(n_dof_arm_) = act_arm_joint_vel_;
  }

  void AsarControl::update_arm_act_acc()
  {
    // act_arm_joint_acc_ = (act_arm_joint_vel_ - prev_arm_joint_vel_) /
    // (cycle_t_); Based on /arm/joint_state rate. Check the config file.
    act_arm_joint_acc_ = (act_arm_joint_vel_ - prev_arm_joint_vel_) / 0.002;
    prev_arm_joint_vel_ = act_arm_joint_vel_;
    act_joints_acc_.head(n_dof_arm_) = act_arm_joint_acc_;
  }

  void AsarControl::update_forceps_act_vel()
  {
    // act_forceps_joint_vel_ =
    //     (act_forceps_joint_pos_ - prev_forceps_joint_pos_) / (cycle_t_);
    // Based on /arm/joint_state rate. Check the config file.
    act_forceps_joint_vel_ =
        (act_forceps_joint_pos_ - prev_forceps_joint_pos_) / 0.004;

    prev_forceps_joint_pos_ = act_forceps_joint_pos_;
    act_joints_vel_.tail(n_dof_forceps_ - 1) =
        act_forceps_joint_vel_.head(n_dof_forceps_ - 1);
  }

  void AsarControl::update_forceps_act_acc()
  {
    // act_forceps_joint_acc_ =
    //     (act_forceps_joint_vel_ - prev_forceps_joint_vel_) / (cycle_t_);
    // Based on /arm/joint_state rate. Check the config file.
    act_forceps_joint_acc_ =
        (act_forceps_joint_vel_ - prev_forceps_joint_vel_) / 0.004;
    prev_forceps_joint_vel_ = act_forceps_joint_vel_;
    act_joints_acc_.tail(n_dof_forceps_ - 1) =
        act_forceps_joint_acc_.head(n_dof_forceps_ - 1);
  }

  //* Control loop
  void AsarControl::control_loop()
  {
    int cycle_freq = 100;

    ros::Rate loop_rate(cycle_freq);
    // rt_clock_.Init();

    while (not(*kill_this_node_))
    {
      // now_timestamp_ = ros::Time::now();
      // period_ = now_timestamp_ - prev_timestamp_;
      // prev_timestamp_ = now_timestamp_;
      // // ROS_INFO_STREAM("TIME " << period_);
      if (arm_state_ >= ARM_CONNECTED && forceps_state_ >= FORCEPS_CONNECTED &&
          flag_asar_joint_state_initialized_)
      {

        publishStates();
      }
      loop_rate.sleep();
      // rt_clock_.SleepToCompleteCycle();
      // PublishRobotState();
      // End while not kill this node
    }
  }

} // namespace asar_ns
