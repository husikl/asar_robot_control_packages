/******************************************************************************
# gen3_planner.cpp:  Gen3 Planner                                   #
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

#include <gen3_planner/gen3_planner.h>
#include <signal.h>
#include <thread>

namespace gen3_nu
{

  enum
  {
    ACT_RESET = -1,
    ACT_NONE = 0,
    ACT_FOLLOWJOINT,
    ACT_FOLLOWTOOL,
  };

  // Constructor
  Gen3Planner::Gen3Planner(ros::NodeHandle &node_handle, bool *kill_this_node)
  {
    nh_ = node_handle;
    kill_this_node_ = kill_this_node;

    //* ROS PARAMETER SERVER

    if (!nh_.getParam("prefix", prefix_))
    {
      prefix_ = "";
    }

    // Thread Sampling Time
    if (nh_.getParam("cyclic_time_usec", cycle_t_us_))
    {
      ROS_INFO_STREAM(
          "Thread cyclic time for PLANNER (Online Traj. Gen. step) [us]: "
          << cycle_t_us_);
    }
    else
    {
      ROS_INFO_STREAM(
          "No Thread cyclic time information in parameter server, using "
          "default 2ms");
      cycle_t_us_ = 2000;
    }

    // if (use_sim)
    // {
    //   sub_joint_state_ = nh_.subscribe("sim/joint/state", 1,
    //                                    &Gen3Planner::updateJointStateCb,
    //                                    this);
    // }
    // else
    // {
    //   ROS_WARN("Using real robot. Be careful");
    //   sub_joint_state_ =
    //       nh_.subscribe("joint_states", 1, &Gen3Planner::updateJointStateCb,
    //       this);
    // }

    sub_joint_state_ = nh_.subscribe("joint_states", 1,
                                     &Gen3Planner::updateJointStateCb, this);
    sub_arm_state_ =
        nh_.subscribe("arm_state", 1, &Gen3Planner::updateArmStateCb, this);

    // Publishers
    pub_joint_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(
        "position_controller/command", 1);
    pub_tool_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("gen3/tool", 1);

    // Action Server
    actFollowJointCmd = std::make_shared<
        SimpleActionServer<gen3_control::FollowJointCmdAction>>(
        nh_, "FollowJoint",
        boost::bind(&Gen3Planner::actFollowJointCmdCb, this, _1), false);

    actFollowJointCmd->registerPreemptCallback(
        boost::bind(&Gen3Planner::actCancelCb, this));

    actFollowJointCmd->start();

    actFollowToolCmd =
        std::make_shared<SimpleActionServer<gen3_control::FollowToolCmdAction>>(
            nh_, "FollowTool",
            boost::bind(&Gen3Planner::actFollowToolCmdCb, this, _1), false);

    actFollowToolCmd->registerPreemptCallback(
        boost::bind(&Gen3Planner::actCancelCb, this));

    actFollowToolCmd->start();

    // Control loop timekeeping
    cycle_t_us_d_ = (double)cycle_t_us_;
    cycle_t_ = cycle_t_us_d_ / 1000000.0;

    //* TRAC IK *//
    urdf_param_ = prefix_ + "/robot_description";
    ROS_INFO_STREAM("Extracting robot model from " << urdf_param_);
    // tracik_solver_.reset(new TRAC_IK::TRAC_IK("base_link", "link_7",
    // urdf_param_, 0.004,
    //                                           1e-5, TRAC_IK::Distance));
    tracik_solver_.reset(new TRAC_IK::TRAC_IK("base_link", "hook_eef_link",
                                              urdf_param_, 0.004, 1e-5,
                                              TRAC_IK::Distance));

    // Extract KDL chain
    if (!(tracik_solver_->getKDLChain(kdl_chain_)))
    {
      ROS_ERROR_STREAM("There was no valid KDL chain found");
    }

    // Get number of joints in model
    n_dof_ = kdl_chain_.getNrOfJoints();
    ROS_INFO_STREAM("Model contains " << n_dof_ << " joints");

    // Get joint position limits for IK
    if (!(tracik_solver_->getKDLLimits(kdl_ll_, kdl_ul_)))
    {
      ROS_ERROR_STREAM("There was no valid KDL joint limits found");
    }

    // Verify the number of joint position limits
    assert(kdl_chain_.getNrOfJoints() == kdl_ll_.data.size());
    assert(kdl_chain_.getNrOfJoints() == kdl_ul_.data.size());

    // Create FK solver with KDL
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    // Resize publishing messages
    pub_joint_cmd_msg_.data.resize(n_dof_);

    m_curAct = ACT_NONE;

    // Initialize temporal variables for KDL
    qtmp_.resize(n_dof_);
    nominal_.resize(n_dof_);
    for (uint j = 0; j < nominal_.data.size(); j++)
    {
      nominal_(j) = (kdl_ll_(j) + kdl_ul_(j)) / 2.0;
    }

    //* Dynamic Reconfigure Server *//
    joint_vel_limit_ = 10.0;
    joint_acc_limit_ = 10.0;

    // max_joint_vel_ = 0.87; //[rad/s]=50deg/s (real:150 deg/s)
    // max_joint_acc_ = 0.44; //[rad/s2]=25deg/s2

    dyn_cb_ = boost::bind(&Gen3Planner::setDynVarCb, this, _1, _2);
    dyn_srv_.setCallback(dyn_cb_);

    //* Trajectory Generator *//
    joint_traj_gen_ = new OnlineTrajGen(n_dof_, cycle_t_);

    flag_joint_pos_initialized_ = false;
    state_ = 0;
  }

  Gen3Planner::~Gen3Planner()
  {
    // spinner->stop();
    delete joint_traj_gen_;
    tracik_solver_.reset();
    fk_solver_.reset();
  }

  bool Gen3Planner::start_joint_traj(VectorXd des_joint_pos)
  {
    joint_traj_gen_->setCurrIn(act_joint_pos_, act_joint_vel_, act_joint_acc_);

    joint_traj_gen_->setLimit(joint_vel_limit_ * VectorXd::Ones(n_dof_),
                              joint_acc_limit_ * VectorXd::Ones(n_dof_),
                              200 * VectorXd::Ones(n_dof_));
    joint_traj_gen_->setTarget(des_joint_pos, VectorXd::Zero(n_dof_));
    joint_traj_gen_->setStateReset();
    return true;
  }

  int Gen3Planner::generate_joint_traj(VectorXd &joint_traj_step)
  {

    if (joint_traj_gen_->getState() != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
      int ResultValue = joint_traj_gen_->initRml();
      if (ResultValue < 0)
      {
        ROS_ERROR_STREAM(
            "Traj. Generation failed. Check following information: "
            << ResultValue);
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
            VectorXd::Map(&OP->NewPositionVector->VecData[0], n_dof_);
        return 0;
      }
    }
    else
      return 1;
  }

  //* Callbacks

  void Gen3Planner::actFollowJointCmdCb(
      const gen3_control::FollowJointCmdGoalConstPtr &goal)
  {
    int hr;
    gen3_control::FollowJointCmdResult res;

    // Set current action
    boost::mutex::scoped_lock lockAct(mtx_Act_);

    // if (state_ < R_SLAVE)
    // {
    //   res.result = EXIT_FAILURE;
    //   actFollowJointCmd->setAborted(res);
    //   return;
    // }

    if (m_curAct != ACT_NONE)
    {
      if (m_curAct != ACT_RESET)
      {
        res.result = EXIT_FAILURE;
        actFollowJointCmd->setAborted(res);
      }
      return;
    }

    m_curAct = ACT_FOLLOWJOINT;
    lockAct.unlock();

    // Execute action
    sensor_msgs::JointState joint_target;
    joint_target = goal->joint_pos;

    VectorXd joint_cmd = VectorXd::Map(&goal->joint_pos.position[0],
                                       goal->joint_pos.position.size());
    ROS_INFO_STREAM(" FOLLOW JOINT action received " << joint_cmd.transpose());

    hr = followJointCmd(joint_cmd);

    // Reset current action
    mtx_Act_.lock();
    if (m_curAct == ACT_FOLLOWJOINT)
    {
      if (hr == EXIT_SUCCESS)
      {
        res.result = EXIT_SUCCESS;
        actFollowJointCmd->setSucceeded(res);
      }
      else
      {
        res.result = EXIT_FAILURE;
      }
      m_curAct = ACT_NONE;
    }
    mtx_Act_.unlock();
  }

  void Gen3Planner::actFollowToolCmdCb(
      const gen3_control::FollowToolCmdGoalConstPtr &goal)
  {
    int hr;
    gen3_control::FollowToolCmdResult res;

    // Set current action
    boost::mutex::scoped_lock lockAct(mtx_Act_);

    // if (state_ < R_SLAVE)
    // {
    //   res.result = EXIT_FAILURE;
    //   actFollowToolCmd->setAborted(res);
    //   return;
    // }

    if (m_curAct != ACT_NONE)
    {
      if (m_curAct != ACT_RESET)
      {
        res.result = EXIT_FAILURE;
        actFollowToolCmd->setAborted(res);
      }
      return;
    }

    m_curAct = ACT_FOLLOWTOOL;
    lockAct.unlock();

    Vector3d pd;
    Matrix3d Rd;
    Quaterniond qd;

    tf::pointMsgToEigen(goal->tool_pose.position, pd);
    tf::quaternionMsgToEigen(goal->tool_pose.orientation, qd);
    Rd = qd.toRotationMatrix();
    ROS_INFO_STREAM(" FOLLOW TOOL action received ");

    hr = followToolCmd(pd, Rd);

    // Reset current action
    mtx_Act_.lock();
    if (m_curAct == ACT_FOLLOWTOOL)
    {
      if (hr == EXIT_SUCCESS)
      {
        res.result = EXIT_SUCCESS;
        actFollowToolCmd->setSucceeded(res);
      }
      else
      {
        res.result = EXIT_FAILURE;
      }
      m_curAct = ACT_NONE;
    }
    mtx_Act_.unlock();
  }

  void Gen3Planner::actCancelCb()
  {
    boost::mutex::scoped_lock lockAct(mtx_Act_);

    if (m_curAct > ACT_NONE)
    {

      ROS_INFO_STREAM("Preempting Controller Actions");
      switch (m_curAct)
      {
      case ACT_FOLLOWJOINT:
      {
        gen3_control::FollowJointCmdResult res_joint;
        res_joint.result = EXIT_FAILURE;
        actFollowJointCmd->setPreempted(res_joint);
        break;
      }
      case ACT_FOLLOWTOOL:
      {
        gen3_control::FollowToolCmdResult res_tool;
        res_tool.result = EXIT_FAILURE;
        actFollowToolCmd->setPreempted(res_tool);
        break;
      }
      }
    }
  }

  void
  Gen3Planner::updateJointStateCb(const sensor_msgs::JointState::ConstPtr &msg)
  {
    act_joint_pos_ = (VectorXd::Map(&msg->position[0], n_dof_)); //[deg]

    if (!flag_joint_pos_initialized_)
    {
      flag_joint_pos_initialized_ = true;
      joint_traj_gen_->setCurrIn(act_joint_pos_, VectorXd::Zero(n_dof_),
                                 VectorXd::Zero(n_dof_));
      joint_traj_gen_->setLimit(joint_vel_limit_ * VectorXd::Ones(n_dof_),
                                joint_acc_limit_ * VectorXd::Ones(n_dof_),
                                100 * VectorXd::Ones(n_dof_));
      joint_traj_gen_->setTarget(act_joint_pos_, VectorXd::Zero(n_dof_));
      joint_traj_gen_->setStateReset();
      // joint_traj_gen_->printLimits();

      ROS_INFO_STREAM("Intial pose " << act_joint_pos_.transpose());
    }

    update_act_vel();
    update_act_acc();

    Vector3d B_p_TOOL;
    Matrix3d B_R_TOOL;

    solveFK(act_joint_pos_, B_p_TOOL, B_R_TOOL);
    Affine3d B_x_TOOL;
    B_x_TOOL.linear() = B_R_TOOL;
    B_x_TOOL.translation() = B_p_TOOL;

    tf::poseEigenToMsg(B_x_TOOL, pub_tool_pose_msg_.pose);
    pub_tool_pose_msg_.header.stamp = ros::Time::now();

    pub_tool_pose_.publish(pub_tool_pose_msg_);
  }

  void
  Gen3Planner::updateArmStateCb(const std_msgs::Int32MultiArray::ConstPtr &msg)
  {
    state_ = msg->data[0];
  }

  void Gen3Planner::setDynVarCb(gen3_control::DynVarConfig &config,
                                uint32_t level)
  {
    ROS_WARN("Reconfigure Request velLimCoeff: %f accLimCoeff: %f",
             config.vel_lim_coeff, config.acc_lim_coeff);
    joint_vel_limit_ = (double)(config.vel_lim_coeff) / 100 * kMaxJointVel;
    joint_acc_limit_ = (double)(config.acc_lim_coeff) / 100 * kMaxJointAcc;
    return;
  }

  int Gen3Planner::followJointCmd(VectorXd joint_cmd)
  {
    VectorXd joint_cmd_step;
    int traj_finished;
    ros::Rate loop_rate(1 / cycle_t_);

    bool flag_traj_started = false;
    traj_finished = 0;
    joint_cmd_step.resize(n_dof_);
    // ROS_INFO_STREAM("Initial target: " << act_joint_pos_.transpose());
    // ros::Duration(0.2).sleep();

    while (actFollowJointCmd->isActive() and traj_finished != 1 and
           state_ >= R_SLAVE)
    {
      if (flag_joint_pos_initialized_)
      {
        if (!flag_traj_started)
        {
          start_joint_traj(joint_cmd);
          flag_traj_started = true;
          joint_cmd_step = act_joint_pos_;
        }
        traj_finished = generate_joint_traj(joint_cmd_step);
        if (traj_finished != -1)
        {
          sendCmdController(joint_cmd_step);
        }
      }
      loop_rate.sleep();
      ros::spinOnce();
    }
    if (traj_finished == 1)
      return 0;
    else
      return -1;
  }

  int Gen3Planner::followToolCmd(Vector3d pd, Matrix3d Rd)
  {
    VectorXd joint_target;
    VectorXd joint_cmd_step;
    int traj_finished;
    int res_ik = 0;
    ros::Rate loop_rate(1 / cycle_t_);
    joint_target.resize(n_dof_);

    res_ik = solveIK(pd, Rd, joint_target);

    bool flag_traj_started = false;

    traj_finished = 0;
    joint_cmd_step.resize(n_dof_);
    // joint_cmd_step = act_joint_pos_;
    // ros::Duration(0.2).sleep();

    if (res_ik == 0)
    {
      // start_joint_traj(joint_target);

      while (actFollowToolCmd->isActive() and traj_finished != 1 and
             state_ >= R_SLAVE)
      {

        if (flag_joint_pos_initialized_)
        {
          if (!flag_traj_started)
          {
            start_joint_traj(joint_target);
            flag_traj_started = true;
            joint_cmd_step = act_joint_pos_;
          }
          traj_finished = generate_joint_traj(joint_cmd_step);
          // ROS_INFO_STREAM("CMD: " << joint_cmd_step.transpose());
          if (traj_finished != -1)
          {
            sendCmdController(joint_cmd_step);
          }
        }
        loop_rate.sleep();
        ros::spinOnce();
      }
    }

    if (traj_finished == 1)
      return 0;
    else
      return -1;
  }

  void Gen3Planner::sendCmdController(VectorXd joint_cmd_step)
  {
    // ROS_INFO_STREAM("PUB: " << joint_cmd_step.transpose());

    VectorXd::Map(&pub_joint_cmd_msg_.data[0], pub_joint_cmd_msg_.data.size()) =
        joint_cmd_step;
    pub_joint_cmd_.publish(pub_joint_cmd_msg_);
  }

  //* Kinematics functions

  int Gen3Planner::solveFK(VectorXd joint_val, Vector3d &B_p_TOOL,
                           Matrix3d &R_B_TOOL)
  {
    qtmp_.data = joint_val * kDEG2RAD;

    bool kinematics_status;
    kinematics_status = fk_solver_->JntToCart(qtmp_, xtmp_);

    Affine3d T01;

    if (!kinematics_status)
    {
      tf::transformKDLToEigen(xtmp_, T01);
      R_B_TOOL = T01.rotation();
      B_p_TOOL = 1000 * T01.translation();

      return 0;
    }
    else
      ROS_ERROR_STREAM("No Forward Kinematics solution");
    return -1;
  }

  int Gen3Planner::solveIK(Vector3d pd, Matrix3d Rd, VectorXd &joint_cmd)
  {
    int rc;
    KDL::JntArray qd;
    KDL::Frame ee;
    Affine3d Td01;

    Td01.linear() = Rd;
    Td01.translation() = pd / 1000;

    tf::transformEigenToKDL(Td01, ee);

    // nominal_ = qd;
    nominal_.data = act_joint_pos_;
    rc = tracik_solver_->CartToJnt(nominal_, ee, qd);
    if (rc >= 0)
    {

      joint_cmd = qd.data;

      return 0;
    }
    else
    {
      ROS_WARN_STREAM("No IK solution found for \np:\n"
                      << pd.transpose());
      ROS_WARN_STREAM("R: \n"
                      << Rd);
      return -1;
    }
  }

  void Gen3Planner::update_act_vel()
  {
    static VectorXd prev_joint_pos(VectorXd::Zero(n_dof_));
    act_joint_vel_ = (act_joint_pos_ - prev_joint_pos) / (cycle_t_);
    prev_joint_pos = act_joint_pos_;
  }

  void Gen3Planner::update_act_acc()
  {
    static VectorXd prev_joint_vel(VectorXd::Zero(n_dof_));
    act_joint_acc_ = (act_joint_vel_ - prev_joint_vel) / (cycle_t_);
    prev_joint_vel = act_joint_vel_;
  }

  void Gen3Planner::control_loop()
  {
    while (not(*kill_this_node_))
    {
    }
  }

} // namespace gen3_nu

using namespace gen3_nu;

bool kill_this_process = false;

void SigIntHandler(int signal)
{
  kill_this_process = true;
  ROS_WARN_STREAM("SHUTDOWN SIGNAL RECEIVED");
}

int main(int argc, char **argv)
{
  // Ros related
  ros::init(argc, argv, "gen3_arm_planner");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigIntHandler);

  Gen3Planner dp(node_handle, &kill_this_process);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::thread t(boost::bind(&Gen3Planner::control_loop, &dp));

  t.join();
  spinner.stop();

  return 0;
}