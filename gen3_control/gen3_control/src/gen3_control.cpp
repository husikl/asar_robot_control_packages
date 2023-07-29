/******************************************************************************
# gen3_control.cpp:  Gen3 Arm Control                               #
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

#include <gen3_control/gen3_control.h>
// C
#include <signal.h>

// Additional libraries
#include <ros/ros.h>
// #include <ros/package.h>

// #define DEBUG_ENABLED false

namespace gen3_nu
{

  // Constructor
  Gen3Control::Gen3Control(ros::NodeHandle &node_handle, bool *kill_this_node)
  {
    nh_ = node_handle;

    //* ROS PARAMETER SERVER
    // Robot Identifier
    if (nh_.getParam("robot_id", robot_id_))
    {
      ROS_INFO_STREAM(
          "Arm ID obtained from ROS Parameter server as: " << robot_id_);
    }
    else
    {
      ROS_WARN_STREAM(
          "No ID information in parameter server, using default: [0]");
      robot_id_ = 0;
    }

    // Robot IP Address
    if (nh_.getParam("ip_address", ip_address_))
    {
      ROS_INFO_STREAM("Arm Ip address obtained from ROS Parameter server as: "
                      << ip_address_);
      // robot_ = DensoRobot(ip_address_, 5007, kill_this_node_);
    }
    else
    {
      ROS_WARN_STREAM(
          "No ip information in parameter server, using default 192.168.0.1");
      // robot_ = DensoRobot(std::string("192.168.0.1"), 5007, kill_this_node_);
    }

    // Thread Sampling Time
    if (nh_.getParam("cyclic_time_usec", cycle_t_us_))
    {
      ROS_INFO_STREAM("Thread cyclic time for ARM CONTROL (Hardware-ROS "
                      "control comm) [us]: "
                      << cycle_t_us_);
    }
    else
    {
      ROS_WARN_STREAM("No thread cyclic time information, using "
                      "default 2ms");
      cycle_t_us_ = 2000;
    }

    // Robot simulaton
    if (nh_.getParam("use_sim", use_sim_))
    {
      ROS_INFO_STREAM("Arm simulator: " << std::boolalpha << use_sim_);
    }
    else
    {
      ROS_WARN_STREAM("No arm simulator option set, using "
                      "default: Start Arm simulator");
      use_sim_ = true;
    }

    // Joint names
    if (!nh_.getParam("position_controller/joints", joint_names_))
    {
      joint_names_.clear();
    }

    n_dof_ = joint_names_.size();
    ROS_INFO_STREAM("Number of Arm Joints:" << n_dof_);

    // Services
    srv_server_robot_request_ = nh_.advertiseService(
        "arm_request", &Gen3Control::SrvRobotCommandCb, this);

    // Subscribers
    sub_sim_joint_state_ = nh_.subscribe(
        "sim/joint/state", 1, &Gen3Control::SubUpdateSimJointStateCb, this);

    // Publishers
    pub_state_ = nh_.advertise<std_msgs::Int32MultiArray>("arm_state", 1);
    pub_sim_joint_cmd_ =
        nh_.advertise<sensor_msgs::JointState>("sim/joint/cmd", 1);

    // Resize publishing messages
    pub_sim_joint_cmd_msg_.name.resize(n_dof_);
    pub_sim_joint_cmd_msg_.position.resize(n_dof_);
    pub_sim_joint_cmd_msg_.velocity.resize(n_dof_);
    pub_sim_joint_cmd_msg_.effort.resize(n_dof_);

    // Real time clock intialization
    rt_clock_ = RTClock(cycle_t_us_);
    // real_time_clock_ = RealtimeClock(cycle_t_us_);

    kill_this_node_ = kill_this_node;

    // Flags
    // flag_buffer_full = false;

    // buffer_full_counter = 0;

    // Command frame
    des_joint_pos_ = VectorXd::Zero(n_dof_);
    act_joint_pos_ = VectorXd::Zero(n_dof_);
    act_joint_pos_raw_ = VectorXd::Zero(n_dof_);
    sim_joint_pos_ = VectorXd::Zero(n_dof_);

    // Initialize ros messages
    pub_state_msg_.data.resize(1);

    // Temporarily
    state_ = R_UNINITIALIZED;

    jointInterfaceResize(n_dof_);
    jointInterfaceSetZero();

    // Initializing Kortex Arm
    // ROS_INFO_STREAM("Args: " << arm_args_.ip_address << " " <<
    // arm_args_.username);
    arm_args_.ip_address = ip_address_;
    arm_.reset(new KortexArm(arm_args_));

    ROS_INFO_STREAM("Kortex Arm initialized");
    // Initialize controllers
    for (int i = 0; i < n_dof_; i++)
    {
      // Create Joint state interface
      JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i],
                                        &joint_velocity_[i], &joint_effort_[i]);
      joint_state_interface_.registerHandle(jointStateHandle);

      // Create Position Joint interface
      JointHandle jointPositionHandle(jointStateHandle,
                                      &joint_position_cmd_[i]);
      position_interface_.registerHandle(jointPositionHandle);

      // Create Velocity Joint interface
      JointHandle jointVelocityHandle(jointStateHandle,
                                      &joint_velocity_cmd_[i]);
      velocity_interface_.registerHandle(jointVelocityHandle);

      // Create Effort joint interface
      JointHandle jointEffortHandle(jointStateHandle, &joint_effort_cmd_[i]);
      effort_interface_.registerHandle(jointEffortHandle);

      JointLimits limits;
      getJointLimits(joint_names_[i], nh_, limits);

      joint_limits_interface::SoftJointLimits softLimits;
      joint_limits_interface::PositionJointSoftLimitsHandle jointLimitsHandle(
          jointEffortHandle, limits, softLimits);
      position_limits_interface_.registerHandle(jointLimitsHandle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_interface_);
    registerInterface(&velocity_interface_);
    registerInterface(&effort_interface_);
    registerInterface(&position_limits_interface_);

    now_timestamp_ = ros::Time::now();
    prev_timestamp_ = ros::Time::now();

    offset_0_ = 0.0;
    offset_2_ = 0.0;
    offset_4_ = 0.0;
    offset_6_ = 0.0;

    ROS_INFO_STREAM("Kinova Gen3 controller succesfully loaded");
  }

  Gen3Control::~Gen3Control() {}

  //* Callbacks
  // Subscriber
  void Gen3Control::SubUpdateSimJointStateCb(
      const sensor_msgs::JointState::ConstPtr &msg)
  {
    std::unique_lock<std::mutex> lockAct(m_mtxAct);
    sim_joint_pos_ = VectorXd::Map(&msg->position[0], n_dof_);
    return;
  }

  // Service Server
  bool Gen3Control::SrvRobotCommandCb(
      gen3_control::robot_request::Request &request,
      gen3_control::robot_request::Response &response)
  {
    bool res;
    ROS_INFO_STREAM("Arm Command Called with request: " << request.message);
    if (request.message == "connect")
      res = connect();
    else if (request.message == "disconnect")
      res = disconnect();
    else if (request.message == "motor_on")
      res = motorOn();
    else if (request.message == "motor_off")
      res = motorOff();
    else if (request.message == "low_level_on")
      res = startLowLevelControl();
    else if (request.message == "low_level_off")
      res = stopLowLevelControl();

    else
      return false;

    return res;
  }

  //* Control Loop
  int Gen3Control::ControlLoop()
  {
    controller_manager::ControllerManager cm(this);

    ROS_INFO_STREAM("Starting control loop");
    // real_time_clock_.init();
    rt_clock_.Init();

    while (not(*kill_this_node_))
    {
      now_timestamp_ = ros::Time::now();
      period_ = now_timestamp_ - prev_timestamp_;
      prev_timestamp_ = now_timestamp_;
      // ROS_INFO_STREAM("TIME " << period_);
      if (state_ >= R_CONNECTED)
      {
        Read();
        cm.update(now_timestamp_, period_);
        if (state_ == R_SLAVE)
        {
          Write();
        }
      }

      rt_clock_.SleepToCompleteCycle();
      // real_time_clock_.updateAndSleep();
      PublishRobotState();
      // End while not kill this node
    }

    // }
    // Just to be safe, call this two functions in any circumstance.
    ROS_WARN_STREAM("Robot_node will try to turn off motors even "
                    "if they are already off.");
    // motor_off();
    disconnect();

    return 0;
  }

  // Ros Control functions
  void Gen3Control::Read()
  {
    std::vector<double> joint_pos;

    if (!use_sim_)
    {
      if (state_ >= R_CONNECTED)
      {
        VectorXd act_joint_pos_km1;
        act_joint_pos_km1 = act_joint_pos_raw_;

        // Get arm joint positions [deg]
        arm_->getJointsPosition(joint_pos);
        if (joint_pos[1] > 231.1)
          joint_pos[1] = -360 + joint_pos[1];
        if (joint_pos[3] > 212.2)
          joint_pos[3] = -360 + joint_pos[3];
        if (joint_pos[5] > 239.7)
          joint_pos[5] = -360 + joint_pos[5];

        act_joint_pos_raw_ =
            VectorXd::Map(&joint_pos[0], joint_pos.size()) * kDeg2Rad;
        act_joint_pos_ = act_joint_pos_raw_;

        if (act_joint_pos_raw_[0] - act_joint_pos_km1[0] > M_PI)
        {
          offset_0_ += -2 * M_PI;
        }
        else if (act_joint_pos_raw_[0] - act_joint_pos_km1[0] < -M_PI)
        {
          offset_0_ += 2 * M_PI;
        }
        if (act_joint_pos_raw_[2] - act_joint_pos_km1[2] > M_PI)
        {
          offset_2_ += -2 * M_PI;
        }
        else if (act_joint_pos_raw_[2] - act_joint_pos_km1[2] < -M_PI)
        {
          offset_2_ += 2 * M_PI;
        }
        if (act_joint_pos_raw_[4] - act_joint_pos_km1[4] > M_PI)
        {
          offset_4_ += -2 * M_PI;
        }
        else if (act_joint_pos_raw_[4] - act_joint_pos_km1[4] < -M_PI)
        {
          offset_4_ += 2 * M_PI;
        }
        if (act_joint_pos_raw_[6] - act_joint_pos_km1[6] > M_PI)
        {
          offset_6_ += -2 * M_PI;
        }
        else if (act_joint_pos_raw_[6] - act_joint_pos_km1[6] < -M_PI)
        {
          offset_6_ += 2 * M_PI;
        }

        act_joint_pos_[0] = act_joint_pos_raw_[0] + offset_0_;
        act_joint_pos_[2] = act_joint_pos_raw_[2] + offset_2_;
        act_joint_pos_[4] = act_joint_pos_raw_[4] + offset_4_;
        act_joint_pos_[6] = act_joint_pos_raw_[6] + offset_6_;
      }
    }
    else
    {
      act_joint_pos_ = sim_joint_pos_;
    }
    joint_position_ = act_joint_pos_;
  }

  void Gen3Control::Write()
  {
    des_joint_pos_ = joint_position_cmd_;

    if (!use_sim_)
      SendCmdRobot();

    SendCmdSim();
  }

  //* Robot Command Functions
  bool Gen3Control::connect()
  {
    if (!use_sim_)
    {
      ROS_INFO_STREAM("Gen3Control::ConnectCallback called " << state_);
      if (state_ < R_CONNECTED)
      {
        arm_->connect();
      }

      /*Get current joint values*/
      int i = 0; // An upper limit of tries so that it doesn't get locked here
                 // if something goes wrong.

      while (act_joint_pos_.norm() < 1 && not(*kill_this_node_))
      {
        std::vector<double> joint_pos;
        arm_->getJointsPosition(joint_pos);

        act_joint_pos_ =
            VectorXd::Map(&joint_pos[0], joint_pos.size()) * kDeg2Rad;
        SafeSleepSeconds(0.1);
        if (i > 1000)
        {
          ROS_ERROR_STREAM(
              "Timeout reached when getting robot's initial posture.");
          return false;
        }
        i++;
      }

      des_joint_pos_ = act_joint_pos_;
      joint_position_cmd_ = act_joint_pos_;
      ROS_INFO_STREAM("Robot State change to: CONNECTED");
      ROS_INFO_STREAM(
          "Current joint positions are: " << act_joint_pos_.transpose());
      state_ = R_CONNECTED;
    }
    else
    {
      act_joint_pos_ = sim_joint_pos_;
      des_joint_pos_ = sim_joint_pos_;
      joint_position_cmd_ = sim_joint_pos_;

      ROS_INFO_STREAM("Robot State change to: CONNECTED");
      ROS_INFO_STREAM(
          "Current joint positions are: " << act_joint_pos_.transpose());
      state_ = R_CONNECTED;
    }

    return true;
  }

  bool Gen3Control::disconnect()
  {
    if (!use_sim_)
    {
      ROS_INFO_STREAM("Gen3Control::DisconnectCallback called" << state_);
      if (state_ == R_CONNECTED)
      {
        arm_->disconnect();
      }
      else if (state_ > R_CONNECTED)
      {
        arm_->disconnect();
      }
    }
    state_ = R_UNINITIALIZED;
    ROS_INFO_STREAM("Robot State change to: UNINITIALIZED");

    return true;
  }

  bool Gen3Control::motorOn()
  {
    if (!use_sim_)
    {
      if (state_ == R_CONNECTED)
      {
        // MotorOn();
      }
    }

    state_ = R_READY;
    ROS_INFO_STREAM("Robot State change to: R_READY");
    return true;
  }

  bool Gen3Control::motorOff()
  {
    if (!use_sim_)
    {
      if (state_ < R_READY)
        ROS_WARN_STREAM(
            "Motors shouldnt be ON but will try to turn them off anyways...");
      // MotorOff();
    }
    state_ = R_CONNECTED;
    ROS_INFO_STREAM("Robot State change to: CONNECTED");
    return true;
  }

  bool Gen3Control::startLowLevelControl()
  {

    if (!use_sim_)
    {
      if (state_ == R_READY)
      {
        arm_->lowLevelServoModeOn();
        // Initialize each actuator to its current position
        arm_->initSetJointsPosition();
      }
    }

    state_ = R_SLAVE;
    ROS_INFO_STREAM("Robot State change to: SLAVE");
    return true;
  }

  bool Gen3Control::stopLowLevelControl()
  {
    if (!use_sim_)
    {
      if (state_ < R_READY)
      {
        ROS_WARN_STREAM(
            "Motors shouldnt be ON but will try to turn them off anyways...");
      }
      arm_->lowLevelServoModeOff();
    }
    state_ = R_READY;
    ROS_INFO_STREAM("Robot State change to: READY");
    return true;
  }

  void Gen3Control::SafeSleepSeconds(double seconds)
  {
    int n_cycles = int(1000000.0 * seconds / cycle_t_us_);
    for (int i = 0; i < n_cycles && not(*kill_this_node_); i++)
    {
      rt_clock_.SleepToCompleteCycle();
    }
  }

  void Gen3Control::BlockSleepSeconds(double seconds)
  {
    int n_cycles = int(1000000.0 * seconds / cycle_t_us_);
    for (int i = 0; i < n_cycles; i++)
    {
      rt_clock_.SleepToCompleteCycle();
    }
  }

  //* Auxiliar Functions

  void Gen3Control::SendCmdRobot()
  {
    arm_->setJointsPosition(des_joint_pos_ * kRad2Deg);
  }

  void Gen3Control::SendCmdSim()
  {
    pub_sim_joint_cmd_msg_.header.stamp = ros::Time::now();
    VectorXd::Map(&pub_sim_joint_cmd_msg_.position[0],
                  pub_sim_joint_cmd_msg_.position.size()) = des_joint_pos_;
    pub_sim_joint_cmd_.publish(pub_sim_joint_cmd_msg_);
  }

  void Gen3Control::PublishRobotState()
  {
    pub_state_msg_.data[0] = state_;
    // pub_state_msg_.data[1] = state_;
    pub_state_.publish(pub_state_msg_);
  }

  // Auxiliary function: taken from kortex_math_util.cpp
  double Gen3Control::wrapDegreesFromZeroTo360(double deg_not_wrapped)
  {
    int n;
    return wrapDegreesFromZeroTo360(deg_not_wrapped, n);
  }

  double Gen3Control::wrapDegreesFromZeroTo360(double deg_not_wrapped,
                                               int &number_of_turns)
  {
    bool properly_wrapped = false;
    number_of_turns = 0;
    do
    {
      if (deg_not_wrapped > 360.0)
      {
        number_of_turns += 1;
        deg_not_wrapped -= 360.0;
      }
      else if (deg_not_wrapped < 0.0)
      {
        number_of_turns -= 1;
        deg_not_wrapped += 360.0;
      }
      else
      {
        properly_wrapped = true;
      }
    } while (!properly_wrapped);
    return deg_not_wrapped;
  }

} // namespace gen3_nu
