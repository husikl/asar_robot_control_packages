/******************************************************************************
# gen3_control.h:  Kinova Gen3 Control                                        #
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

#ifndef GEN3_CONTROL_H
#define GEN3_CONTROL_H

#include <gen3_control/gen3_hw.h>

// C
#include <pthread.h>
#include <time.h>
#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

// C++
#include <cmath>

// ROS related
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

// ROS-control
#include <controller_manager/controller_manager.h>

// RT
// #include <rt_s/realtime_clock.h>
// #include <realtime_scheduler_interface/realtime_scheduler_interface.h>

// Package headers
#include <rt_utils/rt_clock.h>
#include <gen3_control/robot_request.h>
#include <gen3_control/gen3_status_code.h>

#include <kortex_arm/kortex_arm.h>

using namespace Eigen;
using namespace hardware_interface;
using namespace realtime_utils;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace gen3_nu
{

  const double kDeg2Rad = (M_PI) / 180;
  const double kRad2Deg = 180 / (M_PI);

  class Gen3Control : public Gen3HW
  {

    struct ArmJointLimits
    {
      std::string name;
      bool has_position_limits = {false};
      double min_pos = {0.0};
      double max_pos = {0.0};
      bool has_velocity_limits = {false};
      double max_vel = {0.0};
      bool has_effort_limits = {false};
      double max_eff = {0.0};
    };

  public:
    //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor
    Gen3Control(ros::NodeHandle &node_handle, bool *kill_this_node);
    ~Gen3Control();

    // Callbacks
    bool SrvRobotCommandCb(gen3_control::robot_request::Request &request,
                           gen3_control::robot_request::Response &response);
    void SubUpdateSimJointStateCb(const sensor_msgs::JointState::ConstPtr &msg);

    // Accessors

    // Mutators

    // Functions
    int ControlLoop();

    void PublishRobotState();

    bool connect();
    bool disconnect();
    bool motorOn();
    bool motorOff();
    bool startLowLevelControl();
    bool stopLowLevelControl();

    void SafeSleepSeconds(double seconds);
    void BlockSleepSeconds(double useconds);
    void SendCmdRobot();
    void SendCmdSim();
    void Read();
    void Write();
    bool Read(const ros::Time time, const ros::Duration period);
    void Write(const ros::Time time, const ros::Duration period);

    double wrapDegreesFromZeroTo360(double deg_not_wrapped);
    double wrapDegreesFromZeroTo360(double deg_not_wrapped, int &number_of_turns);

  private:
    // ROS
    ros::NodeHandle nh_;

    // Robot variables
    int robot_id_;
    int n_dof_;
    std::string ip_address_;

    // Services
    ros::ServiceServer srv_server_robot_request_;

    // Suscribers
    ros::Subscriber sub_sim_joint_state_;

    // Publishers
    ros::Publisher pub_state_;
    std_msgs::Int32MultiArray pub_state_msg_;

    ros::Publisher pub_sim_joint_cmd_;
    sensor_msgs::JointState pub_sim_joint_cmd_msg_;

    // Joint variables
    VectorXd des_joint_pos_;
    VectorXd act_joint_pos_;
    VectorXd act_joint_pos_raw_;
    VectorXd sim_joint_pos_;

    // Offsets
    double offset_0_;
    double offset_2_;
    double offset_4_;
    double offset_6_;
    

    // RT
    RTClock rt_clock_;
    int cycle_t_us_;

    // Current state of this process
    int state_;

    // b-Cap error storage
    int bcap_error_code_;

    // Bool to kill loops
    bool *kill_this_node_;

    // Gen3 comm
    //   Gen3Robot robot_;

    // Asynchronous spinner
    ros::AsyncSpinner *spinner;

    // Mutex
    std::mutex m_mtxAct;

    // Joint limits vector
    std::vector<ArmJointLimits> joint_limits_;

    // Joint names
    std::vector<std::string> joint_names_;

    // Simulator
    bool use_sim_;

    // Kortex args
    ArmArgs arm_args_;

    // Kortex robot
    std::unique_ptr<KortexArm> arm_;

    // Time variables
    ros::Time now_timestamp_;
    ros::Time prev_timestamp_;
    ros::Duration period_;
  };
} // namespace gen3_nu

#endif
