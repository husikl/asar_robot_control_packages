/******************************************************************************
# gen3_hw.h: Gen3 Hardware Header                                           # #
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

#ifndef _GEN3_HW_H
#define _GEN3_HW_H

// #include <boost/scoped_ptr.hpp>

// Ros related
#include <ros/ros.h>

// Hardware interface related
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
// #include <joint_limits_interface/joint_limits_urdf.h>

// Eigen
#include <Eigen/Dense>

using namespace Eigen;

namespace gen3_nu
{

class Gen3HW : public hardware_interface::RobotHW
{

  public:
  protected:
  // Ros-control interfaces
  hardware_interface::JointStateInterface    joint_state_interface_;
  hardware_interface::PositionJointInterface position_interface_;
  hardware_interface::VelocityJointInterface velocity_interface_;
  hardware_interface::EffortJointInterface   effort_interface_;

  joint_limits_interface::PositionJointSoftLimitsInterface position_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface position_saturation_interface_;

  // Configuration
  //   VectorXd encoder_ranges, effort_limits, velocity_limits;

  // Shared memory
  std::vector<std::string> joint_names_;
  int                      n_dof_;
  VectorXd                 joint_position_;
  VectorXd                 joint_velocity_;
  VectorXd                 joint_effort_;
  VectorXd                 joint_position_cmd_;
  VectorXd                 joint_velocity_cmd_;
  VectorXd                 joint_effort_cmd_;

  void jointInterfaceSetZero()
  {
    joint_position_.setZero();
    joint_velocity_.setZero();
    joint_effort_.setZero();
    joint_position_cmd_.setZero();
    joint_velocity_cmd_.setZero();
    joint_effort_cmd_.setZero();
  }

  void jointInterfaceResize(int n_dof)
  {
    joint_position_.resize(n_dof);
    joint_velocity_.resize(n_dof);
    joint_effort_.resize(n_dof);
    joint_position_cmd_.resize(n_dof);
    joint_velocity_cmd_.resize(n_dof);
    joint_effort_cmd_.resize(n_dof);
  }

}; //

} // namespace gen3_nu

#endif