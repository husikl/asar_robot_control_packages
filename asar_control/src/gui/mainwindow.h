/******************************************************************************
# mainwindow.h: Gen3 Robot GUI                                              # #
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
#   Author: Jacinto Colan,    email: colan@robo.mein.nagoya-u.ac.jp           #
#                                                                             #
# ###########################################################################*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QLabel>
#include <QMainWindow>
#include <QString>
#include <QTimer>
#include <fstream>

// ROS related
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>

// Eigen
#include <Eigen/Dense>

// Project related
// #include <gen3_control/gen3_status_code.h>
#include <forceps_control/forceps_request.h>
#include <gen3_control/robot_request.h>

using namespace Eigen;

namespace Ui
{

  class MainWindow;
} // namespace Ui

class MainWindow : public QMainWindow
{
  Q_OBJECT

  // Robot Status
  enum
  {
    R_UNINITIALIZED = 0,
    R_CONNECTED,
    R_READY,
    R_SLAVE,
  };
  // Forceps Status
  enum
  {
    F_UNINITIALIZED = 0,
    F_CONNECTED,
    F_CALIBRATED,
    F_POS_CONTROL,
    F_TORQUE_CONTROL,
  };

  public:
  explicit MainWindow(ros::NodeHandle &node_handle, QWidget *parent = nullptr);
  ~MainWindow();

  QTimer *ui_update_timer_;

  ros::NodeHandle nh_;

  std::vector<QLabel *> arm_joint_pos_labels_;
  std::vector<QLabel *> forceps_joint_pos_labels_;

  private slots:

  void cyclic_update();

  void on_button_connect_clicked();
  void on_button_motoron_clicked();
  void on_button_slave_clicked();
  void on_button_connect_forceps_clicked();
  void on_button_calibrate_forceps_clicked();
  void on_button_pos_control_forceps_clicked();
  void on_button_torque_control_forceps_clicked();

  int call_arm_srv_();
  int call_forceps_srv_();

  void armConnectionOn();
  void armConnectionOff();
  void armMotorOn();
  void armMotorOff();
  void armSlaveOn();
  void armSlaveOff();

  void forcepsConnectionOn();
  void forcepsConnectionOff();
  void forcepsCalibrate();
  void forcepsPositionControlOn();
  void forcepsPositionControlOff();
  void forcepsTorqueControlOn();
  void forcepsTorqueControlOff();

  void updateArmJointStateLabels();
  void updateArmStateLabel();

  void updateForcepsJointStateLabels();
  void updateForcepsStateLabel();

  // Callbacks
  void updateArmJointStateCb(const sensor_msgs::JointState::ConstPtr &msg);
  void updateForcepsJointStateCb(const sensor_msgs::JointState::ConstPtr &msg);
  void updateArmStateCb(const std_msgs::Int32MultiArray::ConstPtr &msg);
  void updateForcepsStateCb(const std_msgs::Int32MultiArray::ConstPtr &msg);

  private:
  Ui::MainWindow *ui;

  std::string prefix_;

  // Subscribers
  ros::Subscriber sub_arm_joint_state_;
  ros::Subscriber sub_forceps_joint_state_;
  ros::Subscriber sub_arm_state_;
  ros::Subscriber sub_forceps_state_;

  // Service Client
  ros::ServiceClient          srv_client_arm_cmd_;
  gen3_control::robot_request srv_req_arm_cmd_;

  ros::ServiceClient               srv_client_forceps_cmd_;
  forceps_control::forceps_request srv_req_forceps_cmd_;

  // Flags
  bool flag_arm_connected_;
  bool flag_arm_motor_on_;
  bool flag_arm_slave_on_;

  bool flag_forceps_connected_;
  bool flag_forceps_calibrated_;
  bool flag_forceps_pos_control_on_;
  bool flag_forceps_torque_control_on_;

  int      arm_status_;
  VectorXd arm_joint_status_;
  int      forceps_status_;
  VectorXd forceps_joint_status_;
};

#endif // MAINWINDOW_H
