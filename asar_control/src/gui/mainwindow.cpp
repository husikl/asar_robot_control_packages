/******************************************************************************
# mainwindow.cpp: ASAR Robot GUI                                            # #
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

#include "mainwindow.h"

#include "ui_mainwindow.h"

MainWindow::MainWindow(ros::NodeHandle &node_handle, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  nh_ = node_handle;

  //* ROS PARAMETER SERVER

  // Subscribers
  sub_arm_joint_state_ = nh_.subscribe(
      "arm/joint_states", 1, &MainWindow::updateArmJointStateCb, this);
  sub_forceps_joint_state_ = nh_.subscribe(
      "forceps/joint_states", 1, &MainWindow::updateForcepsJointStateCb, this);

  sub_arm_state_ =
      nh_.subscribe("arm/arm_state", 1, &MainWindow::updateArmStateCb, this);
  sub_forceps_state_ = nh_.subscribe("forceps/forceps_state", 1,
                                     &MainWindow::updateForcepsStateCb, this);

  // Services
  srv_client_arm_cmd_ =
      nh_.serviceClient<gen3_control::robot_request>("arm/arm_request");
  srv_client_forceps_cmd_ =
      nh_.serviceClient<gen3_control::robot_request>("forceps/forceps_request");

  // Minitab labels
  arm_joint_pos_labels_.push_back(ui->label_j_pose_1);
  arm_joint_pos_labels_.push_back(ui->label_j_pose_2);
  arm_joint_pos_labels_.push_back(ui->label_j_pose_3);
  arm_joint_pos_labels_.push_back(ui->label_j_pose_4);
  arm_joint_pos_labels_.push_back(ui->label_j_pose_5);
  arm_joint_pos_labels_.push_back(ui->label_j_pose_6);
  arm_joint_pos_labels_.push_back(ui->label_j_pose_7);

  forceps_joint_pos_labels_.push_back(ui->label_j_pose_8);
  forceps_joint_pos_labels_.push_back(ui->label_j_pose_9);
  forceps_joint_pos_labels_.push_back(ui->label_j_pose_10);

  ui->button_motoron->setEnabled(false);
  ui->button_slave->setEnabled(false);

  flag_arm_connected_ = false;
  flag_arm_motor_on_ = false;
  flag_arm_slave_on_ = false;
  flag_forceps_connected_ = false;
  flag_forceps_calibrated_ = false;
  flag_forceps_pos_control_on_ = false;
  flag_forceps_torque_control_on_ = false;

  ui_update_timer_ = new QTimer(this);
  connect(ui_update_timer_, SIGNAL(timeout()), this, SLOT(cyclic_update()));
  ui_update_timer_->start(100);

  arm_status_ = R_UNINITIALIZED;
  arm_joint_status_.resize(7);
  arm_joint_status_.setZero();
  forceps_joint_status_.resize(3);
  forceps_joint_status_.setZero();
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::updateArmStateLabel()
{
  QLabel *label_status;
  label_status = ui->label_status;

  switch (arm_status_)
  {
  case R_UNINITIALIZED:
    label_status->setText(
        "<html><head/><body><p><span style=' font-size:14pt; "
        "font-weight:600; "
        "color:#ff0000;'>UNINITIALIZED</span></p></body></html>");
    break;
  case R_CONNECTED:
    label_status->setText(
        "<html><head/><body><p><span style=' font-size:14pt; "
        "font-weight:600; "
        "color:#0000FF;'>CONNECTED</span></p></body></html>");
    break;
  case R_READY:
    label_status->setText(
        "<html><head/><body><p><span style=' font-size:14pt; "
        "font-weight:600; "
        "color:#964B00;'>READY</span></p></body></html>");
    break;
  case R_SLAVE:
    label_status->setText(
        "<html><head/><body><p><span style=' font-size:14pt; "
        "font-weight:600; "
        "color:#00FF00;'>SLAVE</span></p></body></html>");
    break;
  }
}

void MainWindow::updateArmJointStateLabels()
{
  for (int i = 0; i < 7; i++)
  {
    arm_joint_pos_labels_[i]->setText(
        QString::number(arm_joint_status_(i), 'f', 2));
  }
}

void MainWindow::updateForcepsStateLabel()
{
  QLabel *label_status;
  label_status = ui->label_status_forceps;

  switch (forceps_status_)
  {
  case F_UNINITIALIZED:
    label_status->setText(
        "<html><head/><body><p><span style=' font-size:14pt; "
        "font-weight:600; "
        "color:#ff0000;'>UNINITIALIZED</span></p></body></html>");
    break;
  case F_CONNECTED:
    label_status->setText(
        "<html><head/><body><p><span style=' font-size:14pt; "
        "font-weight:600; "
        "color:#0000FF;'>CONNECTED</span></p></body></html>");
    break;
  case F_CALIBRATED:
    label_status->setText(
        "<html><head/><body><p><span style=' font-size:14pt; "
        "font-weight:600; "
        "color:#964B00;'>CALIBRATED</span></p></body></html>");
    break;
  case F_POS_CONTROL:
    label_status->setText(
        "<html><head/><body><p><span style=' font-size:14pt; "
        "font-weight:600; "
        "color:#00FF00;'>POS CONTROL</span></p></body></html>");
    break;
  case F_TORQUE_CONTROL:
    label_status->setText(
        "<html><head/><body><p><span style=' font-size:14pt; "
        "font-weight:600; "
        "color:#00FF00;'>TORQUE CONTROL</span></p></body></html>");
    break;
  }
}

void MainWindow::updateForcepsJointStateLabels()
{
  for (int i = 0; i < 3; i++)
  {
    forceps_joint_pos_labels_[i]->setText(
        QString::number(forceps_joint_status_(i), 'f', 2));
  }
}

void MainWindow::updateArmJointStateCb(
    const sensor_msgs::JointState::ConstPtr &msg)
{
  arm_joint_status_ = VectorXd::Map(&msg->position[0], msg->position.size());
}

void MainWindow::updateArmStateCb(
    const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  arm_status_ = msg->data[0];
}

void MainWindow::updateForcepsJointStateCb(
    const sensor_msgs::JointState::ConstPtr &msg)
{
  forceps_joint_status_ =
      VectorXd::Map(&msg->position[0], msg->position.size());
}

void MainWindow::updateForcepsStateCb(
    const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  forceps_status_ = msg->data[0];
}

void MainWindow::cyclic_update()
{
  updateArmStateLabel();
  updateForcepsStateLabel();
  updateArmJointStateLabels();
  updateForcepsJointStateLabels();

  ros::spinOnce();
}

void MainWindow::on_button_connect_clicked()
{
  if (!flag_arm_connected_)
  {
    armConnectionOn();
  }
  else
  {
    armConnectionOff();
  }
  call_arm_srv_();
}

void MainWindow::on_button_motoron_clicked()
{
  if (!flag_arm_motor_on_)
  {
    armMotorOn();
  }
  else
  {
    armMotorOff();
  }
  call_arm_srv_();
}

void MainWindow::on_button_slave_clicked()
{
  if (!flag_arm_slave_on_)
  {
    armSlaveOn();
  }
  else
  {
    armSlaveOff();
  }
  call_arm_srv_();
}

void MainWindow::on_button_connect_forceps_clicked()
{
  if (!flag_forceps_connected_)
  {
    forcepsConnectionOn();
  }
  else
  {
    forcepsConnectionOff();
  }
  call_forceps_srv_();
}

void MainWindow::on_button_calibrate_forceps_clicked()
{
  // if (!flag_forceps_calibrated_) {
  forcepsCalibrate();
  // }
  // else {
  //   armMotorOff();
  // }
  call_forceps_srv_();
}

void MainWindow::on_button_pos_control_forceps_clicked()
{
  if (!flag_forceps_pos_control_on_)
  {
    forcepsPositionControlOn();
  }
  else
  {
    forcepsPositionControlOff();
  }
  call_forceps_srv_();
}

void MainWindow::on_button_torque_control_forceps_clicked()
{
  if (!flag_forceps_torque_control_on_)
  {
    forcepsTorqueControlOn();
  }
  else
  {
    forcepsTorqueControlOff();
  }
  call_forceps_srv_();
}

int MainWindow::call_arm_srv_()
{
  if (srv_client_arm_cmd_.call(srv_req_arm_cmd_))
  {
    if (srv_req_arm_cmd_.response.succeeded == EXIT_SUCCESS)
    {
      ROS_INFO_STREAM("{" << srv_req_arm_cmd_.request.message << "} SUCCEED");

      return EXIT_SUCCESS;
    }
    else
    {
      ROS_ERROR_STREAM("{" << srv_req_arm_cmd_.request.message << "} FAILED");
      return EXIT_FAILURE;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Service failed ");
    return EXIT_FAILURE;
  }
}

int MainWindow::call_forceps_srv_()
{
  if (srv_client_forceps_cmd_.call(srv_req_forceps_cmd_))
  {
    if (srv_req_forceps_cmd_.response.succeeded == EXIT_SUCCESS)
    {
      ROS_INFO_STREAM("{" << srv_req_forceps_cmd_.request.message
                          << "} SUCCEED");

      return EXIT_SUCCESS;
    }
    else
    {
      ROS_ERROR_STREAM("{" << srv_req_forceps_cmd_.request.message
                           << "} FAILED");
      return EXIT_FAILURE;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Service failed ");
    return EXIT_FAILURE;
  }
}

void MainWindow::armConnectionOn()
{
  srv_req_arm_cmd_.request.message = "connect";

  flag_arm_connected_ = true;
  ui->button_connect->setChecked(true);

  ui->button_motoron->setEnabled(true);
  ui->button_slave->setEnabled(false);
}
void MainWindow::armConnectionOff()
{
  srv_req_arm_cmd_.request.message = "disconnect";

  flag_arm_connected_ = false;
  ui->button_connect->setChecked(false);

  ui->button_connect->setEnabled(true);
  ui->button_motoron->setEnabled(false);
  ui->button_slave->setEnabled(false);
}

void MainWindow::armMotorOn()
{
  srv_req_arm_cmd_.request.message = "motor_on";

  flag_arm_motor_on_ = true;
  ui->button_motoron->setChecked(true);

  ui->button_connect->setEnabled(false);
  ui->button_slave->setEnabled(true);
}

void MainWindow::armMotorOff()
{
  srv_req_arm_cmd_.request.message = "motor_off";

  flag_arm_motor_on_ = false;
  ui->button_motoron->setChecked(false);

  ui->button_connect->setEnabled(true);
  ui->button_slave->setEnabled(false);
  ui->button_motoron->setEnabled(true);
}

void MainWindow::armSlaveOn()
{
  srv_req_arm_cmd_.request.message = "low_level_on";

  flag_arm_slave_on_ = true;
  ui->button_slave->setChecked(true);

  ui->button_connect->setEnabled(false);
  ui->button_motoron->setEnabled(false);
}

void MainWindow::armSlaveOff()
{
  srv_req_arm_cmd_.request.message = "low_level_off";

  flag_arm_slave_on_ = false;
  ui->button_motoron->setChecked(false);

  ui->button_connect->setEnabled(false);
  ui->button_motoron->setEnabled(true);
  ui->button_slave->setEnabled(true);
}

// Forceps
void MainWindow::forcepsConnectionOn()
{
  srv_req_forceps_cmd_.request.message = "connect";

  flag_forceps_connected_ = true;
  ui->button_connect_forceps->setChecked(true);

  ui->button_calibrate_forceps->setEnabled(true);
  ui->button_pos_control_forceps->setEnabled(false);
  ui->button_torque_control_forceps->setEnabled(false);
}
void MainWindow::forcepsConnectionOff()
{
  srv_req_forceps_cmd_.request.message = "disconnect";

  flag_forceps_connected_ = false;
  ui->button_connect_forceps->setChecked(false);

  ui->button_connect_forceps->setEnabled(true);
  ui->button_calibrate_forceps->setEnabled(false);
  ui->button_pos_control_forceps->setEnabled(false);
  ui->button_torque_control_forceps->setEnabled(false);
}

void MainWindow::forcepsCalibrate()
{
  srv_req_forceps_cmd_.request.message = "calibrate";

  flag_forceps_calibrated_ = true;
  ui->button_calibrate_forceps->setChecked(true);

  ui->button_connect_forceps->setEnabled(true);
  ui->button_pos_control_forceps->setEnabled(true);
  ui->button_torque_control_forceps->setEnabled(true);
}

void MainWindow::forcepsPositionControlOn()
{
  srv_req_forceps_cmd_.request.message = "pos_control_on";

  flag_forceps_pos_control_on_ = true;
  ui->button_pos_control_forceps->setChecked(true);

  ui->button_connect_forceps->setEnabled(false);
  ui->button_calibrate_forceps->setEnabled(false);
  ui->button_torque_control_forceps->setEnabled(false);
}

void MainWindow::forcepsPositionControlOff()
{
  srv_req_forceps_cmd_.request.message = "control_off";

  flag_forceps_pos_control_on_ = false;
  ui->button_pos_control_forceps->setChecked(false);

  ui->button_connect_forceps->setEnabled(true);
  ui->button_calibrate_forceps->setEnabled(false);
  ui->button_torque_control_forceps->setEnabled(true);
}

void MainWindow::forcepsTorqueControlOn()
{
  srv_req_forceps_cmd_.request.message = "torque_control_on";

  flag_forceps_torque_control_on_ = true;
  ui->button_torque_control_forceps->setChecked(true);

  ui->button_connect_forceps->setEnabled(false);
  ui->button_calibrate_forceps->setEnabled(false);
  ui->button_pos_control_forceps->setEnabled(false);
}

void MainWindow::forcepsTorqueControlOff()
{
  srv_req_forceps_cmd_.request.message = "control_off";

  flag_forceps_torque_control_on_ = false;
  ui->button_torque_control_forceps->setChecked(false);

  ui->button_connect_forceps->setEnabled(true);
  ui->button_calibrate_forceps->setEnabled(false);
  ui->button_pos_control_forceps->setEnabled(true);
}