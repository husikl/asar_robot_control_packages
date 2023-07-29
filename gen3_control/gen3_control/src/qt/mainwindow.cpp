/******************************************************************************
# mainwindow.cpp: Gen3 Robot GUI                                            # #
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
  sub_arm_joint_state_ =
      nh_.subscribe("joint_states", 1, &MainWindow::updateArmJointStateCb, this);

  sub_arm_state_ =
      nh_.subscribe("arm_state", 1, &MainWindow::updateArmStateCb, this);

  // Services
  srv_client_arm_cmd_ =
      nh_.serviceClient<gen3_control::robot_request>("arm_request");

  // Minitab labels
  arm_joint_pos_labels_.push_back(ui->label_j_pose_1);
  arm_joint_pos_labels_.push_back(ui->label_j_pose_2);
  arm_joint_pos_labels_.push_back(ui->label_j_pose_3);
  arm_joint_pos_labels_.push_back(ui->label_j_pose_4);
  arm_joint_pos_labels_.push_back(ui->label_j_pose_5);
  arm_joint_pos_labels_.push_back(ui->label_j_pose_6);
  arm_joint_pos_labels_.push_back(ui->label_j_pose_7);

  ui->button_motoron->setEnabled(false);
  ui->button_slave->setEnabled(false);

  flag_connected_ = false;
  flag_motor_on_ = false;
  flag_slave_on_ = false;

  ui_update_timer_ = new QTimer(this);
  connect(ui_update_timer_, SIGNAL(timeout()), this, SLOT(cyclic_update()));
  ui_update_timer_->start(100);

  arm_status_ = R_UNINITIALIZED;
  arm_joint_status_.resize(7);
  arm_joint_status_.setZero();
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
    arm_joint_pos_labels_[i]->setText(QString::number(arm_joint_status_(i), 'f', 2));
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

void MainWindow::cyclic_update()
{
  updateArmStateLabel();
  updateArmJointStateLabels();

  ros::spinOnce();
}

void MainWindow::on_button_connect_clicked()
{
  if (!flag_connected_)
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
  if (!flag_motor_on_)
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
  if (!flag_slave_on_)
  {
    armSlaveOn();
  }
  else
  {
    armSlaveOff();
  }
  call_arm_srv_();
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

void MainWindow::armConnectionOn()
{
  srv_req_arm_cmd_.request.message = "connect";

  flag_connected_ = true;
  ui->button_connect->setChecked(true);

  ui->button_motoron->setEnabled(true);
  ui->button_slave->setEnabled(false);
}
void MainWindow::armConnectionOff()
{
  srv_req_arm_cmd_.request.message = "disconnect";

  flag_connected_ = false;
  ui->button_connect->setChecked(false);

  ui->button_connect->setEnabled(true);
  ui->button_motoron->setEnabled(false);
  ui->button_slave->setEnabled(false);
}

void MainWindow::armMotorOn()
{
  srv_req_arm_cmd_.request.message = "motor_on";

  flag_motor_on_ = true;
  ui->button_motoron->setChecked(true);

  ui->button_connect->setEnabled(false);
  ui->button_slave->setEnabled(true);
}

void MainWindow::armMotorOff()
{
  srv_req_arm_cmd_.request.message = "motor_off";

  flag_motor_on_ = false;
  ui->button_motoron->setChecked(false);

  ui->button_connect->setEnabled(true);
  ui->button_slave->setEnabled(false);
  ui->button_motoron->setEnabled(true);
}

void MainWindow::armSlaveOn()
{
  srv_req_arm_cmd_.request.message = "low_level_on";

  flag_slave_on_ = true;
  ui->button_slave->setChecked(true);

  ui->button_connect->setEnabled(false);
  ui->button_motoron->setEnabled(false);
}

void MainWindow::armSlaveOff()
{
  srv_req_arm_cmd_.request.message = "low_level_off";

  flag_slave_on_ = false;
  ui->button_motoron->setChecked(false);

  ui->button_connect->setEnabled(false);
  ui->button_motoron->setEnabled(true);
  ui->button_slave->setEnabled(true);
}
