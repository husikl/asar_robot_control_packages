/******************************************************************************
# asar_planner.cpp:  Autonomous Surgical Assistant Robot Motion Planner       #
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

#include <asar_planner/asar_planner.h>

namespace asar_ns
{
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
    AsarPlanner::AsarPlanner(ros::NodeHandle &node_handle, bool *kill_this_node) : nh_(node_handle)
    {
        kill_this_node_ = kill_this_node;

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
            pkg_path + std::string("/urdf/") + "asar_v2_revolute.urdf";

        // Creating solver options class
        SolverOptions so;

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

        // ik_solver_.reset(new CODCS_IK(urdf_path, "R_base_link", "R_ee", ik_solver, so,
        //                               max_time, max_error, max_iter, dt));

        ik_solver_.reset(new CODCS_IK(urdf_path, "base_link", "ee_link", ik_solver, so,
                                      max_time, max_error, max_iter, dt));

        // Variables initialization
        n_joints_ = ik_solver_->get_n_joints();
        act_joints_state_.resize(n_joints_);
        act_joints_state_ = VectorXd::Zero(n_joints_);

        // Action Server
        actSolveIK_ = std::make_shared<
            SimpleActionServer<asar_control::SolveIKAction>>(
            nh_, "SolveIK",
            boost::bind(&AsarPlanner::actSolveIKCb, this, _1), false);

        actSolveIK_->registerPreemptCallback(
            boost::bind(&AsarPlanner::actCancelCb, this));

        actSolveIK_->start();
    }

    void AsarPlanner::actSolveIKCb(
        const asar_control::SolveIKGoalConstPtr &goal)
    {
        int hr;
        asar_control::SolveIKResult res;
        pin::SE3 Xd;
        VectorXd q_sol;

        ROS_INFO_STREAM(" SOLVE IK action received ");

        // Set current action
        boost::mutex::scoped_lock lockAct(mtx_Act_);
        if (m_curAct != ACT_NONE)
        {
            if (m_curAct != ACT_RESET)
            {
                res.succeed = EXIT_FAILURE;
                actSolveIK_->setAborted(res);
                ROS_INFO_STREAM("Aborting Action");
            }
            return;
        }

        m_curAct = ACT_SOLVEIK;
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
        mtx_Act_.lock();
        if (m_curAct == ACT_SOLVEIK)
        {
            if (hr == EXIT_SUCCESS)
            {
                res.succeed = EXIT_SUCCESS;
                VectorXd::Map(&res.joint_sol.position[0], n_joints_) = q_sol;
                actSolveIK_->setSucceeded(res);
            }
            else
            {
                res.succeed = EXIT_FAILURE;
                actSolveIK_->setAborted(res);
            }
            m_curAct = ACT_NONE;
        }
        mtx_Act_.unlock();
    }

    void AsarPlanner::actCancelCb()
    {
        boost::mutex::scoped_lock lockAct(mtx_Act_);

        if (m_curAct > ACT_NONE)
        {
            ROS_INFO_STREAM("Preempting Controller Actions from state " << m_curAct);
            switch (m_curAct)
            {
            case ACT_SOLVEIK:
            {
                asar_control::SolveIKResult res;
                res.succeed = EXIT_FAILURE;
                actSolveIK_->setPreempted(res);
                break;
            }
            }
            m_curAct = ACT_NONE;

            ROS_INFO_STREAM("Preempted");
        }
    }

    //* Kinematics functions

    bool AsarPlanner::solveIK(const pin::SE3 &Xd, VectorXd q_sol)
    {
        // VectorXd q_sol(n_joints_);

        int res_ik = 1;

        res_ik = ik_solver_->IkSolve(act_joints_state_, Xd, q_sol);

        if (!res_ik)
            return EXIT_SUCCESS;
        else
            return EXIT_FAILURE;
    }

    void AsarPlanner::solveFK()
    {
        // ik_solver_->solveFk(act_joints_state_, ee_pose_act_);
    }
}