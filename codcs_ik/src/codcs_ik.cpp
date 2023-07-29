/******************************************************************************
# codcs_ik.cpp:  Constrained Dual Concurrent IK Solver                        #
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
#include <codcs_ik/codcs_ik.hpp>

namespace codcs_ik
{
  // Task Types
  enum
  {
    CONSTRAINT = 0,
    OPERATIONAL,
    POSTURE
  };

  // Constraints Types
  enum
  {
    JOINT_LIMITS = 0,
    JOINT_SPEED,
    CART_SPEED
  };

  // Operational Task Subtypes
  enum
  {
    CART_VEL_TRANSLATION = 0,
    CART_VEL_ORIENTATION,
    CART_VEL_POSE,
  };

  typedef struct
  {
    std::string name;
    int priority;
    std::string type;
    int type_id;
    std::string subtype;
    int subtype_id;

  } VarTask;

  double fRand(double min, double max)
  {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
  }

  CODCS_IK::CODCS_IK(const std::string &_urdf_file,
                     const std::string &_base_link, const std::string &_ee_link,
                     const std::string &_ik_solver, SolverOptions _solver_opts,
                     double _max_time, double _max_error, int _max_iter,
                     double _dt)
      : initialized_(false), max_error_(_max_error), max_time_(_max_time),
        max_iter_(_max_iter), urdf_file_(_urdf_file), base_link_(_base_link),
        ee_link_(_ee_link), ik_solver_(_ik_solver), dt_(_dt),
        solver_name_(_ik_solver), solver_opts_(_solver_opts)
  {
    initialize();
  }

  CODCS_IK::~CODCS_IK()
  {
    if (task1_.joinable())
      task1_.join();
    if (task2_.joinable())
      task2_.join();
  }

  int CODCS_IK::solveIk(const pin::SE3 x_des)
  {
    int res = 1;
    VectorXd q_init = pin::neutral(model_);
    VectorXd q_sol = pin::neutral(model_);

    std::cout << "Solving IK with " << ik_solver_ << std::endl;
    auto start_cb_time = std::chrono::high_resolution_clock::now();

    if (ik_solver_ == "codcs_tp")
      res = tp_solver_->IkSolve(q_init, x_des, q_sol);
    else if (ik_solver_ == "codcs_nlo")
      res = nlo_solver_->IkSolve(q_init, x_des, q_sol);
    else if (ik_solver_ == "codcs")
      res = IkSolve(q_init, x_des, q_sol);
    else
      std::cout << "No IK solver found" << std::endl;

    auto stop_cb_time = std::chrono::high_resolution_clock::now();
    auto duration_cb = std::chrono::duration_cast<std::chrono::microseconds>(
        stop_cb_time - start_cb_time);
    std::cout << "Time IK [us]: " << duration_cb.count() << " usec"
              << std::endl;
    std::cout << "Solution found: " << q_sol.transpose() << std::endl;
    std::cout << "Solution found (deg): " << q_sol.transpose() * (180 / M_PI)
              << std::endl;

    return res;
  }

  int CODCS_IK::solveIk(const VectorXd q_init, const pin::SE3 &x_des,
                        VectorXd &q_sol)
  {
    int res = 1;

    std::cout << "Solving IK with " << ik_solver_ << std::endl;
    auto start_cb_time = std::chrono::high_resolution_clock::now();

    if (ik_solver_ == "codcs_tp")
      res = tp_solver_->IkSolve(q_init, x_des, q_sol);
    else if (ik_solver_ == "codcs_nlo")
      res = nlo_solver_->IkSolve(q_init, x_des, q_sol);
    else if (ik_solver_ == "codcs")
      res = IkSolve(q_init, x_des, q_sol);
    else
      std::cout << "No IK solver found" << std::endl;

    auto stop_cb_time = std::chrono::high_resolution_clock::now();
    auto duration_cb = std::chrono::duration_cast<std::chrono::microseconds>(
        stop_cb_time - start_cb_time);
    std::cout << "Time IK [us]: " << duration_cb.count() << " usec"
              << std::endl;
    std::cout << "Solution found: " << q_sol.transpose() << std::endl;
    std::cout << "Solution found (deg): " << q_sol.transpose() * (180 / M_PI)
              << std::endl;

    return res;
  }
  int CODCS_IK::solveFk(const VectorXd q_act, pin::SE3 &x_B_Fee)
  {
    int res = 1;

    pin::forwardKinematics(model_, mdl_data_, q_act);
    pin::updateFramePlacements(model_, mdl_data_);

    x_B_Fee = mdl_data_.oMf[ee_id_];
    return res;
  }

  int CODCS_IK::IkSolve(const VectorXd q_init, const pin::SE3 &x_des,
                        VectorXd &q_sol)
  {
    start_iksolve_time_ = std::chrono::high_resolution_clock::now();
    int res = 1;

    if (ik_solver_ == "codcs_tp")
    {
      res = tp_solver_->IkSolve(q_init, x_des, q_sol);
      return res;
    }

    if (ik_solver_ == "codcs_nlo")
    {
      res = nlo_solver_->IkSolve(q_init, x_des, q_sol);
      return res;
    }

    nlo_solver_->reset();
    tp_solver_->reset();

    q_solutions_.clear();

    task1_ = std::thread(&CODCS_IK::runTPIK, this, q_init, x_des);
    task2_ = std::thread(&CODCS_IK::runNLOIK, this, q_init, x_des);

    task1_.join();
    // auto diff_solver = std::chrono::duration_cast<std::chrono::microseconds>(
    //     std::chrono::high_resolution_clock::now() - start_iksolve_time_);
    // std::cout << "COODCS total 1: " << diff_solver.count() << " uS" <<
    // std::endl;
    task2_.join();

    // diff_solver = std::chrono::duration_cast<std::chrono::microseconds>(
    //     std::chrono::high_resolution_clock::now() - start_iksolve_time_);
    // std::cout << "COODCS total 2: " << diff_solver.count() << " uS" <<
    // std::endl;

    if (!q_solutions_.empty())
    {
      q_sol = q_solutions_[0];
      // std::cout << "Solutions found: " << q_solutions_.size() << std::endl;
      res = 0;
    }

    return res;
  }

  void CODCS_IK::printStatistics()
  {
    std::cout << "TP"
              << " found " << succ_sol_tp_ << " solutions" << std::endl;
    std::cout << "NLO"
              << " found " << succ_sol_nlo_ << " solutions" << std::endl;
  }

  int CODCS_IK::ReadDescriptionFile(std::string file_path)
  {
    YAML::Node desc_file;
    std::vector<std::string> keys;
    std::vector<std::string> solvers;
    std::vector<std::string> tasks;
    std::vector<std::string> constraints;
    try
    {
      desc_file = YAML::LoadFile(file_path);
    }
    catch (...)
    {
      std::cout << "Calibration file couldn't be loaded. Please verify paths."
                << std::endl;
      return -1;
    }

    // Extracting Problem description Map
    YAML::const_iterator initial_key = desc_file.begin();
    // std::cout << initial_key->first.as<std::string>() << std::endl;
    YAML::Node prob_desc = desc_file[initial_key->first.as<std::string>()];

    // Verifying is a Map Type
    switch (prob_desc.Type())
    {
    case YAML::NodeType::Null:
      std::cout << "NULL" << std::endl;
      break;
    case YAML::NodeType::Scalar:
      std::cout << "Scalar" << std::endl;
      break;
    case YAML::NodeType::Sequence:
      std::cout << "Sequence" << std::endl;
      break;
    case YAML::NodeType::Map:
      std::cout << "Map" << std::endl;
      break;
    case YAML::NodeType::Undefined:
      std::cout << "Undefined" << std::endl;
      break;
    }

    // Getting Description Keys
    for (YAML::const_iterator it = prob_desc.begin(); it != prob_desc.end();
         ++it)
    {
      std::string key;
      key = it->first.as<std::string>();
      keys.push_back(key);
      std::cout << "KEY: " << key << std::endl;
    }

    solvers = prob_desc["solvers"].as<std::vector<std::string>>();
    constraints = prob_desc["constraints"].as<std::vector<std::string>>();
    tasks = prob_desc["tasks"].as<std::vector<std::string>>();

    std::vector<VarTask> tasks_container;
    tasks_container.resize(tasks.size());

    for (int idx = 0; idx < tasks.size(); idx++)
    {
      tasks_container[idx].name = tasks[idx];
      tasks_container[idx].priority =
          prob_desc[tasks[idx]]["priority"].as<int>();
      tasks_container[idx].type =
          prob_desc[tasks[idx]]["type"].as<std::string>();
      tasks_container[idx].subtype =
          prob_desc[tasks[idx]]["subtype"].as<std::string>();
    }

    for (int idx = 0; idx < tasks_container.size(); idx++)
    {
      std::cout << "Task[" << idx << "]: " << tasks_container[idx].name
                << std::endl;
      std::cout << "-priority: " << tasks_container[idx].priority << std::endl;
      std::cout << "-type: " << tasks_container[idx].type << std::endl;
      std::cout << "-subtype: " << tasks_container[idx].subtype << std::endl;
    }

    // std::vector<std::string> solvers =
    // desc_file["constraints"].as<std::vector<std::string>>();

    // std::cout << "Solvers found:" << desc_file["constraints"].size() <<
    // std::endl; for (YAML::const_iterator it = solvers.begin(); it !=
    // solvers.end(); it++)
    // {
    //   std::cout << "\t" << *it;
    // }
    // std::cout << std::endl;
    return 0;
  }

  bool CODCS_IK::initialize()
  {
    std::cout << "Initializing CODCS_IK with Max. Error: " << max_error_
              << " Max. Time:" << max_time_ << " Max. It.:" << max_iter_
              << " Delta T:" << dt_ << std::endl;
    //   Load the urdf model
    pin::urdf::buildModel(urdf_file_, model_);
    mdl_data_ = pin::Data(model_);

    n_joints_ = model_.nq;
    // q_sol_.resize(n_joints_);
    q_solutions_.clear();
    errors_.clear();
    succ_sol_tp_ = 0;
    succ_sol_nlo_ = 0;

    // Getting Joints Limits
    q_ul_ = model_.upperPositionLimit;
    q_ll_ = model_.lowerPositionLimit;

    printModelInfo();

    std::cout << "EE link name: " << ee_link_ << std::endl;
    ee_id_ = model_.getFrameId(ee_link_);
    std::cout << "EE link frame id: " << ee_id_ << std::endl;

    if (ik_solver_ == "codcs_tp")
      tp_solver_.reset(new TP_IkSolver<TP_PINOCCHIO>(
          model_, ee_id_, solver_opts_, max_time_, max_error_, max_iter_, dt_));
    else if (ik_solver_ == "codcs_nlo")
      // nlo_solver_.reset(new NLO_IkSolver<NLO_NLOPT>(
      //     model_, ee_id_, solver_opts_, max_time_, max_error_, max_iter_,
      //     dt_));
      nlo_solver_.reset(new NLO_IkSolver<NLO_CASADI>(
          model_, ee_id_, solver_opts_, max_time_, max_error_, max_iter_, dt_));
    else if (ik_solver_ == "codcs")
      initialize_codcs();
    else
      std::cout << "No IK solver found" << std::endl;

    initialized_ = true;

    return true;
  }

  bool CODCS_IK::initialize_codcs()
  {
    tp_solver_.reset(new TP_IkSolver<TP_PINOCCHIO>(
        model_, ee_id_, solver_opts_, max_time_, max_error_, max_iter_, dt_));
    // nlo_solver_.reset(new NLO_IkSolver<NLO_NLOPT>(
    //     model_, ee_id_, solver_opts_, max_time_, max_error_, max_iter_,
    //     dt_));
    nlo_solver_.reset(new NLO_IkSolver<NLO_CASADI>(
        model_, ee_id_, solver_opts_, max_time_, max_error_, max_iter_, dt_));

    return true;
  }

  bool CODCS_IK::printModelInfo()
  {
    // std::cout << "Joints found by size: " << model_.joints.size() << "\n";
    std::cout << "Number of Joints found in model: " << model_.njoints << "\n";
    std::cout << "Model nq (positon states): " << model_.nq << "\n";
    std::cout << "Model nv (velocity states): " << model_.nv << "\n";
    std::cout << "Joints lower limits [rad]: "
              << model_.lowerPositionLimit.transpose() << "\n";
    std::cout << "Joints upper limits [rad]: "
              << model_.upperPositionLimit.transpose() << "\n";

    // Print Joints and frame information
    // std::cout << "Joints ID for [R_joint_1] : "
    //           << model_.getJointId("R_joint_1") << "\n";

    // // Printing Joints info
    // std::cout << "Joints found by njoints: " << model_.njoints << "\n";
    // for (int i = 0; i < model_.njoints; i++)
    //   std::cout << "Joint[" << i << "]:  " << model_.joints[i] << "\n";
    // // Printin Frames info
    // std::cout << "Frames found by nframes: " << model_.nframes << "\n";
    // for (int i = 0; i < model_.nframes; i++)
    //   std::cout << "Frame[" << i << "]:  " << model_.frames[i] << "\n";

    return true;
  }

  template <typename T1, typename T2>
  bool CODCS_IK::runSolver(T1 &solver, T2 &other_solver, const VectorXd q_init,
                           const pin::SE3 &x_des, int id)
  {
    VectorXd q_sol;
    double time_left;

    std::chrono::microseconds diff;
    std::chrono::microseconds diff_solver;

    while (true)
    {
      diff = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now() - start_iksolve_time_);
      time_left = max_time_ - diff.count() / 1000000.0;

      if (time_left <= 0)
        break;

      solver.set_max_time(time_left);

      bool res = solver.IkSolve(q_init, x_des, q_sol);

      // if (id == 1)
      // {
      //   std::cout << "TP: " << diff.count() << " uS" << std::endl;
      //   diff_solver = std::chrono::duration_cast<std::chrono::microseconds>(
      //       std::chrono::high_resolution_clock::now() - start_iksolve_time_);
      //   std::cout << "TP: " << diff_solver.count() << " uS" << std::endl;
      // }
      // else
      // {
      //   std::cout << "NLO: " << diff.count() << " uS" << std::endl;
      //   diff_solver = std::chrono::duration_cast<std::chrono::microseconds>(
      //       std::chrono::high_resolution_clock::now() - start_iksolve_time_);
      //   std::cout << "NLO: " << diff_solver.count() << " uS" << std::endl;
      // }

      mtx_.lock();

      if (res)
      {
        if (id == 1)
        {
          succ_sol_tp_++;
        }
        else
        {
          succ_sol_nlo_++;
        }
        q_solutions_.push_back(q_sol);
      }
      mtx_.unlock();

      if (!q_solutions_.empty())
      {
        break;
      }
    }

    other_solver.abort();
    solver.set_max_time(max_time_);

    return true;
  }

} // namespace codcs_ik