/******************************************************************************
# tp_pinocchio.h:  Task priority based IK                                     #
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

#ifndef TP_PINOCCHIO_HPP
#define TP_PINOCCHIO_HPP

// Source HPP
#include <codcs_ik/solver_base.hpp>
#include <codcs_ik/solver_options.hpp>
// Pinocchio
#include <pinocchio/algorithm/joint-configuration.hpp>
// C++
#include <chrono>

namespace codcs_ik
{

  class CODCS_IK;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;

  class TP_PINOCCHIO : public SolverBase
  {
    friend class codcs_ik::CODCS_IK;

  public:
    TP_PINOCCHIO(const pin::Model &_model, const pin::FrameIndex &_Fid,
                 SolverOptions _solver_opts, const double _max_time,
                 const double _max_error, const int _max_iter = 1000,
                 const double _dt = 1);
    ~TP_PINOCCHIO();

    int IkSolve(const VectorXd q_init, const pin::SE3 &x_des, VectorXd &q_sol);
    MatrixXd pseudoInverse(const Eigen::MatrixXd &a, double epsilon);
    MatrixXd weightedPseudoInverse(const Eigen::MatrixXd &a,
                                   const Eigen::VectorXd w);
    void GetOptions(SolverOptions _solver_opts);

    inline void abort() { aborted_ = true; }
    inline void reset() { aborted_ = false; }
    inline void set_max_time(double _max_time) { max_time_ = _max_time; };
    inline void set_max_error(double _max_error) { max_error_ = _max_error; };

  private:
    pin::Model mdl_;
    pin::Data mdl_data_;
    pin::FrameIndex fid_;

    int n_joints_;
    int max_iter_;

    double max_time_;
    double max_error_;
    double dt_;

    bool aborted_;
    bool success_;

    VectorXd q_ul_;
    VectorXd q_ll_;
    VectorXd q_sol_;

    std::string err_method_;
    std::vector<double> cost_coeff_;

    // Penalty gains
    double mu0_;
    double mu1_;

    Vector3d trocar_pos_;
    bool constrained_control_;
  };

  void TP_PINOCCHIO::GetOptions(SolverOptions _solver_opts)
  {
    SolverOptions so;

    so = _solver_opts;
    err_method_ = so.err_method_;
    cost_coeff_ = so.cost_coeff_;
    trocar_pos_ = so.trocar_pos_;
    constrained_control_ = so.constrained_control_;

    std::cout << "\n------\nOptions summary:" << std::endl;
    std::cout << "Constrained control: " << constrained_control_ << std::endl;
    std::cout << "Error method: " << err_method_ << std::endl;
  }

  TP_PINOCCHIO::TP_PINOCCHIO(const pin::Model &_model,
                             const pin::FrameIndex &_Fid,
                             SolverOptions _solver_opts,
                             const double _max_time_, const double _max_error,
                             const int _max_iter, const double _dt)
      : aborted_(false), success_(false), mdl_(_model), n_joints_(_model.nq),
        fid_(_Fid), max_time_(_max_time_), max_error_(_max_error),
        max_iter_(_max_iter), dt_(_dt)
  {
    GetOptions(_solver_opts);

    mdl_data_ = pin::Data(mdl_);
    q_ll_ = mdl_.lowerPositionLimit;
    q_ul_ = mdl_.upperPositionLimit;

    mu0_ = cost_coeff_[0]; // 10
    mu1_ = cost_coeff_[1]; // 0.005
  }

  TP_PINOCCHIO::~TP_PINOCCHIO() {}

  // Taken from https://armarx.humanoids.kit.edu/pinv_8hh_source.html
  MatrixXd TP_PINOCCHIO::pseudoInverse(const Eigen::MatrixXd &a, double epsilon)
  {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinU |
                                                 Eigen::ComputeThinV);

    double tolerance = epsilon * std::max(a.cols(), a.rows()) *
                       svd.singularValues().array().abs()(0);
    // ROS_INFO_STREAM("Singular values: " << svd.singularValues().array());
    // ROS_INFO_STREAM("U: \n" << svd.matrixV());
    // ROS_INFO_STREAM("V: \n" << svd.matrixU());

    return svd.matrixV() *
           (svd.singularValues().array().abs() > tolerance)
               .select(svd.singularValues().array().inverse(), 0)
               .matrix()
               .asDiagonal() *
           svd.matrixU().adjoint();
  }

  // Taken from https://armarx.humanoids.kit.edu/pinv_8hh_source.html
  MatrixXd TP_PINOCCHIO::weightedPseudoInverse(const Eigen::MatrixXd &a,
                                               const Eigen::VectorXd w)
  {
    int lenght = w.size();

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Winv(lenght);
    Winv = w.asDiagonal().inverse(); // diag(1./w)

    Eigen::MatrixXd tmp(lenght, lenght);

    //(a*Winv*a.transpose())
    tmp = pseudoInverse(a * Winv * a.transpose(), 10E-10);

    return Winv * a.transpose() * tmp;
  }

  int TP_PINOCCHIO::IkSolve(const VectorXd q_init, const pin::SE3 &x_d,
                            VectorXd &q_sol)
  {
    std::chrono::time_point<std::chrono::high_resolution_clock>
        start_solve_time = std::chrono::high_resolution_clock::now();
    std::chrono::microseconds diff;

    // std::cout << "Starting IK Solve with TP: " << std::endl;

    double time_left;
    Vector6d err_ee;
    Vector3d err3d_ee;
    VectorXd q_out = q_init;

    success_ = false;
    q_sol = q_init;

    Vector6d err_rcm = VectorXd::Zero(6);
    pin::Data::Matrix6x Jb_B_Fee(6, mdl_.nv);
    Jb_B_Fee.setZero();

    pin::FrameIndex id_Fee_ = fid_;
    pin::SE3 x_B_Fee = mdl_data_.oMf[id_Fee_];

    pin::Data::Matrix6x Jb_task_2;
    MatrixXd pinv_Jtask_2;

    diff = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - start_solve_time);
    time_left = max_time_ - diff.count() / 1000000.0;

    for (int it = 0;; it++)
    {
      //* Computing FK
      pin::forwardKinematics(mdl_, mdl_data_, q_out);
      pin::updateFramePlacements(mdl_, mdl_data_);

      x_B_Fee = mdl_data_.oMf[id_Fee_];

      //* Computation Errors
      if (err_method_ == "log6")
      {
        //? Using log6
        pin::SE3 T_Fee_d_Fee_a = x_d.actInv(x_B_Fee);
        err_ee = pin::log6(T_Fee_d_Fee_a).toVector();
        err3d_ee = err_ee.head(3);
      }
      else if (err_method_ == "log3")
      {
        //? Using log3
        Vector3d err_tr = mu0_ * (x_d.translation() - x_B_Fee.translation());
        Vector3d err_rot =
            mu1_ * pin::log3(x_d.rotation() * x_B_Fee.rotation().transpose());
        err_ee << err_tr, err_rot;
        err3d_ee = err_tr;
      }
      else
      {
        //? Using only p error
        Vector3d err_tr = mu0_ * (x_d.translation() - x_B_Fee.translation());
        Vector3d err_rot = Vector3d::Zero();
        err_ee << err_tr, err_rot;
        err3d_ee = err_tr;
      }

      if (err_ee.norm() < max_error_)
      {
        success_ = true;
        break;
      }

      if (time_left < 0)
      {
        std::cout << "Maximum time exceeded: " << time_left << std::endl;
        break;
      }

      if (aborted_)
      {
        std::cout << "Aborted by NL " << std::endl;
        break;
      }

      if (it >= max_iter_)
      {
        std::cout << "Maximum number of iteration reached. " << std::endl;
        break;
      }

      //*  Computing Jacobians

      pin::computeFrameJacobian(mdl_, mdl_data_, q_out, fid_, Jb_B_Fee);

      //* Defining Task Jacobians
      Jb_task_2 = Jb_B_Fee;

      //* Computing pseudoinverses
      pinv_Jtask_2 = pseudoInverse(Jb_task_2, 1e-10);

      VectorXd q_dot(mdl_.nv);
      if (constrained_control_)
      {
        //? Use constrained control
      }
      else
      {
        //? Use non-constrained control
        q_dot.noalias() = -pinv_Jtask_2 * err_ee;
      }

      VectorXd q_prev;
      q_prev.resize(9);

      q_prev = q_out;

      q_out = pin::integrate(mdl_, q_prev, q_dot * dt_);

      for (int i = 0; i < n_joints_; i++)
      {
        if (q_out[i] > q_ul_[i])
          q_out[i] = q_out[i] - 2 * M_PI;
        if (q_out[i] < q_ll_[i])
          q_out[i] = q_out[i] + 2 * M_PI;
      }

      std::cout << "Iteration [" << it << "]: error = " << err_ee.transpose()
                << std::endl;

      diff = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now() - start_solve_time);
      time_left = max_time_ - diff.count() / 1000000.0;
    }

    if (success_)
    {
      // std::cout << "TP: Convergence achieved!" << std::endl;
      q_sol = q_out;
      return 0;
    }
    else
    {
      std::cout << "\nTP: Warning: the iterative algorithm has not reached "
                   "convergence to the desired precision "
                << std::endl;
      return 1;
    }
  }
} // namespace codcs_ik

#endif