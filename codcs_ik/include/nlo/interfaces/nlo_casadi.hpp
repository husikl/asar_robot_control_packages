/******************************************************************************
# nlo_casadi.h:  Nonlinear Optimization based IK                              #
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

#ifndef NLO_CASADI_HPP
#define NLO_CASADI_HPP

#include <codcs_ik/solver_base.hpp>
#include <codcs_ik/solver_options.hpp>
// Casadi
#include <casadi/casadi.hpp>
// C++
#include <map>
// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace ca = casadi;

namespace codcs_ik
{

  class NLO_CASADI : public SolverBase
  {
  private:
    pin::Data mdl_data_;

    int max_iter_;
    double max_time_;
    double max_error_;
    double dt_;

    int verb_level_;
    std::string time_stats_;

    VectorXd q_ul_;
    VectorXd q_ll_;
    VectorXd q_sol_;
    std::vector<double> q_sol_vec_;
    VectorXd q_max_;
    VectorXd q_min_;

    Vector3d trocar_pos_;
    std::string nlp_linear_solver_;
    bool constrained_control_;
    bool rcm_is_cost_;

    ca::Function ca_perr2_;
    ca::Function ca_ercm_;
    ca::Function solver_;

    pin::SE3 x_B_Fnostril_;

    // Penalty gains
    double mu0_;
    double mu1_;
    double mu2_;
    double mu3_;

    ca::Function ca_log3_;
    ca::Function ca_log6_;
    ca::Function FK_;
    ca::Function FK_J7_;
    ca::Function FK_J8_;

    pin::FrameIndex id_Fee_; // Getting EE Frame id
    pin::FrameIndex id_Fj7_; // Getting EE Frame id
    pin::FrameIndex id_Fj8_; // Getting EE Frame id

    double rcm_error_max_;
    std::string err_method_;
    std::vector<double> cost_coeff_;

    SolverOptions solver_opts_;

  public:
    NLO_CASADI(const pin::Model &_model, const pin::FrameIndex &_Fid,
               SolverOptions _solver_opts, const double _max_time_,
               const double _eps, const int _max_iter, const double _dt);
    ~NLO_CASADI();

    int IkSolve(const VectorXd q_init, const pin::SE3 &x_d, VectorXd &q_sol);

    ca::DM eig_to_casDM(const VectorXd &eig);
    ca::DM eigmat_to_casDM(const MatrixXd &eig);
    MatrixXd casDM_to_eig(const casadi::DM &cas);

    void generate_ca_RCM_error();
    void generate_ca_EE_error();
    void generate_ca_log3();
    void generate_ca_log6();
    void generate_nlsolver();
    void GetOptions(SolverOptions _solver_opts);

    inline void abort() { aborted_ = true; }
    inline void reset() { aborted_ = false; }
    inline void set_max_time(double _max_time) { max_time_ = _max_time; }
    inline void set_max_error(double _max_error) { max_error_ = _max_error; }
  };

  void NLO_CASADI::GetOptions(SolverOptions _solver_opts)
  {
    SolverOptions so;

    so = _solver_opts;

    int n_cost_coeff = 1;
    cost_coeff_.clear();
    nlp_linear_solver_ = "ma57";
    constrained_control_ = false;
    rcm_is_cost_ = false;

    time_stats_ = so.time_stats_;
    verb_level_ = so.verb_level_;
    rcm_is_cost_ = so.rcm_is_cost_;
    constrained_control_ = so.constrained_control_;
    nlp_linear_solver_ = so.nlp_linear_solver_;
    cost_coeff_ = so.cost_coeff_;
    trocar_pos_ = so.trocar_pos_;
    rcm_error_max_ = so.rcm_error_max_;
    err_method_ = so.err_method_;
    std::cout << "constrained_control_: " << constrained_control_ << std::endl;
    std::cout << "cost_coeff_[0]: " << cost_coeff_[0] << std::endl;
    std::cout << "cost_coeff_[1]: " << cost_coeff_[1] << std::endl;
    std::cout << "cost_coeff_[2]: " << cost_coeff_[2] << std::endl;
    std::cout << "cost_coeff_[3]: " << cost_coeff_[3] << std::endl;
  }

  NLO_CASADI::NLO_CASADI(const pin::Model &_model, const pin::FrameIndex &_Fid,
                         SolverOptions _solver_opts, const double _max_time_,
                         const double _eps, const int _max_iter,
                         const double _dt)
  {
    GetOptions(_solver_opts);
    max_error_ = _eps;
    max_iter_ = _max_iter;
    max_time_ = _max_time_;

    x_B_Fnostril_ = pin::SE3(Eigen::Matrix3d::Identity(), trocar_pos_);
    mu0_ = cost_coeff_[0]; // 10
    mu1_ = cost_coeff_[1]; // 0.005
    mu2_ = cost_coeff_[2]; // 0.001
    mu3_ = cost_coeff_[3]; // 100

    mdl_ = _model;
    q_max_ = mdl_.upperPositionLimit;
    q_min_ = mdl_.lowerPositionLimit;

    id_Fee_ = mdl_.getFrameId("ee_link");
    id_Fj7_ = mdl_.getFrameId("joint_7");
    // id_Fj7_ = mdl_.getFrameId("R_link1");
    id_Fj8_ = mdl_.getFrameId("joint_f0_pitch");

    //* Exporting Kinematics Casadi Functions
    //? Generate Functions
    // Cast the model into casadi::SX
    pin::ModelTpl<ca::SX> model = mdl_.cast<ca::SX>();
    // Create Data model as casadi::SX
    pinocchio::DataTpl<ca::SX> data(model);

    // Create casadi::SX joint variable
    ca::SX ca_q = ca::SX::sym("ca_q", 9, 1);
    // Create associated Eigen matrix
    Eigen::Matrix<ca::SX, 9, 1> _q;
    // Copy casadi::SX into Eigen::Matrix
    pin::casadi::copy(ca_q, _q);

    // Compute symbolic FK
    pin::framesForwardKinematics(model, data, _q);
    // Extract Eigen::Matrix results
    Eigen::Matrix<ca::SX, 3, 1> eig_fk_pos = data.oMf.at(id_Fee_).translation();
    Eigen::Matrix<ca::SX, 3, 3> eig_fk_rot = data.oMf.at(id_Fee_).rotation();
    // Create associated casadi::SX variables

    ca::SX ca_fk_tr =
        ca::SX(ca::Sparsity::dense(eig_fk_pos.rows(), eig_fk_pos.cols()));
    ca::SX ca_fk_rot =
        ca::SX(ca::Sparsity::dense(eig_fk_rot.rows(), eig_fk_rot.cols()));
    // Copy Eigen::Matrix into casadi::SX
    pinocchio::casadi::copy(eig_fk_pos, ca_fk_tr);
    pinocchio::casadi::copy(eig_fk_rot, ca_fk_rot);
    // Generate function
    FK_ = ca::Function("forward_kinematics", {ca_q}, {ca_fk_tr, ca_fk_rot},
                       {"q"}, {"ee_pos", "ee_rot"});

    // Extract Eigen::Matrix results
    Eigen::Matrix<ca::SX, 3, 1> eig_fk_pos_j7 =
        data.oMf.at(id_Fj7_).translation();
    Eigen::Matrix<ca::SX, 3, 3> eig_fk_rot_j7 = data.oMf.at(id_Fj7_).rotation();
    // Create associated casadi::SX variables
    ca::SX ca_fk_j7_tr =
        ca::SX(ca::Sparsity::dense(eig_fk_pos_j7.rows(), eig_fk_pos_j7.cols()));
    ca::SX ca_fk_j7_rot =
        ca::SX(ca::Sparsity::dense(eig_fk_rot_j7.rows(), eig_fk_rot_j7.cols()));
    // Copy Eigen::Matrix into casadi::SX
    pinocchio::casadi::copy(eig_fk_pos_j7, ca_fk_j7_tr);
    pinocchio::casadi::copy(eig_fk_rot_j7, ca_fk_j7_rot);

    // Generate function
    FK_J7_ =
        ca::Function("forward_kinematics_j7", {ca_q},
                     {ca_fk_j7_tr, ca_fk_j7_rot}, {"q"}, {"j7_pos", "j7_rot"});

    // Extract Eigen::Matrix results
    Eigen::Matrix<ca::SX, 3, 1> eig_fk_pos_j8 =
        data.oMf.at(id_Fj8_).translation();
    Eigen::Matrix<ca::SX, 3, 3> eig_fk_rot_j8 = data.oMf.at(id_Fj8_).rotation();
    // Create associated casadi::SX variables
    ca::SX ca_fk_j8_tr =
        ca::SX(ca::Sparsity::dense(eig_fk_pos_j8.rows(), eig_fk_pos_j8.cols()));
    ca::SX ca_fk_j8_rot =
        ca::SX(ca::Sparsity::dense(eig_fk_rot_j8.rows(), eig_fk_rot_j8.cols()));
    // Copy Eigen::Matrix into casadi::SX
    pinocchio::casadi::copy(eig_fk_pos_j8, ca_fk_j8_tr);
    pinocchio::casadi::copy(eig_fk_rot_j8, ca_fk_j8_rot);
    // Generate function
    FK_J8_ =
        ca::Function("forward_kinematics_j8", {ca_q},
                     {ca_fk_j8_tr, ca_fk_j8_rot}, {"q"}, {"j8_pos", "j8_rot"});

    generate_ca_log3();
    generate_ca_log6();
    generate_ca_EE_error();
    generate_ca_RCM_error();
    generate_nlsolver();

    // Getting Joints
    // std::cout << "Joints found by size: " << model_.joints.size() << "\n";
    std::cout << "Joints found by njoints: " << mdl_.njoints << std::endl;
    std::cout << "Joints lower limits: " << mdl_.lowerPositionLimit.transpose()
              << std::endl;
    std::cout << "Joints upper limits: " << mdl_.upperPositionLimit.transpose()
              << std::endl;
  }

  NLO_CASADI::~NLO_CASADI() {}

  int NLO_CASADI::IkSolve(const VectorXd q_init, const pin::SE3 &x_d,
                          VectorXd &q_sol)
  {
    auto start_cb_time = std::chrono::high_resolution_clock::now();

    q_sol = q_init;

    ca::SX ca_pd = eig_to_casDM(x_d.translation());
    ca::SX ca_Rd = eigmat_to_casDM(x_d.rotation());
    ca::SX ca_qin = eigmat_to_casDM(q_init);
    ca::SX ca_pnostril = eig_to_casDM(x_B_Fnostril_.translation());
    ca::DMDict arg_nlopt;

    ca::SX ca_qmin = eig_to_casDM(q_min_ - q_init);
    ca::SX ca_qmax = eig_to_casDM(q_max_ - q_init);

    auto x_act = FK_(ca_qin);
    ca::SX p_act = x_act[0];
    ca::SX R_act = x_act[1];

    auto x_act_j7 = FK_J7_(ca_qin);
    ca::SX p_act_j7 = x_act_j7[0];
    ca::SX R_act_j7 = x_act_j7[1];

    auto x_act_j8 = FK_J8_(ca_qin);
    ca::SX p_act_j8 = x_act_j8[0];
    ca::SX R_act_j8 = x_act_j8[1];

    ca::DM par = ca::DM::zeros(29);
    ca::DM mu({mu0_, mu1_, mu2_, mu3_});

    par(ca::Slice(0, 9)) = ca_qin;
    par(ca::Slice(9, 12)) = ca_pd;
    par(ca::Slice(12, 21)) = ca::DM::reshape(ca_Rd, 9, 1);
    par(ca::Slice(21, 24)) = ca_pnostril;
    par(ca::Slice(24, 28)) = mu;
    par(28) = ca::DM(rcm_error_max_);

    if (constrained_control_)
    {
      //? Use constrained control
      if (rcm_is_cost_)
      {
        //?  Using RCM error as cost
        // std::cout << "Using RCM as cost function" << std::endl;
        arg_nlopt = {{"x0", ca::SX::zeros((9, 1))},
                     {"p", par},
                     {"lbx", ca_qmin},
                     {"ubx", ca_qmax}};
      }
      else
      {
        //? Using RCM as constraint
        // std::cout << "Using RCM as constrained function" << std::endl;

        arg_nlopt = {{"x0", ca::SX::zeros((9, 1))},
                     {"p", par},
                     {"lbx", ca_qmin},
                     {"ubx", ca_qmax},
                     {"lbg", ca::SX(0.0)},
                     {"ubg", ca::SX(rcm_error_max_)}};
      }
    }
    else
    {
      //? Use non-constrained control
      // std::cout << "No constrained motion" << std::endl;

      arg_nlopt = {{"x0", ca::SX::zeros((9, 1))},
                   {"p", par},
                   {"lbx", ca_qmin},
                   {"ubx", ca_qmax}};
    }

    auto start_solver_time = std::chrono::high_resolution_clock::now();

    ca::DMDict res_nlopt = solver_(arg_nlopt);

    auto stop_solver_time = std::chrono::high_resolution_clock::now();
    auto duration_solver =
        std::chrono::duration_cast<std::chrono::microseconds>(
            stop_solver_time - start_solver_time);

    ca::DM q_opt = res_nlopt["x"];
    ca::DM f_opt = res_nlopt["f"];

    double opt_cost = static_cast<double>(f_opt);

    // Verify costs
    std::vector<ca::DM> xcost =
        ca_perr2_(std::vector<ca::DM>{ca::DM(ca_qin) + q_opt, ca_pd, ca_Rd});
    ca::DM c0 = xcost[0]; // P cost
    ca::DM c1 = xcost[1]; // R cost
    ca::DM c2 = ca::DM::mtimes(q_opt.T(), q_opt);
    ca::DM c3 = ca_ercm_(std::vector<ca::DM>{
        ca::DM(ca_qin) + q_opt, eig_to_casDM(x_B_Fnostril_.translation())})[0];

    double p_err = static_cast<double>(c0);
    double r_err = static_cast<double>(c1);
    double pose_err = sqrt(p_err * p_err + r_err * r_err);

    if (pose_err > max_error_)
      return 1;

    q_sol = casDM_to_eig(ca::DM(ca_qin) + q_opt);

    auto stop_cb_time = std::chrono::high_resolution_clock::now();
    auto duration_cb = std::chrono::duration_cast<std::chrono::microseconds>(
        stop_cb_time - start_cb_time);
    // ROS_INFO_STREAM("Time IK [us]: " << duration_cb.count() << " usec");
    return 0;
  }

  void NLO_CASADI::generate_nlsolver()
  {
    // Optimization Variable
    ca::SX q_delta = ca::SX::sym("qdelta", 9, 1);

    // Fixed parameters
    ca::SX par = ca::SX::sym("par", 29, 1);

    ca::SX q_in = par(ca::Slice(0, 9));
    ca::SX ca_pd = par(ca::Slice(9, 12));
    ca::SX ca_Rd = ca::SX::reshape(par(ca::Slice(12, 21)), 3, 3);
    ca::SX p_nostril = par(ca::Slice(21, 24));
    ca::SX mu = par(ca::Slice(24, 28));
    ca::SX eps = par(28);

    // Optimization variable boundaries
    ca::SX ca_qmin = ca::SX::sym("qmin", 9, 1);
    ca::SX ca_qmax = ca::SX::sym("qmax", 9, 1);

    // Optimizer options
    ca::Dict opts;
    opts["verbose"] = false; // false
    opts["print_time"] = 0;
    opts["ipopt.linear_solver"] = nlp_linear_solver_;    //
    opts["ipopt.print_level"] = verb_level_;             // 0
    opts["ipopt.print_timing_statistics"] = time_stats_; //"no"
    // opts["ipopt.hessian_approximation"] = "limited-memory"; //"exact"
    opts["ipopt.warm_start_init_point"] = "yes"; //"no"
    opts["ipopt.max_wall_time"] = max_time_;     //"no"
    opts["ipopt.max_iter"] = max_iter_;          //"no"
    opts["ipopt.tol"] = max_error_;              //"no"

    // Objective Function
    ca::SX obj;

    // Inequality constraints
    ca::SX cineq;

    // Nonlinear problem declaration
    ca::SXDict nlp;

    // Nonlinear problem arguments definition
    // ca::DMDict arg_nlopt;

    if (constrained_control_)
    {
      //? Use constrained control
      if (rcm_is_cost_)
      {
        //?  Using RCM error as cost
        ROS_INFO_STREAM("Using RCM as cost function");
        std::vector<ca::SX> xcost =
            ca_perr2_(std::vector<ca::SX>{q_in + q_delta, ca_pd, ca_Rd});
        ca::SX cost0 = xcost[0];
        ca::SX cost1 = xcost[1];
        ca::SX cost2 = ca::SX::mtimes(q_delta.T(), q_delta);
        ca::SX cost3 =
            ca_ercm_(std::vector<ca::SX>{q_in + q_delta, p_nostril})[0];
        obj = mu(0) * cost0 + mu(1) * cost1 + mu(2) * cost2 + mu(3) * cost3;

        nlp = {{"x", q_delta}, {"p", par}, {"f", obj}};
      }
      else
      {
        //? Using RCM as constraint
        ROS_INFO_STREAM("Using RCM as constrained function");
        std::vector<ca::SX> xcost =
            ca_perr2_(std::vector<ca::SX>{q_in + q_delta, ca_pd, ca_Rd});
        ca::SX cost0 = xcost[0];
        ca::SX cost1 = xcost[1];
        ca::SX cost2 = ca::SX::mtimes(q_delta.T(), q_delta);
        obj = mu(0) * cost0 + mu(1) * cost1 + mu(2) * cost2;

        cineq = ca_ercm_(std::vector<ca::SX>{q_in + q_delta, p_nostril})[0];

        nlp = {{"x", q_delta}, {"p", par}, {"f", obj}, {"g", cineq}};
      }
    }
    else
    {
      //? Use non-constrained control
      ROS_INFO_STREAM("No constrained motion");
      std::vector<ca::SX> xcost =
          ca_perr2_(std::vector<ca::SX>{q_in + q_delta, ca_pd, ca_Rd});
      ca::SX cost0 = xcost[0];
      ca::SX cost1 = xcost[1];
      ca::SX cost2 = ca::SX::mtimes(q_delta.T(), q_delta);
      obj = mu(0) * cost0 + mu(1) * cost1 + mu(2) * cost2;

      nlp = {{"x", q_delta}, {"p", par}, {"f", obj}};
    }

    solver_ = nlpsol("nlpsol", "ipopt", nlp, opts);
  }

  void NLO_CASADI::generate_ca_EE_error()
  {
    ROS_INFO_STREAM("Generating CASADI EE error function");
    //* Generate perr2 = perr.T*perr Casadi function
    // Inputs
    ca::SX R_des = ca::SX::sym("R_act", 3, 3);
    ca::SX p_des = ca::SX::sym("p_act", 3, 1);
    ca::SX q_init = ca::SX::sym("q_init", 9, 1);

    ca::SX p_act = FK_(q_init)[0];
    ca::SX R_act = FK_(q_init)[1];

    ca::SX p_e =
        p_des - ca::SX::mtimes(R_des, ca::SX::mtimes(R_act.T(), p_act));
    ca::SX R_e = ca::SX::mtimes(R_des, R_act.T());

    ca::SX err;

    ca::SX p_error;
    ca::SX R_error;
    ca::SX p_error2;
    ca::SX R_error2;

    if (err_method_ == "log6")
    {
      //? Using log6
      std::vector<ca::SX> err_tmp = ca_log6_(std::vector<ca::SX>{p_e, R_e});
      p_error2 = ca::SX::mtimes(err_tmp[0].T(), err_tmp[0]);
      R_error2 = ca::SX(0.0);
    }
    else if (err_method_ == "log3")
    {
      //? Using log3
      p_error = p_des - p_act;
      p_error2 = ca::SX::mtimes(p_error.T(), p_error);
      R_error = ca_log3_(std::vector<ca::SX>{R_e})[0];
      R_error2 = ca::SX::mtimes(R_error.T(), R_error);
    }
    else
    {
      //? Using only p error
      p_error = p_des - p_act;
      p_error2 = ca::SX::mtimes(p_error.T(), p_error);
      R_error2 = ca::SX(0.0);
    }

    ca_perr2_ =
        ca::Function("p_err2", {q_init, p_des, R_des}, {p_error2, R_error2},
                     {"q_init", "pd", "Rd"}, {"p_err2", "R_err2"});
  }

  void NLO_CASADI::generate_ca_RCM_error()
  {
    // Generating RCM error CASADI function
    ROS_INFO_STREAM("Generating CASADI RCM error function");
    ca::SX qvec = ca::SX::sym("qvec", 9, 1);
    ca::SX p_nostril = ca::SX::sym("p_nostril", 3, 1);

    ca::SX p_j7 = FK_J7_(qvec)[0];
    ca::SX p_j8 = FK_J8_(qvec)[0];
    ca::SX p_ee = FK_(qvec)[0];
    ca::SX R_j7 = FK_J7_(qvec)[1];
    ca::SX R_j8 = FK_J8_(qvec)[1];
    ca::SX R_ee = FK_(qvec)[1];

    ca::SX ca_pd = p_j8 - p_j7;
    ca::SX ca_pr = p_nostril - p_j7;

    ca::SX e_rcm =
        ca_pr - ca::SX::mtimes((ca::SX::mtimes(ca_pr.T(), ca_pd)), ca_pd) /
                    (ca::SX::mtimes(ca_pd.T(), ca_pd));
    ca::SX e_rcm2 = ca::SX::mtimes(e_rcm.T(), e_rcm);

    ca_ercm_ = ca::Function("e_rcm", {qvec, p_nostril}, {e_rcm2},
                            {"q", "p_nostril"}, {"e_rcm"});
  }

  // log3
  void NLO_CASADI::generate_ca_log3()
  {
    ROS_INFO_STREAM("Generating log3");
    ca::SX tolerance = 1e-8;

    ca::SX R = ca::SX::sym("R", 3, 3);
    ca::SX omega = ca::SX::sym("omega", 3, 1);
    ca::SX val = (ca::SX::trace(R) - ca::SX(1)) / ca::SX(2);
    val = ca::SX::if_else(val > ca::SX(1), ca::SX(1),
                          ca::SX::if_else(val < ca::SX(-1), ca::SX(-1), val));
    ca::SX theta = ca::SX::acos(val);
    ca::SX stheta = ca::SX::sin(theta);
    ca::SX tr = ca::SX::if_else(theta < tolerance, ca::SX::zeros((3, 3)),
                                (R - R.T()) * theta / (ca::SX(2) * stheta));
    omega = ca::SX::inv_skew(tr);
    ca_log3_ =
        ca::Function("ca_log3", {R}, {omega, theta}, {"R"}, {"w", "theta"});
  }

  // log6
  void NLO_CASADI::generate_ca_log6()
  {
    ROS_INFO_STREAM("Generating log6");
    ca::SX tolerance = 1e-8;
    ca::SX tolerance2 = 1e-16;

    ca::SX tau = ca::SX::sym("tau", 6, 1);
    ca::SX R = ca::SX::sym("R", 3, 3);
    ca::SX p = ca::SX::sym("p", 3, 1);

    std::vector<ca::SX> log_res = ca_log3_(R);
    ca::SX omega = log_res[0];
    ca::SX theta = log_res[1];

    ca::SX stheta = ca::SX::sin(theta);
    ca::SX ctheta = ca::SX::cos(theta);

    ca::SX A_inv = ca::SX::if_else(
        ca::SX::mtimes(p.T(), p) < tolerance2, ca::SX::zeros((3, 3)),
        ca::SX::if_else(
            theta < tolerance, ca::SX::eye(3),
            ca::SX::eye(3) - ca::SX::skew(omega) / ca::SX(2) +
                (ca::SX(2) * stheta - theta * (ca::SX(1) + ctheta)) *
                    (ca::SX::mtimes(ca::SX::skew(omega), ca::SX::skew(omega))) /
                    (ca::SX(2) * (ca::SX::pow(theta, 2)) * stheta)));

    ca::SX v = ca::SX::mtimes(A_inv, p);

    // tau = ca::SX::vertcat(v, omega);
    tau(ca::Slice(0, 3)) = v;
    tau(ca::Slice(3, 6)) = omega;

    ca_log6_ = ca::Function("ca_log6", {p, R}, {tau}, {"p", "R"}, {"tau"});
  }

  //* Casadi-Eigen conversion functions

  ca::DM NLO_CASADI::eig_to_casDM(const VectorXd &eig)
  {
    auto dm = casadi::DM(casadi::Sparsity::dense(eig.size()));
    for (int i = 0; i < eig.size(); i++)
    {
      dm(i) = eig(i);
    }
    return dm;
  }

  ca::DM NLO_CASADI::eigmat_to_casDM(const MatrixXd &eig)
  {
    auto dm = casadi::DM(casadi::Sparsity::dense(eig.rows(), eig.cols()));
    for (int i = 0; i < eig.rows(); i++)
    {
      for (int j = 0; j < eig.cols(); j++)
      {
        dm(i, j) = eig(i, j);
      }
    }
    return dm;
  }

  MatrixXd NLO_CASADI::casDM_to_eig(const casadi::DM &cas)
  {
    auto vector_x = static_cast<std::vector<double>>(cas);

    MatrixXd eig = MatrixXd::Zero(cas.size1(), cas.size2());

    for (int i = 0; i < cas.size1(); i++)
    {
      for (int j = 0; j < cas.size2(); j++)
      {
        eig(i, j) = vector_x[i + j * cas.size2()];
      }
    }
    return eig;
  }

} // namespace codcs_ik

#endif
