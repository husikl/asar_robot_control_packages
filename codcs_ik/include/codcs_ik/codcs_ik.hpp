#ifndef CODCS_IK_HPP
#define CODCS_IK_HPP

#define PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR

#include <cmath>
#include <memory>
#include <string>
#include <thread>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

// Eigen conversions
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

// ROS related
#include <ros/ros.h>

// Eigen
#include <Eigen/Dense>

// Nonlinear Optimization IK
#include <nlo/nlo_ik.hpp>

// Task-prioritized IK
#include <codcs_ik/ik_solver.hpp>
#include <tp/tp_ik.hpp>

// YAML parser library
#include "yaml-cpp/yaml.h"

// Casadi
#include <casadi/casadi.hpp>

// SolverOptions
#include <codcs_ik/solver_options.hpp>

using namespace Eigen;
namespace pin = pinocchio;

namespace codcs_ik
{

  class CODCS_IK : public IkSolver
  {
  public:
    CODCS_IK(const std::string &_urdf_file, const std::string &_base_link,
             const std::string &_tip_link, const std::string &_ik_solver,
             SolverOptions solver_opts, double _max_time = 10e-3,
             double _max_error = 1e-5, int _max_iter = 1e2, double _dt = 1.0);

    ~CODCS_IK();

    int solveIk(const VectorXd q_init, const pin::SE3 &x_des, VectorXd &q_out);
    int solveIk(const pin::SE3 x_des);
    int solveFk(const VectorXd q_act, pin::SE3 &x_B_Fee);

    int IkSolve(const VectorXd q_init, const pin::SE3 &x_des, VectorXd &q_out);

    void printStatistics();

    int ReadDescriptionFile(std::string file_path);

    int get_n_joints() { return n_joints_; }
    std::string get_solver_name() { return solver_name_; }

  private:
    //   Pinocchio variables
    pin::Model model_;
    pin::Data mdl_data_;
    pin::FrameIndex ee_id_;

    std::string urdf_file_;
    std::string base_link_;
    std::string ee_link_;
    // std::string urdf_param_;
    std::string ik_solver_;

    std::string solver_name_;
    int n_joints_;
    int max_iter_;
    int succ_sol_tp_;
    int succ_sol_nlo_;
    double max_time_;
    double max_error_;
    double dt_;

    bool initialized_;

    VectorXd q_ul_;
    VectorXd q_ll_;
    // VectorXd q_sol_;

    std::vector<VectorXd> q_solutions_;
    std::vector<double> errors_;

    bool initialize();
    bool initialize_codcs();
    bool printModelInfo();

    template <typename T1, typename T2>
    bool runSolver(T1 &solver, T2 &other_solver, const VectorXd q_init,
                   const pin::SE3 &x_des, int id);
    bool runTPIK(const VectorXd q_init, const pin::SE3 &x_Fee_d);
    bool runNLOIK(const VectorXd q_init, const pin::SE3 &x_Fee_d);

    std::unique_ptr<TP_IkSolver<TP_PINOCCHIO>> tp_solver_;
    // std::unique_ptr<NLO_IkSolver<NLO_NLOPT>> nlo_solver_;
    std::unique_ptr<NLO_IkSolver<NLO_CASADI>> nlo_solver_;

    std::thread task1_, task2_;
    std::mutex mtx_;

    std::chrono::time_point<std::chrono::high_resolution_clock>
        start_iksolve_time_;

    SolverOptions solver_opts_;
  };

  inline bool CODCS_IK::runTPIK(const VectorXd q_init, const pin::SE3 &x_Fee_d)
  {
    return runSolver(*tp_solver_.get(), *nlo_solver_.get(), q_init, x_Fee_d, 1);
  }

  inline bool CODCS_IK::runNLOIK(const VectorXd q_init, const pin::SE3 &x_Fee_d)
  {
    return runSolver(*nlo_solver_.get(), *tp_solver_.get(), q_init, x_Fee_d, 2);
  }

} // namespace codcs_ik

#endif
