
#include <codcs_ik/codcs_ik.hpp>
#include <codcs_ik/ik_solver.hpp>
#include <csignal>
#include <string>

// ROS
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

// Orocos KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/velocityprofile_trap.hpp>

// Trac_IK
#include <trac_ik/trac_ik.hpp>

// KDL Parser
#include <kdl_parser/kdl_parser.hpp>

using namespace codcs_ik;
namespace pin = pinocchio;

class BenchmarkKDL : public IkSolver
{
  public:
  BenchmarkKDL(const std::string &_urdf_file, const std::string &_base_link,
               const std::string &_ee_link, const std::string &_ik_solver,
               double _max_time, double _max_error, int _max_iter, double _dt)
      : initialized_(false), max_error_(_max_error), max_iter_(_max_iter),
        urdf_file_(_urdf_file), base_link_(_base_link), ee_link_(_ee_link), dt_(_dt),
        solver_name_("kdl")
  {
    initialize_kdl();
  }

  bool initialize_kdl()
  {
    std::cout << "Initializing KDL with Max. Error: " << max_error_
              << " Max. It.:" << max_iter_ << " Delta T:" << dt_ << std::endl;

    double maxtime = 0.01;

    // Parsing URDF
    if (!kdl_parser::treeFromFile(urdf_file_, kdl_tree_))
    {
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }
    bool exit_value = kdl_tree_.getChain(base_link_, ee_link_, kdl_chain_);
    // Resize variables
    n_joints_ = kdl_chain_.getNrOfJoints();
    qtmp_.resize(kdl_chain_.getNrOfJoints());
    nominal_.resize(kdl_chain_.getNrOfJoints());
    ll_.resize(kdl_chain_.getNrOfJoints());
    ul_.resize(kdl_chain_.getNrOfJoints());

    //   Load the urdf model
    pin::Model model_;
    pin::urdf::buildModel(urdf_file_, model_);

    // Getting Joints Limits
    q_ul_ = model_.upperPositionLimit;
    q_ll_ = model_.lowerPositionLimit;

    // Storing Joint limits
    for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
      ll_.data(i) = q_ll_[i];
      ul_.data(i) = q_ul_[i];
    }

    assert(kdl_chain_.getNrOfJoints() == ll_.data.size());
    assert(kdl_chain_.getNrOfJoints() == ul_.data.size());

    // KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);  // Forward kin.
    // solver KDL::ChainIkSolverVel_pinv      vik_solver(kdl_chain_); //
    // PseudoInverse vel solver KDL::ChainIkSolverPos_NR_JL
    // kdl_solver(kdl_chain_, ll_, ul_, fk_solver, vik_solver, 1,
    //  eps); //

    kdl_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    kdl_vik_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    kdl_ik_solver_.reset(
        new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, ll_, ul_, *kdl_fk_solver_,
                                        *kdl_vik_solver_, max_iter_, max_error_));

    // Initialize nominal vector
    for (uint j = 0; j < nominal_.data.size(); j++)
    {
      nominal_(j) = (ll_(j) + ul_(j)) / 2.0;
    }
    std::cout << "KDL initialized" << std::endl;
    initialized_ = true;

    return true;
  }

  // * KDL solver
  int IkSolve(const VectorXd q_init, const pin::SE3 &x_Fee_d, VectorXd &q_out)
  {
    bool          success = false;
    int           rc;
    KDL::JntArray qd(n_joints_);
    KDL::Frame    ee;

    Affine3d Tdes;
    Tdes.linear() = x_Fee_d.rotation();
    Tdes.translation() = x_Fee_d.translation();

    tf::transformEigenToKDL(Tdes, ee);

    rc = kdl_ik_solver_->CartToJnt(nominal_, ee, qd);

    if (rc >= 0)
    {
      // std::cout << "Solution found" << std::endl;
      q_out = VectorXd::Map(&qd.data[0], qd.data.size());
      // std::cout << "Solution found: " << qd.data * (180 / M_PI) << std::endl;
      // std::cout << "Time IK [us]: " << duration_cb.count() << " usec";

      success = true;
      return 0;
      // ROS_WARN_STREAM("IK solution found: " <<
      // des_joint_pos_.transpose());
    }
    else
    {
      // ROS_WARN_STREAM("No IK solution found");
      success = false;
      return 1;
    }
    // return success;
  }

  int         get_n_joints() { return n_joints_; }
  std::string get_solver_name() { return solver_name_; }

  private:
  // Temporary variables for KDL
  KDL::Tree  kdl_tree_;
  KDL::Chain kdl_chain_;

  KDL::JntArray nominal_;
  KDL::JntArray qtmp_;
  KDL::JntArray ll_, ul_;
  KDL::Frame    xtmp_;
  KDL::Jacobian Jtmp_;

  KDL::Twist    xdot_temp_;
  KDL::JntArray qdot_tmp_;

  // KDL
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv>      kdl_vik_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL>     kdl_ik_solver_;

  std::string urdf_file_;

  std::string base_link_;
  std::string ee_link_;
  VectorXd    q_ul_;
  VectorXd    q_ll_;

  int         n_joints_;
  std::string solver_name_;
  int         max_iter_;
  double      max_error_;

  bool   initialized_;
  double dt_;
};

class BenchmarkTRACIK : public IkSolver
{
  public:
  BenchmarkTRACIK(const std::string &_urdf_file, const std::string &_base_link,
                  const std::string &_ee_link, const std::string &_ik_solver,
                  double _max_time, double _max_error, int _max_iter, double _dt)
      : initialized_(false), max_error_(_max_error), max_time_(_max_time),
        urdf_file_(_urdf_file), base_link_(_base_link), ee_link_(_ee_link), dt_(_dt),
        solver_name_("trac_ik")
  {
    initialize_tracik();
  }

  bool initialize_tracik()
  {
    std::cout << "Initializing TRACIK with Max. Error: " << max_error_
              << " Max. Time:" << max_time_ << " Delta T:" << dt_ << std::endl;

    // Parsing URDF
    if (!kdl_parser::treeFromFile(urdf_file_, kdl_tree_))
    {
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }
    bool exit_value = kdl_tree_.getChain(base_link_, ee_link_, kdl_chain_);

    // Resize variables
    n_joints_ = kdl_chain_.getNrOfJoints();
    qtmp_.resize(kdl_chain_.getNrOfJoints());
    nominal_.resize(kdl_chain_.getNrOfJoints());
    ll_.resize(kdl_chain_.getNrOfJoints());
    ul_.resize(kdl_chain_.getNrOfJoints());

    //   Load the urdf model
    pin::Model model_;
    pin::urdf::buildModel(urdf_file_, model_);

    // Getting Joints Limits
    q_ul_ = model_.upperPositionLimit;
    q_ll_ = model_.lowerPositionLimit;

    // Storing Joint limits
    for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
      ll_.data(i) = q_ll_[i];
      ul_.data(i) = q_ul_[i];
    }

    assert(kdl_chain_.getNrOfJoints() == ll_.data.size());
    assert(kdl_chain_.getNrOfJoints() == ul_.data.size());

    // Initialize Trac-IK
    tracik_solver_.reset(new TRAC_IK::TRAC_IK(kdl_chain_, ll_, ul_, max_time_,
                                              max_error_, TRAC_IK::Speed));
    // tracik_solver_.reset(new TRAC_IK::TRAC_IK("base_link", "J6", urdf_param_,
    //                                           0.001, eps,
    //                                           TRAC_IK::Distance));

    // Initialize nominal vector
    for (uint j = 0; j < nominal_.data.size(); j++)
    {
      nominal_(j) = (ll_(j) + ul_(j)) / 2.0;
    }
    std::cout << "TRACIK initialized" << std::endl;

    return true;
  }

  // * TRAC-IK solver
  int IkSolve(const VectorXd q_init, const pin::SE3 &x_Fee_d, VectorXd &q_out)
  {
    bool          success = false;
    int           rc;
    KDL::JntArray qd(n_joints_);
    KDL::Frame    ee;

    Affine3d Tdes;
    Tdes.linear() = x_Fee_d.rotation();
    Tdes.translation() = x_Fee_d.translation();

    tf::transformEigenToKDL(Tdes, ee);

    // std::cout << "Size q_init: " << q_init.size() << std::endl;
    // std::cout << "Size nominal_: " << nominal_.data.size() << std::endl;
    // for (int i = 0; i < n_joints_; i++)
    // {
    //   nominal_.data(i) = q_init[i];
    // }

    rc = tracik_solver_->CartToJnt(nominal_, ee, qd);

    if (rc >= 0)
    {
      // VectorXd des_joints_tmp = VectorXd::Zero(6);
      // des_joints_tmp          = qd.data * RAD2DEG;
      // if ((des_joints_tmp - des_joint_pos_).cwiseAbs().maxCoeff() <= 1.4) {
      //   des_joint_pos_ = qd.data * RAD2DEG;
      //   return 0;
      // } else {
      //   ROS_WARN_STREAM("Solution Found but High Speed des  ");
      //   return -1;
      // }
      // std::cout << "Solution found" << std::endl;
      q_out = VectorXd::Map(&qd.data[0], qd.data.size());
      // std::cout << "Solution found" << qd.data * (180 / M_PI) << std::endl;

      // std::cout << "Time IK [us]: " << duration_cb.count() << " usec";

      success = true;
      return 0;
      // ROS_WARN_STREAM("IK solution found: " <<
      // des_joint_pos_.transpose());
    }
    else
    {
      // ROS_WARN_STREAM("No IK solution found");
      success = false;
      return 1;
    }
    // return success;
  }

  int         get_n_joints() { return n_joints_; }
  std::string get_solver_name() { return solver_name_; }

  private:
  std::unique_ptr<TRAC_IK::TRAC_IK> tracik_solver_;

  std::string urdf_file_;

  std::string base_link_;
  std::string ee_link_;
  VectorXd    q_ul_;
  VectorXd    q_ll_;

  int         n_joints_;
  std::string solver_name_;
  double      max_error_;
  double      max_time_;

  // Temporary variables for KDL
  KDL::Tree  kdl_tree_;
  KDL::Chain kdl_chain_;

  KDL::JntArray nominal_;
  KDL::JntArray qtmp_;
  KDL::JntArray ll_, ul_;

  bool   initialized_;
  double dt_;
};

double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

// template <typename T>
void solveIkFromRandomList(std::string urdf_file_, int num_samples,
                           IkSolver *ik_solver, std::string ee_link_,
                           bool print_all = false)
{
  double total_time = 0;
  double total_time_it = 0;
  uint   n_success = 0;

  //   Pinocchio variables
  pin::Model      model_;
  pin::Data       mdl_data_;
  pin::FrameIndex ee_id_;

  VectorXd q_ul_;
  VectorXd q_ll_;
  VectorXd q_sol_;

  // std::vector<VectorXd> q_solutions_;
  std::vector<double> errors_;
  int                 n_joints_;

  //   Load the urdf model
  pin::urdf::buildModel(urdf_file_, model_);
  ee_id_ = model_.getFrameId(ee_link_);

  n_joints_ = model_.nq;
  q_sol_.resize(n_joints_);
  // q_solutions_.clear();
  errors_.clear();

  // Getting Joints Limits
  q_ul_ = model_.upperPositionLimit;
  q_ll_ = model_.lowerPositionLimit;

  std::cout << "Solving IK with " << ik_solver->get_solver_name() << " for "
            << num_samples << " random configurations for link " << ee_link_
            << std::endl;

  // Create desired number of valid, random joint configurations
  std::vector<VectorXd> JointList;
  VectorXd              q(ik_solver->get_n_joints());

  for (uint i = 0; i < num_samples; i++)
  {
    for (uint j = 0; j < q_ll_.size(); j++)
    {
      q(j) = fRand(q_ll_(j), q_ul_(j));
    }
    JointList.push_back(q);
  }

  pin::Data mdl_data(model_);
  VectorXd  q_init = pin::neutral(model_);
  for (uint i = 0; i < q_init.size(); i++)
  {
    q_init[i] = (q_ul_[i] + q_ll_[i]) / 2.0;
  }

  VectorXd q_sol = pin::neutral(model_);

  auto start_cb_time = std::chrono::high_resolution_clock::now();
  auto start_it_time = std::chrono::high_resolution_clock::now();
  auto stop_it_time = std::chrono::high_resolution_clock::now();

  for (uint i = 0; i < num_samples; i++)
  {
    if (print_all)
      std::cout << "\n[Prob " << i << "] Solving for: " << JointList[i].transpose()
                << std::endl;
    int res = 1;
    pin::forwardKinematics(model_, mdl_data, JointList[i]);
    pin::updateFramePlacements(model_, mdl_data);
    pin::SE3 x_des = mdl_data.oMf[ee_id_];

    start_it_time = std::chrono::high_resolution_clock::now();

    // Call IK Solver
    res = ik_solver->IkSolve(q_init, x_des, q_sol);

    stop_it_time = std::chrono::high_resolution_clock::now();

    if (res && print_all)
      ROS_WARN("Solution not found");

    auto duration_it = std::chrono::duration_cast<std::chrono::microseconds>(
        stop_it_time - start_it_time);
    if (print_all)
    {
      std::cout << "Time: " << duration_it.count() << std::endl;
      if (!res)
      {
        std::cout << "Solution: " << q_sol.transpose() << std::endl;
      }
    }
    total_time_it += duration_it.count();

    if (!res)
      n_success++;
  }

  auto stop_cb_time = std::chrono::high_resolution_clock::now();
  auto duration_cb = std::chrono::duration_cast<std::chrono::microseconds>(
      stop_cb_time - start_cb_time);

  total_time = duration_cb.count();
  // std::cout << "Time IK [us]: " << duration_cb.count() << " usec" <<
  // std::endl; std::cout << "Solution found: " << q_sol.transpose() <<
  // std::endl; std::cout << "Solution found (deg): " << q_sol.transpose() *
  // (180 / M_PI) << std::endl;

  std::cout << "------------------------------" << std::endl;
  std::cout << "Summary:" << std::endl;

  std::cout << ik_solver->get_solver_name() << " found " << n_success
            << " solutions (" << 100.0 * n_success / num_samples
            << "\%) with a total average of " << total_time / num_samples
            << " usec/config."
            << " and solving average of " << total_time_it / num_samples
            << " usec/config." << std::endl;
  if (ik_solver->get_solver_name() == "codcs")
  {
    ik_solver->printStatistics();
  }
  std::cout << "------------------------------" << std::endl;

  return;
}

bool kill_process = false;
void SigIntHandler(int signal)
{
  kill_process = true;
  ROS_INFO_STREAM("SHUTDOWN SIGNAL RECEIVED");
}

int main(int argc, char **argv)
{
  ROS_INFO("CODCS Benchmarking vs TRAC-IK and KDL");
  ros::init(argc, argv, "codcs_ik_test");
  ros::NodeHandle nh;
  std::signal(SIGINT, SigIntHandler);

  // ROS parameters
  std::string ik_solver;
  int         max_iter;
  int         n_random_config;
  double      max_time;
  double      max_error;
  double      dt;
  int         print_all;

  if (!nh.getParam("ik_solver", ik_solver))
  {
    ik_solver = "tracik";
  }
  if (!nh.getParam("max_iter", max_iter))
  {
    max_iter = 10000;
  }
  if (!nh.getParam("max_time", max_time))
  {
    max_time = 5e-3;
  }
  if (!nh.getParam("max_error", max_error))
  {
    max_error = 1e-5;
  }
  if (!nh.getParam("delta_integration", dt))
  {
    dt = 1.0;
  }
  if (!nh.getParam("n_random_config", n_random_config))
  {
    n_random_config = 100;
  }
  if (!nh.getParam("print_all", print_all))
  {
    print_all = 0;
  }
  bool rcm_is_cost;
  if (!nh.getParam("rcm_is_cost", rcm_is_cost))
  {
    rcm_is_cost = true;
  }
  int ik_method;
  if (!nh.getParam("ik_method", ik_method))
  {
    ik_method = 1;
  }
  std::string nlp_linear_solver;
  if (!nh.getParam("nlp_linear_solver", nlp_linear_solver))
  {
    nlp_linear_solver = "ma57";
  }
  std::string error_method;
  if (!nh.getParam("error_method", error_method))
  {
    error_method = "log6";
  }
  double mu0;
  if (!nh.getParam("mu0", mu0))
  {
    mu0 = 1.0;
  }
  double mu1;
  if (!nh.getParam("mu1", mu1))
  {
    mu1 = 0.005;
  }
  double mu2;
  if (!nh.getParam("mu2", mu2))
  {
    mu2 = 0.001;
  }
  double mu3;
  if (!nh.getParam("mu3", mu3))
  {
    mu3 = 1000.0;
  }
  double rcm_error_max;
  if (!nh.getParam("rcm_error_max", rcm_error_max))
  {
    rcm_error_max = 2.5e-5;
  }
  bool constrained_control;
  if (!nh.getParam("constrained_control", constrained_control))
  {
    constrained_control = false;
  }
  // Setting up URDF path
  std::string pkg_path = ros::package::getPath("codcs_ik");
  // std::string urdf_path = pkg_path + std::string("/urdf/") +
  // "smart_arm_r.urdf";
  std::string urdf_path = pkg_path + std::string("/urdf/") + "gen3_4dof_v4.urdf";

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
  so.trocar_pos_ = Vector3d(0.5197, 0.0248, 0.4351);

  ROS_INFO_STREAM("Error type: " << so.err_method_);

  // Initialiing CODCS_IK
  ROS_INFO("Starting CODCS-IK");
  CODCS_IK iks(urdf_path, "R_base_link", "R_ee", ik_solver, so, max_time, max_error,
               max_iter, dt);

  ROS_INFO("Starting KDL");
  BenchmarkKDL kdl_ik(urdf_path, "R_base_link", "R_ee", ik_solver, max_time,
                      max_error, max_iter, dt);

  ROS_INFO("Starting TRAC-IK");
  BenchmarkTRACIK trac_ik(urdf_path, "R_base_link", "R_ee", ik_solver, max_time,
                          max_error, max_iter, dt);

  ROS_WARN("Running random configurations for KDL");
  solveIkFromRandomList(urdf_path, n_random_config, &kdl_ik, "R_ee",
                        (print_all & 1));

  ROS_WARN("Running random configurations for TRAC-IK");
  solveIkFromRandomList(urdf_path, n_random_config, &trac_ik, "R_ee",
                        (print_all & 2) >> 1);

  ROS_WARN("Running random configurations for CODCS-IK");
  solveIkFromRandomList(urdf_path, n_random_config, &iks, "R_ee",
                        (print_all & 4) >> 2);
}
