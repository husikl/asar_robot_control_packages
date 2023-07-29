#ifndef TPIK_HPP
#define TPIK_HPP

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

// TP Interfaces
#include <tp/interfaces/tp_pinocchio.hpp>

using namespace Eigen;
namespace pin = pinocchio;

namespace codcs_ik {

class CODCS_IK;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

template <typename T>
class TP_IkSolver {
  friend class codcs_ik::CODCS_IK;

 public:
  TP_IkSolver(const pin::Model &_model, const pin::FrameIndex &_Fid,
              SolverOptions _solver_opts, const double _max_time,
              const double _max_error, const int _max_iter = 1000,
              const double _dt = 1) {
    tp_solver_.reset(new T(_model, _Fid, _solver_opts, _max_time, _max_error,
                           _max_iter, _dt));
  }
  ~TP_IkSolver(){};

  int IkSolve(const VectorXd q_init, const pin::SE3 &x_des, VectorXd &q_out) {
    int res;
    res = tp_solver_->IkSolve(q_init, x_des, q_out);
    return res;
  }
  MatrixXd pseudoInverse(const Eigen::MatrixXd &a, double epsilon) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        a, Eigen::ComputeThinU | Eigen::ComputeThinV);

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
  MatrixXd weightedPseudoInverse(const Eigen::MatrixXd &a,
                                 const Eigen::VectorXd w) {
    int lenght = w.size();

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Winv(lenght);
    Winv = w.asDiagonal().inverse();  // diag(1./w)

    Eigen::MatrixXd tmp(lenght, lenght);

    //(a*Winv*a.transpose())
    tmp = pseudoInverse(a * Winv * a.transpose(), 10E-10);

    return Winv * a.transpose() * tmp;
  }

  void abort();
  void reset();
  void set_max_time(double _max_time);
  void set_max_error(double _max_error);

 private:
  std::unique_ptr<T> tp_solver_;
};

template <typename T>
inline void TP_IkSolver<T>::abort() {
  tp_solver_->abort();
}
template <typename T>
inline void TP_IkSolver<T>::reset() {
  tp_solver_->reset();
}
template <typename T>
inline void TP_IkSolver<T>::set_max_time(double _max_time) {
  tp_solver_->set_max_time(_max_time);
}
template <typename T>
inline void TP_IkSolver<T>::set_max_error(double _max_error) {
  tp_solver_->set_max_error(_max_error);
}

}  // namespace codcs_ik

#endif