#ifndef NLO_NLOPT_HPP
#define NLO_NLOPT_HPP

#include <chrono>
#include <codcs_ik/solver_base.hpp>
#include <nlopt.hpp>
// CODCS
#include <codcs_ik/solver_options.hpp>
namespace codcs_ik
{
  double minfuncSumSquared(const std::vector<double> &x,
                           std::vector<double> &grad, void *data);
  class NLO_NLOPT : public SolverBase
  {
  private:
    nlopt::opt opt;

    pin::Data mdl_data_;

    int max_iter_;
    double dt_;

    VectorXd q_ul_;
    VectorXd q_ll_;
    VectorXd q_sol_;
    std::vector<double> q_sol_vec_;

  public:
    NLO_NLOPT(const pin::Model &_model, const pin::FrameIndex &_Fid,
              SolverOptions _solver_opts, const double _max_time_,
              const double _eps, const int _max_iter, const double _dt);
    ~NLO_NLOPT();
    void cartSumSquaredError(const std::vector<double> &x, double error[]);

    int IkSolve(const VectorXd q_init, const pin::SE3 &x_Fee_d, VectorXd &q_out);
    inline void abort() { aborted_ = true; }
    inline void reset() { aborted_ = false; }
    inline void set_max_time(double _max_time) { max_time_ = _max_time; }
    inline void set_max_error(double _max_error) { max_error_ = _max_error; }
  };

  double minfuncSumSquared(const std::vector<double> &x,
                           std::vector<double> &grad, void *data)
  {

    NLO_NLOPT *c = (NLO_NLOPT *)data;

    std::vector<double> vals(x);

    double jump = std::numeric_limits<float>::epsilon();
    double result[1];
    c->cartSumSquaredError(vals, result);

    if (!grad.empty())
    {
      // std::cout << "using grads" << std::endl;
      double v1[1];
      for (uint i = 0; i < x.size(); i++)
      {
        double original = vals[i];

        vals[i] = original + jump;
        c->cartSumSquaredError(vals, v1);

        vals[i] = original;
        grad[i] = (v1[0] - result[0]) / (2.0 * jump);
      }
    }

    return result[0];
  }

  NLO_NLOPT::NLO_NLOPT(const pin::Model &_model, const pin::FrameIndex &_Fid,
                       SolverOptions _solver_opts, const double _max_time_,
                       const double _eps, const int _max_iter, const double _dt)
      : max_iter_(_max_iter), dt_(_dt)
  {
    aborted_ = false;
    success_ = false;
    mdl_ = _model;
    n_joints_ = _model.nq;
    fid_ = _Fid;
    max_time_ = _max_time_;
    max_error_ = _eps;

    mdl_data_ = pin::Data(mdl_);

    q_ll_ = mdl_.lowerPositionLimit;
    q_ul_ = mdl_.upperPositionLimit;

    opt = nlopt::opt(nlopt::LD_SLSQP, n_joints_);
    std::vector<double> tolerance(1, std::numeric_limits<float>::epsilon());
    opt.set_xtol_abs(tolerance[0]);
    opt.set_min_objective(minfuncSumSquared, this);
  }

  NLO_NLOPT::~NLO_NLOPT() {}

  int NLO_NLOPT::IkSolve(const VectorXd q_init, const pin::SE3 &x_Fee_d,
                         VectorXd &q_out)
  {
    std::chrono::time_point<std::chrono::high_resolution_clock>
        start_iksolve_time = std::chrono::high_resolution_clock::now();

    std::chrono::microseconds diff;

    Vector6d err;
    q_sol_vec_.resize(n_joints_);
    x_Fee_d_ = x_Fee_d;
    success_ = false;

    opt.set_maxtime(max_time_);

    double minf; /* the minimum objective value, upon return */

    std::vector<double> q(q_init.data(), q_init.data() + q_init.size());

    std::vector<double> artificial_ll(q_ll_.data(), q_ll_.data() + q_ll_.size());
    opt.set_lower_bounds(artificial_ll);

    std::vector<double> artificial_ul(q_ul_.data(), q_ul_.data() + q_ul_.size());
    opt.set_upper_bounds(artificial_ul);

    try
    {
      opt.optimize(q, minf);
    }
    catch (...)
    {
    }

    if (!aborted_ && !success_)
    {

      double time_left;
      diff = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now() - start_iksolve_time);
      time_left = max_time_ - diff.count() / 1000000.0;

      // while (time_left > 0 && !aborted && progress < 0)
      while (time_left > 0 && !aborted_ && !success_)
      {
        // std::cout << "NL: Solution not found. Using random seeds." <<
        // std::endl;
        for (int i = 0; i < q.size(); i++)
        {
          q[i] = artificial_ll[i] + ((double)std::rand() / RAND_MAX) *
                                        (artificial_ul[i] - artificial_ll[i]);
          // std::cout << "q[" << i << "]: " << q[i] << std::endl;
        }

        opt.set_maxtime(time_left);
        // opt.set_maxtime(max_time_);

        try
        {
          opt.optimize(q, minf);
        }
        catch (...)
        {
        }

        //   if (progress == -1) // Got NaNs
        //     progress = -3;

        diff = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start_iksolve_time);
        time_left = max_time_ - diff.count() / 1000000.0;
      }
    }
    // if (aborted_)
    //   std::cout << "NL: Aborted by TP" << std::endl;

    if (success_)
    {
      // std::cout << "NL: Solution found" << std::endl;
      q_out = VectorXd::Map(&q_sol_vec_[0], q_sol_vec_.size());
      return 0;
    }
    else
    {
      q_out = q_init;
      return 1;
    }
  }
  void NLO_NLOPT::cartSumSquaredError(const std::vector<double> &x,
                                      double error[])
  {

    if (aborted_)
    {
      opt.force_stop();
      return;
    }

    Vector6d err;

    VectorXd q_sol;
    q_sol = VectorXd::Map(&x[0], x.size());

    pin::framesForwardKinematics(mdl_, mdl_data_, q_sol);

    pin::SE3 T_Fee_d_Fee_a = x_Fee_d_.actInv(mdl_data_.oMf[fid_]);
    err = pin::log6(T_Fee_d_Fee_a).toVector();

    error[0] = err.dot(err);

    if (err.isZero(max_error_))
    {
      success_ = true;
      q_sol_vec_ = x;
      return;
    }
  }
} // namespace codcs_ik
#endif