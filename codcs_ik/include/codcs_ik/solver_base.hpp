#ifndef SOLVER_BASE_HPP
#define SOLVER_BASE_HPP

#include <pinocchio/algorithm/frames.hpp>
// SolverOptions
#include <codcs_ik/solver_options.hpp>

namespace pin = pinocchio;
using namespace Eigen;

namespace codcs_ik
{
    const double kDEG2RAD = (M_PI) / 180;
    const double kRAD2DEG = 180 / (M_PI);

    typedef Matrix<double, 6, 1> Vector6d;
    typedef Matrix<double, 6, 1> Twist;

    class SolverBase
    {
    protected:
        pin::Model mdl_;
        pin::FrameIndex fid_;
        pin::SE3 x_Fee_d_;

        int n_joints_;

        double max_time_;
        double max_error_;

        bool aborted_;
        bool success_;

    public:
        virtual int IkSolve(const VectorXd _q_init, const pin::SE3 &_x_Fee_d,
                            VectorXd &_q_out) { return 0; };
    };
} // namespace codcs_ik
#endif