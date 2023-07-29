#ifndef SOLVER_OPTIONS_HPP
#define SOLVER_OPTIONS_HPP

// Eigen
#include <Eigen/Dense>
// C++
#include <map>
#include <vector>

using namespace Eigen;

namespace codcs_ik
{

  class SolverOptions
  {
public:
    int                                verb_level_;
    std::string                        time_stats_;
    std::vector<double>                cost_coeff_;
    std::map<std::string, std::string> other_opts_;
    bool                               constrained_control_;
    bool                               rcm_is_cost_;
    double                             rcm_error_max_;
    std::string                        nlp_linear_solver_;
    std::string                        err_method_;
    Vector3d                           trocar_pos_;
  };
} // namespace codcs_ik
#endif