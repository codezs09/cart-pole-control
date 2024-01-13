#pragma once

#include <cppad/ipopt/solve.hpp>

#include "utils/utils.hpp"

namespace CartPole {

typedef CPPAD_TESTVECTOR(double) Dvector;

/**
 * @brief FG_eval class definition implementation.
 *
 */
class FG_eval {
 public:
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
  FG_eval() = delete;
  FG_eval(json super_param, json control_param)
      : super_param_(super_param), control_param_(control_param) {
    hp_ = control_param_["hp"];
    hc_ = control_param_["hc"];
  }

  void LoadState(const std::vector<double>& state,
                 const std::vector<double>& target, double last_control);

  // MPC implementation
  void operator()(ADvector& fg, const ADvector& vars);

  void GetStateSequenceValues(const std::vector<double>& vars_val,
                              std::vector<double>* ts_val,
                              std::vector<double>* us_val,
                              std::vector<double>* xs_val,
                              std::vector<double>* dxs_val,
                              std::vector<double>* thetas_val,
                              std::vector<double>* dthetas_val) const;

  void GetCostsValues(const std::vector<double>& vars_val, double* cost_x_val,
                      double* cost_theta_val, double* cost_u_val,
                      double* cost_du_val) const;

 private:
  void _UpdateADStateSequence(const ADvector& vars, ADvector* us, ADvector* xs,
                              ADvector* dxs, ADvector* thetas,
                              ADvector* dthetas);

  void _UpdateADCosts(const ADvector& vars, const ADvector& us,
                      const ADvector& xs, const ADvector& thetas,
                      CppAD::AD<double>* cost_x, CppAD::AD<double>* cost_theta,
                      CppAD::AD<double>* cost_u, CppAD::AD<double>* cost_du);

  json super_param_;
  json control_param_;

  size_t hp_;
  size_t hc_;

  std::vector<double> state_;
  std::vector<double> target_;
  double last_control_;
};

}  // namespace CartPole