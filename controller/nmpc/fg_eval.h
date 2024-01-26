#pragma once

#include <cppad/ipopt/solve.hpp>

#include "utils/utils.hpp"

namespace CartPole {

typedef CPPAD_TESTVECTOR(double) Dvector;

/**
 * @brief FG_eval class interface.
 *
 */
class FG_eval {
 public:
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
  FG_eval() = delete;
  FG_eval(json super_param, json control_param)
      : super_param_(super_param), control_param_(control_param) {
    hp_ = control_param_["nmpc_cfg"]["hp"];
    hc_ = control_param_["nmpc_cfg"]["hc"];
  }

  void LoadState(const std::vector<double>& state,
                 const std::vector<double>& target, double last_control) {
    state_ = state;
    target_ = target;
    last_control_ = last_control;
  }

  // MPC implementation
  virtual void operator()(ADvector& fg, const ADvector& vars) = 0;

  virtual void GetStateSequenceValues(const std::vector<double>& vars_val,
                                      std::vector<double>* ts_val,
                                      std::vector<double>* us_val,
                                      std::vector<double>* xs_val,
                                      std::vector<double>* dxs_val,
                                      std::vector<double>* thetas_val,
                                      std::vector<double>* dthetas_val) = 0;

  virtual void GetCostsValues(const std::vector<double>& vars_val,
                              double* cost_x_val, double* cost_theta_val,
                              double* cost_u_val, double* cost_du_val) = 0;

 protected:
  json super_param_;
  json control_param_;

  size_t hp_;
  size_t hc_;

  std::vector<double> state_;
  std::vector<double> target_;
  double last_control_;
};

}  // namespace CartPole