#pragma once

#include <cppad/ipopt/solve.hpp>

#include "controller/nmpc/fg_eval.h"
#include "utils/utils.hpp"

namespace CartPole {

/**
 * @brief FG_eval_regular class definition implementation.
 *
 */
class FG_eval_regular : public FG_eval {
 public:
  FG_eval_regular() = delete;
  FG_eval_regular(json super_param, json control_param)
      : FG_eval(super_param, control_param) {}

  // MPC implementation
  virtual void operator()(ADvector& fg, const ADvector& vars) override;

  virtual void GetStateSequenceValues(
      const std::vector<double>& vars_val, std::vector<double>* ts_val,
      std::vector<double>* us_val, std::vector<double>* xs_val,
      std::vector<double>* dxs_val, std::vector<double>* thetas_val,
      std::vector<double>* dthetas_val) override;

  virtual void GetCostsValues(const std::vector<double>& vars_val,
                              double* cost_x_val, double* cost_theta_val,
                              double* cost_u_val, double* cost_du_val) override;

 protected:
  virtual void _UpdateADStateSequence(const ADvector& vars, ADvector* us,
                                      ADvector* xs, ADvector* dxs,
                                      ADvector* thetas, ADvector* dthetas);

  virtual void _UpdateADCosts(const ADvector& vars, const ADvector& us,
                              const ADvector& xs, const ADvector& thetas,
                              CppAD::AD<double>* cost_x,
                              CppAD::AD<double>* cost_theta,
                              CppAD::AD<double>* cost_u,
                              CppAD::AD<double>* cost_du);
};

}  // namespace CartPole