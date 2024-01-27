#pragma once

#include <cppad/ipopt/solve.hpp>

#include "controller/nmpc/fg_eval.h"
#include "controller/nmpc/regular/fg_eval_regular.h"
#include "utils/utils.hpp"

namespace CartPole {

/**
 * @brief FG_eval_loose class definition implementation.
 *
 */
class FG_eval_loose : public FG_eval_regular {
 public:
  FG_eval_loose() = delete;
  FG_eval_loose(json super_param, json control_param)
      : FG_eval_regular(super_param, control_param) {}

  // MPC implementation
  void operator()(ADvector& fg, const ADvector& vars) override;

 private:
  void _UpdateDeltaUsBoundsCost(const ADvector& delta_us,
                                CppAD::AD<double>* cost_du_bounds);
  void _UpdateUsBoundsCost(const ADvector& us,
                           CppAD::AD<double>* cost_u_bounds);
  void _UpdateThetasBoundsCost(const ADvector& thetas,
                               CppAD::AD<double>* cost_theta_bounds);
};

}  // namespace CartPole