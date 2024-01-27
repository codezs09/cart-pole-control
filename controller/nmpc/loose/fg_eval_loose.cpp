#include "fg_eval_loose.h"

namespace CartPole {

namespace {
/**
 * @brief The loose barrier function takes the form similar to the
 * following reference, except that there's a 1/t multiplication factor:
 *
 * Reference:
 *  - Hauser, John, and Alessandro Saccon. "A barrier function method for
 *    the optimization of trajectory functionals with constraints."
 *    Proceedings of the 45th IEEE Conference on Decision and Control. IEEE,
 *    2006.
 */
CppAD::AD<double> loose_barrier(CppAD::AD<double> z, double t, int k,
                                double delta) {
  CppAD::AD<double> res;
  if (z > delta) {
    res = -1.0 / t * CppAD::log(z);
  } else {
    res = (k - 1.0) / k *
              (CppAD::pow((z - k * delta) / (k - 1) / delta, k) - 1) / t -
          CppAD::log(z) / t;
  }
}

}  // namespace

void FG_eval_loose::operator()(ADvector& fg, const ADvector& vars) {
  ADvector us, xs, dxs, thetas, dthetas;
  _UpdateADStateSequence(vars, &us, &xs, &dxs, &thetas, &dthetas);

  // fg[0]: cost function
  CppAD::AD<double> cost_x, cost_theta, cost_u, cost_du;
  _UpdateADCosts(vars, us, xs, thetas, &cost_x, &cost_theta, &cost_u, &cost_du);

  fg[0] = cost_x + cost_theta + cost_u + cost_du;

  // hard constraints
  CppAD::AD<double> cost_delta_u_bounds, cost_u_bounds, cost_theta_bounds;
  _UpdateDeltaUsBoundsCost(vars, &cost_delta_u_bounds);
  _UpdateUsBoundsCost(us, &cost_u_bounds);
  _UpdateThetasBoundsCost(thetas, &cost_theta_bounds);

  fg[0] += cost_delta_u_bounds + cost_u_bounds + cost_theta_bounds;
}

void FG_eval_loose::_UpdateDeltaUsBoundsCost(
    const ADvector& delta_us, CppAD::AD<double>* cost_du_bounds) {
  const double mpc_dt = control_param_["nmpc_cfg"]["mpc_dt"];
  const double force_rate_limit = control_param_["force_rate_limit"];
  const double delta_u_limit = force_rate_limit * mpc_dt;
  const double delta_u_min = -std::fabs(delta_u_limit);
  const double delta_u_max = std::fabs(delta_u_limit);
  const double regulator = delta_u_max - delta_u_min;
  assert(regulator > 0.0);

  // Factors for loose barrier function
  const double factor_t = 1.0;
  const int factor_k = 2.0;           // even integer > 1
  const double factor_delta = 0.001;  // 0 < delta <=1

  // delta_u[k] - delta_u_min >= 0
  for (size_t i = 0; i < hc_; i++) {
    CppAD::AD<double> delta_u_normalized =
        (delta_us[i] - delta_u_min) / regulator;
    *cost_du_bounds +=
        loose_barrier(delta_u_normalized, factor_t, factor_k, factor_delta);
  }

  // delta_u_max - delta_u[k] >= 0
  for (size_t i = 0; i < hc_; i++) {
    CppAD::AD<double> delta_u_normalized =
        (delta_u_max - delta_us[i]) / regulator;
    *cost_du_bounds +=
        loose_barrier(delta_u_normalized, factor_t, factor_k, factor_delta);
  }
}

void FG_eval_loose::_UpdateUsBoundsCost(const ADvector& us,
                                        CppAD::AD<double>* cost_u_bounds) {
  const double force_limit = control_param_["force_limit"];
  const double u_min = -std::fabs(force_limit);
  const double u_max = std::fabs(force_limit);
  const double regulator = u_max - u_min;
  assert(regulator > 0.0);

  // Factors for loose barrier function
  const double factor_t = 1.0;
  const int factor_k = 2.0;           // even integer > 1
  const double factor_delta = 0.001;  // 0 < delta <=1

  // u[k] - u_min >= 0
  for (size_t i = 0; i < hc_; i++) {
    CppAD::AD<double> u_normalized = (us[i] - u_min) / regulator;
    *cost_u_bounds +=
        loose_barrier(u_normalized, factor_t, factor_k, factor_delta);
  }

  // u_max - u[k] >= 0
  for (size_t i = 0; i < hc_; i++) {
    CppAD::AD<double> u_normalized = (u_max - us[i]) / regulator;
    *cost_u_bounds +=
        loose_barrier(u_normalized, factor_t, factor_k, factor_delta);
  }
}

void FG_eval_loose::_UpdateThetasBoundsCost(
    const ADvector& thetas, CppAD::AD<double>* cost_theta_bounds) {
  const double theta_limit = control_param_["theta_limit"];
  const double theta_min = -std::fabs(theta_limit);
  const double theta_max = std::fabs(theta_limit);
  const double regulator = theta_max - theta_min;
  assert(regulator > 0.0);

  // Factors for loose barrier function
  const double factor_t = 1.0;
  const int factor_k = 2.0;           // even integer > 1
  const double factor_delta = 0.001;  // 0 < delta <=1

  // theta[k] - theta_min >= 0
  for (size_t i = 0; i < hp_; i++) {
    CppAD::AD<double> theta_normalized = (thetas[i] - theta_min) / regulator;
    *cost_theta_bounds +=
        loose_barrier(theta_normalized, factor_t, factor_k, factor_delta);
  }

  // theta_max - theta[k] >= 0
  for (size_t i = 0; i < hp_; i++) {
    CppAD::AD<double> theta_normalized = (theta_max - thetas[i]) / regulator;
    *cost_theta_bounds +=
        loose_barrier(theta_normalized, factor_t, factor_k, factor_delta);
  }
}

}  // namespace CartPole