#include "fg_eval.h"

namespace CartPole {

void FG_eval::LoadState(const std::vector<double>& state,
                        const std::vector<double>& target,
                        double last_control) {
  state_ = state;
  target_ = target;
  last_control_ = last_control;
}

void FG_eval::operator()(ADvector& fg, const ADvector& vars) {
  ADvector us, xs, dxs, thetas, dthetas;
  _UpdateADStateSequence(vars, &us, &xs, &dxs, &thetas, &dthetas);

  // fg[0]: cost function
  CppAD::AD<double> cost_x, cost_theta, cost_u, cost_du;
  _UpdateADCosts(vars, us, xs, thetas, &cost_x, &cost_theta, &cost_u, &cost_du);

  fg[0] = cost_x + cost_theta + cost_u + cost_du;

  // fg[1:(hp+hc+1)]: constraints
  for (size_t i = 0; i < hc_; i++) {
    fg[i + 1] = us[i];
  }
  for (size_t i = 0; i < hp_; i++) {
    fg[i + hc_ + 1] = thetas[i];
  }
}

void FG_eval::GetStateSequenceValues(const std::vector<double>& vars_val,
                                     std::vector<double>* ts_val,
                                     std::vector<double>* us_val,
                                     std::vector<double>* xs_val,
                                     std::vector<double>* dxs_val,
                                     std::vector<double>* thetas_val,
                                     std::vector<double>* dthetas_val) {
  assert(vars_val.size() == hc_);
  ADvector vars(hc_);
  CppAD::Independent(vars);

  ADvector us, xs, dxs, thetas, dthetas;
  _UpdateADStateSequence(vars, &us, &xs, &dxs, &thetas, &dthetas);

  ADvector outputs(hc_ + 4 * hp_);
  std::copy(us.begin(), us.end(), outputs.begin());
  std::copy(xs.begin(), xs.end(), outputs.begin() + hc_);
  std::copy(dxs.begin(), dxs.end(), outputs.begin() + hc_ + hp_);
  std::copy(thetas.begin(), thetas.end(), outputs.begin() + hc_ + 2 * hp_);
  std::copy(dthetas.begin(), dthetas.end(), outputs.begin() + hc_ + 3 * hp_);

  CppAD::ADFun<double> fun(vars, outputs);
  std::vector<double> outputs_val = fun.Forward(0, vars_val);

  // update
  ts_val->resize(hp_);
  for (size_t i = 0; i < hp_; i++) {
    ts_val->at(i) =
        i * static_cast<double>(control_param_["nmpc_cfg"]["mpc_dt"]);
  }

  us_val->assign(hp_, outputs_val[hc_ - 1]);
  std::copy(outputs_val.begin(), outputs_val.begin() + hc_, us_val->begin());

  xs_val->assign(outputs_val.begin() + hc_, outputs_val.begin() + hc_ + hp_);
  dxs_val->assign(outputs_val.begin() + hc_ + hp_,
                  outputs_val.begin() + hc_ + 2 * hp_);
  thetas_val->assign(outputs_val.begin() + hc_ + 2 * hp_,
                     outputs_val.begin() + hc_ + 3 * hp_);
  dthetas_val->assign(outputs_val.begin() + hc_ + 3 * hp_, outputs_val.end());
}

void FG_eval::GetCostsValues(const std::vector<double>& vars_val,
                             double* cost_x_val, double* cost_theta_val,
                             double* cost_u_val, double* cost_du_val) {
  assert(vars_val.size() == hc_);
  ADvector vars(hc_);
  CppAD::Independent(vars);

  ADvector us, xs, dxs, thetas, dthetas;
  _UpdateADStateSequence(vars, &us, &xs, &dxs, &thetas, &dthetas);

  CppAD::AD<double> cost_x, cost_theta, cost_u, cost_du;
  _UpdateADCosts(vars, us, xs, thetas, &cost_x, &cost_theta, &cost_u, &cost_du);

  ADvector outputs(4);
  outputs[0] = cost_x;
  outputs[1] = cost_theta;
  outputs[2] = cost_u;
  outputs[3] = cost_du;

  CppAD::ADFun<double> fun(vars, outputs);
  std::vector<double> outputs_val = fun.Forward(0, vars_val);

  *cost_x_val = outputs_val[0];
  *cost_theta_val = outputs_val[1];
  *cost_u_val = outputs_val[2];
  *cost_du_val = outputs_val[3];
}

void FG_eval::_UpdateADStateSequence(const ADvector& vars, ADvector* us,
                                     ADvector* xs, ADvector* dxs,
                                     ADvector* thetas, ADvector* dthetas) {
  us->resize(hc_);
  xs->resize(hp_);
  dxs->resize(hp_);
  thetas->resize(hp_);
  dthetas->resize(hp_);

  // control parameters
  const double dt = control_param_["nmpc_cfg"]["mpc_dt"];

  // model parameters (based on knowledge from control side, may have errors
  // compared to trueth)
  const double M = control_param_["model_param"]["mass_cart"];
  const double b = control_param_["model_param"]["damping_cart"];
  const double m = control_param_["model_param"]["mass_pole"];
  const double I = control_param_["model_param"]["inertia_pole"];
  const double l = control_param_["model_param"]["length_com_pole"];
  const double g = control_param_["model_param"]["gravity"];

  // given states
  const double x0 = state_[0];
  const double dx0 = state_[1];
  const double theta0 = state_[2];
  const double dtheta0 = state_[3];
  const double last_u = last_control_;

  // forumalation of sequence of control, u[i], i=0,...,hc-1
  CppAD::AD<double> u = last_u;
  for (size_t i = 0; i < hc_; i++) {
    u += vars[i];
    (*us)[i] = u;
  }

  // formulation of sequence of state, x[i], i=1,...,hp
  CppAD::AD<double> x = x0;
  CppAD::AD<double> dx = dx0;
  CppAD::AD<double> theta = theta0;
  CppAD::AD<double> dtheta = dtheta0;
  for (size_t i = 0; i < hp_; i++) {
    CppAD::AD<double> F;
    if (i < hc_) {
      F = (*us)[i];
    } else {
      F = (*us)[hc_ - 1];
    }

    // dynamics, check `plant.py` for equation details.
    CppAD::AD<double> ddx =
        ((I + m * l * l) *
             (F - b * dx + m * l * dtheta * dtheta * CppAD::sin(theta)) -
         m * m * l * l * g * CppAD::sin(theta) * CppAD::cos(theta)) /
        ((I + m * l * l) * (M + m) - CppAD::pow(m * l * CppAD::cos(theta), 2));

    CppAD::AD<double> ddtheta =
        ((M + m) * m * g * l * CppAD::sin(theta) +
         m * l * b * CppAD::cos(theta) * dx -
         CppAD::pow(m * l * dtheta, 2) * CppAD::sin(theta) * CppAD::cos(theta) -
         F * m * l * CppAD::cos(theta)) /
        ((I + m * l * l) * (M + m) - CppAD::pow(m * l * CppAD::cos(theta), 2));

    // x[k+1] = x[k] + f(x[k], u[k]) * dt
    x += dx * dt;
    dx += ddx * dt;
    theta += dtheta * dt;
    dtheta += ddtheta * dt;

    // update
    (*xs)[i] = x;
    (*dxs)[i] = dx;
    (*thetas)[i] = theta;
    (*dthetas)[i] = dtheta;
  }
}

void FG_eval::_UpdateADCosts(const ADvector& vars, const ADvector& us,
                             const ADvector& xs, const ADvector& thetas,
                             CppAD::AD<double>* cost_x,
                             CppAD::AD<double>* cost_theta,
                             CppAD::AD<double>* cost_u,
                             CppAD::AD<double>* cost_du) {
  const double q_x = control_param_["nmpc_cfg"]["Qx"];
  const double q_th = control_param_["nmpc_cfg"]["Qtheta"];
  const double r_u = control_param_["nmpc_cfg"]["R_u"];
  const double r_du = control_param_["nmpc_cfg"]["R_du"];

  const double x_r = target_[0];
  const double theta_r = target_[1];

  *cost_x = 0.0;
  *cost_theta = 0.0;
  *cost_u = 0.0;
  *cost_du = 0.0;
  for (size_t i = 0; i < hc_; i++) {
    *cost_u += r_u * CppAD::pow(us[i], 2);
    *cost_du += r_du * CppAD::pow(vars[i], 2);
  }
  for (size_t i = 0; i < hp_; ++i) {
    *cost_x += q_x * CppAD::pow(xs[i] - x_r, 2);
    *cost_theta += q_th * CppAD::pow(thetas[i] - theta_r, 2);
  }
}

}  // namespace CartPole