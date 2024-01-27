#include "nmpc_problem_loose.h"
#include "fg_eval_loose.h"

namespace CartPole {

bool NmpcProblemLoose::Init(const std::string& super_param_path,
                            const std::string& control_param_path) {
  utils::load_json(super_param_path, &super_param_);
  utils::load_json(control_param_path, &control_param_);

  const size_t hc = control_param_["nmpc_cfg"]["hc"];
  initial_guess_.clear();
  initial_guess_.resize(hc, 0.0);
}

/**
 * @brief
 * @note
 * vars: increment of force, delta_u_[i], i=0,...,hc-1
 * X: [x, dx, theta, dtheta, u]
 * Update: X[k+1] = f(X[k], delta_u[k], *args), k=0,...,hp-1
 * Cost: sum_{k=1}^{hp} (X[k] - X_target[k])^T Q (X[k] - X_target[k]) +
 *       sum_{k=0}^{hc-1} delta_u[k]^T R delta_u[k]
 * Constraints:
 *       u_lb <= u[k] <= u_ub, k=0,...,hc-1
 *       theta_lb <= theta[k] <= theta_ub, k=0,...,hp-1
 */
void NmpcProblemLoose::Solve(const std::vector<double>& state,
                             const std::vector<double>& target,
                             double last_control, cart_pole::Frame* frame) {
  bool ok = true;
  const size_t hc = control_param_["nmpc_cfg"]["hc"];
  const size_t hp = control_param_["nmpc_cfg"]["hp"];

  size_t n_vars = hc;
  Dvector vars(n_vars);
  _SetInitialGuess(&vars);

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  _SetVariableBounds(&vars_lowerbound, &vars_upperbound);

  // s.t. x[k+1] = f(x[k], du[k], args), k=0,...,hp-1

  FG_eval_loose fg_eval(super_param_, control_param_);
  fg_eval.LoadState(state, target, last_control);

  std::string options = NmpcProblemLoose::_SetSolverOptions();

  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, FG_eval_loose>(options, vars, vars_lowerbound,
                                              vars_upperbound, Dvector(),
                                              Dvector(), fg_eval, solution);
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  frame->set_x(state[0]);
  frame->set_dx(state[1]);
  frame->set_theta(state[2]);
  frame->set_dtheta(state[3]);
  if (ok) {
    frame->set_status(true);
    _UpdateResults(solution, fg_eval, frame);
  } else {
    frame->set_status(false);
    std::cerr << "Optimization failed! " << std::endl;
  }
}

void NmpcProblemLoose::_UpdateResults(
    const CppAD::ipopt::solve_result<Dvector>& solution, FG_eval_loose& fg_eval,
    cart_pole::Frame* frame) {
  // OPTIONAL: output solution.x for next initial guess

  // fill state and control optimal sequence
  initial_guess_.assign(solution.x.begin(), solution.x.end());
  const auto& vars_val = initial_guess_;

  std::vector<double> ts_val, us_val, xs_val, dxs_val, thetas_val, dthetas_val;
  fg_eval.GetStateSequenceValues(vars_val, &ts_val, &us_val, &xs_val, &dxs_val,
                                 &thetas_val, &dthetas_val);

  frame->set_force(us_val[0]);
  frame->clear_mpc_horizon();
  auto mpc_horizon = frame->mutable_mpc_horizon();

  mpc_horizon->mutable_t()->Add(ts_val.begin(), ts_val.end());
  mpc_horizon->mutable_x()->Add(xs_val.begin(), xs_val.end());
  mpc_horizon->mutable_dx()->Add(dxs_val.begin(), dxs_val.end());
  mpc_horizon->mutable_theta()->Add(thetas_val.begin(), thetas_val.end());
  mpc_horizon->mutable_dtheta()->Add(dthetas_val.begin(), dthetas_val.end());
  mpc_horizon->mutable_force()->Add(us_val.begin(), us_val.end());

  // fill value of costs
  double cost_x_val, cost_theta_val, cost_u_val, cost_du_val;
  fg_eval.GetCostsValues(vars_val, &cost_x_val, &cost_theta_val, &cost_u_val,
                         &cost_du_val);
  double cost_total = cost_x_val + cost_theta_val + cost_u_val + cost_du_val;

  auto costs = frame->mutable_costs();
  costs->set_cost_total(cost_total);
  costs->set_cost_x(cost_x_val);
  costs->set_cost_theta(cost_theta_val);
  costs->set_cost_u(cost_u_val);
  costs->set_cost_du(cost_du_val);
}

std::string NmpcProblemLoose::_SetSolverOptions() {
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";
  options += "String  linear_solver         mumps\n";  // ma57 ma27 mumps

  return options;
}

void NmpcProblemLoose::_SetInitialGuess(Dvector* vars) {
  for (size_t i = 0; i < vars->size(); ++i) {
    (*vars)[i] = initial_guess_[i];
  }
}

void NmpcProblemLoose::_SetVariableBounds(Dvector* vars_lowerbound,
                                          Dvector* vars_upperbound) {
  for (size_t i = 0; i < vars_lowerbound->size(); ++i) {
    (*vars_lowerbound)[i] = -1e19;  // -inf
    (*vars_upperbound)[i] = 1e19;   // +inf
  }
}

}  // namespace CartPole
