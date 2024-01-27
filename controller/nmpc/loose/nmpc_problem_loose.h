#pragma once

#include "controller/nmpc/loose/fg_eval_loose.h"
#include "controller/nmpc/nmpc_problem_interface.h"

namespace CartPole {

/**
 * @brief NmpcProblemLoose class defines the CartPole NMPC problem.
 * Regular:
 *     Replace the hard constrained on states and control with soft constraints.
 */
class NmpcProblemLoose : public NmpcProblem {
 public:
  NmpcProblemLoose() = default;
  ~NmpcProblemLoose() = default;

  bool Init(const std::string& super_param_path,
            const std::string& control_param_path) override;

  void Solve(const std::vector<double>& state,
             const std::vector<double>& target, double last_control,
             cart_pole::Frame* frame) override;

 private:
  std::string _SetSolverOptions();

  void _SetInitialGuess(Dvector* vars);
  void _SetVariableBounds(Dvector* vars_lowerbound, Dvector* vars_upperbound);

  void _UpdateResults(const CppAD::ipopt::solve_result<Dvector>& solution,
                      FG_eval_loose& fg_eval, cart_pole::Frame* frame);
};

}  // namespace CartPole