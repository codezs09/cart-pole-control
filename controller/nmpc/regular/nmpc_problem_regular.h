#pragma once

#include "controller/nmpc/nmpc_problem_interface.h"
#include "controller/nmpc/regular/fg_eval_regular.h"

namespace CartPole {

/**
 * @brief NmpcProblemRegular class defines the CartPole NMPC problem.
 * Regular:
 *     Optimize the control sequences over the horizon. Hard constrained on
 *     states and control.
 */
class NmpcProblemRegular : public NmpcProblem {
 public:
  NmpcProblemRegular() = default;
  ~NmpcProblemRegular() = default;

  bool Init(const std::string& super_param_path,
            const std::string& control_param_path) override;

  void Solve(const std::vector<double>& state,
             const std::vector<double>& target, double last_control,
             cart_pole::Frame* frame) override;

 private:
  std::string _SetSolverOptions();

  void _SetInitialGuess(Dvector* vars);
  void _SetVariableBounds(Dvector* vars_lowerbound, Dvector* vars_upperbound);
  void _SetConstraintsBounds(Dvector* constraints_lowerbound,
                             Dvector* constraints_upperbound);

  void _UpdateResults(const CppAD::ipopt::solve_result<Dvector>& solution,
                      FG_eval_regular& fg_eval, cart_pole::Frame* frame);
};

}  // namespace CartPole