#pragma once

#include <cppad/ipopt/solve.hpp>
#include <string>

#include "fg_eval.h"
#include "proto/proto_gen/data.pb.h"
#include "utils/utils.hpp"

namespace CartPole {

/**
 * @brief NmpcProblem class defines the CartPole NMPC problem.
 *
 */
class NmpcProblem {
 public:
  NmpcProblem() = default;
  ~NmpcProblem() = default;

  bool Init(const std::string& super_param_path,
            const std::string& control_param_path);

  void Solve(const std::vector<double>& state,
             const std::vector<double>& target, double last_control,
             cart_pole::Frame* frame);

 private:
  std::string _SetSolverOptions();

  void _SetInitialGuess(Dvector* vars);
  void _SetVariableBounds(Dvector* vars_lowerbound, Dvector* vars_upperbound);
  void _SetConstraintsBounds(Dvector* constraints_lowerbound,
                             Dvector* constraints_upperbound);

  void _UpdateResults(const CppAD::ipopt::solve_result<Dvector>& solution,
                      FG_eval& fg_eval, cart_pole::Frame* frame);

  json super_param_;
  json control_param_;
};

}  // namespace CartPole