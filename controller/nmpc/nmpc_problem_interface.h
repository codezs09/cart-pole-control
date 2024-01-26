#pragma once

#include <cppad/ipopt/solve.hpp>
#include <string>

#include "proto/proto_gen/data.pb.h"
#include "utils/utils.hpp"

namespace CartPole {

/**
 * @brief NmpcProblem class interface defines the CartPole NMPC problem.
 *
 */
class NmpcProblem {
 public:
  NmpcProblem() = default;
  ~NmpcProblem() = default;

  virtual bool Init(const std::string& super_param_path,
                    const std::string& control_param_path) = 0;

  virtual void Solve(const std::vector<double>& state,
                     const std::vector<double>& target, double last_control,
                     cart_pole::Frame* frame) = 0;

 protected:
  json super_param_;
  json control_param_;
  std::vector<double> initial_guess_;
};

}  // namespace CartPole