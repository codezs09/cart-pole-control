#pragma once

#include <iostream>
#include <memory>
#include <string>

#include "nmpc_problem_interface.h"
#include "nmpc_problem_regular.h"

namespace CartPole {

class NmpcProblemFactory {
 public:
  NmpcProblemFactory() = delete;
  ~NmpcProblemFactory() = default;

  static std::unique_ptr<NmpcProblem> Create(
      const std::string& super_param_path,
      const std::string& control_param_path) {
    std::unique_ptr<NmpcProblem> nmpc_problem_ptr = nullptr;

    json control_param;
    utils::load_json(control_param_path, &control_param);
    std::string nmpc_type = control_param["nmpc_cfg"]["nmpc_type"];
    if (nmpc_type == "regular") {
      nmpc_problem_ptr = std::make_unique<NmpcProblemRegular>();
    } else if (nmpc_type == "loose") {
      // TODO
      // TODO: do nothing
    } else {
      std::string s = "Unsupported NMPC type: " + nmpc_type +
                      "\nPlease set \"nmpc_type\" to one of {\"regular\", "
                      "\"loose\"} in control_param.json\n";
      std::cout << s << std::endl;
      return nullptr;
    }

    nmpc_problem_ptr->Init(super_param_path, control_param_path);
    return nmpc_problem_ptr;
  }
};

}  // namespace CartPole
