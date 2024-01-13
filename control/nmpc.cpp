#include "nmpc.h"

#include <fstream>
#include <iostream>

namespace CartPole {

Nmpc::Nmpc(const std::string& super_param_path,
           const std::string& control_param_path) {
  nmpc_problem_.Init(super_param_path, control_param_path);
}

Nmpc::~Nmpc() { utils::clean_c_str(c_str_); }

bool Nmpc::RunSolver(const std::vector<double>& state,
                     const std::vector<double>& target, double last_control) {
  nmpc_problem_.Solve(state, target, last_control, &frame_);
  _SerializedFrameMsgToCString();
}

void Nmpc::_SerializedFrameMsgToCString() {
  std::string msg_serialized = frame_.SerializeAsString();
  utils::clean_c_str(c_str_);
  c_str_ = new char[msg_serialized.length() + 1];
  std::strcpy(c_str_, msg_serialized.c_str());
}

const char* Nmpc::GetCStringOfSerializedFrameMsg() { return c_str_; }

extern "C" {
EXPORT_API Nmpc* CreateNMPC(const char* super_param_path,
                            const char* control_param_path) {
  return new Nmpc(super_param_path, control_param_path);
}

EXPORT_API bool RunSolver(Nmpc* nmpc, double* c_state, int c_state_size,
                          double* c_target, int c_target_size,
                          double last_control) {
  std::vector<double> state(c_state, c_state + c_state_size);
  std::vector<double> target(c_target, c_target + c_target_size);

  return nmpc->RunSolver(state, target, last_control);
}

EXPORT_API const char* GetSerializedFrameMsg(Nmpc* nmpc) {
  return nmpc->GetCStringOfSerializedFrameMsg();
}

EXPORT_API bool DestroyNMPC(Nmpc* nmpc) { delete nmpc; }
}

}  // namespace CartPole