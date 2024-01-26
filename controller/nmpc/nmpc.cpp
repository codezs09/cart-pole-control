#include "nmpc.h"

#include <fstream>
#include <iostream>

namespace CartPole {

Nmpc::Nmpc(const std::string& super_param_path,
           const std::string& control_param_path) {
  nmpc_problem_ptr_ =
      NmpcProblemFactory::Create(super_param_path, control_param_path);
  assert(nmpc_problem_ptr_ != nullptr);
}

Nmpc::~Nmpc() { utils::clean_c_str(c_str_); }

bool Nmpc::RunSolver(const std::vector<double>& state,
                     const std::vector<double>& target, double last_control) {
  nmpc_problem_ptr_->Solve(state, target, last_control, &frame_);
  _SerializedFrameMsgToCString();
}

void Nmpc::_SerializedFrameMsgToCString() {
  std::string msg_serialized = frame_.SerializeAsString();
  utils::clean_c_str(c_str_);
  c_str_size_ = msg_serialized.length();
  c_str_ = new char[c_str_size_];  // NOTE: no need to +1 for '\0' for the
                                   // binary string
  memcpy(c_str_, msg_serialized.data(),
         msg_serialized.size());  // NOTE: strcpy not working for binary string

  // std::cout << "Optimization success! ";
  // std::cout << "x = " << frame_.x() << ", dx = " << frame_.dx()
  //           << ", theta = " << frame_.theta()
  //           << ", dtheta = " << frame_.dtheta()
  //           << ", force = " << frame_.force() << std::endl;
  // for (size_t i = 0; i < frame_.mpc_horizon().t_size(); i++) {
  //   std::cout << "t = " << frame_.mpc_horizon().t(i)
  //             << ", x = " << frame_.mpc_horizon().x(i)
  //             << ", dx = " << frame_.mpc_horizon().dx(i)
  //             << ", theta = " << frame_.mpc_horizon().theta(i)
  //             << ", dtheta = " << frame_.mpc_horizon().dtheta(i)
  //             << ", force = " << frame_.mpc_horizon().force(i) << std::endl;
  // }

  // // Debug: serialized back to protobuf message and test if fields are
  // std::string msg_serialized2(c_str_, msg_serialized.length() + 1);
  // cart_pole::Frame frame2;
  // frame2.ParseFromString(msg_serialized2);
  // std::cout << "FRAME2: x = " << frame2.x() << ", dx = " << frame2.dx()
  //           << ", theta = " << frame2.theta()
  //           << ", dtheta = " << frame2.dtheta()
  //           << ", force = " << frame2.force() << std::endl;
  // for (size_t i = 0; i < frame2.mpc_horizon().t_size(); i++) {
  //   std::cout << "FRAME2: t = " << frame2.mpc_horizon().t(i)
  //             << ", x = " << frame2.mpc_horizon().x(i)
  //             << ", dx = " << frame2.mpc_horizon().dx(i)
  //             << ", theta = " << frame2.mpc_horizon().theta(i)
  //             << ", dtheta = " << frame2.mpc_horizon().dtheta(i)
  //             << ", force = " << frame2.mpc_horizon().force(i) << std::endl;
  // }
}

// const char* Nmpc::GetCStringOfSerializedFrameMsg() { return c_str_; }

void Nmpc::GetCStringOfSerializedFrameMsg(char** c_str_ptr, int* c_str_size) {
  *c_str_ptr = c_str_;
  *c_str_size = c_str_size_;
}

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

EXPORT_API void GetSerializedFrameMsg(Nmpc* nmpc, char** c_str_ptr,
                                      int* c_str_size) {
  nmpc->GetCStringOfSerializedFrameMsg(c_str_ptr, c_str_size);
}

EXPORT_API bool DestroyNMPC(Nmpc* nmpc) { delete nmpc; }
}

}  // namespace CartPole