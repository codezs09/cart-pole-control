#pragma once

#if defined(_WIN32)
#define EXPORT_API __declspec(dllexport)
#elif defined(__linux__)
#ifndef EXPORT_API
#define EXPORT_API
#endif
#else
#error "Unsupported OS."
#endif

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "nmpc_problem_factory.h"
#include "proto/proto_gen/data.pb.h"
#include "utils/utils.hpp"

namespace CartPole {

class Nmpc {
 public:
  Nmpc() = delete;
  Nmpc(const std::string& super_param_path,
       const std::string& control_param_path);
  ~Nmpc();

  bool RunSolver(const std::vector<double>& state,
                 const std::vector<double>& target, double last_control);

  //   const char* GetCStringOfSerializedFrameMsg();
  void GetCStringOfSerializedFrameMsg(char** c_str_ptr, int* c_str_size);

 private:
  void _SerializedFrameMsgToCString();

  std::unique_ptr<NmpcProblem> nmpc_problem_ptr_ = nullptr;
  cart_pole::Frame frame_;
  char* c_str_ = nullptr;
  int c_str_size_ = 0;
};

extern "C" {
EXPORT_API Nmpc* CreateNMPC(const char* super_param_path,
                            const char* control_param_path);
EXPORT_API bool RunSolver(Nmpc* nmpc, double* c_state, int c_state_size,
                          double* c_target, int c_target_size,
                          double last_control);
EXPORT_API void GetSerializedFrameMsg(Nmpc* nmpc, char** c_str_ptr,
                                      int* c_str_size);
EXPORT_API bool DestroyNMPC(Nmpc* nmpc);
}

}  // namespace CartPole