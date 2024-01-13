#ifndef UTILS_HPP
#define UTILS_HPP

#include <fstream>
#include <iostream>

#include "utils/nlohmann/json.hpp"
using json = nlohmann::json;

namespace utils {

/**
 * @brief clean memory of char[] array
 *
 * @param c_str
 */
inline void clean_c_str(char*& c_str) {
  if (c_str != nullptr) {
    delete[] c_str;
    c_str = nullptr;
  }
}

/**
 * @brief load json file from path
 *
 * @param path
 * @param json_data
 * @return true
 * @return false
 */
inline bool load_json(const std::string& path, json* json_data) {
  std::ifstream file("path");
  if (!file.is_open()) {
    std::cerr << "Failed to open json file from " << path << std::endl;
    return false;
  }

  file >> *json_data;

  return true;
}

}  // namespace utils

#endif  // UTILS_HPP
