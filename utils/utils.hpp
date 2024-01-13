#ifndef UTILS_HPP
#define UTILS_HPP

#include <boost/filesystem.hpp>
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
 * @brief check if file exists
 *
 */
inline bool is_file_exist(const std::string& path) {
  boost::filesystem::path p(path);
  if (boost::filesystem::exists(p)) {
    if (boost::filesystem::is_regular_file(p)) {
      return true;
    } else {
      std::cerr << p << " exists, but is not a regular file" << std::endl;
    }
  } else {
    std::cerr << "The file path " << p << " does not exist" << std::endl;
  }
  return false;
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
  if (!is_file_exist(path)) {
    return false;
  }

  std::ifstream file(path);
  if (!file.is_open()) {
    std::cerr << "Failed to open json file from " << path << std::endl;
    return false;
  }

  file >> *json_data;

  // std::cerr << "Successfully load json file from " << path << std::endl;

  return true;
}

}  // namespace utils

#endif  // UTILS_HPP
