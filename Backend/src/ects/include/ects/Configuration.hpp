#pragma once

#include "nlohmann/json.hpp"
#include "ros/ros.h"
#include <fstream>
#include <iostream>

using json = nlohmann::json;

namespace ects {
class Configuration {
public:
  static auto load_configuration(std::string path) -> Configuration *;

  template <typename T> auto get_value(std::string key) const -> T {
    return m_config[json::json_pointer(key)].get<T>();
  }

  auto dump() const -> std::string { return m_config.dump(); }

private:
  Configuration(json config) : m_config(config) {}

  json m_config;
};
} // namespace ects