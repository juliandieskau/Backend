#pragma once

#include "nlohmann/json.hpp"
#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <optional>

using json = nlohmann::json;

namespace ects {
class Configuration {
  public:
    static auto load_configuration(std::string path)
        -> std::optional<Configuration>;

    template <typename T> auto get_value(std::string key) const -> T {
        return config_json.at(json::json_pointer(key)).get<T>();
    }

    auto dump() const -> std::string { return config_json.dump(); }

  private:
    explicit Configuration(json config_json) : config_json(config_json) {}

    json config_json;
};
} // namespace ects
