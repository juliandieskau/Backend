#pragma once

#include "nlohmann/json.hpp"
#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <optional>
#include <utility>

using json = nlohmann::json;

namespace ects {
class Configuration {
  public:
    static auto load_configuration(std::string path)
        -> std::optional<Configuration>;

    template <typename T> auto get_value(const std::string &key) const -> T {
        return config_json.at(json::json_pointer(key)).get<T>();
    }
    template <typename T>
    auto get_value_or_default(const std::string &key,
                              const T &default_value) const -> T {
        try {
            auto x = config_json.at(json::json_pointer(key));
            if (!x.is_null())
                return x.get<T>();
        } catch (json::exception &e) {
        }
        ROS_INFO_STREAM("default config at " << key << " to " << default_value);
        return default_value;
    }

    auto dump() const -> std::string { return config_json.dump(); }

  private:
    explicit Configuration(json config_json)
        : config_json(std::move(config_json)) {}

    json config_json;
};
} // namespace ects
