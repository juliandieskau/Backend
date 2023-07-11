#pragma once

#include "nlohmann/json.hpp"
#include <fstream>
#include <iostream>
#include "ros/ros.h"

using json = nlohmann::json;

namespace ects {
    class Configuration {
    public:
        static auto load_configuration(std::string path) -> Configuration *;

        template<typename T>
        auto get_value(std::string key) -> T {
            return m_config[json::json_pointer(key)].get<T>();
        }

        auto dump() -> std::string { return m_config.dump(); }

    private:
        Configuration(json config) : m_config(config) {}

        json m_config;
    };
}