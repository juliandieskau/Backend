#pragma once

#include "ects/ECTS.hpp"
#include "ects/PluginLoader.hpp"
#include "ros/duration.h"
#include "ros/ros.h"
#include "gtest/gtest.h"
#include <functional>
#include <string>

struct {
    struct owned_ects {
        ects::Configuration configuration;
        ects::RosNode ros_interface;
        ects::TimerManager timer_manager = {};
    };
    std::optional<owned_ects> ects = std::nullopt;

    auto set_config(const ects::Configuration &config) -> void {
        this->ects = {config, {}, {}};
    }

    auto get_ects() -> ects::ECTS {
        if (!ects)
            throw std::runtime_error("not created yet");
        return {ects->configuration, ects->ros_interface, ects->timer_manager};
    }

} global;

static auto ects_with_config(std::string config) -> ects::ECTS {
    std::string configFilePath = std::tmpnam(nullptr);
    {
        std::ofstream configFile(configFilePath);
        configFile << config;
    }
    auto cfg = ects::Configuration::load_configuration(configFilePath);
    if (!cfg)
        throw std::runtime_error("invalid test config");
    global.set_config(*cfg);
    auto ects = global.get_ects();

    auto plugin_loader = ects::PluginLoader();
    auto core_plugin = plugin_loader.load("core");
    core_plugin->init(ects);
    ects.add_plugin(std::move(core_plugin));
    return ects;
}

static auto spin_predicate(std::function<bool()> predicate, int timeout_ms)
    -> void {
    auto spin_count = timeout_ms / 10;
    for (int i = 0; i < spin_count; i++) {
        if (predicate()) {
            return;
        }
        ros::spinOnce();
        ros::WallDuration(0.01).sleep();
    }
}

static auto load_test_plugin(ects::ECTS &ects, std::string plugin_name)
    -> void {
    auto plugin_loader = ects::PluginLoader();
    auto plugin = plugin_loader.load(plugin_name);
    ASSERT_NO_THROW(plugin->init(ects));
    EXPECT_EQ(plugin->name(), plugin_name);
    ects.add_plugin(std::move(plugin));
}

