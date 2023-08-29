#pragma once
#ifndef ECTS_TESTUTIL_HPP
#define ECTS_TESTUTIL_HPP

#include "ects/ECTS.hpp"
#include "ects/PluginLoader.hpp"
#include "ros/duration.h"
#include "ros/ros.h"
#include "gtest/gtest.h"
#include <functional>
#include <string>

static auto ects_with_config(std::string config) -> ects::ECTS {
    std::string configFilePath = std::tmpnam(nullptr);
    {
        std::ofstream configFile(configFilePath);
        configFile << config;
    }
    auto cfg = ects::Configuration::load_configuration(configFilePath).value();
    auto ros_interface = new ects::RosNode();
    auto timer_manager = new ects::TimerManager();
    auto ects = ects::ECTS(*cfg, *ros_interface, *timer_manager);

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
#endif
