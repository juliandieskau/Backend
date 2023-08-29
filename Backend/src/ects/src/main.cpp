/* ECTS - main.cpp
 * Entry point of Application
 */
#include "ects/Configuration.hpp"
#include "ects/ECTS.hpp"
#include "ects/PluginLoader.hpp"
#include "ects/RosInterface.hpp"
#include "ects/Timer.hpp"
#include "ros/ros.h"

#include <iostream>
#include <sstream>
#include <unistd.h>

using namespace ects;
auto main(int argc, char **argv) -> int {
    if (argc < 2) {
        ROS_ERROR_STREAM("Usage: " << argv[0] << " [config-file]");
        return -1;
    }
    ROS_INFO_STREAM("Starting ECTS with config file " << argv[1]);
    auto config_opt = Configuration::load_configuration(argv[1]);
    if (!config_opt.has_value()) {
        ROS_FATAL("Can not continue without valid Configuration.");
        return -1;
    }
    auto config = config_opt.value();
    ROS_INFO_STREAM("Running with config: " << config->dump());

    ros::init(argc, argv, "ects");
    ROS_INFO("Initialized ROS node");

    RosNode ros;
    TimerManager timer_manager;
    PluginLoader plugin_loader;
    ECTS ects{*config, ros, timer_manager};
    std::vector<std::unique_ptr<Plugin>> uninitialized_plugins;
    for (const auto &plugin_json :
         config->get_value<json::array_t>("/core/load_plugins")) {
        auto plugin_name = plugin_json.get<std::string>();
        ROS_INFO_STREAM("Loading plugin " << plugin_name);
        auto plugin = plugin_loader.load(plugin_name);
        if (!plugin)
            ROS_ERROR_STREAM("Could not load plugin " << plugin_name);
        else
            uninitialized_plugins.push_back(std::move(plugin));
    }

    for (auto &plugin : uninitialized_plugins) {
        try {
            plugin->init(ects);
            ects.add_plugin(std::move(plugin));
        } catch (std::exception &e) {
            ROS_ERROR_STREAM("failed to initialize " << plugin->name() << ": "
                                                     << e.what());
        }
    }
    // clean up all plugins which failed to initialize
    uninitialized_plugins.clear();

    ROS_INFO("Listening for callbacks.");
    ros::spin();
    return 0;
}
