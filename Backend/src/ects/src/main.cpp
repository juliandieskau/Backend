/* ECTS - main.cpp
 * Entry point of Application
 */
#include "ects/Configuration.hpp"
#include "ects/ECTS.hpp"
#include "ects/ForceRetransmit.h"
#include "ects/PluginLoader.hpp"
#include "ects/Timer.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ects/ros_interface.h"

#include <iostream>
#include <climits>
#include <sstream>
#include <cstdio>
#include <unistd.h>

using namespace ects;
auto main(int argc, char **argv) -> int {
  if (argc < 1) {
    ROS_ERROR_STREAM("Usage: " << argv[0] << " [config-file]");
    return -1;
  }
  ROS_INFO_STREAM("Starting ECTS with config file " << argv[1]);
  auto *config = Configuration::load_configuration(argv[1]);
  if (config == nullptr) {
    ROS_FATAL("Can not continue without valid Configuration.");
    return -1;
  }
  ROS_INFO_STREAM("Running with config" << config->dump());

  ros::init(argc, argv, "ects");
  ROS_INFO("Initialized ROS node");

  ros_node ros;
  auto *ects = new ECTS(config, ros, nullptr);

  PluginLoader pluginLoader;
  std::vector<Plugin *> plugins;
  for (auto plugin_json :
       config->get_value<json::array_t>("/core/load_plugins")) {
    auto plugin_name = plugin_json.get<std::string>();
    ROS_INFO_STREAM("Loading plugin " << plugin_name);
    auto plugin = pluginLoader.load(plugin_name);
    if (plugin == nullptr) {
      ROS_ERROR_STREAM("Could not load plugin " << plugin_name);
    }
    plugins.push_back(plugin);
  }

  for (auto plugin : plugins) {
    plugin->init(ects);
  }

  ROS_INFO("Listening for callbacks.");
  ros::spin();

  //   while (ros::ok()) {
  //     ros::spinOnce();
  //     loop_rate.sleep();
  //   }
  return 0;
}
