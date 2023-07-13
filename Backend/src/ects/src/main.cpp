/* ECTS - main.cpp
 * Entry point of Application
 */
#include "ects/Configuration.hpp"
#include "ects/ECTS.hpp"
#include "ects/PluginLoader.hpp"
#include "ects/RosNode.hpp"
#include "ects/TimerManager.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <limits.h>
#include <sstream>
#include <stdio.h>
#include <unistd.h>

using namespace ects;
auto main(int argc, char **argv) -> int {
  if (argc < 1) {
    ROS_ERROR_STREAM("Usage: " << argv[0] << " [config-file]");
    return -1;
  }
  ROS_INFO_STREAM("Starting ECTS with config file " << argv[1]);
  auto config = Configuration::load_configuration(argv[1]);
  if (config == nullptr) {
    ROS_FATAL("Can not continue without valid Configuration.");
    return -1;
  }
  ROS_INFO_STREAM("Running with config" << config->dump());

  PluginLoader pluginLoader;
  for (auto plugin_json :
       config->get_value<json::array_t>("/core/load_plugins")) {
    auto plugin_name = plugin_json.get<std::string>();
    ROS_INFO_STREAM("Loading plugin " << plugin_name);
    auto plugin = pluginLoader.load(plugin_name);
    if (plugin == nullptr) {
      ROS_ERROR_STREAM("Could not load plugin " << plugin_name);
    }
  }

  ros::init(argc, argv, "ects");
  ROS_INFO("Initialized ROS node");
  //   while (ros::ok()) {
  //     ros::spinOnce();
  //     loop_rate.sleep();
  //   }
  return 0;
}
