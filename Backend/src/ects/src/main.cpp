/* ECTS - main.cpp
 * Entry point of Application
 */
#include "ects/Configuration.hpp"
#include "ros/ros.h"
#include <iostream>
#include <limits.h>
#include <stdio.h>
#include <unistd.h>

auto main(int argc, char **argv) -> int {
  if (argc < 1) {
    std::cout << "Usage: " << argv[0] << " [config-file]" << std::endl;
    return -1;
  }
  std::cout << "Starting ECTS: " << argv[0] << std::endl;
  char cwd[PATH_MAX];
  if (getcwd(cwd, sizeof(cwd)) == NULL) {
    return -1;
  }
  std::cout << "Current working dir: " << cwd << std::endl;
  auto a = ects::Configuration::load_configuration(argv[1]);
  if (a == nullptr) {
    std::cout << "Error loading configuration file. Can not continue."
              << std::endl;
    return -1;
  }
  std::cout << "Robot name: " << a->get_value<std::string>("/core/robot_name")
            << std::endl;
  ros::init(argc, argv, "ects");
  return 0;
}
