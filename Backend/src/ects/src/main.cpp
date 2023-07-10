/* ECTS - main.cpp
 * Entry point of Application
 */
#include "ects/Configuration.hpp"
#include "ros/ros.h"
#include <iostream>

int main(int argc, char **argv) {
  std::cout << "Starting ECTS" << std::endl;
  auto a = ects::load_configuration();
  ros::init(argc, argv, "ects");
  return 0;
}
