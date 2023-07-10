/* ECTS - main.cpp
 * Entry point of Application
 */
#include "ros/ros.h"

#include <iostream>

int main(int argc, char **argv) {
  std::cout << "Hello World!" << std::endl;
  ros::init(argc, argv, "ects");

  return 0;
}
