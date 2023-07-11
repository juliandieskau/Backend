/* ECTS - main.cpp
 * Entry point of Application
 */
#include <iostream>
#include "ects/Configuration.hpp"
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <limits.h>
#include <stdio.h>
#include <unistd.h>


auto main(int argc, char **argv) -> int {
    if (argc < 1) {
        ROS_ERROR_STREAM( "Usage: " << argv[0] << " [config-file]");
        return -1;
    }
    ROS_INFO_STREAM("Starting ECTS with config file " << argv[1]);
    auto config = ects::Configuration::load_configuration(argv[1]);
    if (config == nullptr) {
        ROS_FATAL("Can not continue without valid Configuration.");
        return -1;
    }
    ROS_INFO_STREAM("Running with config" << config->dump());

    ros::init(argc, argv, "ects");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);
    ROS_INFO("Looping");
    while(ros::ok()) {
        std_msgs::String msg;
        msg.data = "foo";

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
