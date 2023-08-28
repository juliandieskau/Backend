#include "ects/Configuration.hpp"
#include "ects/ECTS.hpp"
#include "ects/PluginLoader.hpp"
#include "ros/ros.h"
#include "gtest/gtest.h"
#include "spot_msgs/BatteryStateArray.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/String.h"
#include "ros/duration.h"
#include <chrono>
#include <thread>

TEST(EctsBattery, all) {
    std::string configFilePath = std::tmpnam(nullptr);
    {
        std::ofstream configFile(configFilePath);
        configFile << "{\n"
                      "  \"core\": {\n"
                      "    \"robot_name\": \"battery_test\",\n"
                      "    \"load_plugins\": [\n"
                      "      \"battery\"\n"
                      "    ]\n"
                      "  },\n"
                      "  \"battery\": {\n"
                      "    \"battery_topic\": \"/test/battery_states\"\n"
                      "  }\n"
                      "}\n";
    }
    auto config = ects::Configuration::load_configuration(configFilePath).value();
    auto ros_interface = new ects::RosNode();
    auto timer_manager = new ects::TimerManager();
    auto ects = ects::ECTS(config, *ros_interface, *timer_manager);

    auto plugin_loader = ects::PluginLoader();
    auto battery = plugin_loader.load("battery");

    ASSERT_NO_THROW(battery->init(ects));

    ros::NodeHandle nh;
    auto recv_battery_state = false;
    nh.subscribe<sensor_msgs::BatteryState>(
        "/ects/battery/usage", 0,
        [&](const sensor_msgs::BatteryState::ConstPtr &msg) {
            recv_battery_state = true;
            EXPECT_EQ(msg->charge, 0.69);
            EXPECT_EQ(msg->voltage, 39.0);
            EXPECT_EQ(msg->current, 12.3);
            EXPECT_EQ(msg->temperature, 30.0);
            EXPECT_EQ(msg->percentage, 0.69);
            EXPECT_EQ(msg->cell_voltage.size(), 0);
            EXPECT_EQ(msg->cell_temperature.size(), 0);
            EXPECT_EQ(msg->location, "");
            EXPECT_EQ(msg->serial_number, "");
            EXPECT_EQ(msg->design_capacity, 0.0);
            EXPECT_EQ(msg->capacity, 0.0);
            EXPECT_EQ(msg->power_supply_health,
                      sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN);
            EXPECT_EQ(msg->power_supply_status,
                      sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN);
            EXPECT_EQ(msg->power_supply_technology,
                      sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN);
            EXPECT_EQ(msg->present, true);
            EXPECT_EQ(msg->cell_voltage.size(), 0);
            EXPECT_EQ(msg->cell_temperature.size(), 0);
            ROS_INFO("Received battery state");
        });

    auto publisher = nh.advertise<spot_msgs::BatteryStateArray>("/test/battery_states", 0);
    auto test_array = spot_msgs::BatteryStateArray();
    auto test_state = spot_msgs::BatteryState();
    test_state.charge_percentage = 0.69;
    test_state.status = 3; // DISCHARGING
    test_state.voltage = 39.0;
    test_state.current = 12.3;
    test_state.temperatures = {10,20,30};
    test_state.estimated_runtime = ros::Duration(600.0);
    test_array.battery_states.push_back(test_state);

    publisher.publish(test_array);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();
    EXPECT_TRUE(recv_battery_state);
}
