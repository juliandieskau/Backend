#include "../plugins/battery/BatteryMessages.hpp"
#include "TestUtil.hpp"
#include "ects/Configuration.hpp"
#include "ects/ECTS.hpp"
#include "ects/ForceRetransmit.h"
#include "ects/PluginLoader.hpp"
#include "ros/duration.h"
#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "spot_msgs/BatteryStateArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "gtest/gtest.h"
#include <chrono>
#include <thread>

const auto battery_config = "{\n"
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

auto recv_battery_state = false, recv_battery_percent = false,
     recv_battery_critical = false, recv_battery_remaining = false;

TEST(EctsPlugins, battery) {
    auto ects = ects_with_config(battery_config);
    load_test_plugin(ects, "battery");

    ros::NodeHandle nh;
    auto publisher =
        nh.advertise<spot_msgs::BatteryStateArray>("/test/battery_states", 0);
    auto sub = nh.subscribe<sensor_msgs::BatteryState>(
        "/ects/battery/usage", 0,
        [&](const sensor_msgs::BatteryState::ConstPtr &msg) {
            ROS_INFO_STREAM("got battery state");
            recv_battery_state = true;
            EXPECT_NEAR(msg->charge, 0.69, 0.01);
            EXPECT_NEAR(msg->voltage, 39.0, 0.01);
            EXPECT_NEAR(msg->current, 12.3, 0.01);
            EXPECT_NEAR(msg->temperature, 30.0, 0.01);
            EXPECT_NEAR(msg->percentage, 0.69, 0.01);
            EXPECT_EQ(msg->cell_voltage.size(), 1);
            // cell voltage is not known, as such is not tested.
            EXPECT_EQ(msg->cell_temperature.size(), 1);
            EXPECT_NEAR(msg->cell_temperature[0], 30.0, 0.01);
            EXPECT_EQ(msg->location, "");
            EXPECT_EQ(msg->serial_number, "");
            EXPECT_TRUE(msg->design_capacity != msg->design_capacity); // is NaN
            EXPECT_TRUE(msg->capacity != msg->capacity);               // is NaN
            EXPECT_EQ(msg->power_supply_health,
                      sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN);
            EXPECT_EQ(msg->power_supply_status,
                      sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN);
            EXPECT_EQ(
                msg->power_supply_technology,
                sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN);
            EXPECT_EQ(msg->present, true);
        });
    auto sub2 = nh.subscribe<std_msgs::Float32>(
        "/ects/battery/percentage", 0,
        [&](const std_msgs::Float32::ConstPtr &msg) {
            recv_battery_percent = true;
            EXPECT_NEAR(msg->data, 0.69, 0.01);
        });
    auto sub3 =
        nh.subscribe<std_msgs::Bool>("/ects/battery/is_critical", 0,
                                     [&](const std_msgs::Bool::ConstPtr &msg) {
                                         recv_battery_critical = true;
                                         EXPECT_EQ(msg->data, false);
                                     });
    auto sub4 = nh.subscribe<std_msgs::Float32>(
        "/ects/battery/estimated_time_remaining", 0,
        [&](const std_msgs::Float32::ConstPtr &msg) {
            recv_battery_remaining = true;
            EXPECT_NEAR(msg->data, 600.0, 0.01);
        });

    auto test_array = spot_msgs::BatteryStateArray();
    auto test_state = spot_msgs::BatteryState();
    test_state.charge_percentage = 0.69;
    test_state.status = 3; // DISCHARGING
    test_state.voltage = 39.0;
    test_state.current = 12.3;
    test_state.temperatures = {10, 20, 30};
    test_state.estimated_runtime = ros::Duration(600.0);
    test_array.battery_states.push_back(test_state);
    publisher.publish(test_array);

    auto recv_all = [&]() {
        return recv_battery_state && recv_battery_percent &&
               recv_battery_critical && recv_battery_remaining;
    };
    spin_predicate(recv_all, 1000);
    EXPECT_TRUE(recv_all());

    auto pub_retransmit =
        nh.advertise<ects::ForceRetransmit>("/ects/retransmit", 0);

    auto test_topic_retransmit = [&](std::string topic) {
        auto retransmit = ects::ForceRetransmit();
        retransmit.topic = topic;
        retransmit.reload_all = topic == "";
        pub_retransmit.publish(retransmit);
        spin_predicate(recv_all, 1000);
        EXPECT_TRUE(recv_all());
    };
    recv_battery_state = false;
    test_topic_retransmit("/ects/battery/usage");

    recv_battery_percent = false;
    test_topic_retransmit("/ects/battery/percentage");

    recv_battery_critical = false;
    test_topic_retransmit("/ects/battery/is_critical");

    recv_battery_remaining = false;
    test_topic_retransmit("/ects/battery/estimated_time_remaining");

    recv_battery_state = false;
    recv_battery_percent = false;
    recv_battery_critical = false;
    recv_battery_remaining = false;
    test_topic_retransmit("");
}

TEST(ECTSPlugins, battery_conversion) {
    // NOTE: correctness of conversion is tested above
    auto test_array = spot_msgs::BatteryStateArray();
    ASSERT_THROW(ects::plugins::battery::BatteryState::from_ros(test_array),
                 std::runtime_error);
    auto test_state = spot_msgs::BatteryState();
    test_state.charge_percentage = 0.69;
    test_array.battery_states.push_back(test_state);
    ASSERT_NO_THROW(ects::plugins::battery::BatteryState::from_ros(test_array));
    auto res = ects::plugins::battery::BatteryState::from_ros(test_array);
}
