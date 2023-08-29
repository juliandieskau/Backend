#include "../plugins/battery/BatteryMessages.hpp"
#include "TestUtil.hpp"
#include "ects/ECTS.hpp"
#include "ects/ForceRetransmit.h"
#include "iosb_localization_filter/FilterState.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "gtest/gtest.h"
#include <chrono>
#include <thread>

static const auto position_config = "{\n"
                                    "  \"core\": {\n"
                                    "    \"robot_name\": \"position_test\",\n"
                                    "    \"load_plugins\": [\n"
                                    "      \"position\"\n"
                                    "    ]\n"
                                    "  },\n"
                                    "  \"position\": {\n"
                                    "    \"odometry_topic\": \"/test/o\",\n"
                                    "    \"imu_topic\": \"/test/i\",\n"
                                    "    \"status_topic\": \"/test/s\"\n"
                                    "  }\n"
                                    "}\n";

static bool recv_imu = false, recv_odom = false, recv_status = false;

TEST(EctsPlugins, position) {
    auto ects = ects_with_config(position_config);
    load_test_plugin(ects, "position");

    ros::NodeHandle nh;
    auto pub_imu = nh.advertise<sensor_msgs::Imu>("/test/i", 0);
    auto pub_odom = nh.advertise<nav_msgs::Odometry>("/test/o", 0);
    auto pub_status =
        nh.advertise<iosb_localization_filter::FilterState>("/test/s", 0);

    sensor_msgs::Imu imu;
    imu.orientation.x = 0.69;
    auto sub_imu = nh.subscribe<sensor_msgs::Imu>(
        "/ects/imu/current", 0, [&](const sensor_msgs::Imu::ConstPtr &msg) {
            recv_imu = true;
            EXPECT_NEAR(msg->orientation.x, 0.69, 0.01);
        });

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = 39.0;
    auto sub_odom = nh.subscribe<nav_msgs::Odometry>(
        "/ects/control/position", 0,
        [&](const nav_msgs::Odometry::ConstPtr &msg) {
            ("got odom");
            recv_odom = true;
            EXPECT_NEAR(msg->pose.pose.position.x, 39.0, 0.01);
        });

    iosb_localization_filter::FilterState status;
    status.gps_active = true;
    auto sub_status = nh.subscribe<iosb_localization_filter::FilterState>(
        "/ects/control/filter_state", 0,
        [&](const iosb_localization_filter::FilterState::ConstPtr &msg) {
            recv_status = true;
            EXPECT_TRUE(msg->gps_active);
        });

    pub_imu.publish(imu);
    pub_odom.publish(odom);
    pub_status.publish(status);

    auto recv_all = [&]() { return recv_imu && recv_odom && recv_status; };

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
    recv_imu = false;
    test_topic_retransmit("/ects/imu/current");

    recv_odom = false;
    test_topic_retransmit("/ects/control/position");

    recv_status = false;
    test_topic_retransmit("/ects/control/filter_state");

    recv_imu = false;
    recv_odom = false;
    recv_status = false;
    test_topic_retransmit("");
}
