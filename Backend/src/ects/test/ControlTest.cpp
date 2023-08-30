#include "TestUtil.hpp"
#include "ects/Configuration.hpp"
#include "ects/ForceRetransmit.h"
#include "geometry_msgs/Twist.h"
#include "gtest/gtest.h"
#include <thread>

static const auto control_config = "{\n"
                                   "  \"core\": {\n"
                                   "    \"robot_name\": \"battery_test\",\n"
                                   "    \"load_plugins\": [\n"
                                   "      \"control\"\n"
                                   "    ]\n"
                                   "  },\n"
                                   "  \"control\": {\n"
                                   "    \"command_topic\": \"/test/cmd\"\n"
                                   "  }\n"
                                   "}\n";

static bool has_recv_cmd = false;

TEST(EctsPlugins, control) {
    auto ects = ects_with_config(control_config);
    load_test_plugin(ects, "control");

    ros::NodeHandle nh;
    auto publisher = nh.advertise<geometry_msgs::Twist>("/ects/control/cmd", 0);
    auto sub = nh.subscribe<geometry_msgs::Twist>(
        "/test/cmd", 0, [&](const geometry_msgs::Twist::ConstPtr &msg) {
            has_recv_cmd = true;
            EXPECT_NEAR(msg->linear.x, 0.69, 0.01);
            EXPECT_NEAR(msg->linear.y, 39.0, 0.01);
            EXPECT_NEAR(msg->linear.z, 12.3, 0.01);
            EXPECT_NEAR(msg->angular.x, 30.0, 0.01);
            EXPECT_NEAR(msg->angular.y, 0.69, 0.01);
            EXPECT_NEAR(msg->angular.z, 39.0, 0.01);
        });

    auto cmd = geometry_msgs::Twist();
    cmd.linear.x = 0.69;
    cmd.linear.y = 39.0;
    cmd.linear.z = 12.3;
    cmd.angular.x = 30.0;
    cmd.angular.y = 0.69;
    cmd.angular.z = 39.0;
    publisher.publish(cmd);

    auto recv_cmd = [&]() { return has_recv_cmd; };
    spin_predicate(recv_cmd, 1000);
    EXPECT_TRUE(has_recv_cmd);

    auto pub_retransmit =
        nh.advertise<ects::ForceRetransmit>("/ects/retransmit", 0);

    // check that nothing will be retransmitted
    has_recv_cmd = false;
    auto retransmit = ects::ForceRetransmit();
    retransmit.topic = "/test/cmd";
    retransmit.reload_all = false;
    pub_retransmit.publish(retransmit);
    spin_predicate(recv_cmd, 100);

    EXPECT_FALSE(has_recv_cmd);

    retransmit.reload_all = true;
    pub_retransmit.publish(retransmit);
    spin_predicate(recv_cmd, 100);
    EXPECT_FALSE(has_recv_cmd);
}
