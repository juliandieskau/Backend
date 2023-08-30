#include "ects/RosInterface.hpp"
#include "TestUtil.hpp"
#include "ects/EmptyMessage.hpp"
#include "std_msgs/Float64.h"
#include "std_srvs/Trigger.h"
#include "gtest/gtest.h"
#include <thread>

using namespace ects;

struct srv {};

namespace ects {
template <> struct server_traits<srv> {
    using ros_t = std_srvs::Trigger;
    using request_from_ros_t = EmptyMessage<ros_t::Request>;
    using response_to_ros_t = EmptyMessage<ros_t::Response>;
};
template <> struct client_traits<srv> {
    using ros_t = std_srvs::Trigger;
    using request_to_ros_t = EmptyMessage<ros_t::Request>;
    using response_from_ros_t = EmptyMessage<ros_t::Response>;
};
} // namespace ects

TEST(RosInterface, register_twice) {
    RosNode ros;
    using msg = EmptyMessage<std_msgs::Float64>;
    auto sub = ros.create_subscriber<msg>("topic");
    sub.subscribe([](msg m) {});
    sub.subscribe([](msg m) {});

    auto serv = ros.create_server<srv>("service");
    auto callback = [](server_traits<srv>::request_from_ros_t r) {
        return server_traits<srv>::response_to_ros_t{};
    };
    serv.register_service(callback);
    serv.register_service(callback);
}

template <typename ros_t> struct ThrowingMessage {
    using from_ros_t = ros_t;
    using to_ros_t = ros_t;

    static auto from_ros(const from_ros_t &) -> ThrowingMessage<ros_t> {
        throw std::runtime_error("from ros throw");
    }
    static auto to_ros(const ThrowingMessage<ros_t> &) -> to_ros_t {
        throw std::runtime_error("to ros throw");
    }
};

TEST(RosInterface, throwing_functions) {
    RosNode ros;

    {
        using msg = ThrowingMessage<std_msgs::Float64>;
        auto pub = ros.create_publisher<msg>("topic");
        pub.publish(msg{});
    }
    using msg = EmptyMessage<std_msgs::Float64>;
    auto sub = ros.create_subscriber<msg>("topic");
    sub.subscribe(
        [](msg m) { throw std::runtime_error("subscriber callback throw"); });
    auto pub = ros.create_publisher<msg>("topic");
    pub.publish(msg{});

    ros::Duration(0.1).sleep();
    ros::spinOnce();

    auto serv = ros.create_server<srv>("service");
    auto throwing_callback = [](server_traits<srv>::request_from_ros_t r)
        -> server_traits<srv>::response_to_ros_t {
        throw std::runtime_error("server callback throw");
    };
    serv.register_service(throwing_callback);
    auto cli = ros.create_client<srv>("service");

    bool done = false;
    std::thread call_thread([&]() {
        ASSERT_ANY_THROW(
            cli.call_service(client_traits<srv>::request_to_ros_t{}));
        done = true;
    });

    spin_predicate([&] { return done; }, 1000);
    ASSERT_TRUE(done);
    call_thread.join();
}

TEST(RosInterface, server_client) {
    RosNode ros;

    auto serv = ros.create_server<srv>("service");
    auto callback = [](server_traits<srv>::request_from_ros_t r) {
        return server_traits<srv>::response_to_ros_t{};
    };
    serv.register_service(callback);
    auto cli = ros.create_client<srv>("service");

    bool done = false;
    std::thread call_thread([&]() {
        ASSERT_NO_THROW(
            cli.call_service(client_traits<srv>::request_to_ros_t{}));
        done = true;
    });

    spin_predicate([&] { return done; }, 1000);
    ASSERT_TRUE(done);
    call_thread.join();
}
