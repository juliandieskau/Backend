#pragma once
/* ECTS - RosInterface.hpp
 * RosInterface provides the interface Plugins use to interact with ROS, in a
 * ROS-version-agnostic way. ROS Subscibers, Publishers, Services, Clients and
 * Forwarders are provided. These objects can be instantiated via the
 * RosInterface factory class.
 *
 * Forwarders, the only ECTS-specific term, are a convenient way to forward ROS
 * messages from one topic to another, without having to implement a
 * ECTS-message for the topic's type.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

#include "MessageInterface.hpp"
#include "ros/ros.h"
#include <functional>
#include <string>
#include <utility>

namespace ects {

namespace detail {

template <typename T>
auto try_conversion(std::function<T()> f, const std::string &name)
    -> std::optional<T> {
    try {
        return f();
    } catch (const std::exception &e) {
        ROS_WARN_STREAM("ros conversion on '" << name
                                              << "' failed: " << e.what());
    }
    return std::nullopt;
}
template <typename T>
inline auto try_service(std::function<T()> f, const std::string &name)
    -> std::optional<T> {
    try {
        return f();
    } catch (const std::exception &e) {
        ROS_WARN_STREAM(name << ": " << e.what());
    }
    return std::nullopt;
}
inline auto try_callback(std::function<void()> f, const std::string &name)
    -> void {
    try {
        f();
    } catch (const std::exception &e) {
        ROS_WARN_STREAM(name << ": " << e.what());
    }
}

} // namespace detail

struct RosNode;

template <from_ros_message internal_t> struct Subscriber {
    void subscribe(std::function<void(internal_t)> callback) {
        if (ros_subscriber.has_value()) {
            ROS_ERROR("subscriber: subscribing multiple times on this "
                      "subscriber (for '%s') not supported",
                      topic_name.c_str());
            return;
        }
        auto internal_callback =
            [=, name = topic_name](
                const typename internal_t::from_ros_t::ConstPtr &ros_input) {
                auto r = detail::try_conversion<internal_t>(
                    [&] { return internal_t::from_ros(*ros_input); }, name);
                if (r)
                    detail::try_callback([&] { callback(*r); }, name);
            };
        ros_subscriber = ros_handle.subscribe<typename internal_t::from_ros_t>(
            topic_name, 1000, internal_callback);
    }

  private:
    Subscriber(std::string topic_name, ros::NodeHandle handle)
        : topic_name(std::move(topic_name)), ros_handle(handle),
          ros_subscriber(std::nullopt) {}
    friend RosNode;

    std::string topic_name;
    ros::NodeHandle ros_handle;
    std::optional<ros::Subscriber> ros_subscriber;
};

template <to_ros_message internal_t> struct Publisher {
    void publish(const internal_t &internal_input) {
        auto r = detail::try_conversion<typename internal_t::to_ros_t>(
            [&] { return internal_t::to_ros(internal_input); }, topic_name);
        if (r)
            ros_publisher.publish(*r);
    }

  private:
    Publisher(std::string topic_name, ros::NodeHandle handle)
        : topic_name(std::move(topic_name)),
          ros_publisher(handle.advertise<typename internal_t::to_ros_t>(
              this->topic_name, 1000)) {}
    friend RosNode;

    std::string topic_name;
    ros::Publisher ros_publisher;
};

template <server_messages internal_t> struct Server {
    using ros_t = typename server_traits<internal_t>::ros_t;
    using request_from_ros = typename server_traits<internal_t>::request_from_ros_t;
    using response_to_ros = typename server_traits<internal_t>::response_to_ros_t;

    void register_service(
        std::function<response_to_ros(request_from_ros)> callback) {
        if (ros_server.has_value()) {
            ROS_ERROR("Server: server (for '%s') can only be registered once",
                      service_name.c_str());
            return;
        }
        using from_ros_t = typename request_from_ros::from_ros_t;
        using to_ros_t = typename response_to_ros::to_ros_t;
        auto service = [=, name = service_name](from_ros_t &ros_input,
                                                to_ros_t &ros_output) -> bool {
            auto r = detail::try_conversion<request_from_ros>(
                [&]() { return request_from_ros::from_ros(ros_input); }, name);
            if (!r)
                return false;
            auto internal_output = detail::try_service<response_to_ros>(
                [&] { return callback(*r); }, name);
            if (!internal_output)
                return false;
            auto converted_output = detail::try_conversion<to_ros_t>(
                [&] { return response_to_ros::to_ros(*internal_output); },
                name);
            if (!converted_output)
                return false;
            ros_output = *converted_output;
            return true;
        };
        // without the boost function template argument deduction fails for all
        // overloads
        const boost::function<bool(from_ros_t &, to_ros_t &)> s = service;
        ros_server = ros_handle.advertiseService(service_name, s);
    }

  private:
    Server(std::string service_name, ros::NodeHandle handle)
        : service_name(std::move(service_name)), ros_handle(handle),
          ros_server(std::nullopt) {}
    friend RosNode;

    std::string service_name;
    ros::NodeHandle ros_handle;
    std::optional<ros::ServiceServer> ros_server;
};

template <client_messages internal_t> struct Client {
    using ros_t = typename client_traits<internal_t>::ros_t;
    using request_to_ros = typename client_traits<internal_t>::request_to_ros_t;
    using response_from_ros = typename client_traits<internal_t>::response_from_ros_t;

    response_from_ros call_service(request_to_ros internal_input) {
        ros_t srv;
        srv.request = request_to_ros::to_ros(internal_input);
        if (!ros_client.call(srv)) {
            ROS_ERROR("Client: failed to call service '%s'",
                      ros_client.getService().c_str());
            throw std::runtime_error("failed to call service");
        }
        return response_from_ros::from_ros(srv.response);
    }

  private:
    Client(std::string service_name, ros::NodeHandle handle)
        : ros_client(handle.serviceClient<ros_t>(service_name)) {}
    friend RosNode;

    ros::ServiceClient ros_client;
};

template <typename ros_t> struct TopicForwarder {
    virtual auto transmit_all() -> void {
        if (value->has_value())
            ros_publisher.publish(**value);
    }
    virtual auto transmit(const std::string &topic_name) -> void {
        if (ros_publisher.getTopic() == topic_name)
            transmit_all();
    }

  private:
    TopicForwarder(std::string from_topic, std::string to_topic,
                   ros::NodeHandle handle)
        : value(std::make_unique<std::optional<ros_t>>(std::nullopt)),
          ros_handle(handle),
          ros_publisher(handle.advertise<ros_t>(to_topic, 1000)),
          ros_subscriber(handle.subscribe<ros_t>(
              from_topic, 100,
              [ros_publisher = this->ros_publisher, val = this->value.get()](
                  const typename ros_t::ConstPtr &ros_input) {
                  ros_publisher.publish(*ros_input);
                  *val = *ros_input;
              })) {}
    std::unique_ptr<std::optional<ros_t>> value;
    ros::NodeHandle ros_handle;
    ros::Publisher ros_publisher;
    ros::Subscriber ros_subscriber;
    friend RosNode;
};

struct RosNode {
    template <from_ros_message internal_t>
    Subscriber<internal_t> create_subscriber(std::string topic_name) {
        ROS_INFO_STREAM("Creating subscriber for topic " << topic_name);
        return Subscriber<internal_t>(topic_name, ros_handle);
    }

    template <to_ros_message internal_t>
    Publisher<internal_t> create_publisher(std::string topic_name) {
        ROS_INFO_STREAM("Creating publisher for topic " << topic_name);
        return Publisher<internal_t>(topic_name, ros_handle);
    }

    template <server_messages internal_t>
    Server<internal_t> create_server(std::string service_name) {
        ROS_INFO_STREAM("Creating server for service " << service_name);
        return Server<internal_t>(service_name, ros_handle);
    }

    template <client_messages internal_t>
    Client<internal_t> create_client(std::string service_name) {
        ROS_INFO_STREAM("Creating client for service " << service_name);
        return Client<internal_t>(service_name, ros_handle);
    }

    template <typename ros_t>
    TopicForwarder<ros_t> create_topic_forwarder(std::string from_topic,
                                                 std::string to_topic) {
        ROS_INFO_STREAM("Creating topic forwarder from " << from_topic << " to "
                                                         << to_topic);
        return TopicForwarder<ros_t>(from_topic, to_topic, ros_handle);
    }

    RosNode() : ros_handle() {}

  private:
    ros::NodeHandle ros_handle;
};

} // namespace ects
