#pragma once

#include "MessageInterface.h"
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
                const internal_t::from_ros_t::ConstPtr &ros_input) {
                auto r = detail::try_conversion<internal_t>(
                    [&] { return internal_t::from_ros(*ros_input); }, name);
                if (r)
                    callback(*r);
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
    void publish(internal_t internal_input) {
        auto r = detail::try_conversion<typename internal_t::to_ros_t>(
            [&] { return internal_t::to_ros(internal_input); }, topic_name);
        if (r)
            ros_publisher.publish(*r);
    }

  private:
    Publisher(std::string topic_name, ros::NodeHandle handle)
        : topic_name(std::move(topic_name)),
          ros_publisher(handle.advertise<typename internal_t::to_ros_t>(
              topic_name, 1000)) {}
    friend RosNode;

    std::string topic_name;
    ros::Publisher ros_publisher;
};

template <server_messages internal_t> struct Server {
    using ros_t = server_traits<internal_t>::ros_t;
    using request_from_ros = server_traits<internal_t>::request_from_ros_t;
    using response_to_ros = server_traits<internal_t>::response_to_ros_t;

    void register_service(
        std::function<response_to_ros(request_from_ros)> callback) {
        if (ros_server.has_value()) {
            ROS_ERROR("Server: server (for '%s') can only be registered once",
                      service_name.c_str());
            return;
        }
        using from_ros_t = request_from_ros::from_ros_t;
        using to_ros_t = response_to_ros::to_ros_t;
        auto service = [=, name = service_name](from_ros_t &ros_input,
                                                to_ros_t &ros_output) -> bool {
            auto r = detail::try_conversion<request_from_ros>(
                [&]() { return request_from_ros::from_ros(ros_input); }, name);
            if (!r)
                return false;
            auto internal_output = callback(*r);
            auto converted_output = detail::try_conversion<to_ros_t>(
                [&] { return response_to_ros::to_ros(internal_output); }, name);
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
    using ros_t = client_traits<internal_t>::ros_t;
    using request_to_ros = client_traits<internal_t>::request_to_ros_t;
    using response_from_ros = client_traits<internal_t>::response_from_ros_t;

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

struct RosNode {
    template <from_ros_message internal_t>
    Subscriber<internal_t> create_subscriber(std::string topic_name) {
        return Subscriber<internal_t>(topic_name, ros_handle);
    }

    template <to_ros_message internal_t>
    Publisher<internal_t> create_publisher(std::string topic_name) {
        return Publisher<internal_t>(topic_name, ros_handle);
    }

    template <server_messages internal_t>
    Server<internal_t> create_server(std::string service_name) {
        return Server<internal_t>(service_name, ros_handle);
    }

    template <client_messages internal_t>
    Client<internal_t> create_client(std::string service_name) {
        return Client<internal_t>(service_name, ros_handle);
    }

    RosNode() : ros_handle() {}

  private:
    ros::NodeHandle ros_handle;
};

} // namespace ects
