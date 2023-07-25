#pragma once

#include <string>
#include <functional>
#include <utility>
#include "message_interface.h"
#include "ros/ros.h"

namespace ects {

namespace detail {

    template<typename T>
    auto try_conversion(std::function<T()> f, const std::string& name) -> std::optional<T> {
        try {
            return f();
        } catch (const std::exception& e) {
            ROS_WARN_STREAM("ros conversion on '" << name << "' failed: " << e.what());
        }
        return std::nullopt;
    }

}

struct ros_node;

template<from_ros_message internal_t>
struct subscriber {
    void subscribe(std::function<void(internal_t)> callback) {
        if (m_subscriber.has_value()) {
            ROS_ERROR("subscriber: subscribing multiple times on this subscriber (for '%s') not supported", m_topic_name.c_str());
            return;
        }
        auto internal_callback = [=, name = m_topic_name](const internal_t::from_ros_t::ConstPtr& ros_input) {
            auto r = detail::try_conversion<internal_t>([&]{ return internal_t::from_ros(*ros_input); }, name);
            if (r) callback(*r);
        };
        m_subscriber = m_handle.subscribe<typename internal_t::from_ros_t>(m_topic_name, 1000, internal_callback);
    }

private:
    subscriber(std::string topic_name, ros::NodeHandle handle)
            : m_topic_name(std::move(topic_name)), m_handle(handle), m_subscriber(std::nullopt) {

    }
    friend ros_node;

    std::string m_topic_name;
    ros::NodeHandle m_handle;
    std::optional<ros::Subscriber> m_subscriber;
};

template<to_ros_message internal_t>
struct publisher {
    void publish(internal_t internal_input) {
        auto r = detail::try_conversion<typename internal_t::to_ros_t>(
                [&]{ return internal_t::to_ros(internal_input); }, m_topic_name);
        if (r) m_publisher.publish(*r);
    }
private:
    publisher(std::string topic_name, ros::NodeHandle handle)
            : m_topic_name(std::move(topic_name)),
            m_publisher(handle.advertise<typename internal_t::to_ros_t>(m_topic_name, 1000)) {
    }
    friend ros_node;

    std::string m_topic_name;
    ros::Publisher m_publisher;
};

template<server_messages internal_t>
struct server {
    using ros_t = server_traits<internal_t>::ros_t;
    using request_from_ros = server_traits<internal_t>::request_from_ros_t;
    using response_to_ros = server_traits<internal_t>::response_to_ros_t;

    void register_service(std::function<response_to_ros(request_from_ros)> callback) {
        if (m_server.has_value()) {
            ROS_ERROR("server: server (for '%s') can only be registered once", m_service_name.c_str());
            return;
        }
        using from_ros_t = request_from_ros::from_ros_t;
        using to_ros_t = response_to_ros::to_ros_t;
        auto service = [=, name = m_service_name](from_ros_t& ros_input, to_ros_t& ros_output) -> bool {
            auto r = detail::try_conversion<request_from_ros>(
                    [&]() { return request_from_ros::from_ros(ros_input); }, name);
            if (!r) return false;
            auto internal_output = callback(*r);
            auto converted_output = detail::try_conversion<to_ros_t>(
                    [&]{ return response_to_ros::to_ros(internal_output); }, name);
            if (!converted_output) return false;
            ros_output = *converted_output;
            return true;
        };
        //without the boost function template argument deduction fails for all overloads
        const boost::function<bool(from_ros_t&, to_ros_t&)> s = service;
        m_server = m_handle.advertiseService(m_service_name, s);
    }
private:
    server(std::string service_name, ros::NodeHandle handle)
            : m_service_name(std::move(service_name)), m_handle(handle), m_server(std::nullopt) {
    }
    friend ros_node;

    std::string m_service_name;
    ros::NodeHandle m_handle;
    std::optional<ros::ServiceServer> m_server;
};

template<client_messages internal_t>
struct client {
    using ros_t = client_traits<internal_t>::ros_t;
    using request_to_ros = client_traits<internal_t>::request_to_ros_t;
    using response_from_ros = client_traits<internal_t>::response_from_ros_t;

    response_from_ros call_service(request_to_ros internal_input) {
        ros_t srv;
        srv.request = request_to_ros::to_ros(internal_input);
        if (!m_client.call(srv)) {
            ROS_ERROR("client: failed to call service '%s'", m_client.getService().c_str());
            throw std::runtime_error("failed to call service");
        }
        return response_from_ros::from_ros(srv.response);
    }
private:
    client(std::string service_name, ros::NodeHandle handle)
            : m_client(handle.serviceClient<ros_t>(service_name)) {
    }
    friend ros_node;

    ros::ServiceClient m_client;
};

struct ros_node {
    template<from_ros_message internal_t>
    subscriber<internal_t> create_subscriber(std::string topic_name) {
        return subscriber<internal_t>(topic_name, m_handle);
    }

    template<to_ros_message internal_t>
    publisher<internal_t> create_publisher(std::string topic_name) {
        return publisher<internal_t>(topic_name, m_handle);
    }

    template<server_messages internal_t>
    server<internal_t> create_server(std::string service_name) {
        return server<internal_t>(service_name, m_handle);
    }

    template<client_messages internal_t>
    client<internal_t> create_client(std::string service_name) {
        return client<internal_t>(service_name, m_handle);
    }

    ros_node() : m_handle() {
    }
private:
    ros::NodeHandle m_handle;
};


}
