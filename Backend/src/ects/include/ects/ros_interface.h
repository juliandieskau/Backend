#pragma once

#include <string>
#include <functional>
#include "message_interface.h"
#include "ros/ros.h"

namespace ects {

struct ros_node;

template<from_ros_message internal_t>
struct subscriber {
    void subscribe(std::function<void(internal_t)> callback) {
        if (m_callback.has_value()) {
            ROS_ERROR("subscriber: subscribing multiple times on this subscriber (for '%s') not supported", m_subscriber.getTopic().c_str());
            return;
        }
        m_callback = callback;
    }
    void unsubscribe() {
        m_callback.reset();
    }
private:
    subscriber(std::string topic_name, ros::NodeHandle handle) {
        std::function<void(const typename internal_t::from_ros_t::ConstPtr&)> callback = [this](const internal_t::from_ros_t::ConstPtr& ros_input) {
            ROS_INFO_STREAM("cal " << this->m_subscriber.getTopic() << " " << this->m_callback.has_value());
            if (!this->m_callback.has_value()) {
                ROS_WARN("subscriber: topic '%s' called, but no callback registered", m_subscriber.getTopic().c_str());
                return;
            }
            (*m_callback)(internal_t::from_ros(*ros_input));
        };
        m_subscriber = handle.subscribe<typename internal_t::from_ros_t>(topic_name, 1000, callback);
    }
    friend ros_node;
public://FIXME
    ros::Subscriber m_subscriber;
    std::optional<std::function<void(internal_t)>> m_callback;
};

template<to_ros_message internal_t>
struct publisher {
    void publish(internal_t internal_input) {
        m_publisher.publish(internal_t::to_ros(internal_input));
    }
private:
    publisher(std::string topic_name, ros::NodeHandle handle)
            : m_publisher(handle.advertise<typename internal_t::to_ros_t>(topic_name, 1000)) {
    }
    friend ros_node;

    ros::Publisher m_publisher;
};

template<server_messages internal_t>
struct server {
    using ros_t = server_traits<internal_t>::to_ros_t;
    using request_from_ros = server_traits<internal_t>::request_from_ros_t;
    using response_to_ros = server_traits<internal_t>::response_to_ros_t;

    void register_service(std::function<response_to_ros(request_from_ros)> callback) {
        if (m_callback.has_value()) {
            ROS_ERROR("server: server (for '%s') can only be registered once", m_server.getService().c_str());
            return;
        }
        m_callback = callback;
    }
    void unregister() {
        m_callback.reset();
    }
private:
    server(std::string service_name, ros::NodeHandle handle) {
        using from_ros_t = request_from_ros::from_ros_t;
        using to_ros_t = response_to_ros::to_ros_t;
        auto service = [this](from_ros_t ros_input) -> to_ros_t {
            if (!m_callback.has_value()) {
                ROS_WARN("server: service '%s' called, but no callback registered", m_server.getService().c_str());
                return;
            }
            return (*m_callback)(from_ros_t::from_ros(ros_input));
        };
        m_server = handle.advertiseService(service, service_name);
    }
    friend ros_node;

    ros::ServiceServer m_server;
    std::function<response_to_ros(request_from_ros)> m_callback;
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
