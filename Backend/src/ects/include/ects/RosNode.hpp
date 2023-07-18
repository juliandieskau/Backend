#pragma once
#include "ros/ros.h"
#include <functional>
#include <memory>
#include <string>

namespace ects {
template <typename ROSMSG, typename IMSG> class MessageConverter {
  static constexpr auto rosType = ROSMSG::type;
  static constexpr auto ectsType = IMSG::type;

  auto toRos(IMSG message) -> ROSMSG { return message.toRos(); }
  auto fromRos(ROSMSG message) -> IMSG { return IMSG::fromRos(message); }
};

template <typename MSGC> class RosSubscriber {
  auto subscribe(const std::function<void(int)> callback) -> void;
  ~RosSubscriber() = 0; // unsubscribe

private:
  RosSubscriber(const std::string topic);
  auto callback(MSGC::rosType message) -> void;
  ros::NodeHandle m_nodeHandle;
  ros::Subscriber m_subscriber;
  std::string m_topic;
};

template <typename MSGC> class RosPublisher {
  auto publish(MSG::ectsType message) -> void;

private:
  RosPublisher(std::string topic);
  ros::NodeHandle m_nodeHandle;
  ros::Publisher m_publisher;
  std::string m_topic;
};

template <typename MSGC> class RosServer {
  auto registerServiceHandler(void(std::function < MSGC::ectsType &) > callback)
      -> void;
  ~RosServer() = 0; // unadvertise
private:
  RosServer(std::string topic);
  std::string m_topic;
  ros::NodeHandle m_nodeHandle;
  ros::ServiceServer m_serviceServer;
};

template <typename MSG> class RosClient {
  auto call(MSG message) -> void = 0;

private:
  RosClient(std::string topic);
  std::string m_topic;
};

class RosNode {
  template <typename MSG> friend class RosSubscriber;
  template <typename MSG> friend class RosPublisher;
  template <typename MSG> friend class RosServer;
  template <typename MSG> friend class RosClient;

public:
  template <typename MSG>
  auto createSubscriber(std::string topic)
      -> std::unique_ptr<RosSubscriber<MSG>>;
  template <typename MSG>
  auto createPublisher(std::string topic) -> std::unique_ptr<RosPublisher<MSG>>;
  template <typename MSG>
  auto createServer(std::string topic) -> std::unique_ptr<RosServer<MSG>>;
  template <typename MSG>
  auto createClient(std::string topic) -> std::unique_ptr<RosClient<MSG>>;
};
} // namespace ects