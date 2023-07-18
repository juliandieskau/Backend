#include "ects/RosNode.hpp"

namespace ects {

template <typename MSGC>
auto RosNode::createSubscriber(std::string topic)
    -> std::unique_ptr<RosSubscriber<MSGC>> {
  return std::make_unique<RosSubscriber<MSGC>>(topic);
}
template <typename MSGC>
auto RosNode::createPublisher(std::string topic)
    -> std::unique_ptr<RosPublisher<MSGC>> {
  return std::make_unique<RosPublisher<MSGC>>(topic);
}

template <typename MSGC>
auto RosNode::createServer(std::string topic)
    -> std::unique_ptr<RosServer<MSGC>> {
  return std::make_unique<RosServer<MSGC>>(topic);
}

template <typename MSGC>
auto RosNode::createClient(std::string topic)
    -> std::unique_ptr<RosClient<MSGC>> {
  return std::make_unique<RosClient<MSGC>>(topic);
}

template <typename MSGC>
RosPublisher<MSGC>::RosPublisher(std::string topic) : m_topic(topic) {
  m_publisher = m_nodeHandle.advertise<MSGC::rosType>(topic, 1000);
}

template <typename MSGC>
RosSubscriber<MSGC>::RosSubscriber(std::string topic) : m_topic(topic) {
  m_subscriber =
      m_nodeHandle.subscribe(topic, 1000, &RosSubscriber::callback, this);
}

template <typename MSGC>
RosSubscriber<MSGC>::RosSubscriber(std::string topic) : m_topic(topic) {
  m_subscriber =
      m_nodeHandle.subscribe(topic, 1000, &RosSubscriber::callback, this);
}

template <typename MSGC> RosSubscriber<MSGC>::callback(MSGC::rosType message) {
  ROS_LOG_INFO("Received message");
}

} // namespace ects