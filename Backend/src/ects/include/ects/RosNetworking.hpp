#pragma once
#include <functional>
#include <memory>
#include <string>

namespace ects {
class Message {};

template <typename M> class RosPublisher {};

template <typename M> class RosServer {};

class RosInterface {
public:
  RosInterface() {}

  auto subscribe(std::string topic, std::function<void(Message)> callback)
      -> void;
  auto call(std::string topic, Message message, std::function<void(Message)>)
      -> void;
  template <typename M>
  auto publishOn(std::string topic) -> std::unique_ptr<RosPublisher<M>>;
  auto advertiseOn(std::string topic) -> std::unique_ptr<RosServer<Message>>;
};
} // namespace ects