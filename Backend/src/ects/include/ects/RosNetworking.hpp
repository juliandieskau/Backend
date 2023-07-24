#pragma once
// #include <functional>
// #include <memory>
// #include <string>

// namespace ects {

// template <typename M> class RosPublisher {};

// template <typename S> class RosServer {
// public:
//   // std::function<void(S::Request, S::Response)> m_handlerFn;
// };

// class RosInterface {
// public:
//   RosInterface() {}

//   auto subscribe(std::string topic,
//                  std::function<void(MessageConverter)> callback) -> void;
//   auto call(std::string topic, Message message, std::function<void(Message)>)
//       -> void;
//   template <typename S>
//   auto publishOn(std::string topic) -> std::unique_ptr<RosPublisher<S>>;
//   auto advertiseOn(std::string topic) -> std::unique_ptr<RosServer<Message>>;
// };
// } // namespace ects