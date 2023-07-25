#include "battery_monitor.hpp"

namespace ects::plugins::battery {

auto battery_monitor::init(ECTS *ects) -> void {
  auto robot_battery_topic =
      ects->getConfig().get_value<std::string>("/battery/topic");
  data = {
      std::nullopt,
      ects->getRosIf().create_subscriber<battery_state>(robot_battery_topic),
      ects->getRosIf().create_publisher<battery_state>(battery_state_topic),
      ects->getRosIf().create_publisher<charge_percentage>(percentage_topic),
      ects->getRosIf().create_publisher<estimated_runtime>(runtime_topic),
      ects->getRosIf().create_publisher<warning>(warning_topic),
  };
  data->battery_subscriber.subscribe([this](battery_state state) {
    data->state = state;
    transmit_all();
  });
}

auto battery_monitor::transmit_all() -> void {
  if (data->state.has_value()) {
    publish_battery_state();
    publish_percentage();
    publish_runtime();
    publish_warning();
  }
}

auto battery_monitor::transmit(std::string &topic_name) -> void {
  if (data->state.has_value()) {
    if (topic_name == battery_state_topic)
      publish_battery_state();
    if (topic_name == percentage_topic)
      publish_percentage();
    if (topic_name == runtime_topic)
      publish_runtime();
    if (topic_name == warning_topic)
      publish_warning();
  }
}

auto battery_monitor::publish_battery_state() -> void {
  data->battery_state_publisher.publish(*data->state);
}

auto battery_monitor::publish_percentage() -> void {
  data->percentage_publisher.publish(data->state->get_charge());
}

auto battery_monitor::publish_runtime() -> void {
  data->runtime_publisher.publish(data->state->get_estimated_runtime());
}

auto battery_monitor::publish_warning() -> void {
  data->warning_publisher.publish(data->state->is_critical());
}

} // namespace ects::plugins::battery
