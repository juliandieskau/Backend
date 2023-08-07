#pragma once
/* ECTS - Control.hpp
 * The control plugin allows the user to control the basic movement of the robot
 * with a joystick. (by sending Twist messages)
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

#include "ects/Common.hpp"
#include "ects/ECTS.hpp"
#include "ects/Plugin.hpp"

namespace ects::plugins::control {
using ects::common::Twist;

class Control : public Plugin {
  public:
    auto init(ECTS &) -> void override;
    auto transmit_all() -> void override;
    auto transmit(const std::string &topic_name) -> void override;
    auto name() const -> const std::string override { return "control"; }

  private:
    struct data {
        Subscriber<Twist> control_subscriber;
        Publisher<Twist> control_publisher;
    };
    std::optional<data> data;
    static constexpr auto command_topic = "/ects/control/cmd";
    static constexpr auto command_topic_key = "/control/command_topic";
};

} // namespace ects::plugins::control
