#pragma once
/* ECTS - BatteryMessages.hpp
 * Messages used by the battery plugin.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */
#include "sensor_msgs/BatteryState.h"
#include "spot_msgs/BatteryStateArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <chrono>

namespace ects::plugins::battery {

struct ChargePercentage {
    using to_ros_t = std_msgs::Float32;

    static auto to_ros(const ChargePercentage &) -> to_ros_t;

    double charge;
};

struct EstimatedRuntime {
    using to_ros_t = std_msgs::Float32;

    static auto to_ros(const EstimatedRuntime &) -> to_ros_t;

    std::chrono::duration<double> duration;
};

struct Warning {
    using to_ros_t = std_msgs::Bool;

    static auto to_ros(const Warning &) -> to_ros_t;

    bool is_critical;
};

struct BatteryState {
    using from_ros_t = spot_msgs::BatteryStateArray;
    using to_ros_t = sensor_msgs::BatteryState;

    auto get_charge() -> ChargePercentage;
    auto get_estimated_runtime() -> EstimatedRuntime;
    auto is_critical() -> Warning;

    static auto from_ros(const from_ros_t &ros_input) -> BatteryState;
    static auto to_ros(const BatteryState &) -> to_ros_t;

  private:
    ChargePercentage charge;
    EstimatedRuntime runtime;
    double average_voltage;
    std::vector<double> cell_currents;
    std::vector<double> cell_temperatures;
    static constexpr auto critical_threshold = 0.15;
};

} // namespace ects::plugins::battery
