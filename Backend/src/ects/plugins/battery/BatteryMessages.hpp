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

    static to_ros_t to_ros(const ChargePercentage &);

    double charge;
};

struct EstimatedRuntime {
    using to_ros_t = std_msgs::Float32;

    static to_ros_t to_ros(const EstimatedRuntime &);

    std::chrono::duration<double> duration;
};

struct Warning {
    using to_ros_t = std_msgs::Bool;

    static to_ros_t to_ros(const Warning &);

    bool is_critical;
};

struct BatteryState {
    using from_ros_t = spot_msgs::BatteryStateArray;
    using to_ros_t = sensor_msgs::BatteryState;

    ChargePercentage get_charge();
    EstimatedRuntime get_estimated_runtime();
    Warning is_critical();

    static BatteryState from_ros(const from_ros_t &ros_input);
    static to_ros_t to_ros(const BatteryState &);

  private:
    ChargePercentage charge;
    EstimatedRuntime runtime;
    double average_voltage;
    std::vector<double> cell_currents;
    std::vector<double> cell_temperatures;
    static constexpr auto critical_threshold = 0.15;
};

} // namespace ects::plugins::battery
