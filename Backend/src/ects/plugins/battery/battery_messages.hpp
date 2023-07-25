#pragma once
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/BatteryState.h"
#include "spot_msgs/BatteryStateArray.h"
#include <chrono>

namespace ects::plugins::battery {

struct charge_percentage {
    using to_ros_t = std_msgs::Float32;

    static to_ros_t to_ros(charge_percentage);

    double charge;
};

struct estimated_runtime {
    using to_ros_t = std_msgs::Float32;

    static to_ros_t to_ros(estimated_runtime);

    std::chrono::duration<double> duration;
};

struct warning {
    using to_ros_t = std_msgs::Bool;

    static to_ros_t to_ros(warning);

    bool is_critical;
};

struct battery_state {
    using from_ros_t = spot_msgs::BatteryStateArray;
    using to_ros_t = sensor_msgs::BatteryState;

    charge_percentage get_charge();
    estimated_runtime get_estimated_runtime();
    warning is_critical();

    static battery_state from_ros(from_ros_t ros_input);
    static to_ros_t to_ros(battery_state);
private:
    charge_percentage charge;
    estimated_runtime runtime;
    double average_voltage;
    std::vector<double> cell_currents;
    std::vector<double> cell_temperatures;
};

}
