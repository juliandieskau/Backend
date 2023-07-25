#pragma once
#include "spot_msgs/BatteryStateArray.h"
#include <chrono>

namespace ects::plugins::battery {

struct charge_percentage {
    double charge;
};

struct estimated_runtime {
    std::chrono::duration<double> duration;
};

struct warning {
    bool is_critical;
};

struct battery_state {
    using from_ros_t = spot_msgs::BatteryStateArray;

    charge_percentage get_charge();
    estimated_runtime get_estimated_runtime();
    warning is_critical();

    static battery_state from_ros(from_ros_t ros_input);
private:
    charge_percentage charge;
    estimated_runtime runtime;
    double average_voltage;
    std::vector<double> cell_currents;
    std::vector<double> cell_temperatures;
};

}
