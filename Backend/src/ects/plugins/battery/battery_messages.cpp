#include "battery_messages.hpp"
#include <algorithm>

namespace ects::plugins::battery {

battery_state battery_state::from_ros(battery_state::from_ros_t ros_input) {
    if (ros_input.battery_states.empty())
        throw std::runtime_error("no batteries in battery state list");
    battery_state s{};
    for (const auto& state : ros_input.battery_states) {
        s.charge.charge += state.charge_percentage;
        s.average_voltage += state.voltage;
        s.runtime.duration += std::chrono::nanoseconds{state.estimated_runtime.toNSec()};
        s.cell_currents.push_back(state.current);
        if (!state.temperatures.empty()) {
            auto max_cell_temp = std::max_element(state.temperatures.begin(), state.temperatures.end());
            s.cell_temperatures.push_back(*max_cell_temp);
        }
    }
    auto batteries = static_cast<double>(ros_input.battery_states.size());
    s.charge.charge /= batteries;
    s.average_voltage /= batteries;
    s.runtime.duration /= batteries;
    return s;
}

charge_percentage battery_state::get_charge() {
    return charge;
}

estimated_runtime battery_state::get_estimated_runtime() {
    return runtime;
}

warning battery_state::is_critical() {
    return { charge.charge < 0.15 };
}
}
