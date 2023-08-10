#include "BatteryMessages.hpp"
#include <algorithm>

namespace ects::plugins::battery {

auto BatteryState::from_ros(const BatteryState::from_ros_t &ros_input)
    -> BatteryState {
    if (ros_input.battery_states.empty())
        throw std::runtime_error("no batteries in battery state list");
    BatteryState s{};
    for (const auto &state : ros_input.battery_states) {
        s.charge.charge += state.charge_percentage;
        s.average_voltage += state.voltage;
        s.runtime.duration +=
            std::chrono::nanoseconds{state.estimated_runtime.toNSec()};
        s.cell_currents.push_back(state.current);
        if (!state.temperatures.empty()) {
            auto max_cell_temp = std::max_element(state.temperatures.begin(),
                                                  state.temperatures.end());
            s.cell_temperatures.push_back(*max_cell_temp);
        }
    }
    auto batteries = static_cast<double>(ros_input.battery_states.size());
    s.charge.charge /= batteries;
    s.average_voltage /= batteries;
    s.runtime.duration /= batteries;
    return s;
}

auto BatteryState::to_ros(const BatteryState &s) -> BatteryState::to_ros_t {
    auto fl = [](auto d) { return static_cast<float>(d); };
    auto nan = std::numeric_limits<float>::quiet_NaN();
    BatteryState::to_ros_t r{};
    r.charge = fl(s.charge.charge);
    for (double current : s.cell_currents)
        r.current += fl(current);
    r.current /= fl(s.cell_currents.size());
    r.voltage = fl(s.average_voltage);
    r.capacity = nan;
    for (auto temp : s.cell_temperatures) {
        r.cell_temperature.push_back(fl(temp));
        r.cell_voltage.push_back(nan);
    }
    r.design_capacity = nan;
    r.location = "";
    r.percentage = fl(s.charge.charge);
    r.power_supply_health = BatteryState::to_ros_t::POWER_SUPPLY_HEALTH_UNKNOWN;
    r.power_supply_status = BatteryState::to_ros_t::POWER_SUPPLY_STATUS_UNKNOWN;
    r.power_supply_technology =
        BatteryState::to_ros_t::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    r.present = true;
    r.serial_number = "";
    r.temperature = fl(*std::max_element(s.cell_temperatures.begin(),
                                         s.cell_temperatures.end()));
    return r;
}

auto BatteryState::get_charge() -> ChargePercentage { return charge; }

auto ChargePercentage::to_ros(const ChargePercentage &c)
    -> ChargePercentage::to_ros_t {
    ChargePercentage::to_ros_t r{};
    r.data = static_cast<float>(c.charge);
    return r;
}

auto BatteryState::get_estimated_runtime() -> EstimatedRuntime {
    return runtime;
}

auto EstimatedRuntime::to_ros(const EstimatedRuntime &er)
    -> EstimatedRuntime::to_ros_t {
    EstimatedRuntime::to_ros_t r{};
    r.data =
        std::chrono::duration_cast<std::chrono::duration<float>>(er.duration)
            .count();
    return r;
}

auto BatteryState::is_critical() -> Warning {
    return {charge.charge < BatteryState::critical_threshold};
}

auto Warning::to_ros(const Warning &w) -> Warning::to_ros_t {
    Warning::to_ros_t r{};
    r.data = w.is_critical;
    return r;
}
} // namespace ects::plugins::battery
