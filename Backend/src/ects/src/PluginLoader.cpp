#include "ects/PluginLoader.hpp"
#include "BatteryMonitor.hpp"
#include "PluginCore.hpp"
#include "WaypointManager.hpp"

namespace ects {
auto PluginLoader::load(std::string &plugin_name) -> Plugin * {
    // TODO: Implement modular loading
    if (plugin_name == "core")
        return new plugins::core::PluginCore();
    if (plugin_name == "battery")
        return new plugins::battery::BatteryMonitor();
    if (plugin_name == "waypoints")
        return new plugins::waypoints::WaypointManager();
    return nullptr;
};
} // namespace ects
