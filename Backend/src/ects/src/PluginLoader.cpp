#include "ects/PluginLoader.hpp"
#include "BatteryMonitor.hpp"
#include "PluginCore.hpp"
#include "WaypointManager.hpp"
#include <dlfcn.h>

namespace ects {
typedef Plugin *(*plugin_creator)();
auto PluginLoader::load(std::string &plugin_name) -> Plugin * {
    auto plugin_so_name = "libects_plugin_" + plugin_name + ".so";
    void *so = dlopen(plugin_so_name.c_str(), RTLD_NOW);
    if (so == nullptr) {
        ROS_ERROR_STREAM("Could not load plugin " << plugin_name << ": "
                                                  << dlerror());
        return nullptr;
    }
    plugin_creator create_plugin =
        (plugin_creator)dlsym(so, "create_plugin_instance");
    if (create_plugin == nullptr) {
        ROS_ERROR_STREAM("Could not load plugin " << plugin_name << ": "
                                                  << dlerror());
        return nullptr;
    }
    return create_plugin();
};
} // namespace ects
