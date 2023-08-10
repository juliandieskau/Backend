#include "ects/PluginLoader.hpp"
#include "ros/ros.h"
#include <dlfcn.h>

namespace ects {
using plugin_creator = Plugin *(*)();

auto PluginLoader::load(const std::string &plugin_name)
    -> std::unique_ptr<Plugin> {
    auto plugin_so_name = "libects_plugin_" + plugin_name + ".so";
    void *so = dlopen(plugin_so_name.c_str(), RTLD_NOW);
    plugin_creator create_plugin = nullptr;
    if (so != nullptr)
        create_plugin = (plugin_creator)dlsym(so, "create_plugin_instance");
    if (so == nullptr || create_plugin == nullptr) {
        ROS_ERROR_STREAM("Could not load plugin " << plugin_name << ": "
                                                  << dlerror());
        return nullptr;
    }
    return std::unique_ptr<Plugin>{create_plugin()};
}
} // namespace ects
