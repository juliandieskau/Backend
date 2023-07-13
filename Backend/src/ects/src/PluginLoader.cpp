#include "ects/PluginLoader.hpp"
#include "PluginCore.hpp"

namespace ects {
auto PluginLoader::load(std::string &plugin_name) -> Plugin * {
  // TODO: Implement modular loading
  if (plugin_name == "core") {
    return new plugins::core::PluginCore();
  }
  return nullptr;
};

} // namespace ects