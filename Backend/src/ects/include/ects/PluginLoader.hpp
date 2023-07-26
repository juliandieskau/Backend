#pragma once
#include "Plugin.hpp"
#include <memory>

namespace ects {
class PluginLoader {
  public:
    auto load(std::string &plugin_name) -> std::unique_ptr<Plugin>;
};
} // namespace ects
