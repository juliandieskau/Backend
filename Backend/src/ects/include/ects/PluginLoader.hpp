#pragma once
#include "Plugin.hpp"

namespace ects {
class PluginLoader {
public:
  auto load(std::string &plugin_name) -> Plugin *;
};
} // namespace ects