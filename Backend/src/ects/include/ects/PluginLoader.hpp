#pragma once
/* ECTS - PluginLoader.hpp
 * PluginLoader is a class that loads plugins from shared libraries.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

#include "Plugin.hpp"
#include <memory>

namespace ects {
class PluginLoader {
  public:
    auto load(const std::string &plugin_name) -> std::unique_ptr<Plugin>;
};
} // namespace ects
