#pragma once
/* ECTS - Plugin.hpp
 * Plugin is the interface all plugins must implement.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

#include <string>

namespace ects {
class ECTS;

class Plugin {
  public:
    virtual auto init(ECTS &ects) -> void = 0;
    virtual auto transmit_all() -> void = 0;
    virtual auto transmit(const std::string &topic_name) -> void = 0;
    virtual auto name() const -> const std::string = 0;
    virtual ~Plugin() = default;
};
} // namespace ects
