#pragma once

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
