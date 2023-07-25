#pragma once
#include <string>
namespace ects {
class ECTS;
class Plugin {
  public:
    virtual auto init(ECTS *) -> void = 0;
    virtual auto transmit_all() -> void = 0;
    virtual auto transmit(std::string &topic_name) -> void = 0;
};
} // namespace ects
