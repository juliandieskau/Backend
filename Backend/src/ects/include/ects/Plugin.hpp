#pragma once
#include <iostream>
namespace ects {
class Plugin {
public:
  virtual auto init() -> void = 0;
  virtual auto transmit_all() -> void = 0;
  virtual auto transmit(std::string &topic_name) -> void = 0;
};
}