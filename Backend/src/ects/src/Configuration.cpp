#include "ects/Configuration.hpp"
#include "nlohmann/json.hpp"
#include <iostream>
using json = nlohmann::json;

namespace ects {
Configuration::Configuration() {}
Configuration::~Configuration() {}
auto load_configuration() -> Configuration * {
  auto cfg = new Configuration();
  auto j3 = json::parse(R"({"testing": "json", "pi": 3.141})");
  std::cout << j3.dump() << std::endl;
  return cfg;
}

} // namespace ects