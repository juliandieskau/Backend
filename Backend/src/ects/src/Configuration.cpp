#include "ects/Configuration.hpp"

namespace ects {
auto Configuration::load_configuration(std::string path) -> Configuration * {
  json j;
  try {
    std::ifstream i(path);
    i >> j;
  } catch (std::exception &e) {
    std::cout << "Error loading configuration file: " << e.what() << std::endl;
    return nullptr;
  }
  auto cfg = new Configuration(j);
  return cfg;
}

} // namespace ects