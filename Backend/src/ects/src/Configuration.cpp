#include "ects/Configuration.hpp"

namespace ects {
auto Configuration::load_configuration(std::string path) -> Configuration * {
    json j;
    try {
        std::ifstream i(path);
        i >> j;
    } catch (std::exception &e) {
        ROS_ERROR("Error loading configuration file:");
        ROS_ERROR_STREAM(e.what());
        return nullptr;
    }
    auto cfg = new Configuration(j);
    return cfg;
}
} // namespace ects
