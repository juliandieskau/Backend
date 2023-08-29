#include "ects/Configuration.hpp"

namespace ects {
auto Configuration::load_configuration(std::string path)
    -> std::optional<Configuration> {
    json j;
    try {
        std::ifstream i(path);
        i >> j;
    } catch (std::exception &e) {
        ROS_ERROR("Error loading configuration file:");
        ROS_ERROR_STREAM(e.what());
        return std::nullopt;
    }
    return Configuration{j};
}
} // namespace ects
