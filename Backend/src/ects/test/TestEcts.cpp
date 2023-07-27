#include "ects/Configuration.hpp"
#include "ects/ECTS.hpp"
#include "ects/PluginLoader.hpp"
#include "ros/ros.h"
#include "gtest/gtest.h"
TEST(EctsCore, loadConfig) {
    std::string configFilePath = std::tmpnam(nullptr);
    {
        std::ofstream configFile(configFilePath);
        configFile << "{\n"
                      "  \"core\": {\n"
                      "    \"robot_name\": \"test_robot\",\n"
                      "    \"load_plugins\": [\n"
                      "      \"core\"\n"
                      "    ]\n"
                      "  }\n"
                      "}\n";
    }
    auto config = ects::Configuration::load_configuration(configFilePath);
    ASSERT_TRUE(config.has_value());
    ASSERT_EQ(config->get_value<std::string>("/core/robot_name"), "test_robot");
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}