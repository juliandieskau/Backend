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
    ASSERT_EQ(config->get_value_or_default<std::string>("/core/robot_name",
                                                        "default"),
              "test_robot");
    ASSERT_ANY_THROW(config->get_value<int>("/core/robot_name"))
        << "throw on wrong type";
    ASSERT_ANY_THROW(config->get_value<int>("/nonexistent"))
        << "throw on nonexistent key";
    ASSERT_EQ(config->get_value_or_default<int>("/nonexistent", -11), -11);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
