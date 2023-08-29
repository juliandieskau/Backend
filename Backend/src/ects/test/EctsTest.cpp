#include "ects/ECTS.hpp"
#include "ects/Configuration.hpp"
#include "ects/PluginLoader.hpp"
#include "ros/ros.h"
#include "gtest/gtest.h"

TEST(EctsCore, config) {
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

TEST(EctsCore, config_load_error) {
    auto config_opt = ects::Configuration::load_configuration("/non/existent");
    ASSERT_FALSE(config_opt.has_value());

    std::string configFilePath = std::tmpnam(nullptr);
    {
        std::ofstream configFile(configFilePath);
        configFile << "invalid json";
    }
    config_opt = ects::Configuration::load_configuration(configFilePath);
    ASSERT_FALSE(config_opt.has_value());
}

TEST(EctsCore, pluginloader) {
    // NOTE: The core plugin should always be built, so it can be used to test
    ects::PluginLoader loader;
    auto res = loader.load("core");
    ASSERT_NE(res, nullptr);
    ASSERT_EQ(res->name(), "core");

    res = loader.load("nonexistent");
    ASSERT_EQ(res, nullptr);
}

auto main(int argc, char **argv) -> int {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
