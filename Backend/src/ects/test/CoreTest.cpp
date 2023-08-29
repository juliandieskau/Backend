#include "TestUtil.hpp"
#include "ects/Configuration.hpp"
#include "ects/ECTS.hpp"
#include "ects/ECTSStatus.h"
#include "gtest/gtest.h"
#include <thread>

static const auto core_config = "{\n"
                                "  \"core\": {\n"
                                "    \"robot_name\": \"core_test\",\n"
                                "    \"load_plugins\": [\n"
                                "      \"core\"\n"
                                "    ]\n"
                                "  }\n"
                                "}\n";
static bool recv_status = false;

TEST(EctsPlugins, core) {
    auto ects = ects_with_config(core_config);
    // NOTE: ects_with_config loads the core plugin, so we don't need to do it
    // load_test_plugin(ects, "control");

    std::thread call_thread([&]() {
        ros::NodeHandle nh;
        auto status_client =
            nh.serviceClient<ects::ECTSStatus>("/ects/ects_status");
        auto status = ects::ECTSStatus();
        ASSERT_TRUE(status_client.call(status));
        recv_status = true;
        ASSERT_EQ(status.response.robot_name, "core_test");
        ASSERT_EQ(status.response.plugins_loaded.size(), 1);
        ASSERT_EQ(status.response.plugins_loaded[0], "core");
    });

    spin_predicate([&]() { return recv_status; }, 1000);
    call_thread.join();
}
