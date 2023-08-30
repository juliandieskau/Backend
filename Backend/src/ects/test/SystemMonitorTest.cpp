#include "../plugins/systemmonitor/aggregations/Aggregation.hpp"
#include "../plugins/systemmonitor/cpu/Cpu.hpp"
#include "../plugins/systemmonitor/disk/Disk.hpp"
#include "../plugins/systemmonitor/memory/Memory.hpp"
#include "../plugins/systemmonitor/network/Network.hpp"
#include "../plugins/systemmonitor/programs/Programs.hpp"
#include "TestUtil.hpp"
#include "ects/AdapterList.h"
#include "ects/AggregationList.h"
#include "ects/CpuUsage.h"
#include "ects/ForceRetransmit.h"
#include "ects/MountpointList.h"
#include "gtest/gtest.h"
#include <chrono>
#include <nlohmann/json.hpp>
#include <thread>

using json = nlohmann::json;
using namespace ects::plugins::systemmonitor;

template <typename T, typename P> auto is(P &d) -> bool {
    return dynamic_cast<T *>(d.get());
}

TEST(SystemMonitor, aggregations) {
    json aggregation_json = {{{"name", "second"},
                              {"type", "interval"},
                              {"keep_count", 60},
                              {"interval", 1.0}},
                             {{"name", "minute"},
                              {"type", "interval"},
                              {"keep_count", 240},
                              {"interval", 60.0}},
                             {{"name", "smoothed"},
                              {"type", "readings"},
                              {"keep_count", 100},
                              {"readings", 5}}};
    auto aggregations = std::vector<std::unique_ptr<AggregationStrategy>>();
    for (auto &entry : aggregation_json)
        ASSERT_NO_THROW(aggregations.push_back(aggregation_from_json(entry)));

    ASSERT_EQ(aggregations[0]->get_name(), "second");
    ASSERT_EQ(aggregations[0]->get_keep_count(), 60);
    ASSERT_TRUE(is<IntervalAggregationStrategy>(aggregations[0]));
    ASSERT_FALSE(is<ReadingsAggregationStrategy>(aggregations[0]));
    ASSERT_EQ(aggregations[0]->to_ros().interval, 1.0);

    ASSERT_EQ(aggregations[1]->get_name(), "minute");
    ASSERT_EQ(aggregations[1]->get_keep_count(), 240);
    ASSERT_TRUE(is<IntervalAggregationStrategy>(aggregations[1]));
    ASSERT_EQ(aggregations[1]->to_ros().interval, 60.0);

    ASSERT_EQ(aggregations[2]->get_name(), "smoothed");
    ASSERT_EQ(aggregations[2]->get_keep_count(), 100);
    ASSERT_TRUE(is<ReadingsAggregationStrategy>(aggregations[2]));
    ASSERT_EQ(aggregations[2]->to_ros().nreadings, 5);

    json bad_aggregation = {{"name", "minute"},
                            {"type", "invalid"},
                            {"keep_count", 240},
                            {"interval", 60.0}};
    ASSERT_ANY_THROW(aggregation_from_json(bad_aggregation));

    struct Usage : UsageData {};

    Usage u1, u2;
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    Usage u3, u4, u5;

    auto a1 = aggregations[0]->make_aggregator();
    ASSERT_FALSE(a1->new_data(&u1));
    ASSERT_FALSE(a1->new_data(&u2));
    ASSERT_TRUE(a1->new_data(&u3));
    ASSERT_FALSE(a1->new_data(&u4));
    ASSERT_FALSE(a1->new_data(&u5));

    auto a2 = aggregations[1]->make_aggregator();
    ASSERT_FALSE(a2->new_data(&u1));
    ASSERT_FALSE(a2->new_data(&u2));
    ASSERT_FALSE(a2->new_data(&u3));
    ASSERT_FALSE(a2->new_data(&u4));
    ASSERT_FALSE(a2->new_data(&u5));

    auto a3 = aggregations[2]->make_aggregator();
    for (int i = 0; i < 3; ++i) {
        ASSERT_FALSE(a3->new_data(&u1));
        ASSERT_FALSE(a3->new_data(&u2));
        ASSERT_FALSE(a3->new_data(&u3));
        ASSERT_FALSE(a3->new_data(&u4));
        ASSERT_TRUE(a3->new_data(&u5));
    }
}

TEST(SystemMonitor, cpu) {
    auto cpu = ects::plugins::systemmonitor::Cpu();
    ASSERT_NO_THROW(cpu.get_usage());
    ros::WallDuration(0.1).sleep(); // cpu needs two samples to calculate usage
    auto usage = cpu.get_usage();
    ASSERT_GE(usage.get_cpu_percentage().get_value(), 0.0);
}

TEST(SystemMonitor, disk) {
    auto mountpoints =
        ects::plugins::systemmonitor::Disk::get_mountpoints().get_mountpoints();
    ASSERT_FALSE(mountpoints.empty());
    for (auto &mountpoint : mountpoints) {
        ASSERT_NO_THROW(
            ects::plugins::systemmonitor::Disk::get_usage(mountpoint));
    }
}

TEST(SystemMonitor, memory) {
    auto memory = ects::plugins::systemmonitor::Memory();
    ASSERT_NO_THROW(memory.get_usage());
}

TEST(SystemMonitor, network) {
    auto network = ects::plugins::systemmonitor::Network();
    auto adapters = network.get_adapters();
    ASSERT_FALSE(adapters.empty());
    for (auto &adapter : adapters) {
        ASSERT_NO_THROW(network.get_usage(adapter));
        ASSERT_NO_THROW(network.get_info(adapter));
    }
}

TEST(SystemMonitor, programs) {
    auto programs = ects::plugins::systemmonitor::Programs();
    ASSERT_NO_THROW(programs.get_usage());
    auto usage = programs.get_usage();
    ASSERT_GT(usage.get_total(), 1);
}

static const auto systemmonitor_config =
    "{\n"
    "  \"core\": {\n"
    "    \"robot_name\": \"systemmon_test\",\n"
    "    \"load_plugins\": [\n"
    "      \"systemmonitor\"\n"
    "    ]\n"
    "  },\n"
    "  \"systemmonitor\": {\n"
    "    \"update_interval\": 0.1\n"
    "  },\n"
    "  \"aggregations\": [\n"
    "    {\n"
    "      \"name\": \"ta\",\n"
    "      \"type\": \"readings\",\n"
    "      \"keep_count\": 2,\n"
    "      \"readings\": 2\n"
    "    }\n"
    "  ]\n"
    "}\n";

static bool recv_cpu = false, recv_cpu_agg = false, recv_agg = false;
TEST(EctsPlugins, systemmonitor_cpu) {
    auto ects = ects_with_config(systemmonitor_config);
    load_test_plugin(ects, "systemmonitor");
    ros::NodeHandle nh;
    auto pub_retransmit =
        nh.advertise<ects::ForceRetransmit>("/ects/retransmit", 0);
    auto retransmit_topic = [&](const std::string &topic) {
        ects::ForceRetransmit retransmit;
        retransmit.topic = topic;
        retransmit.reload_all = topic.empty();
        pub_retransmit.publish(retransmit);
    };
    {
        auto cpu_sub = nh.subscribe<ects::CpuUsage>(
            "/ects/system/cpu/usage", 0,
            [&](const ects::CpuUsage::ConstPtr &msg) {
                ROS_INFO("got cpu");
                EXPECT_GT(msg->total_usage, 0.0);
                recv_cpu = true;
            });
        auto cpu_agg_sub = nh.subscribe<ects::CpuUsageHistory>(
            "/ects/system/averages/ta/cpu/usage", 0,
            [&](const ects::CpuUsageHistory::ConstPtr &msg) {
                ROS_INFO("got cpu agg");
                if (!msg->measurements.empty()) {
                    EXPECT_GT(msg->measurements[0].total_usage, 0.0);
                }
                EXPECT_EQ(msg->aggregation.ectsname, "ta");
                recv_cpu_agg = true;
            });
        auto recv_all_cpu = [&]() { return recv_cpu && recv_cpu_agg; };

        spin_predicate(recv_all_cpu, 1000);
        EXPECT_TRUE(recv_all_cpu());
        auto reset_recv = [&]() {
            recv_cpu = false;
            recv_cpu_agg = false;
        };
        reset_recv();
        spin_predicate(recv_all_cpu, 50);
        EXPECT_FALSE(recv_all_cpu());

        reset_recv();
        ROS_INFO("retransmitting");
        retransmit_topic("/ects/system/cpu/usage");
        retransmit_topic("/ects/system/averages/ta/cpu/usage");
        spin_predicate(recv_all_cpu, 50);
        EXPECT_TRUE(recv_all_cpu());

        reset_recv();
        retransmit_topic("");
        spin_predicate(recv_all_cpu, 50);
        EXPECT_TRUE(recv_all_cpu());
    }
    {
        volatile bool recv_mp = false;
        std::vector<std::string> rosnames;
        std::thread mountpoints([&]() {
            ros::NodeHandle nh;
            auto mountpoint_caller = nh.serviceClient<ects::MountpointList>(
                "/ects/system/disk/mountpoints");
            auto mountpoints = ects::MountpointList();
            ASSERT_TRUE(mountpoint_caller.call(mountpoints));
            ASSERT_FALSE(mountpoints.response.rosname.empty());
            recv_mp = true;
            rosnames = std::vector(mountpoints.response.rosname.begin(),
                                   mountpoints.response.rosname.end());
        });
        spin_predicate([&]() { return recv_mp; }, 1000);
        mountpoints.join();
        EXPECT_TRUE(recv_mp);
        ASSERT_FALSE(rosnames.empty());

        for (auto rn : rosnames) {
            auto disk_sub = nh.subscribe<ects::DiskUsage>(
                "/ects/system/disk/" + rn + "/usage", 0,
                [&](const ects::DiskUsage::ConstPtr &msg) {
                    ROS_INFO_STREAM("got disk: " << rn);
                    EXPECT_GT(msg->size_total, 0);
                    EXPECT_GT(msg->used, 0);
                    recv_cpu = true;
                });
            auto disk_agg_sub = nh.subscribe<ects::DiskUsageHistory>(
                "/ects/system/averages/ta/disk/" + rn + "/usage", 0,
                [&](const ects::DiskUsageHistory::ConstPtr &msg) {
                    ROS_INFO_STREAM("got disk agg: " << rn);
                    if (!msg->measurements.empty()) {
                        EXPECT_GT(msg->measurements[0].size_total, 0);
                        EXPECT_GT(msg->measurements[0].used, 0);
                    }
                    EXPECT_EQ(msg->aggregation.ectsname, "ta");
                    recv_cpu_agg = true;
                });
            auto recv_all_disk = [&]() { return recv_cpu && recv_cpu_agg; };
            spin_predicate(recv_all_disk, 1000);
            EXPECT_TRUE(recv_all_disk());
            auto reset_recv = [&]() {
                recv_cpu = false;
                recv_cpu_agg = false;
            };
            reset_recv();
            retransmit_topic("/ects/system/disk/" + rn + "/usage");
            retransmit_topic("/ects/system/averages/ta/disk/" + rn + "/usage");
            spin_predicate(recv_all_disk, 50);
            EXPECT_TRUE(recv_all_disk());

            reset_recv();
            retransmit_topic("");
            spin_predicate(recv_all_disk, 50);
            EXPECT_TRUE(recv_all_disk());
        }
    }

    {
        std::thread call_thread([&]() {
            ros::NodeHandle nh;
            auto agg_caller = nh.serviceClient<ects::AggregationList>(
                "/ects/system/aggregation");
            auto agg = ects::AggregationList();
            ASSERT_TRUE(agg_caller.call(agg));
            recv_agg = true;
            ASSERT_EQ(agg.response.available_aggregations.size(), 1);
            ASSERT_EQ(agg.response.available_aggregations[0].ectsname, "ta");
        });
        spin_predicate([&]() { return recv_agg; }, 1000);
        call_thread.join();
        EXPECT_TRUE(recv_agg);
    }
}
