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
    auto CpuUsage =
        ects::plugins::systemmonitor::CpuUsageMessage::to_ros(usage);
    ASSERT_GE(CpuUsage.total_usage, 0.0);
    ASSERT_GT(CpuUsage.per_core_usage.size(), 0);
    for (auto &core : CpuUsage.per_core_usage) {
        ASSERT_GE(core, 0.0);
    }
    ASSERT_EQ(CpuUsage.load_averages.size(), 3);
    for (auto &avg : CpuUsage.load_averages) {
        ASSERT_GE(avg, 0.0);
    }
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
static bool recv_mem = false, recv_mem_agg = false;
static bool recv_mp = false;
static bool recv_adapters = false, recv_adapters_info = false,
            recv_adapter_usage = false, recv_adapters_agg = false;
static bool recv_programs = false, recv_programs_agg = false;
TEST(EctsPlugins, systemmonitor) {
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
                EXPECT_GT(msg->total_usage, 0.0);
                recv_cpu = true;
            });
        auto cpu_agg_sub = nh.subscribe<ects::CpuUsageHistory>(
            "/ects/system/averages/ta/cpu/usage", 0,
            [&](const ects::CpuUsageHistory::ConstPtr &msg) {
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
        spin_predicate(recv_all_cpu, 10);
        EXPECT_FALSE(recv_all_cpu());

        reset_recv();
        retransmit_topic("/ects/system/cpu/usage");
        retransmit_topic("/ects/system/averages/ta/cpu/usage");
        spin_predicate(recv_all_cpu, 100);
        EXPECT_TRUE(recv_all_cpu());

        reset_recv();
        retransmit_topic("");
        spin_predicate(recv_all_cpu, 100);
        EXPECT_TRUE(recv_all_cpu());
    }
    {
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
                    EXPECT_GT(msg->size_total, 0);
                    EXPECT_GT(msg->used, 0);
                    recv_cpu = true;
                });
            auto disk_agg_sub = nh.subscribe<ects::DiskUsageHistory>(
                "/ects/system/averages/ta/disk/" + rn + "/usage", 0,
                [&](const ects::DiskUsageHistory::ConstPtr &msg) {
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
            spin_predicate(recv_all_disk, 100);
            EXPECT_TRUE(recv_all_disk());

            reset_recv();
            retransmit_topic("");
            spin_predicate(recv_all_disk, 100);
            EXPECT_TRUE(recv_all_disk());
        }
    }
    {

        auto memory_sub = nh.subscribe<ects::MemoryUsage>(
            "/ects/system/mem/usage", 0,
            [&](const ects::MemoryUsage::ConstPtr &msg) {
                EXPECT_GT(msg->total, 0);
                EXPECT_GT(msg->used, 0);
                EXPECT_GE(msg->free, 0);
                EXPECT_GE(msg->shared, 0);
                EXPECT_GE(msg->buff_cache, 0);
                EXPECT_GE(msg->available, 0);
                EXPECT_GE(msg->swap_total, 0);
                EXPECT_GE(msg->swap_used, 0);
                EXPECT_GE(msg->swap_free, 0);
                recv_mem = true;
            });
        auto memory_agg_sub = nh.subscribe<ects::MemoryUsageHistory>(
            "/ects/system/averages/ta/mem/usage", 0,
            [&](const ects::MemoryUsageHistory::ConstPtr &msg) {
                if (!msg->measurements.empty()) {
                    EXPECT_GT(msg->measurements[0].total, 0);
                    EXPECT_GT(msg->measurements[0].used, 0);
                    EXPECT_GE(msg->measurements[0].free, 0);
                    EXPECT_GE(msg->measurements[0].shared, 0);
                    EXPECT_GE(msg->measurements[0].buff_cache, 0);
                    EXPECT_GE(msg->measurements[0].available, 0);
                    EXPECT_GE(msg->measurements[0].swap_total, 0);
                    EXPECT_GE(msg->measurements[0].swap_used, 0);
                    EXPECT_GE(msg->measurements[0].swap_free, 0);
                }
                EXPECT_EQ(msg->aggregation.ectsname, "ta");
                recv_mem_agg = true;
            });
        auto recv_all_mem = [&]() { return recv_mem && recv_mem_agg; };
        spin_predicate(recv_all_mem, 1000);
        EXPECT_TRUE(recv_all_mem());
        auto reset_recv = [&]() {
            recv_mem = false;
            recv_mem_agg = false;
        };
        reset_recv();
        retransmit_topic("/ects/system/mem/usage");
        retransmit_topic("/ects/system/averages/ta/mem/usage");
        spin_predicate(recv_all_mem, 100);
        EXPECT_TRUE(recv_all_mem());

        reset_recv();
        retransmit_topic("");
        spin_predicate(recv_all_mem, 100);
        EXPECT_TRUE(recv_all_mem());
    }
    {
        std::vector<std::string> adapter_names;
        std::thread adapter_thread([&]() {
            ros::NodeHandle nh;
            auto adapter_caller = nh.serviceClient<ects::AdapterList>(
                "/ects/system/network/adapters");
            auto adapters = ects::AdapterList();
            ASSERT_TRUE(adapter_caller.call(adapters));
            ASSERT_FALSE(adapters.response.adapters.empty());
            recv_adapters = true;
            adapter_names = std::vector(adapters.response.adapters.begin(),
                                        adapters.response.adapters.end());
        });
        spin_predicate([&]() { return recv_adapters; }, 1000);
        adapter_thread.join();
        EXPECT_TRUE(recv_adapters);

        for (auto adapter : adapter_names) {
            auto info_sub = nh.subscribe<ects::NetworkInfo>(
                "/ects/system/network/" + adapter + "/info", 0,
                [&](const ects::NetworkInfo::ConstPtr &msg) {
                    EXPECT_EQ(msg->interface_name, adapter);
                    recv_adapters_info = true;
                });
            auto usage_sub = nh.subscribe<ects::NetworkUsage>(
                "/ects/system/network/" + adapter + "/usage", 0,
                [&](const ects::NetworkUsage::ConstPtr &msg) {
                    EXPECT_GE(msg->wifi_signal_strength, 0);
                    // NOTE: we can't really test down and up speed here
                    recv_adapter_usage = true;
                });
            auto adapter_agg_sub = nh.subscribe<ects::NetworkUsageHistory>(
                "/ects/system/averages/ta/network/" + adapter + "/usage", 0,
                [&](const ects::NetworkUsageHistory::ConstPtr &msg) {
                    if (!msg->measurements.empty()) {
                        EXPECT_GE(msg->measurements[0].wifi_signal_strength, 0);
                    }
                    EXPECT_EQ(msg->aggregation.ectsname, "ta");
                    recv_adapters_agg = true;
                });
            auto recv_all_net = [&]() {
                return recv_adapters_info && recv_adapter_usage &&
                       recv_adapters_agg;
            };
            spin_predicate(recv_all_net, 1000);
            EXPECT_TRUE(recv_all_net());

            auto reset_recv = [&]() {
                recv_adapters_info = false;
                recv_adapter_usage = false;
                recv_adapters_agg = false;
            };
            reset_recv();
            retransmit_topic("/ects/system/network/" + adapter + "/info");
            retransmit_topic("/ects/system/network/" + adapter + "/usage");
            retransmit_topic("/ects/system/averages/ta/network/" + adapter +
                             "/usage");
            spin_predicate(recv_all_net, 100);
            EXPECT_TRUE(recv_all_net());

            reset_recv();
            retransmit_topic("");
            spin_predicate(recv_all_net, 100);
            EXPECT_TRUE(recv_all_net());
        }
    }
    {
        auto programs_sub = nh.subscribe<ects::ProcessTotal>(
            "/ects/system/processes/total", 0,
            [&](const ects::ProcessTotal::ConstPtr &msg) {
                EXPECT_GT(msg->number_of_processes, 1);
                recv_programs = true;
            });
        auto programs_agg_sub = nh.subscribe<ects::ProcessTotalHistory>(
            "/ects/system/averages/ta/processes/total", 0,
            [&](const ects::ProcessTotalHistory::ConstPtr &msg) {
                if (!msg->measurements.empty()) {
                    EXPECT_GT(msg->measurements[0].number_of_processes, 1);
                }
                EXPECT_EQ(msg->aggregation.ectsname, "ta");
                recv_programs_agg = true;
            });

        auto recv_all_programs = [&]() {
            return recv_programs && recv_programs_agg;
        };
        spin_predicate(recv_all_programs, 1000);

        auto reset_recv = [&]() {
            recv_programs = false;
            recv_programs_agg = false;
        };
        reset_recv();
        retransmit_topic("/ects/system/processes/total");
        retransmit_topic("/ects/system/averages/ta/processes/total");
        spin_predicate(recv_all_programs, 100);
        EXPECT_TRUE(recv_all_programs());

        reset_recv();
        retransmit_topic("");
        spin_predicate(recv_all_programs, 100);
        EXPECT_TRUE(recv_all_programs());
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
