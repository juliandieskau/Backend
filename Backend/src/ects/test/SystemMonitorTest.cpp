#include "../plugins/systemmonitor/aggregations/Aggregation.hpp"
#include "../plugins/systemmonitor/cpu/Cpu.hpp"
#include "../plugins/systemmonitor/disk/Disk.hpp"
#include "../plugins/systemmonitor/memory/Memory.hpp"
#include "../plugins/systemmonitor/network/Network.hpp"
#include "../plugins/systemmonitor/programs/Programs.hpp"
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
