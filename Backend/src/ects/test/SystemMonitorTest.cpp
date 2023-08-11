#include "../plugins/systemmonitor/aggregations/Aggregation.hpp"
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
