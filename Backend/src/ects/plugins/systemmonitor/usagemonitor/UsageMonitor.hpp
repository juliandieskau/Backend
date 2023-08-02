#pragma once

#include "../Usage.hpp"
#include "../aggregations/Aggregation.hpp"
#include "ects/RosInterface.hpp"
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ects::plugins::systemmonitor {

template <typename T> class UsageDataCollection {
  public:
    using ros_t = T::history_ros_t;
    using to_ros_t = ros_t;

    UsageDataCollection(AggregationStrategy *strategy)
        : strategy(strategy->make_aggregator()),
          history(strategy->get_keep_count()), current_aggregation() {}
    auto add(T data) -> void;

    static auto to_ros(UsageDataCollection<T>) -> ros_t;

  private:
    std::unique_ptr<Aggregator> strategy;
    Window<T> history;
    std::vector<T> current_aggregation;
};

template <typename T> class UsageDataMonitor {
  public:
    auto step() -> void;

    // topic_name is like "cpu/usage", "processes/total",
    // "disk/{mountpoint}/usage"
    UsageDataMonitor(
        std::string topic_name,
        const std::vector<AggregationStrategy *> &aggregation_strategies,
        std::unique_ptr<UsageProvider<T>> data_provider);

  private:
    std::string topic_name;
    std::unique_ptr<UsageProvider<T>> data_provider;
    std::vector<UsageDataCollection<T>> collected_usage_data;
    std::vector<Publisher<UsageDataCollection<T>>> collection_publishers;
    Publisher<T> last_usage_publisher;
    size_t size;

    static constexpr auto last_usage_topic_name = "/ects/system/";
    static constexpr auto aggregation_topic_name = "/ects/system/averages/";
};

} // namespace ects::plugins::systemmonitor