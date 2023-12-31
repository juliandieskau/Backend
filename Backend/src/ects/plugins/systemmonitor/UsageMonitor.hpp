#pragma once
/* ECTS - UsageMonitor.hpp
 * The class UsageMonitor is responsible for aggregating and publishing the
 * data collected by it's UsageProvider.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

#include "Usage.hpp"
#include "aggregations/Aggregation.hpp"
#include "ects/ECTS.hpp"
#include "ects/RosInterface.hpp"
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ects::plugins::systemmonitor {

template <typename T> class UsageDataCollection {
  public:
    using ros_t = typename T::history_ros_t;
    using to_ros_t = ros_t;

    UsageDataCollection(AggregationStrategy *strategy)
        : strategy(strategy), aggregator(strategy->make_aggregator()),
          history(strategy->get_keep_count()), current_aggregation() {}
    auto add(T data) -> void {
        // NOTE: this implementation saves all data till it aggregates
        auto average = [this, &data] {
            T acc = data;
            for (auto d : current_aggregation)
                acc = acc + d;
            acc = acc / current_aggregation.size();
            current_aggregation.clear();
            return acc;
        };
        if (aggregator->new_data(&data))
            history.add(average());
        else
            current_aggregation.push_back(data);
    }

    static auto to_ros(const UsageDataCollection<T> &collection) -> ros_t {
        ros_t r;
        r.aggregation = collection.strategy->to_ros();
        for (auto &data : collection.history.get_window())
            r.measurements.push_back(T::to_ros(data));
        return r;
    }

  private:
    AggregationStrategy *strategy;
    std::unique_ptr<Aggregator> aggregator;
    Window<T> history;
    std::vector<T> current_aggregation;
};

template <typename T> class UsageDataMonitor {
  public:
    auto step() -> void {
        T data = data_provider->get_usage();
        last_usage_data = data;
        last_usage_publisher.publish(data);
        for (size_t i = 0; i < size; ++i) {
            collected_usage_data[i].add(data);
            collection_publishers[i].publish(collected_usage_data[i]);
        }
    }
    auto transmit_all() -> void {
        if (last_usage_data.has_value())
            last_usage_publisher.publish(*last_usage_data);
        for (size_t i = 0; i < size; ++i) {
            collection_publishers[0].publish(collected_usage_data[i]);
        }
    }
    auto transmit(const std::string &topic_name) -> void {
        if (last_usage_data.has_value() &&
            topic_name == last_usage_topic_name + this->topic_name)
            last_usage_publisher.publish(*last_usage_data);
        for (size_t i = 0; i < size; ++i) {
            if (topic_name == aggregation_topic_name +
                                  aggregation_strategies[i]->get_name() + "/" +
                                  this->topic_name)
                collection_publishers[0].publish(collected_usage_data[i]);
        }
    }

    // topic_name is like "/cpu/usage", "/processes/total",
    // "/disk/{mountpoint}/usage"
    UsageDataMonitor(std::string topic_name,
                     std::vector<AggregationStrategy *> aggregation_strategies,
                     std::unique_ptr<UsageProvider<T>> data_provider,
                     ECTS &ects)
        : topic_name(std::move(topic_name)),
          aggregation_strategies(std::move(aggregation_strategies)),
          data_provider(std::move(data_provider)), collected_usage_data(),
          collection_publishers(), last_usage_data(std::nullopt),
          last_usage_publisher(ects.ros_interface().create_publisher<T>(
              last_usage_topic_name + this->topic_name)),
          size(this->aggregation_strategies.size()) {
        for (auto aggregation : this->aggregation_strategies) {
            collected_usage_data.emplace_back(aggregation);
            auto pub =
                ects.ros_interface().create_publisher<UsageDataCollection<T>>(
                    aggregation_topic_name + aggregation->get_name() + "/" +
                    this->topic_name);
            collection_publishers.push_back(std::move(pub));
        }
    }

  private:
    std::string topic_name;
    std::vector<AggregationStrategy *> aggregation_strategies;
    std::unique_ptr<UsageProvider<T>> data_provider;
    std::vector<UsageDataCollection<T>> collected_usage_data;
    std::vector<Publisher<UsageDataCollection<T>>> collection_publishers;
    std::optional<T> last_usage_data;
    Publisher<T> last_usage_publisher;
    size_t size;

    static constexpr auto last_usage_topic_name = "/ects/system/";
    static constexpr auto aggregation_topic_name = "/ects/system/averages/";
};

} // namespace ects::plugins::systemmonitor
