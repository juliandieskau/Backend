#pragma once

#include "../Usage.hpp"
#include "../aggregations/Aggregation.hpp"
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ects::plugins::systemmonitor {

template <typename T> class UsageDataCollection {
  public:
    UsageDataCollection(AggregationStrategy &strategy)
        : strategy(strategy), history(strategy.get_keep_count()) {}
    auto add(T data) -> void;

  private:
    AggregationStrategy &strategy;
    Window<T> history;
};

template <typename T> class UsageDataMonitor {
  public:
    virtual auto step() -> void = 0;

  protected:
    UsageDataMonitor(std::string topic_name,
                     std::vector<std::unique_ptr<AggregationStrategy>>
                         aggregation_strategies,
                     std::unique_ptr<UsageProvider<T>> data_provider)
        : topic_name(std::move(topic_name)),
          aggregation_strategies(std::move(aggregation_strategies)),
          data_provider(data_provider) {}

  private:
    std::string topic_name;
    std::vector<std::unique_ptr<AggregationStrategy>> aggregation_strategies;
    std::unique_ptr<UsageProvider<T>> data_provider;
};

} // namespace ects::plugins::systemmonitor