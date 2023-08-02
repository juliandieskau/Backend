#pragma once
#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "aggregations/Aggregation.hpp"

namespace ects::plugins::systemmonitor {
class AggregationStrategy;
class averageable {
  public:
    virtual auto operator/(const averageable rhs) -> averageable;
    virtual auto operator+(const float rhs) -> averageable;
};

class UsageData : public averageable {
  public:
    auto get_timestamp() const
        -> const std::chrono::time_point<std::chrono::system_clock> & {
        return timestamp;
    }

  protected:
    UsageData() : timestamp(std::chrono::system_clock::now()) {}

  private:
    const std::chrono::time_point<std::chrono::system_clock> timestamp;
};

template <typename T> class UsageProvider {
  public:
    virtual auto get_usage() -> T;
};

template <typename T> class UsageProviderAdapter : public UsageProvider<T> {
  public:
    UsageProviderAdapter(std::function<T()> f) : f(f) {}
    auto get_usage() -> T override { return f(); }

  private:
    std::function<T()> f;
};

template <typename T> class UsageDataMonitor {
  public:
    virtual auto step() -> void = 0;
  protected:
    UsageDataMonitor(std::string topic_name,
                     std::vector<AggregationStrategy> aggregation_strategies,
                     UsageProvider<T> data_provider)
        : topic_name(topic_name),
          aggregation_strategies(aggregation_strategies),
          data_providers(data_provider) {}

  private:
    std::string topic_name;
    std::vector<AggregationStrategy> aggregation_strategies;
    std::vector<UsageProvider<T>> data_providers;
};

} // namespace ects::plugins::systemmonitor
