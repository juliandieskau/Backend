#pragma once
#include "../Usage.hpp"
#include "ects/Aggregation.h"
#include "nlohmann/json.hpp"
#include <chrono>
#include <cstdint>
#include <utility>
#include <vector>

namespace ects::plugins::systemmonitor {
template <typename T> class Window {
  public:
    Window(uint32_t keep_count) : keep_count(keep_count) {}
    auto get_keep_count() -> uint32_t { return keep_count; }
    auto add(T) -> void;
    auto get_window() -> std::vector<T>;

  private:
    uint32_t keep_count;
    std::vector<T> window;
};

class Aggregator {
  public:
    virtual auto new_data(UsageData *data) -> bool = 0;

  protected:
    Aggregator() = default;
    Aggregator(const Aggregator &) = default;
    Aggregator(Aggregator &&) = default;
};

class AggregationStrategy {
  public:
    using ros_t = ects::Aggregation;

    auto get_keep_count() const -> uint32_t { return keep_count; }
    auto get_name() -> std::string & { return name; }
    virtual auto make_aggregator() -> std::unique_ptr<Aggregator> = 0;
    virtual auto to_ros() -> ros_t = 0;

  protected:
    AggregationStrategy(uint32_t keep_count, std::string name)
        : keep_count(keep_count), name(std::move(name)) {}

  private:
    uint32_t keep_count;
    std::string name;
};

class ReadingsAggregationStrategy : public AggregationStrategy {
  public:
    ReadingsAggregationStrategy(uint32_t keep_count, std::string name,
                                uint32_t readings_count)
        : AggregationStrategy(keep_count, std::move(name)),
          readings_count(readings_count) {}
    auto make_aggregator() -> std::unique_ptr<Aggregator> override;
    auto to_ros() -> ros_t override;

  private:
    uint32_t readings_count;
};

class IntervalAggregationStrategy : public AggregationStrategy {
  public:
    IntervalAggregationStrategy(uint32_t keep_count, std::string name,
                                std::chrono::duration<float> interval)
        : AggregationStrategy(keep_count, std::move(name)), interval(interval) {
    }
    auto make_aggregator() -> std::unique_ptr<Aggregator> override;
    auto to_ros() -> ros_t override;

  private:
    std::chrono::duration<float> interval;
};

auto aggregation_from_json(nlohmann::json &) -> AggregationStrategy *;

} // namespace ects::plugins::systemmonitor
