#pragma once
#include "../Usage.hpp"
#include "nlohmann/json.hpp"
#include <chrono>
#include <cstdint>
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

class AggregationStrategy {
  public:
    auto get_keep_count() -> uint32_t { return keep_count; }
    auto get_name() -> std::string & { return name; }
    virtual auto new_data() -> void = 0;
    virtual auto should_aggregate() -> bool = 0;

  protected:
    AggregationStrategy(uint32_t keep_count, std::string name)
        : keep_count(keep_count), name(name) {}

  private:
    uint32_t keep_count;
    std::string name;
};

class ReadingsAggregationStrategy : public AggregationStrategy {
  public:
    ReadingsAggregationStrategy(uint32_t keep_count, std::string name,
                                uint32_t readings_count)
        : AggregationStrategy(keep_count, name),
          readings_count(readings_count) {}
    auto new_data() -> void override;
    auto should_aggregate() -> bool override;

  private:
    uint32_t readings_count;
};

class IntervalAggregationStrategy : public AggregationStrategy {
  public:
    IntervalAggregationStrategy(uint32_t keep_count, std::string name,
                                float interval)
        : AggregationStrategy(keep_count, name), interval(interval) {}
    auto new_data() -> void override;
    auto should_aggregate() -> bool override;

  private:
    float interval;
};

auto aggregation_from_json(nlohmann::json &) -> AggregationStrategy *;

} // namespace ects::plugins::systemmonitor
