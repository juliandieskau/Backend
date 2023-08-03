#include "Aggregation.hpp"

namespace ects::plugins::systemmonitor {

auto aggregation_from_json(nlohmann::json &j) -> AggregationStrategy * {

    auto type = j.at("type").get<std::string>();
    auto keep_amount = j.at("keep_count").get<uint32_t>();
    auto name = j.at("name").get<std::string>();
    if (type == "interval") {
        std::chrono::duration<float> interval{j.at("interval").get<float>()};
        return new IntervalAggregationStrategy(keep_amount, name, interval);
    } else if (type == "readings") {
        auto nreadings = j.at("readings").get<uint32_t>();
        return new ReadingsAggregationStrategy(keep_amount, name, nreadings);
    } else {
        throw std::runtime_error("Invalid aggregation type specified");
    }
}

struct ReadingsAggregator : public Aggregator {
    auto new_data(UsageData *data) -> bool override {
        if (++current_count < readings_count)
            return false;
        current_count = 0;
        return true;
    }

    ReadingsAggregator(uint32_t readings_count)
        : readings_count(readings_count) {}
    uint32_t current_count = 0;
    const uint32_t readings_count;
};
auto ReadingsAggregationStrategy::make_aggregator()
    -> std::unique_ptr<Aggregator> {
    return std::make_unique<ReadingsAggregator>(readings_count);
}
auto ReadingsAggregationStrategy::to_ros() -> AggregationStrategy::ros_t {
    ros_t r;
    r.type = 1;
    r.nreadings = readings_count;
    r.ectsname = get_name();
    r.keep_amount = get_keep_count();
    return r;
}

struct IntervalAggregator : public Aggregator {
    auto new_data(UsageData *data) -> bool override {
        if (!start.has_value()) {
            start = data->get_timestamp();
            return false;
        }
        if (data->get_timestamp() - *start > interval) {
            start = std::nullopt;
            return true;
        }
        return false;
    }

    IntervalAggregator(std::chrono::duration<float> interval)
        : interval(interval) {}
    std::optional<std::chrono::time_point<std::chrono::system_clock>> start =
        std::nullopt;
    const std::chrono::duration<float> interval;
};
auto IntervalAggregationStrategy::make_aggregator()
    -> std::unique_ptr<Aggregator> {
    return std::make_unique<IntervalAggregator>(interval);
}
auto IntervalAggregationStrategy::to_ros() -> AggregationStrategy::ros_t {
    ros_t r;
    r.type = 0;
    r.interval = interval.count();
    r.ectsname = get_name();
    r.keep_amount = get_keep_count();
    return r;
}
} // namespace ects::plugins::systemmonitor
