#include "Aggregation.hpp"

namespace ects::plugins::systemmonitor {
    auto aggregation_from_json(nlohmann::json &j) -> AggregationStrategy*{

        auto type = j.at("type").get<std::string>();
        auto keep_amount = j.at("keep_amount").get<uint32_t>();
        auto name = j.at("name").get<std::string>();
        if(type == "interval"){
            auto interval = j.at("interval").get<float>();
            return new IntervalAggregationStrategy(keep_amount, name, interval);
        } else if(type == "readings"){
            auto nreadings = j.at("readings").get<uint32_t>();
            return new ReadingsAggregationStrategy(keep_amount, name, nreadings);
        } else {
            throw new std::runtime_error("Invalid aggregation type specified");
        }
    };

    auto ReadingsAggregationStrategy::new_data() -> void {

    }

    auto ReadingsAggregationStrategy::should_aggregate() -> bool {
        return false;
    }

    auto IntervalAggregationStrategy::new_data() -> void {

    }
    auto IntervalAggregationStrategy::should_aggregate() -> bool {
        return false;
    }

} // namespace ects::plugin::systemmonitor
