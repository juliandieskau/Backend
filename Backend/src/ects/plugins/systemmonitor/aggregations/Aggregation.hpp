#pragma once
#include <cstdint>

namespace ects::plugins::systemmonitor {
    class AggregationStrategy {
      public:
        virtual auto get_keep_count() -> uint32_t = 0;
        virtual auto new_data() -> void = 0;
        virtual auto should_aggregate() -> bool = 0;

        protected:
        AggregationStrategy(uint32_t keep_count) : keep_count(keep_count) {}

        private:
        uint32_t keep_count;
    };

} // namespace ects::plugins::systemmonitor
