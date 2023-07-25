#pragma once

#include "ects/Plugin.hpp"
#include "battery_messages.hpp"

namespace ects::plugins::battery {

class battery_monitor : public Plugin {
public:
    auto init(ECTS *) -> void override;
    auto transmit_all() -> void override;
    auto transmit(std::string &topic_name) -> void override;

private:
    struct data {
        std::optional<battery_state> state;
        subscriber<battery_state> battery_subscriber;
    };
    std::optional<data> data;
};

}
