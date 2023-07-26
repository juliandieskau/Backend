#pragma once

#include "ects/Plugin.hpp"
#include <optional>

namespace ects::plugins::waypoints {

class WaypointManager : public Plugin {
  public:
    auto init(ECTS *ects1) -> void override;
    auto transmit_all() -> void override;
    auto transmit(const std::string &topic_name) -> void override;
    auto name() const -> const std::string override { return "waypoints"; }

  private:
    struct data {};
    std::optional<data> data;
};

} // namespace ects::plugins::waypoints
