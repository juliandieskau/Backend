#pragma once

#include "ects/ECTS.hpp"
#include "ects/Plugin.hpp"
#include "iosb_localization_filter/FilterState.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

namespace ects::plugins::position {
class Position : public Plugin {
  public:
    auto init(ECTS &) -> void override;
    auto transmit_all() -> void override;
    auto transmit(const std::string &topic_name) -> void override;
    auto name() const -> const std::string override { return "position"; }

  public:
    struct data {
        TopicForwarder<nav_msgs::Odometry> odometry_forwarder;
        TopicForwarder<sensor_msgs::Imu> imu_forwarder;
        TopicForwarder<iosb_localization_filter::FilterState> status_forwarder;
    };
    std::optional<data> data;
    static constexpr auto default_odometry_topic =
        "/localization/odometry_global";
    static constexpr auto default_imu_topic = "/ellipse/imu";
    static constexpr auto default_status_topic =
        "/localization/filter_state_global";

    static constexpr auto forward_odometry_topic = "/ects/control/position";
    static constexpr auto forward_imu_topic = "/ects/imu/current";
    static constexpr auto forward_status_topic = "/ects/control/filter_state";

    static constexpr auto odometry_topic_key = "/position/odometry_topic";
    static constexpr auto imu_topic_key = "/position/imu_topic";
    static constexpr auto status_topic_key = "/position/status_topic";
};

} // namespace ects::plugins::position
