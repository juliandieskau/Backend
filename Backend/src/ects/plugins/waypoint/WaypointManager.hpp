#pragma once

#include "WaypointList.hpp"
#include "WaypointListFileMessages.hpp"
#include "WaypointListStorage.hpp"
#include "WaypointMessages.hpp"
#include "ects/Plugin.hpp"
#include "ects/RosInterface.hpp"
#include <optional>

namespace ects::plugins::waypoints {

class WaypointManager : public Plugin {
  public:
    auto init(ECTS &ects) -> void override;
    auto transmit_all() -> void override;
    auto transmit(const std::string &topic_name) -> void override;
    auto name() const -> const std::string override { return "waypoints"; }

  private:
    struct data {
        WaypointList waypoints;
        WaypointListStorage storage;
        Subscriber<AddWaypointMessage> add_waypoint_subscriber;
        Subscriber<RemoveWaypointMessage> remove_waypoint_subscriber;
        Subscriber<ReplaceWaypointMessage> replace_waypoint_subscriber;
        Subscriber<ReorderWaypointsMessage> reorder_waypoints_subscriber;
        Subscriber<RepeatWaypointsMessage> repeat_waypoints_subscriber;
        Subscriber<ReverseWaypointsMessage> reverse_waypoints_subscriber;
        Server<FileListService> list_files_server;
        Server<WaypointListFileService> save_waypoints_server;
        Server<WaypointListFileService> load_waypoints_server;
    };
    std::optional<data> data;
    static constexpr auto add_waypoint_topic_name =
        "/ects/waypoints/add_waypoint";
    static constexpr auto remove_waypoint_topic_name =
        "/ects/waypoints/remove_waypoint";
    static constexpr auto replace_waypoint_topic_name =
        "/ects/waypoints/replace_waypoint";
    static constexpr auto reorder_waypoints_topic_name =
        "/ects/waypoints/reorder_waypoints";
    static constexpr auto repeat_waypoints_topic_name =
        "/ects/waypoints/repeat_waypoints";
    static constexpr auto reverse_waypoints_topic_name =
        "/ects/waypoints/reverse_waypoints";
    static constexpr auto list_files_service_name =
        "/ects/waypoints/saved_lists";
    static constexpr auto save_waypoints_service_name =
        "/ects/waypoints/save_waypoint_list";
    static constexpr auto load_waypoints_service_name =
        "/ects/waypoints/load_waypoint_list";
};

} // namespace ects::plugins::waypoints
