#include "WaypointManager.hpp"
#include "ects/ECTS.hpp"
#include "ros/ros.h"

extern "C" auto create_plugin_instance() -> ects::Plugin * {
    return new ects::plugins::waypoints::WaypointManager();
}
namespace ects::plugins::waypoints {

void WaypointManager::init(ECTS &ects) {
    ROS_INFO("Initializing WaypointManager");

    data = {
        WaypointList{},
        WaypointListStorage{ects.config().get_value<std::string>(
            "/waypoints/storage_directory")},
        ects.ros_interface().create_subscriber<AddWaypointMessage>(
            add_waypoint_topic_name),
        ects.ros_interface().create_subscriber<RemoveWaypointMessage>(
            remove_waypoint_topic_name),
        ects.ros_interface().create_subscriber<ReplaceWaypointMessage>(
            replace_waypoint_topic_name),
        ects.ros_interface().create_subscriber<ReorderWaypointsMessage>(
            reorder_waypoints_topic_name),
        ects.ros_interface().create_subscriber<RepeatWaypointsMessage>(
            repeat_waypoints_topic_name),
        ects.ros_interface().create_subscriber<ReverseWaypointsMessage>(
            reverse_waypoints_topic_name),
        ects.ros_interface().create_server<FileListService>(
            list_files_service_name),
        ects.ros_interface().create_server<WaypointListFileService>(
            save_waypoints_service_name),
        ects.ros_interface().create_server<WaypointListFileService>(
            load_waypoints_service_name),
    };
    data->add_waypoint_subscriber.subscribe([this](AddWaypointMessage message) {
        ROS_INFO_STREAM("called add waypoint subscriber");
        data->waypoints.add_waypoint(message.get_waypoint(),
                                     message.get_index());
    });
    data->remove_waypoint_subscriber.subscribe(
        [this](RemoveWaypointMessage message) {
            ROS_INFO_STREAM("called remove waypoint subscriber");
            data->waypoints.remove_waypoint(message.get_index());
        });
    data->replace_waypoint_subscriber.subscribe(
        [this](ReplaceWaypointMessage message) {
            ROS_INFO_STREAM("called replace waypoint subscriber");
            data->waypoints.replace_waypoint(message.get_index(),
                                             message.get_waypoint());
        });
    data->reorder_waypoints_subscriber.subscribe(
        [this](ReorderWaypointsMessage message) {
            ROS_INFO_STREAM("called reorder waypoints subscriber");
            data->waypoints.reorder_waypoints(message.get_permutation());
        });
    data->repeat_waypoints_subscriber.subscribe(
        [this](RepeatWaypointsMessage message) {
            ROS_INFO_STREAM("called repeat waypoints subscriber");
            data->waypoints.set_repeat(message.get_repeat());
        });
    data->reverse_waypoints_subscriber.subscribe(
        [this](ReverseWaypointsMessage message) {
            ROS_INFO_STREAM("called reverse waypoints subscriber");
            data->waypoints.reverse_waypoints();
        });
    data->list_files_server.register_service(
        [this](FileListRequest) -> FileListResponse {
            ROS_INFO_STREAM("called list files server");
            return FileListResponse{data->storage.list_directory()};
        });
    data->save_waypoints_server.register_service(
        [this](FileRequest file) -> FileResponse {
            ROS_INFO_STREAM("called save waypoints server");
            data->storage.save(file.get_filename(), data->waypoints);
            return FileResponse{};
        });
    data->load_waypoints_server.register_service(
        [this](FileRequest file) -> FileResponse {
            ROS_INFO_STREAM("called load waypoints server");
            data->waypoints = data->storage.load(file.get_filename());
            return FileResponse{};
        });
}
void WaypointManager::transmit_all() {}
void WaypointManager::transmit(const std::string &topic_name) {}

} // namespace ects::plugins::waypoints