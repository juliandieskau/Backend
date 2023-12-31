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
        WaypointListStorage{ects.config().get_value_or_default<std::string>(
            WaypointManager::storage_location_key, "waypoints")},
        false,
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
        ects.ros_interface().create_publisher<WaypointList>(
            waypoint_list_topic_name),
        ects.ros_interface().create_publisher<NumberOfWaypointsMessage>(
            waypoint_count_topic_name),
        ects.ros_interface().create_subscriber<EmptyStartMessage>(
            start_execution_topic_name),
        ects.ros_interface().create_publisher<IOSBWaypointList>(
            ects.config().get_value<std::string>(start_execution_topic_key)),
        ects.ros_interface().create_subscriber<EmptyStopMessage>(
            stop_execution_topic_name),
        ects.ros_interface().create_publisher<EmptyStopMessage>(
            ects.config().get_value<std::string>(
                stop_execution_topic_name_key)),
        ects.ros_interface().create_publisher<IsExecutingMessage>(
            is_executing_topic_name),
        ects.ros_interface().create_topic_forwarder<std_msgs::UInt32>(
            ects.config().get_value<std::string>(
                current_waypoint_topic_name_key),
            current_waypoint_topic_name),
    };
    data->add_waypoint_subscriber.subscribe([this](AddWaypointMessage message) {
        ROS_INFO_STREAM("called add waypoint subscriber");
        data->waypoints.add_waypoint(message.get_waypoint(),
                                     message.get_index());
        transmit_all();
    });
    data->remove_waypoint_subscriber.subscribe(
        [this](RemoveWaypointMessage message) {
            ROS_INFO_STREAM("called remove waypoint subscriber");
            data->waypoints.remove_waypoint(message.get_index());
            transmit_all();
        });
    data->replace_waypoint_subscriber.subscribe(
        [this](ReplaceWaypointMessage message) {
            ROS_INFO_STREAM("called replace waypoint subscriber");
            data->waypoints.replace_waypoint(message.get_index(),
                                             message.get_waypoint());
            transmit_all();
        });
    data->reorder_waypoints_subscriber.subscribe(
        [this](ReorderWaypointsMessage message) {
            ROS_INFO_STREAM("called reorder waypoints subscriber");
            data->waypoints.reorder_waypoints(message.get_permutation());
            transmit_all();
        });
    data->repeat_waypoints_subscriber.subscribe(
        [this](RepeatWaypointsMessage message) {
            ROS_INFO_STREAM("called repeat waypoints subscriber");
            data->waypoints.set_repeat(message.get_repeat());
            transmit_all();
        });
    data->reverse_waypoints_subscriber.subscribe(
        [this](ReverseWaypointsMessage message) {
            ROS_INFO_STREAM("called reverse waypoints subscriber");
            data->waypoints.reverse_waypoints();
            transmit_all();
        });
    data->list_files_server.register_service(
        [this](FileListRequest) -> FileListResponse {
            ROS_INFO_STREAM("called list files server");
            return FileListResponse{data->storage.list_directory()};
        });
    data->save_waypoints_server.register_service(
        [this](FileRequest file) -> FileResponse {
            ROS_INFO_STREAM("called save waypoints server");
            try {
                data->storage.save(file.get_filename(), data->waypoints);
            } catch (std::exception &e) {
                ROS_INFO_STREAM("failed to load file " << file.get_filename()
                                                       << ": " << e.what());
                return FileResponse{e.what()};
            }
            return {};
        });
    data->load_waypoints_server.register_service(
        [this](FileRequest file) -> FileResponse {
            ROS_INFO_STREAM("called load waypoints server");
            try {
                data->waypoints = data->storage.load(file.get_filename());
            } catch (std::exception &e) {
                ROS_INFO_STREAM("failed to load file " << file.get_filename()
                                                       << ": " << e.what());
                return FileResponse{e.what()};
            }
            transmit_all();
            return {};
        });
    data->start_execution_subscriber.subscribe([this](EmptyStartMessage) {
        if (data->is_executing) {
            ROS_ERROR_STREAM("called start execution while already executing");
            return;
        }
        data->is_executing = true;
        publish_is_executing();
        data->start_execution_publisher.publish(data->waypoints);
    });
    data->stop_execution_subscriber.subscribe([this](EmptyStopMessage) {
        if (!data->is_executing) {
            ROS_ERROR_STREAM("called stop execution while not executing");
            return;
        }
        data->is_executing = false;
        publish_is_executing();
        data->stop_execution_publisher.publish({});
    });
}
void WaypointManager::transmit_all() {
    publish_waypoint_list();
    publish_waypoint_count();
    publish_is_executing();
}
void WaypointManager::transmit(const std::string &topic_name) {
    if (topic_name == waypoint_list_topic_name)
        publish_waypoint_list();
    if (topic_name == waypoint_count_topic_name)
        publish_waypoint_count();
    if (topic_name == is_executing_topic_name)
        publish_is_executing();
}

void WaypointManager::publish_waypoint_list() {
    data->waypoint_list_publisher.publish(data->waypoints);
}

void WaypointManager::publish_waypoint_count() {
    data->waypoint_count_publisher.publish(data->waypoints.size());
}

void WaypointManager::publish_is_executing() {
    data->is_executing_publisher.publish(data->is_executing);
}

} // namespace ects::plugins::waypoints
