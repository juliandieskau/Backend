#pragma once
/* ECTS - WaypointListFileMessages.hpp
 * Message definitions relating to the saved waypoint lists.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

#include "WaypointList.hpp"
#include "ects/RosInterface.hpp"

#include "ects/EmptyMessage.hpp"
#include "ects/WaypointListDirectory.h"
#include "ects/WaypointListFile.h"

#include <string>
#include <vector>

namespace ects::plugins::waypoints {

using file_list_service_ros_t = WaypointListDirectory;
struct FileListService {};
using FileListRequest = EmptyMessage<file_list_service_ros_t::Request>;
class FileListResponse {
  public:
    using ros_t = file_list_service_ros_t::Response;
    using to_ros_t = ros_t;

    FileListResponse(std::vector<std::string> files);
    FileListResponse(std::string error_message);
    static auto to_ros(const FileListResponse &) -> ros_t;

  private:
    std::vector<std::string> files;
    std::optional<std::string> error;
};

using waypoint_list_file_service_ros_t = WaypointListFile;
struct WaypointListFileService {};
class FileRequest {
  public:
    using ros_t = waypoint_list_file_service_ros_t::Request;
    using from_ros_t = ros_t;

    auto get_filename() -> std::string;
    static auto from_ros(const ros_t &) -> FileRequest;

  private:
    FileRequest(std::string filename);

    std::string filename;
};
class FileResponse {
  public:
    using ros_t = waypoint_list_file_service_ros_t::Response;
    using to_ros_t = ros_t;

    FileResponse();
    FileResponse(std::string error_message);
    static auto to_ros(const FileResponse &) -> ros_t;

  private:
    std::optional<std::string> error;
};

} // namespace ects::plugins::waypoints

namespace ects {
using namespace ects::plugins::waypoints;

template <> struct server_traits<FileListService> {
    using ros_t = file_list_service_ros_t;
    using request_from_ros_t = FileListRequest;
    using response_to_ros_t = FileListResponse;
};

template <> struct server_traits<WaypointListFileService> {
    using ros_t = waypoint_list_file_service_ros_t;
    using request_from_ros_t = FileRequest;
    using response_to_ros_t = FileResponse;
};

} // namespace ects
