#include "WaypointListFileMessages.hpp"

#include <utility>

namespace ects::plugins::waypoints {

FileListResponse::FileListResponse(std::vector<std::string> files)
    : files(std::move(files)), error(std::nullopt) {}
FileListResponse::FileListResponse(std::string error_message)
    : files(), error(error_message) {}
auto FileListResponse::to_ros(const FileListResponse &internal_input)
    -> FileListResponse::ros_t {
    FileListResponse::ros_t r{};
    r.success = !internal_input.error.has_value();
    r.error_message = internal_input.error.value_or("");
    r.filenames = internal_input.files;
    return r;
}

auto FileRequest::get_filename() -> std::string { return filename; }
auto FileRequest::from_ros(const FileRequest::ros_t &ros_input) -> FileRequest {
    return {ros_input.filename};
}
FileRequest::FileRequest(std::string filename)
    : filename(std::move(filename)) {}

FileResponse::FileResponse() : error(std::nullopt) {}
FileResponse::FileResponse(std::string error_message) : error(error_message) {}
auto FileResponse::to_ros(const FileResponse &internal_input)
    -> FileResponse::ros_t {
    FileResponse::ros_t r{};
    r.success = !internal_input.error.has_value();
    r.error_message = internal_input.error.value_or("");
    return r;
}

} // namespace ects::plugins::waypoints
