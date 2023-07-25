#pragma once

namespace ects {

template <typename ros_t> struct EmptyMessage {
    using from_ros_t = ros_t;
    using to_ros_t = ros_t;

    static EmptyMessage<ros_t> from_ros(from_ros_t) { return {}; }
    static to_ros_t to_ros(EmptyMessage<ros_t>) { return {}; }
};

} // namespace ects