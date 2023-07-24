#pragma once

namespace ects {

template<typename ros_t>
struct empty_message {
    using from_ros_t = ros_t;
    using to_ros_t = ros_t;

    static empty_message<ros_t> from_ros(from_ros_t) {
        return {};
    }
    static to_ros_t to_ros(empty_message<ros_t>) {
        return {};
    }
};

}
