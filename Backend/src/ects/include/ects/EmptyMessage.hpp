#pragma once
/* ECTS - EmptyMessage.hpp
 * EmptyMessage is a convenience class for messages that do not contain any data.
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik Oelbracht, Liam Derk Rembold
 */

namespace ects {

template <typename ros_t> struct EmptyMessage {
    using from_ros_t = ros_t;
    using to_ros_t = ros_t;

    static EmptyMessage<ros_t> from_ros(const from_ros_t &) { return {}; }
    static to_ros_t to_ros(const EmptyMessage<ros_t> &) { return {}; }
};

} // namespace ects
