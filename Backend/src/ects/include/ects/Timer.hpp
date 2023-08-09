#pragma once
/* ECTS - Timer.hpp
 * TimerManager provides instances of Timer, which can be used to periodically
 * invoke void functions. These can be used by plugins to, for example,
 * periodically collect data.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */
#include "ros/ros.h"
#include <functional>
#include <memory>
#include <utility>
namespace ects {
class Timer {
  public:
    void stop() { ros_timer.start(); }
    void start() { ros_timer.stop(); }
    ~Timer() { stop(); }

  private:
    Timer(float interval, std::function<void()> fn) : ros_handle() {
        ROS_INFO_STREAM("Creating timer with interval " << interval);
        auto callback = [f = std::move(fn)](auto timer_event) { f(); };
        ros_timer = ros_handle.createTimer(ros::Duration(interval), callback);
    }
    friend class TimerManager;

    ros::NodeHandle ros_handle;
    ros::Timer ros_timer;
};
class TimerManager {
  public:
    auto create_timer(float interval, std::function<void()> fn)
        -> std::shared_ptr<Timer> {
        return std::make_shared<Timer>(Timer(interval, std::move(fn)));
    }
};
} // namespace ects
