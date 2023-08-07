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
    void stop() { running = false; }
    void start() { running = true; }
    ~Timer() { stop(); }

    Timer(float interval, std::function<void()> fn)
        : interval(interval), fn(std::move(fn)) {
        ros_handle = ros::NodeHandle();
        ROS_INFO_STREAM("Creating timer with interval " << interval);
        auto callback = [this](auto timerEvent) {
            if (running) {
                this->fn();
            }
        };
        ros_timer = ros_handle.createTimer(ros::Duration(interval), callback);
        running = true;
    }

  private:
    float interval;
    std::function<void()> fn;
    ros::NodeHandle ros_handle;
    ros::Timer ros_timer;
    bool running;
};
class TimerManager {
  public:
    auto create_timer(float interval, std::function<void()> fn)
        -> std::shared_ptr<Timer> {
        return std::make_shared<Timer>(interval, fn);
    }
};
} // namespace ects
