#pragma once
#include "ros/ros.h"
#include <functional>
#include <memory>
#include <utility>
namespace ects {
class Timer {
public:
  void stop() { m_running = false; }
  void start() { m_running = true; }
  ~Timer() { stop(); }

  Timer(float interval, std::function<void()> fn)
      : m_interval(interval), m_fn(std::move(fn)) {
    m_nodeHandle = ros::NodeHandle();
    ROS_INFO_STREAM("Creating timer with interval " << m_interval);
    auto callback = [this](auto timerEvent) {
        if (m_running) {
            m_fn();
        }
    };
    m_timer = m_nodeHandle.createTimer(ros::Duration(m_interval), callback);

    ROS_INFO_STREAM("Timer created");
    m_running = true;
  }

private:
  float m_interval;
  std::function<void()> m_fn;
  ros::NodeHandle m_nodeHandle;
  ros::Timer m_timer;
  bool m_running;
};
class TimerManager {
public:
  auto createTimer(float interval, std::function<void()> fn)
      -> std::shared_ptr<Timer> {
    // auto timer = new Timer<F>(interval, fn);
    // m_timers.push_back(timer);
    return std::make_shared<Timer>(interval, fn);
  }
};

} // namespace ects