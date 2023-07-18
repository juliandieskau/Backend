#pragma once
#include <functional>
#include <memory>
namespace ects {
template <typename F> class Timer {
public:
  void stop() { m_running = false; }
  void start() { m_running = true; }
  ~Timer() { stop(); }

private:
  Timer(float interval, F fn);
  float m_interval;
  F m_fn;
  bool m_running;
};

class TimerManager {
  template <typename F> friend class Timer;
  template <typename F>
  auto createTimer(float interval, F fn) -> std::shared_ptr<Timer<F>>;
};

} // namespace ects