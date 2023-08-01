#pragma once
#include <chrono>

namespace ects::plugins::systemmonitor {

class averageable {
  public:
    virtual auto operator/(const averageable rhs) -> averageable;
    virtual auto operator+(const float rhs) -> averageable;
};

class UsageData : public averageable {
  public:
    auto get_timestamp() const
        -> const std::chrono::time_point<std::chrono::system_clock> & {
        return timestamp;
    }

  protected:
    UsageData() : timestamp(std::chrono::system_clock::now()) {}

  private:
    const std::chrono::time_point<std::chrono::system_clock> timestamp;
};

template <typename T> class UsageProvider {
  public:
    virtual auto get_usage() -> T;
};

} // namespace ects::plugins::systemmonitor
