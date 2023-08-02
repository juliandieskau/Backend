#pragma once

#include <chrono>
#include <functional>

namespace ects::plugins::systemmonitor {

class UsageData {
  public:
    auto get_timestamp() const
        -> const std::chrono::time_point<std::chrono::system_clock> & {
        return timestamp;
    }

  protected:
    UsageData() : timestamp(std::chrono::system_clock::now()) {}

  private:
    std::chrono::time_point<std::chrono::system_clock> timestamp;
};

template <typename T> class UsageProvider {
  public:
    virtual auto get_usage() -> T = 0;
    virtual ~UsageProvider() = default;

  protected:
    UsageProvider() = default;
};

template <typename T> class UsageProviderAdapter : public UsageProvider<T> {
  public:
    UsageProviderAdapter(std::function<T()> f) : f(f) {}
    auto get_usage() -> T override { return f(); }

  private:
    std::function<T()> f;
};

} // namespace ects::plugins::systemmonitor
