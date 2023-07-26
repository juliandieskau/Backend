#pragma once

#include "ects/WaypointList.h"
#include "Waypoint.hpp"

#include <vector>

namespace ects::plugins::waypoints {

class Index {
  public:
    Index(size_t);
    size_t get() const;

  private:
    size_t index;
};

class WaypointList {
  public:
    using ros_t = ects::WaypointList;
    using from_ros_t = ros_t;
    using to_ros_t = ros_t;

    WaypointList();
    void add_waypoint(Waypoint, Index);
    void remove_waypoint(Index);
    void replace_waypoint(Index, Waypoint);
    void reorder_waypoints(std::vector<Index> permutation);
    void reverse_waypoints();
    void set_repeat(bool);
    double total_length();
    std::size_t size();

    static WaypointList from_ros(const ros_t &);
    static ros_t to_ros(const WaypointList &);

  private:
    bool in_bounds(Index i, const std::string& name);

    std::vector<Waypoint> waypoints;
    bool cyclic;
};

} // namespace ects::plugins::waypoints
