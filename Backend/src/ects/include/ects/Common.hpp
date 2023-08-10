#pragma once
/* ECTS - Common.hpp
 * This file contains common message types used by multiple ECTS Plugins.
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik Oelbracht, Liam
 * Derk Rembold
 */

#include "nlohmann/json.hpp"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

namespace ects::common {

class Position2d {
  public:
    Position2d(double x, double y, double radius);
    Position2d() = default;
    auto get_x() const -> double;
    auto get_y() const -> double;
    auto get_radius() const -> double;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Position2d, x, y, radius);

  private:
    double x;
    double y;
    double radius;
};

class Heading2d {
  public:
    Heading2d(double heading, double accuracy);
    Heading2d() = default;
    auto get_heading() const -> double;
    auto get_accuracy() const -> double;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Heading2d, heading, accuracy);

  private:
    double heading;
    double accuracy;
};

class Vector3D {
  public:
    using ros_t = geometry_msgs::Vector3;
    using from_ros_t = ros_t;
    using to_ros_t = ros_t;

    static auto from_ros(const ros_t &) -> Vector3D;
    static auto to_ros(const Vector3D &) -> ros_t;

  private:
    Vector3D(double x, double y, double z);

    double x;
    double y;
    double z;
};

class Twist {
  public:
    using ros_t = geometry_msgs::Twist;
    using from_ros_t = ros_t;
    using to_ros_t = ros_t;

    static auto from_ros(const ros_t &) -> Twist;
    static auto to_ros(const Twist &) -> ros_t;

  private:
    Twist(Vector3D linear, Vector3D angular);

    Vector3D linear;
    Vector3D angular;
};

} // namespace ects::common
