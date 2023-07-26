#pragma once

#include "nlohmann/json.hpp"
#include <geometry_msgs/Pose2D.h>

namespace ects::common {

class Position2d {
  public:
    Position2d(double x, double y, double radius);
    Position2d() = default;
    double get_x() const;
    double get_y() const;
    double get_radius() const;
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
    double get_heading() const;
    double get_accuracy() const;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Heading2d, heading, accuracy);

  private:
    double heading;
    double accuracy;
};

} // namespace ects::common
