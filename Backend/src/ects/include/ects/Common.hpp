#pragma once

#include <geometry_msgs/Pose2D.h>

namespace ects::common {

class Position2d {
  public:
    Position2d(double x, double y, double radius);
    double get_x() const;
    double get_y() const;
    double get_radius() const;

  private:
    double x;
    double y;
    double radius;
};

class Heading2d {
  public:
    Heading2d(double heading, double accuracy);
    double get_heading() const;
    double get_accuracy() const;

  private:
    double heading;
    double accuracy;
};

} // namespace ects::common
