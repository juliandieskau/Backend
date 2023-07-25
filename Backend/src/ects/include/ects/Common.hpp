#pragma once

#include <geometry_msgs/Pose2D.h>

namespace ects::common {

class Position2d {
  public:
    Position2d(double x, double y, double radius);
    double get_x();
    double get_y();
    double get_radius();

  private:
    double x;
    double y;
    double radius;
};

class Heading2d {
  public:
    Heading2d(double heading, double accuracy);
    double get_heading();
    double get_accuracy();

  private:
    double heading;
    double accuracy;
};

} // namespace ects::common
