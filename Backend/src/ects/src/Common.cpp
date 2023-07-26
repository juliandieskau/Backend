#include "ects/Common.hpp"

namespace ects::common {

Position2d::Position2d(double x, double y, double radius)
    : x(x), y(y), radius(radius) {}
double Position2d::get_x() const { return x; }
double Position2d::get_y() const { return y; }
double Position2d::get_radius() const { return radius; }

Heading2d::Heading2d(double heading, double accuracy)
    : heading(heading), accuracy(accuracy) {}
double Heading2d::get_heading() const { return heading; }
double Heading2d::get_accuracy() const { return accuracy; }

} // namespace ects::common
