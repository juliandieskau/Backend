#include "ects/Common.hpp"

namespace ects::common {

Position2d::Position2d(double x, double y, double radius)
    : x(x), y(y), radius(radius) {}
double Position2d::get_x() { return x; }
double Position2d::get_y() { return y; }
double Position2d::get_radius() { return radius; }

Heading2d::Heading2d(double heading, double accuracy)
    : heading(heading), accuracy(accuracy) {}
double Heading2d::get_heading() { return heading; }
double Heading2d::get_accuracy() { return accuracy; }

} // namespace ects::common
