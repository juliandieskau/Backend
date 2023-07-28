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

auto Twist::from_ros(const Twist::ros_t &ros) -> Twist {
    return {Vector3D::from_ros(ros.linear), Vector3D::from_ros(ros.angular)};
}
auto Twist::to_ros(const Twist &tw) -> Twist::ros_t {
    Twist::ros_t r;
    r.linear = Vector3D::to_ros(tw.linear);
    r.angular = Vector3D::to_ros(tw.angular);
    return r;
}
Twist::Twist(Vector3D linear, Vector3D angular)
    : linear(linear), angular(angular) {}

auto Vector3D::from_ros(const Vector3D::ros_t &r) -> Vector3D {
    return {r.x, r.y, r.z};
}
auto Vector3D::to_ros(const Vector3D &v) -> Vector3D::ros_t {
    Vector3D::ros_t r;
    std::tie(r.x, r.y, r.z) = std::tuple{v.x, v.y, v.z};
    return r;
}
Vector3D::Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}
} // namespace ects::common
