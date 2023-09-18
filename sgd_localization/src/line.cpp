#include "sgd_localization/line.hpp"

namespace sgd_localization
{

Line::Line(Point p1, Point p2) : p1_(p1.x, p1.y), p2_(p2.x, p2.y)
{
    // empty constructor
}

float
Line::length()
{
    return std::hypotf(p1_.x-p2_.x, p1_.y-p2_.y);
}

float
Line::angle(Line other)
{
    float a = atanf32((other.p1_.y - other.p2_.y) / (other.p1_.x - other.p2_.x))
            - atanf32((p1_.y - p2_.y) / (p1_.x - p2_.x));

    return (a > M_PI_2) ? (a - M_PI) : (a < -M_PI_2 ? a + M_PI : a);
}

} // namespace sgd_localization
