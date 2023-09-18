#ifndef SGD_LOCALIZATION_POINT_HPP_
#define SGD_LOCALIZATION_POINT_HPP_

#include <math.h>
#include "geometry_msgs/msg/point32.hpp"

namespace sgd_localization
{

class Point
{
private:

public:
    const float x, y;

    /**
     * @brief Construct a new Point from x and y coordinates
     * 
     * @param x 
     * @param y 
     */
    Point(float x_, float y_) : x(x_), y(y_) {}

    /**
     * @brief Construct a new Point from ros2 geometry_msg
     * 
     * @param geometry_msg
     */
    Point(geometry_msgs::msg::Point32 pnt) : x(pnt.x), y(pnt.y) {}

    /**
     * @brief Angular coordinate phi
     * 
     * @return float 
     */
    float get_polar_phi(float robo_x, float robo_y, float robo_yaw)
    {
        return atan2(y-robo_y, x-robo_x) - robo_yaw;
    }

    /**
     * @brief Radial coordinate r
     * 
     * @return float 
     */
    float get_polar_radius(float robo_x, float robo_y)
    {
        return std::hypotf(x-robo_x, y-robo_y);
    }

    Point to_frame(float transform_x, float transform_y, float yaw)
    {
        float x_ = cos(yaw)*x - sin(yaw)*y + transform_x;
        float y_ = cos(yaw)*y + sin(yaw)*x + transform_y;
        return Point(x_, y_);
    }

    bool equals(Point another)
    {
        return (x == another.x && y == another.y);
    }
};

} // namespace sgd_local_costmap

#endif