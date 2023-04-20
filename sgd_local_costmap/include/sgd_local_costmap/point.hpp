#ifndef SGD_LOCAL_COSTMAP_POINT_HPP
#define SGD_LOCAL_COSTMAP_POINT_HPP

#include <math.h>

namespace sgd_local_costmap
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

    bool equals(Point another)
    {
        return (x == another.x && y == another.y);
    }
};

} // namespace sgd_local_costmap

#endif