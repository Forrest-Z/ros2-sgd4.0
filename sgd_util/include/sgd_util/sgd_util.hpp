#ifndef SGD_UTIL_HPP_
#define SGD_UTIL_HPP_

#include <utility>

#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace sgd_util
{

//! \brief Convert from local coordinate system to WGS84 coordinate system.
//! \param x x coordinate
//! \param y y coordinate
//! \return pair with latitude and longitude
std::pair<double, double> local_to_WGS84(double x, double y);

//! \brief Convert from WGS84 coordinate system to local coordinate system.
//! \param lat latitude
//! \param lon longitude
//! \return pair with x and y coordinates
std::pair<double, double> WGS84_to_local(double lat, double lon);

//! \brief Calculate quaternion from rotation around z axis.
//! \param angle_z angle around z axis
//! \return Quaternion as standard ros2 geometry msg
geometry_msgs::msg::Quaternion rotation_around_z(double angle_z);

}   // namespace sgd_util
#endif