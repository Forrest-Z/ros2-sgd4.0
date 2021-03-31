
#include "sgd_util/sgd_util.hpp"

namespace sgd_util
{

std::pair<double, double>
WGS84_to_local(double lat, double lon)
{
  double x = (lon - 10.021944) * 66269.83554449;
  double y = (lat - 53.555833) * 111293.78937166;

  return std::make_pair(x,y);
}

std::pair<double, double>
local_to_WGS84(double x, double y)
{
  double lat = y / 111293.78937166 + 53.555833;
  double lon = x / 66269.83554449 + 10.021944;

  return std::make_pair(lat, lon);
}

geometry_msgs::msg::Quaternion
rotation_around_z(double angle_z)
{
  tf2::Quaternion q;
  q.setRPY(0,0,angle_z);
  return tf2::toMsg(q);
}


}   // namespace sgd_util