
#include "sgd_util/sgd_util.hpp"

namespace sgd_util
{

#define ORIG_LAT 53.55526851472951
#define ORIG_LON 10.021989122747957

std::pair<double, double>
WGS84_to_local(double lat, double lon)
{
  double x = (lon - ORIG_LON) * 66269.83554449;
  double y = (lat - ORIG_LAT) * 111293.78937166;

  return std::make_pair(x,y);
}

std::pair<double, double>
local_to_WGS84(double x, double y)
{
  // Nullpunkt: 53.55526851472951, 10.021989122747957

  double dist_lat = 6378137 * 3.14159 / 180;
  double lat = y / dist_lat + ORIG_LAT;
  double lon = x / (dist_lat * cos(ORIG_LAT/180*3.14159)) + ORIG_LON;

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