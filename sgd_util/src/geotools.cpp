#include "sgd_util/geotools.hpp"

namespace sgd_util
{

LatLon::LatLon()
{
    lat_ = 0.0;
    lon_ = 0.0;
}

LatLon::LatLon(const double lat, const double lon) :
    lat_(lat), lon_(lon)
{}

LatLon::~LatLon() {}

void
LatLon::set_global_coordinates(const double lat, const double lon)
{
    lat_ = lat;
    lon_ = lon;
}

void
LatLon::set_local_coordinates(const LatLon origin, const double x, const double y)
{
    // convert local to WGS84
    lat_ = origin.lat_ + y * METER_TO_LATLON;
    lon_ = origin.lon_ + x * METER_TO_LATLON / cos((lat_) * M_PI/180);
}

double
LatLon::distance_to(const LatLon another_latlon) const
{
    auto xy = to_local(another_latlon);
    //return std::hypot(xy.first, xy.second);
    return std::sqrt(std::pow(xy.first, 2) + std::pow(xy.second, 2));
}

double
LatLon::distance_to(const double lat, const double lon) const
{
    return distance_to(LatLon(lat, lon));
}

std::pair<double, double>
LatLon::to_local(const LatLon origin) const
{
    return to_local(origin.lat_, origin.lon_);
}

std::pair<double, double>
LatLon::to_local(const double lat, const double lon) const
{
    double x = (lon_ - lon) * LATLON_TO_METER * cos((lat + lat_) * M_PI/360);
    double y = (lat_ - lat) * LATLON_TO_METER;
    return {x,y};
}

double
LatLon::lat() {
    return lat_;
}

double
LatLon::lon() {
    return lon_;
}

std::vector<LatLon>
LatLon::interpolate(const LatLon other, int points_to_insert)
{
    // TODO: Take into account the spherical curvature of the earth...
    // Only if we're gonna interpolate points more than 5 km apart.
    std::vector<LatLon> vec_ll;
    double delta_lat = other.lat_ - lat_;
    double delta_lon = other.lon_ - lon_;
    for (int i = 0; i < points_to_insert; i++)
    {
        LatLon ll(lat_ + delta_lat*(i+1)/(points_to_insert+1), 
                lon_ + delta_lon*(i+1)/(points_to_insert+1));
        vec_ll.push_back(ll);
    }
    return vec_ll;
}

double
LatLon::bearing(const LatLon other) const
{
    auto ang = atan2((other.lat_-lat_)/180*M_PI, (other.lon_-lon_)/180*M_PI*cos(lat_/180*M_PI)) * -1.0 + M_PI_2;
    if (ang < 0)
    {
        ang += 2*M_PI;
    } else if (ang > 2*M_PI)
    {
        ang -= 2*M_PI;
    }

    return ang;
}

std::string
LatLon::to_string() const
{
    const char *out = "%3.7f, %3.7f";
    int sz = std::snprintf(nullptr, 0, out, lat_, lon_);
    char buf[sz + 1]; // note +1 for null terminator
    std::snprintf(&buf[0], sz+1, out, lat_, lon_);
    return std::string(buf);
}

}
