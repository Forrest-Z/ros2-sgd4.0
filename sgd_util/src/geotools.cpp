#include "sgd_util/geotools.hpp"

namespace sgd_util
{

LatLon::LatLon()
{
    lat_ = 0.0;
    lon_ = 0.0;
}

LatLon::LatLon(double lat, double lon) :
    lat_(lat), lon_(lon)
{}

LatLon::~LatLon() {}

void
LatLon::set_global_coordinates(double lat, double lon)
{
    lat_ = lat;
    lon_ = lon;
}

void
LatLon::set_local_coordinates(LatLon origin, double x, double y)
{
    // convert local to WGS84
    lat_ = origin.lat_ + y * METER_TO_LATLON;
    lon_ = origin.lon_ + x * METER_TO_LATLON / cos((lat_) * M_PI/180);
}

double
LatLon::distance_to(LatLon another_latlon)
{
    auto xy = to_local(another_latlon);
    return std::sqrt(std::pow(xy.first, 2) + std::pow(xy.second, 2));
}

double
LatLon::distance_to(double lat, double lon)
{
    auto xy = to_local(lat, lon);
    return std::sqrt(std::pow(xy.first, 2) + std::pow(xy.second, 2));
}

std::pair<double, double>
LatLon::to_local(const LatLon origin) {
    return to_local(origin.lat_, origin.lon_);
}

std::pair<double, double>
LatLon::to_local(const double lat, const double lon) {
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
LatLon::interpolate(LatLon other, int points_to_insert)
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

std::string
LatLon::to_string() {
    const char *out = "%3.7f, %3.7f";
    int sz = std::snprintf(nullptr, 0, out, lat_, lon_);
    char buf[sz + 1]; // note +1 for null terminator
    std::snprintf(&buf[0], sz+1, out, lat_, lon_);
    return std::string(buf);
}

}
