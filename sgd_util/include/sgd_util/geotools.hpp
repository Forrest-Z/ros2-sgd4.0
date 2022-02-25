#ifndef SGD_UTIL__GEO_TOOLS
#define SGD_UTIL__GEO_TOOLS

#include <string>
#include <vector>
#include <math.h>

namespace sgd_util
{

class LatLon
{
private:
    static constexpr double LATLON_TO_METER = M_PI / 180.0 * 6378137.0;
    static constexpr double METER_TO_LATLON = 1 / LATLON_TO_METER;

    double lat_, lon_;

public:
    LatLon();
    LatLon(double lat, double lon);
    ~LatLon();

    double lat();
    double lon();

    void set_global_coordinates(double lat, double lon);
    void set_local_coordinates(LatLon origin, double x, double y);

    /**
     * @brief Calculate the euclidian distance from this Lat/Lon to the other Lat/Lon
     * 
     * @param another_latlon the other Lat/Lon
     * @return double the distance
     */
    double distance_to(LatLon another_latlon);

    /**
     * @brief Calculate the euclidian distance from this Lat/Lon to the other Lat/Lon and returns the
     * distance in meters.
     * 
     * @param lat 
     * @param lon 
     * @return double 
     */
    double distance_to(double lat, double lon);

    /**
     * @brief Transform WGS84 coordinate system into local coordinate system.
     * 
     * @param origin the latlon from which the local coordinates are to be computed
     * @return std::pair<double, double> 
     */
    std::pair<double, double> to_local(LatLon origin);

    /**
     * @brief Transform WGS84 coordinate system into local coordinate system.
     * 
     * @param lat 
     * @param lon 
     * @return std::pair<double, double> 
     */
    std::pair<double, double> to_local(double lat, double lon);

    /**
     * @brief Create string <lat, lon> with 7 digits precision
     * 
     * @return string
     */
	std::string to_string();
};

}   // namespace sgd_util

#endif  // SGD_UTIL__GEO_TOOLS