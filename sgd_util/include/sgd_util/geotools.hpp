// Copyright 2022 HAW Hamburg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SGD_UTIL__GEO_TOOLS
#define SGD_UTIL__GEO_TOOLS

#include <string>
#include <vector>
#include <math.h>
#include <iostream>

namespace sgd_util
{

class LatLon
{
private:
    static constexpr double LATLON_TO_METER = M_PI / 180.0 * 6378137.0;
    static constexpr double METER_TO_LATLON = 1.0 / LATLON_TO_METER;

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
     * @brief Calculate the euclidian distance from this Lat/Lon to the other Lat/Lon and returns the
     * distance in meters.
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
    std::pair<double, double> to_local(const LatLon origin);

    /**
     * @brief Transform WGS84 coordinate system into local coordinate system.
     * 
     * @param lat 
     * @param lon 
     * @return std::pair<double, double> 
     */
    std::pair<double, double> to_local(const double lat, const double lon);

    /**
     * @brief Interpolate between two points. The interpolation is based on the euclidian distance,
     * so be careful when using this for large distances
     * 
     * @param other the LatLon to interpolate to
     * @param points_to_insert how many points to insert, defaults to 1
     * @return the vector containing only the new points
     */
    std::vector<LatLon> interpolate(LatLon other, int points_to_insert = 1);

    /**
     * @brief Calculates the bearing between the path from this position to the other position. The north axis is defined as 0.
     * 
     * @param other 
     * @return bearing in the interval [0;2*PI]
     */
    double bearing(LatLon other);

    /**
     * @brief Create string <lat, lon> with 7 digits precision
     * 
     * @return string
     */
	std::string to_string();
};

}   // namespace sgd_util

#endif  // SGD_UTIL__GEO_TOOLS