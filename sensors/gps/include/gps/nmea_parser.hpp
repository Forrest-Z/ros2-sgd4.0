


#ifndef NMEA_PARSER_HPP_
#define NMEA_PARSER_HPP_

#include <string>
#include <vector>
#include <sstream>

#include "rapidxml/"

namespace sensor_gps
{

class Nmea_Parser
{
public:
    Nmea_Parser();
    ~Nmea_Parser();

protected:
    std::vector<std::string> split(const std::string& s, char delimiter);
};

}       // namespace sensor_gps

#endif