
#include "gps/nmea_parser.hpp"

namespace sensor_gps
{

Nmea_Parser::Nmea_Parser(){
/*
    1. XML einlesen

*/
}

Nmea_Parser::~Nmea_Parser() {
    
}

std::vector<std::string>
Nmea_Parser::split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
}

}   // namespace sensor_gps