
#ifndef NMEA_PARSER_HPP_
#define NMEA_PARSER_HPP_

#include <fstream>
#include <iostream>

#include <regex>

#include <sstream>
#include <string>
#include <vector>
#include <sstream>

#include <unordered_map>

#include <variant>

#include "rapidxml/rapidxml.hpp"
#include "gps/nmea_param.hpp"

namespace sgd_sensors
{

class Nmea_Parser
{

public:
    Nmea_Parser(std::string xml_file);
    ~Nmea_Parser();

    //! \brief Parse a line of a NMEA message.
    //! \param line A zero terminated string. Has to start with '$'.
    int parse_line(std::string line);

    //! \brief Returns timestamp of last valid gps message.
    double time();

    //! \brief Returns latitude of last valid gps message.
    double latitude();

    //! \brief Returns longitude of last valid gps message.
    double longitude();

    //! \brief Returns true if gps signal is fixed.
    int fix();

    //! \brief Returns the number of messages to parse.
    bool msg_complete();

    //! \brief Get additional data from last valid gps message.
    //! \param param_name Parameter to search for
    //! \returns Requested data as string or NULL if requested parameter is not valid.
    template<class T>
    T get_data(std::string param_name);

    //! \brief Clear all data. 
    void clear();

protected:
    uint8_t msg_counter_;

    std::unordered_map<std::string, std::string> data_string_;
    std::unordered_map<std::string, double> data_double_;
    std::unordered_map<std::string, int> data_int_;

    rapidxml::xml_document<> osm;
    rapidxml::xml_node<> * root;

    std::unordered_map<std::string, std::vector<NMEA_PARAM>> sentence_ids_;

    std::vector<std::string> split(const std::string& s, char delimiter);

    void add_param(std::string name, std::variant<std::string, int, double> value);

    //! \brief Get value of attribute.
    //! \param node The node to get attribute of.
    //! \param name The name of the attribute
    //! \returns Attribute value as string or empty string if node has no such attribute.
    std::string get_attribute_value(rapidxml::xml_node<> *node, std::string name = "");

};

}       // namespace sgd_sensors

#endif