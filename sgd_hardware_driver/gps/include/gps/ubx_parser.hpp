
#ifndef SGD_HARDWARE__UBX_PARSER_HPP_
#define SGD_HARDWARE__UBX_PARSER_HPP_

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

#include "IGPS_Message.hpp"

namespace sgd_hardware
{

class Ubx_Parser : public IGPS_Message
{

typedef enum {
    NAV = 0x01,
    RXM = 0x02,
    INF = 0x04,
    ACK = 0x05,
    CFG = 0x06,
    MON = 0x0A,
    AID = 0x0B,
    TIM = 0x0D
} NUM_FORMAT;

public:
    Ubx_Parser(std::string xml_file);
    ~Ubx_Parser() = default;

    void parse_msg(std::string msg) override;

protected:
    uint8_t msg_counter_;

    rapidxml::xml_document<> osm;
    rapidxml::xml_node<> * root;

    std::unordered_map<std::string, std::vector<NMEA_PARAM>> sentence_ids_;

    std::vector<std::string> split(const std::string& s, char delimiter);
    
    //! \brief Get value of attribute.
    //! \param node The node to get attribute of.
    //! \param name The name of the attribute
    //! \returns Attribute value as string or empty string if node has no such attribute.
    std::string get_attribute_value(rapidxml::xml_node<> *node, std::string name = "");

};

}       // namespace sgd_hardware

#endif