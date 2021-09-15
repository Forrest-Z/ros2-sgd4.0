
#include "gps/ubx_parser.hpp"
#include <memory>

namespace sgd_hardware
{

Ubx_Parser::Ubx_Parser(std::string xml_file)
{
    std::ifstream t(xml_file);

    std::vector<char> buffer = std::vector<char>((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');
    osm.parse<0>(&buffer[0]);

    root = osm.first_node(0);
    
    for (rapidxml::xml_node<> *node = root->first_node("sentenceId"); node; node = node->next_sibling())
    {
        if (get_attribute_value(node, "parse") == "true")
        {
            std::vector<NMEA_PARAM> v;
            for (rapidxml::xml_node<> *n = node->first_node(); n; n = n->next_sibling())
            {
                std::string s(n->value());
                NMEA_PARAM p(get_attribute_value(n, "name"), get_attribute_value(n, "type"), s);
                v.push_back(p);
            }
            sentence_ids_.insert(std::make_pair(get_attribute_value(node, "name"), v));
        }
    }  
    msg_counter_ = 0;  
}

Ubx_Parser::~Ubx_Parser() {
    
}

void
Ubx_Parser::parse_msg(std::string msg)
{
    if (msg.front() != '$') // || line.back() != '\0')
    {
        //std::cout << "Badly formatted string." << std::endl;
        //std::cout << "Line has to start with '$'. First char is " << line.front() << std::endl;
        return;
    }

    msg.erase(msg.begin());
    std::vector<std::string> msg_split = split(msg, ',');

    auto sid_ = sentence_ids_.find(msg_split.front());
    if (sid_ != sentence_ids_.end())
    {
        msg_split.erase(msg_split.begin());
        msg_counter_++;
        
        //int i = 0;
        std::vector<NMEA_PARAM> v = sid_->second;
        for (uint i = 0; i < v.size(); i++)
        {
            // Parse param
            add_param(v[i].name(), v[i].param(msg_split[i]));
        }
    }
    return;
}


// ##########
// private methods
// ##########

std::vector<std::string>
Ubx_Parser::split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

std::string
Ubx_Parser::get_attribute_value(rapidxml::xml_node<> *node, std::string name)
{
    rapidxml::xml_attribute<> *att;
    if (name.length() <= 0)
    {
        att = node->first_attribute(0);
    }
    else
    {
        att = node->first_attribute(name.c_str());
    }

    return att == nullptr ? "" : att->value();
}

}   // namespace sgd_sensors
