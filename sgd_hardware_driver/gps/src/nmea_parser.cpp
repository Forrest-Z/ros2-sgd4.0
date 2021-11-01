
#include "gps/nmea_parser.hpp"
#include <memory>

namespace sgd_hardware
{

Nmea_Parser::Nmea_Parser(std::string xml_file)
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

Nmea_Parser::~Nmea_Parser() {
    
}

int
Nmea_Parser::parse_line(std::string line)
{
    if (line.front() != '$') // || line.back() != '\0')
    {
        //std::cout << "Badly formatted string." << std::endl;
        //std::cout << "Line has to start with '$'. First char is " << line.front() << std::endl;
        return -1;
    }

    line.erase(line.begin());
    std::vector<std::string> msg = split(line, ',');

    auto sid_ = sentence_ids_.find(msg.front());
    if (sid_ != sentence_ids_.end())
    {
        msg.erase(msg.begin());
        msg_counter_++;
        
        //int i = 0;
        std::vector<NMEA_PARAM> v = sid_->second;
        for (uint i = 0; i < v.size(); i++)
        {
            // Parse param
            add_param(v[i].name(), v[i].param(msg[i]));
        }
    }
    return 0;
}

void
Nmea_Parser::clear()
{
    data_string_.clear();
    data_int_.clear();
    data_double_.clear();
    msg_counter_ = 0;
}

template<class T>
T Nmea_Parser::get_data(std::string param_name)
{
    std::cerr << "No data available for param " << param_name << std::endl;
    return NULL;
}

template<>
int Nmea_Parser::get_data<int>(std::string param_name)
{
    return data_int_.find(param_name) != data_int_.end() ? data_int_.find(param_name)->second : 0;
}

template<>
double Nmea_Parser::get_data<double>(std::string param_name)
{
    return data_double_.find(param_name) != data_double_.end() ? data_double_.find(param_name)->second : 0;
}

template<>
std::string Nmea_Parser::get_data<std::string>(std::string param_name)
{
    return data_string_.find(param_name) != data_string_.end() ? data_string_.find(param_name)->second : 0;
}

double
Nmea_Parser::time()
{
    return get_data<double>("time");
}

double 
Nmea_Parser::latitude()
{
    return get_data<double>("latitude");
}   

double
Nmea_Parser::longitude()
{
    return get_data<double>("longitude");
}

int
Nmea_Parser::fix()
{
    return get_data<int>("fix");
}

bool
Nmea_Parser::msg_complete()
{
    return msg_counter_ == sentence_ids_.size();
}


// ##########
// private methods
// ##########

std::vector<std::string>
Nmea_Parser::split(const std::string& s, char delimiter) {
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
Nmea_Parser::get_attribute_value(rapidxml::xml_node<> *node, std::string name)
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

void
Nmea_Parser::add_param(std::string name, std::variant<std::string, int, double> value)
{
    if (name.length() == 0) {return;}
    
    if (std::holds_alternative<std::string>(value))
    {
        data_string_.insert(std::make_pair(name, std::get<std::string>(value)));
    }
    else if (std::holds_alternative<int>(value))
    {
        data_int_.insert(std::make_pair(name, std::get<int>(value)));
    }
    else
    {
        data_double_.insert(std::make_pair(name, std::get<double>(value)));
    }
}

}   // namespace sgd_sensors
