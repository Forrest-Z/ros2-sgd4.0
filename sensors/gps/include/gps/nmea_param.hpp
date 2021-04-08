#ifndef NMEA_PARAM_HPP_
#define NMEA_PARAM_HPP_

#include <chrono>
#include <regex>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <sstream>

#include <unordered_map>

#include <variant>



class NMEA_PARAM
{
public:
    NMEA_PARAM(std::string name, std::string type, std::string regex)
        : name_(name), type_(type)
    {
        regex_ = std::regex(regex);
    }
    ~NMEA_PARAM() {    }

    //! \brief Tries to find a match in p, compute it and returns the value.
    //! \param p The string to match
    //! \returns A string, int, double or time value according to NMEA_PARAM.type
    std::variant<std::string, int, double> param(std::string p)
    {
        if (name_.length() == 0) {return 0;}

        std::smatch matches;
        std::regex_match(p, matches, regex_);

        try
        {
            if (type_ == "time" && matches.size() >= 4)     // hh mm ss.sss
            {
                double sec = std::stoi(matches[1])*60*60 + std::stoi(matches[2])*60 + std::stod(matches[3]);
                return sec;
            }
            else if (type_ == "latlon" && matches.size() >= 3)  // dd mm.mmm
            {
                double l = std::stoi(matches[1]) + std::stod(matches[2])*5/300;
                return l;
            }
            else if (type_ == "int" && matches.ready())
            {
                int i = std::stoi(matches[0]);
                return i;
            }
            else if (type_ == "double" && matches.ready())
            {
                double d = std::stod(matches[0]);
                return d;
            }
            else if (matches.ready())   // type string / char
            {
                return matches[0];
            }
        } catch (const std::invalid_argument& e)
        {
            std::cerr << e.what() << std::endl;
            std::cerr << "Type " << type_ << " value " << matches[0] << std::endl;
            return 0;
        }
        std::cerr << "Error parsing param " << name_ << ". Value is " << p << std::endl;
        return 0;
    }

    std::string name() {return name_;}
    std::string type() {return type_;}
private:
    const std::string name_, type_;
    std::regex regex_;
};

#endif