// Copyright 2021 HAW Hamburg
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

#ifndef GPS__NMEA_PARAM_HPP_
#define GPS__NMEA_PARAM_HPP_

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
    NMEA_PARAM(const char * name, const char * type, const char * regex)
    {
        if (name != NULL)
        {
            name_ = std::string(name);
        } else {
            name_ = "";
        }

        if (type != NULL)
        {
            type_ = std::string(type);
        } else {
            type_ = "";
        }

        if (regex != NULL)
        {
            regex_ = std::string(regex);
        } else {
            regex_ = "";
        }
    }

    ~NMEA_PARAM() = default;

    //! \brief Tries to find a match in p, compute it and returns the value.
    //! \param p The string to match
    //! \returns A string, int, double or time value according to NMEA_PARAM.type
    std::variant<int, double, std::string> parse_param(std::string p)
    {
        if (name_.length() < 1)
        {
            return 0;
        }

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
                double l = std::stoi(matches[1]) + std::stod(matches[2])/60;
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
            std::string s("Error parsing param ");
            s.append(name_);
            s.append(". Type: ");
            s.append(type_);
            s.append(", value: ");
            s.append(matches[0]);
            throw std::invalid_argument(s);
        }
        std::string s("Error parsing param ");
        s.append(name_);
        s.append(". Type: ");
        s.append(type_);
        s.append(", value: ");
        s.append(matches[0]);
        throw std::invalid_argument(s);
    }

    std::string name()
    {
        return name_;
    }
    
    std::string type()
    {
        return type_;
    }
private:
    std::string name_;
    std::string type_;
    std::regex regex_;
};

#endif