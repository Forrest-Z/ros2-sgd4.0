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

#include "gnss/ubx_parser.hpp"
#include <memory>

namespace sgd_hardware_drivers
{

Ubx_Parser::Ubx_Parser()
{
    
}

int
Ubx_Parser::import_xml(std::string xml_file)
{
    tinyxml2::XMLDocument doc;
    doc.LoadFile(xml_file.c_str());

    // check error
    if (doc.ErrorID())
    {
        std::cerr << doc.ErrorStr();
        return doc.ErrorID();
    }
    return doc.ErrorID();
}

int
Ubx_Parser::parse_msg(std::string msg)
{
    if (msg.front() != '$') // || line.back() != '\0')
    {
        //std::cout << "Badly formatted string." << std::endl;
        //std::cout << "Line has to start with '$'. First char is " << line.front() << std::endl;
        return 1;
    }

    msg.erase(msg.begin());
    std::vector<std::string> msg_split = split(msg, ',');

    auto sid_ = sentence_ids_.find(msg_split.front());
    if (sid_ != sentence_ids_.end())
    {
        msg_split.erase(msg_split.begin());
        
        //int i = 0;
        std::vector<NMEA_PARAM> v = sid_->second;
        for (uint i = 0; i < v.size(); i++)
        {
            // Parse param
            //add_data(v[i].name(), v[i].param(msg_split[i]));
        }
    }
    return 0;
}

bool
Ubx_Parser::msg_complete()
{
    return true;
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

}   // namespace sgd_sensors
