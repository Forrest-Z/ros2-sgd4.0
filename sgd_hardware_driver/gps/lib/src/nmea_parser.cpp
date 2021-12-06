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

#include "nmea_parser.hpp"
#include <memory>

namespace sgd_hardware
{

Nmea_Parser::Nmea_Parser() {}

int
Nmea_Parser::import_xml(std::string xml_file)
{
    tinyxml2::XMLDocument doc;
    doc.LoadFile(xml_file.c_str());

    // check error
    if (doc.Error())
    {
        Error err(Error::XML_PARSE_ERROR, doc.ErrorStr());
        errors.push_back(err);
        return doc.ErrorID();
    }

    num_params = 0;
    if (doc.RootElement() != NULL)
    {
        // iterate through sentenceIds
        auto sentenceID = doc.RootElement()->FirstChildElement("sentenceId");     // <sentenceID ... >
        while (sentenceID != NULL)
        {
            if (sentenceID->Attribute("parse", "true") != NULL) // if parse=false skip this node
            {
                std::vector<NMEA_PARAM> v;
                tinyxml2::XMLElement * param = sentenceID->FirstChildElement("param");  // <param ... >
                
                while (param != NULL)   // iterate through params
                {
                    // save name and type attribute as NMEA_PARAM
                    NMEA_PARAM p(param->Attribute("name"), param->Attribute("type"), param->GetText());
                    v.push_back(p);

                    num_params += param->Attribute("name") != NULL ? 1 : 0;

                    param = param->NextSiblingElement("param");
                }
                sentence_ids_.insert(std::make_pair(sentenceID->Attribute("name"), v));
            }

            sentenceID = sentenceID->NextSiblingElement("sentenceId");
        }
    }

    return 0;
}

int
Nmea_Parser::parse_msg(std::string msg)
{
    if (msg.front() != '$') // || line.back() != '\0')
    {
        Error err(Error::INVALID_ARG, "Badly formatted message: " + msg);
        errors.push_back(err);
        return 1;
    }

    msg.erase(msg.begin());
    std::vector<std::string> msgs = split(msg, ',');

    auto sid_ = sentence_ids_.find(msgs.front());
    if (sid_ != sentence_ids_.end())
    {
        msgs.erase(msgs.begin());

        std::vector<NMEA_PARAM> v = sid_->second;
        for (uint i = 0; i < v.size(); i++)
        {
            // Parse param
            try
            {
                add_data(v[i].name(), v[i].parse_param(msgs[i]));
            }
            catch (const std::invalid_argument& e)
            {
                Error err(Error::INVALID_ARG, e.what());
                errors.push_back(err);
            }
        }
    }
    return 0;
}

bool
Nmea_Parser::msg_complete()
{
    return num_values() == num_params;
}

// ##########
// protected methods
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

}   // namespace sgd_sensors
