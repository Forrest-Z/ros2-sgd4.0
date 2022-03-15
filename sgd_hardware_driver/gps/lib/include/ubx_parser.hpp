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

#ifndef GPS__UBX_PARSER_HPP_
#define GPS__UBX_PARSER_HPP_

#include <fstream>
#include <iostream>

#include <regex>

#include <sstream>
#include <string>
#include <vector>
#include <sstream>

#include <unordered_map>

#include <variant>

#include "tinyxml2.h"

#include "nmea_param.hpp"

#include "IGPS_Message.hpp"

namespace sgd_hardware_drivers
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
    Ubx_Parser();
    ~Ubx_Parser() = default;

    int import_xml(std::string xml_file) override;

    int parse_msg(std::string msg) override;

    bool msg_complete() override;

protected:
    std::unordered_map<std::string, std::vector<NMEA_PARAM>> sentence_ids_;
    std::vector<std::string> split(const std::string& s, char delimiter);
};

}       // namespace sgd_hardware

#endif