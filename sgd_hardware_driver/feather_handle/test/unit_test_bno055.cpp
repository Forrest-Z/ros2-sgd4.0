// Copyright 2022 HAW Hamburg
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

#include <iostream>
#include <fstream>
#include <string>
#include "gtest/gtest.h"

#include "feather_handle/bno055.hpp"

static std::string BASE_PATH("");
static const double error = 1E-6;

TEST(NmeaParserTest, ParseJSON)
{
    auto bno = std::make_unique<sgd_hardware_drivers::BNO055>();

    // read json file
    std::string json;
    std::ifstream file(BASE_PATH + "ok.json");
    if (file.is_open())
    {
        while (getline(file, json))
        {
            ASSERT_NO_THROW(bno->parse_msg(json));
        }
        file.close();
    }
}

int main(int argc, char **argv)
{
    auto exe = std::string(argv[0]);
    BASE_PATH = exe.substr(0, exe.find_last_of("/")) + "/test/data/";

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}