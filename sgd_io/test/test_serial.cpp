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

#include "gtest/gtest.h"

#include "sgd_io/serial.hpp"

static std::string BASE_PATH("");

TEST(SerialTest, NormalMessage)
{
    // create file
    // initialize Serial with created file
    // write to file
    // get msg from serial
}

TEST(SerialTest, VeryLongMessage)
{
}

TEST(SerialTest, EmptyMessage)
{
}

TEST(SerialTest, RawMessage)
{
}

int main(int argc, char **argv)
{
    auto exe = std::string(argv[0]);
    BASE_PATH = exe.substr(0, exe.find_last_of("/")) + "/test/data";

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}