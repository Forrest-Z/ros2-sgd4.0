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

#include <string>
#include "gtest/gtest.h"

#include "sgd_lifecycle_manager/lf_node_factory.hpp"
#include "sgd_util/exception.hpp"

static std::string BASE_PATH("");

TEST(LfNodeFactory, ImportXmlFail)
{
    sgd_lifecycle::LF_Node_Factory lf(BASE_PATH + "/launch_fail.xml");
    ASSERT_THROW(lf.import_launch_file(), sgd_util::XmlError);
}

TEST(LfNodeFactory, GetAndSetNodeState)
{
    sgd_lifecycle::LF_Node_Factory lf(BASE_PATH + "/launch_ok.xml");
    ASSERT_NO_THROW(lf.import_launch_file());

    sgd_lifecycle::lifecycle_node * nd;
    int counter = 0;
    while ((nd = lf.has_node()) != nullptr)
    {
        counter++;
        EXPECT_EQ(nd->state, 1) << "Node " << nd->get_node_name() << " expected state 1, got state " << nd->state;
    }
    EXPECT_EQ(counter, 19) << "Could not read all nodes.";

    // change node state
    counter = 1;
    while ((nd = lf.has_node()) != nullptr)
    {
        nd->state = counter;
        counter++;
    }

    counter = 1;
    while ((nd = lf.has_node()) != nullptr)
    {
        EXPECT_EQ(nd->state, counter) << "Node " << nd->get_node_name() << " expected state " << counter << ", got state " << nd->state;
        counter ++;
    }
}

int main(int argc, char **argv)
{
    auto exe = std::string(argv[0]);
    BASE_PATH = exe.substr(0,exe.find_last_of("/")) + "/test/data";

    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}