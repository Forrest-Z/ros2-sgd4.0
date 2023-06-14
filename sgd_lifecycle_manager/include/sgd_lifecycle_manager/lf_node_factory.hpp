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

#ifndef SGD_LIFECYCLE__LF_Node_Factory_HPP_
#define SGD_LIFECYCLE__LF_Node_Factory_HPP_

#include <fstream>
#include <iostream>
#include <vector>
#include <memory>

#include "tinyxml2.h"

namespace sgd_lifecycle
{

class lifecycle_node
{
private:
    const std::string node_name_;
public:
    lifecycle_node(const char * node_name)
        : node_name_(node_name)
    {
        state = 1;  // set state to unconfigured
    }

    ~lifecycle_node() {}

    uint8_t state;

    std::string get_node_name() {
        return node_name_;
    }
};

class LF_Node_Factory
{

public:
    LF_Node_Factory(std::string launch_xml_file, bool is_sim = false)
        : launch_file_(launch_xml_file), is_sim_(is_sim) {}
    ~LF_Node_Factory() {}

    //! \brief Import xml file containing the nodes to launch
    //! \throw XmlError when an error during import occurs
    void import_launch_file();

    //! \brief returns a pointer to the next node from the nodelist or a nullptr if the end is reached
    lifecycle_node * next_node();

    //! \brief Returns the lowest state in the nodelist
    uint8_t get_lowest_state();

    //! \brief Check whether a node has a certain state or not. Returns -1 to indicate
    //! that the node is in a lower state, 0 to indicate the node is in the expected state
    //! and the state of the node to indicate the node is in a higher state
    uint8_t cmp_node_state(std::string node_name, uint8_t state);

protected:
    // parameters
    bool is_sim_;
    const std::string launch_file_;
    std::vector<lifecycle_node> nodelist_;  // contains all nodes
    uint it_;

    void read_xml_file(tinyxml2::XMLElement * node, lifecycle_node *g = nullptr);
};

}

#endif  // SGD_LIFECYCLE__LF_Node_Factory_HPP_
