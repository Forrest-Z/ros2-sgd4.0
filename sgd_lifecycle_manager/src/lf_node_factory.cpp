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

#include "sgd_lifecycle_manager/lf_node_factory.hpp"

namespace sgd_lifecycle
{

void
LF_Node_Factory::import_launch_file() {
    tinyxml2::XMLDocument doc;
    doc.LoadFile(launch_file_.c_str());

    if (doc.ErrorID() != 0)
    {
        throw sgd_util::XmlError(doc.ErrorStr());
    }

    auto root = doc.RootElement();     // <launch>
    if (root != NULL)
    {
        read_xml_file(root);
    } else
    {
        //RCLCPP_INFO(get_logger(), "Root element is NULL");
    }
    it_ = 0;
}

uint8_t
LF_Node_Factory::get_lowest_state() {
    uint8_t lowest_state_ = 0;
    for (auto n : nodelist_)
    {
        lowest_state_ = lowest_state_ < n.state ? lowest_state_ : n.state;
    }
    return lowest_state_;
}

uint8_t
LF_Node_Factory::cmp_node_state(std::string node_name, uint8_t state) {
    for (auto n : nodelist_)
    {
        if (n.get_node_name() == node_name)
        {
            if (n.state < state)
            {
                return -1;
            } else if (n.state > state)
            {
                return n.state;
            } else
            {
                return 0;
            }
        }
    }
    return -1;
}

lifecycle_node *
LF_Node_Factory::next_node() {
    if (it_ < nodelist_.size())
    {
        return &nodelist_.at(it_++);
    } else
    {
        it_ = 0;
        return nullptr;
    }
}

void
LF_Node_Factory::read_xml_file(tinyxml2::XMLElement * node, lifecycle_node *n) {
    tinyxml2::XMLElement * nd = node->FirstChildElement("node");
    while (nd != NULL)
    {
        // TODO add node dependencies

        // check start attribute
        std::string start = nd->Attribute("start") != NULL ? nd->Attribute("start") : "";

        if ((is_sim_ && start == "real") || (!is_sim_ && start == "sim"))
        {
            // skip this node
            nd = nd->NextSiblingElement();
            continue;
        }

        // create lifecycle node
        nodelist_.push_back(lifecycle_node(nd->Attribute("name")));

        nd = nd->NextSiblingElement();
    }
}

}   // namespace sgd_lifecycle