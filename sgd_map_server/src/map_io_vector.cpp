// Copyright 2023 HAW Hamburg
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

/* Copyright 2019 Rover Robotics
 * Copyright 2010 Brian Gerkey
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sgd_map_server/map_io_vector.hpp"

namespace sgd_map_server
{

Map_IO_Vector::Map_IO_Vector(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
{
    // init regex to parse fill attribute and path coordinates
    regex_fill_ = "fill:#(\\d);.*";
    regex_path_ = "(-?\\d+\\.\\d+),(-?\\d+\\.\\d+)";

    PLOGD << "Map_IO_Vector initialized";
}

Map_IO_Vector::~Map_IO_Vector() {}

// === Map input part ===

void
Map_IO_Vector::loadMapFromFile(const LoadParameters & load_parameters)
{
    // get image name and change extension from png to svg
    std::string svg_name = load_parameters.image_file_name.substr(0, load_parameters.image_file_name.find_last_of('.')) + ".svg";

    // import file with tinyxml2
    PLOGI << "Loading image_file: " << svg_name;
    tinyxml2::XMLDocument doc;
    doc.LoadFile(svg_name.c_str());

    // check error
    if (doc.Error())
    {
        // TODO handle error
        PLOGE << "Error reading svg file: " << doc.ErrorIDToName(doc.ErrorID());
        return;
    }

    sgd_msgs::msg::VecObstacleArray msg;
    if (doc.RootElement() != NULL)
    {
        auto root = doc.RootElement();
        // width and height in m
        msg.info.width = std::strtoul(root->Attribute("width"), nullptr, 10);
        msg.info.height = std::strtoul(root->Attribute("height"), nullptr, 10);
        height_ = msg.info.height;
        PLOGD << "Image width: " << msg.info.width << "; height: " << msg.info.height;

        PLOGD << "Import svg: Object; Points | r;cx;cy";
        try
        {
            import_svg(doc.RootElement(), msg);
        }
        catch(const std::exception& e)
        {
            PLOGE << e.what();
            std::cerr << e.what() << '\n';
        }
        
        PLOGI << "Imported " << msg.obstacles.size() << " obstacles";
    }

    msg.info.resolution = load_parameters.resolution;
    msg.info.origin.position.x = load_parameters.origin[0];
    msg.info.origin.position.y = load_parameters.origin[1];
    msg.info.origin.position.z = 0.0;

    // Since loadMapFromFile() does not belong to any node, publishing in a system time.
    // rclcpp::Clock clock(RCL_SYSTEM_TIME);
    // msg.info.map_load_time = clock.now();
    msg.header.frame_id = "map";
    // msg.header.stamp = clock.now();

    PLOGD << "Read map " << load_parameters.image_file_name << ": " << msg.info.width << " X " << msg.info.height << " map @ " << msg.info.resolution << " m/cell";

    map = msg;
}

void
Map_IO_Vector::import_svg(tinyxml2::XMLElement * element, sgd_msgs::msg::VecObstacleArray &msg)
{
    // switch if group / circle / path
    auto elem = element->FirstChildElement();
    // get height
    auto height = element->Attribute("height");
    // TODO use transformation matrix

    // TODO: error handling for xml attributes
    while (elem != NULL)
    {
        auto name = std::string(elem->Name());
        if (name == "g")
        {
            // open group
            PLOGD << "Group:";
            import_svg(elem, msg);
        }
        else if (name == "circle")
        {
            // parse circle -> create circle
            // Circle c;
            sgd_msgs::msg::VecObstacle c;
            // TODO id
            c.confidence = std::stoi(elem->Attribute("cfd"));
            c.clss.data = std::string(elem->Attribute("class"));
            // get radius and position
            c.radius = char2float(elem->Attribute("r"));
            c.pose.position.x = char2float(elem->Attribute("cx"));
            c.pose.position.y = height_ - char2float(elem->Attribute("cy"));
            
            msg.obstacles.push_back(c);
            PLOGD << "Circle; " << c.radius << "; " << c.pose.position.x << "; " << c.pose.position.y;
        }
        else if (name == "path")
        {
            // parse path -> create polygon
            sgd_msgs::msg::VecObstacle p;
            // TODO id
            p.confidence = std::stoi(elem->Attribute("cfd"));
            p.clss.data = std::string(elem->Attribute("class"));

            std::stringstream path(elem->Attribute("d"));
            std::string s, debug_ = "Polygon";
            char mode = 0;
            while(getline(path, s, ' '))
            {
                // The M indicates a moveto, the Ls indicate linetos, and the z indicates a closepath.
                // example: d="m 62.358,241.096 8.233,10.664 -11.202,8.65 -8.233,-10.676 z"
                if (s.length() == 1)
                {
                    // get command: M/L/Z
                    mode = s.front();
                }
                else
                {
                    // length is > 1 -> coordinate pair
                    std::smatch matches;
                    std::regex_match(s, matches, regex_path_);

                    if (matches.size() < 3)
                    {
                        elem = elem->NextSiblingElement();
                        continue;
                    }

                    geometry_msgs::msg::Point32 pnt;
                    float x_ = std::stof(matches[1]);
                    float y_ = std::stof(matches[2]);
                    switch (mode)
                    {
                    case 'm':
                    {
                        // first coordinate pair is always absolute
                        if (!p.vertices.empty())
                        {
                            // implicit lineto command
                            // get last point
                            auto last_pnt_ = p.vertices.back();
                            pnt.x = last_pnt_.x + x_;
                            pnt.y = last_pnt_.y + y_;
                            break;
                        }
                    } // else fallthrough
                    case 'M':
                    {
                        // absolute line start
                        pnt.x = x_;
                        pnt.y = height_ - y_;
                        break;
                    }
                    case 'L':
                    {
                        // lineto command, absolute mode
                        pnt.x = x_;
                        pnt.y = height_ -  y_;
                        break;
                    }
                    case 'l':
                    {
                        // lineto command, relative mode
                        auto last_pnt_ = p.vertices.back();
                        pnt.x = last_pnt_.x + x_;
                        pnt.y = last_pnt_.y + y_;
                        break;
                    }
                    default:
                        // do nothing, or log error?
                        break;
                    }
                    p.vertices.push_back(pnt);
                    debug_.append("; " + std::to_string(pnt.x) + ", " + std::to_string(pnt.y));
                }
            }
            msg.obstacles.push_back(p);
            PLOGD << debug_;
        } else
        {
            PLOGW << "Unknown svg attribute " << name;
        }
        elem = elem->NextSiblingElement();
    }
}

float
Map_IO_Vector::char2float(const char * attribute)
{
    std::string f(attribute);
    return std::stof(f);
}

// === Map output part ===

void
Map_IO_Vector::checkSaveParameters(SaveParameters &save_parameters)
{
    // Checking map file name
    if (save_parameters.map_file_name == "")
    {
        // rclcpp::Clock clock(RCL_SYSTEM_TIME);
        save_parameters.map_file_name = "map_12345";
                                        // std::to_string(static_cast<int>(clock.now().seconds()));
        PLOGW << "Map file unspecified. Map will be saved to " << save_parameters.map_file_name << " file";
    }

    // Checking thresholds
    if (save_parameters.occupied_thresh == 0.0)
    {
        save_parameters.occupied_thresh = 0.65;
        PLOGW << "Occupied threshold unspecified. Setting it to default value: " << save_parameters.occupied_thresh;
    }
    if (save_parameters.free_thresh == 0.0)
    {
        save_parameters.free_thresh = 0.25;
        PLOGW << "Free threshold unspecified. Setting it to default value: " << save_parameters.free_thresh;
    }
    if (1.0 < save_parameters.occupied_thresh)
    {
        PLOGE << "Threshold_occupied must be 1.0 or less";
        throw std::runtime_error("Incorrect thresholds");
    }
    if (save_parameters.free_thresh < 0.0)
    {
        PLOGE << "Free threshold must be 0.0 or greater";
        throw std::runtime_error("Incorrect thresholds");
    }
    if (save_parameters.occupied_thresh <= save_parameters.free_thresh)
    {
        PLOGE << "Threshold_free must be smaller than threshold_occupied";
        throw std::runtime_error("Incorrect thresholds");
    }

    // Checking image format
    if (save_parameters.image_format == "")
    {
        save_parameters.image_format = save_parameters.mode == MapMode::Scale ? "png" : "pgm";
        PLOGW << "Image format unspecified. Setting it to: " << save_parameters.image_format;
    }

    std::transform(
        save_parameters.image_format.begin(),
        save_parameters.image_format.end(),
        save_parameters.image_format.begin(),
        [](unsigned char c)
        { return std::tolower(c); });

    const std::vector<std::string> BLESSED_FORMATS{"bmp", "pgm", "png"};
    if (
        std::find(BLESSED_FORMATS.begin(), BLESSED_FORMATS.end(), save_parameters.image_format) ==
        BLESSED_FORMATS.end())
    {
        std::stringstream ss;
        bool first = true;
        for (auto &format_name : BLESSED_FORMATS)
        {
            if (!first)
            {
                ss << ", ";
            }
            ss << "'" << format_name << "'";
            first = false;
        }
        PLOGW << "Requested image format '" << save_parameters.image_format << "' is not one of the recommended formats: " << ss.str();
    }
    const std::string FALLBACK_FORMAT = "png";

    // Checking map mode
    if (
        save_parameters.mode == MapMode::Scale &&
        (save_parameters.image_format == "pgm" ||
            save_parameters.image_format == "jpg" ||
            save_parameters.image_format == "jpeg"))
    {
        PLOGW << "Map mode 'scale' requires transparency, but format '" << save_parameters.image_format << "' does not support it. Consider switching image format to 'png'.";
    }
}

void
Map_IO_Vector::tryWriteMapToFile(const SaveParameters &save_parameters)
{
    PLOGI << "Received a " << map.info.width << " X " << map.info.height << " map @ " << map.info.resolution << " m/pix";
    std::string mapmetadatafile = save_parameters.map_file_name + ".yaml";
    {
        std::ofstream yaml(mapmetadatafile);

        geometry_msgs::msg::Quaternion orientation = map.info.origin.orientation;
        // tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
        // double yaw, pitch, roll;
        // mat.getEulerYPR(yaw, pitch, roll);

        YAML::Emitter e;
        e << YAML::Precision(3);
        e << YAML::BeginMap;
        // e << YAML::Key << "image" << YAML::Value << mapdatafile;
        e << YAML::Key << "mode" << YAML::Value << map_mode_to_string(save_parameters.mode);
        e << YAML::Key << "resolution" << YAML::Value << map.info.resolution;
        e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq << map.info.origin.position.x << map.info.origin.position.y << /*yaw*/ "yaw" << YAML::EndSeq;
        e << YAML::Key << "negate" << YAML::Value << 0;
        e << YAML::Key << "occupied_thresh" << YAML::Value << save_parameters.occupied_thresh;
        e << YAML::Key << "free_thresh" << YAML::Value << save_parameters.free_thresh;

        if (!e.good())
        {
            PLOGW << "YAML writer failed with an error " << e.GetLastError() << ". The map metadata may be invalid.";
        }

        PLOGI << "Writing map metadata to " << mapmetadatafile;
        std::ofstream(mapmetadatafile) << e.c_str();
    }
    PLOGI << "Map saved";
}

bool
Map_IO_Vector::saveMapToFile(const SaveParameters & save_parameters)
{
    // Local copy of SaveParameters that might be modified by checkSaveParameters()
    SaveParameters save_parameters_loc = save_parameters;

    try
    {
        // Checking map parameters for consistency
        checkSaveParameters(save_parameters_loc);
        tryWriteMapToFile(save_parameters_loc);
    }
    catch (std::exception &e)
    {
        PLOGE << "Failed to write map for reason: " << e.what();
        return false;
    }
    return true;
}

} // namespace sgd_map_server