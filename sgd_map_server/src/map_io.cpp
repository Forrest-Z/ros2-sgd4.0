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

#include "sgd_map_server/map_io.hpp"

namespace sgd_map_server
{

Map_IO::Map_IO(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
{
    // init regex to parse fill attribute and path coordinates
    regex_path_ = "(-?\\d+\\.\\d+),(-?\\d+\\.\\d+)";

    PLOGD << "Map_IO initialized";
}

Map_IO::~Map_IO() {}

// === Map input part ===

void
Map_IO::loadMapFromFile(const LoadParameters & load_parameters)
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

    nav_msgs::msg::OccupancyGrid msg;
    msg.info.width = load_parameters.width;
    msg.info.height = load_parameters.height;
    msg.info.resolution = load_parameters.resolution;
    msg.info.origin.position.x = load_parameters.origin[0];
    msg.info.origin.position.y = load_parameters.origin[1];
    msg.info.origin.position.z = 0.0;

    PLOGI.printf("Read map %s: %u X %u map @ %.2f m/cell", load_parameters.image_file_name.c_str(), msg.info.width, msg.info.height, msg.info.resolution);

    if (doc.RootElement() != NULL)
    {
        auto root = doc.RootElement();
        // resize occupancy grid to match image size
        msg.data.resize(msg.info.width * msg.info.height);
        height_ = msg.info.height*msg.info.resolution;

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
        
        PLOGI << "SVG rasterization done";
    }

    // Since loadMapFromFile() does not belong to any node, publishing in a system time.
    // rclcpp::Clock clock(RCL_SYSTEM_TIME);
    // msg.info.map_load_time = clock.now();
    msg.header.frame_id = "map";
    // msg.header.stamp = clock.now();

    map = msg;
}

void
Map_IO::import_svg(tinyxml2::XMLElement * element, nav_msgs::msg::OccupancyGrid &msg)
{
    // switch if group / circle / path
    auto elem = element->FirstChildElement();
    // get height
    // auto height = element->Attribute("height");
    // TODO use transformation matrix

    // TODO: error handling for xml attributes
    sgd_rasterizer::RayRasterizer rr(msg.info.width, msg.info.height);
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
            // rasterize circle
            int r = (int)round(char2float(elem->Attribute("r")) / msg.info.resolution);
            int cx = (int)round(char2float(elem->Attribute("cx")) / msg.info.resolution);
            int cy = (int)round(char2float(elem->Attribute("cy")) / msg.info.resolution);
            PLOGD.printf("Circle; %d; %d; %d", r, cx, cy);

            sgd_rasterizer::CircleRasterizer cr(r);

            // save to image array
            for (int x = 0; x < cr.maxs.size(); x++)
            {
                for (int y = cr.mins[x]; y <= cr.maxs[x]; y++)
                {
                    int x_ = x + cx - r;
                    int y_ = y + cy - r;

                    msg.data[msg.info.width * (msg.info.height - y_ - 1) + x_] = 100;
                }
            }
        }
        else if (name == "path")
        {
            PLOGD << "Rasterize polygon";
            // triangulate polygon
            std::vector<float> coords;     /// @brief coordinates in meters
            std::vector<int> icoords;       /// @brief coordinates in cells

            // import svg points
            PLOGD << "Vertices:";
            std::stringstream path(elem->Attribute("d"));
            std::string s;
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

                    // geometry_msgs::msg::Point32 pnt;
                    float x_ = std::stof(matches[1]);
                    float y_ = std::stof(matches[2]);
                    switch (mode)
                    {
                    case 'm':
                    case 'l':
                    {
                        // first coordinate pair is always absolute
                        if (!coords.empty())
                        {
                            // implicit lineto command
                            // get last point
                            float last_x_ = coords[coords.size()-2];
                            float last_y_ = coords[coords.size()-1];
                            coords.push_back(last_x_ + x_);
                            coords.push_back(last_y_ + (height_ - y_));
                            break;
                        }
                    } // else fallthrough
                    case 'M':   // absolute line start
                    case 'L':   // absolute lineto command
                    {
                        coords.push_back(x_);
                        coords.push_back(height_ - y_);
                        break;
                    }
                    default:
                        // do nothing, or log error?
                        break;
                    }
                    PLOGD.printf("%.3f, %.3f", coords[coords.size()-2], coords[coords.size()-1]);
                }
            }

            PLOGD << "RayRasterizer";
            
            // modify coords to match resolution
            for (int c = 0; c < coords.size(); c++)
            {
                coords[c] = coords[c] / msg.info.resolution;
            }
            
            rr.add_object(coords, 100U);

            // PLOGD.printf("Delauny triangulation for %d points", (int)(coords.size()/2));
            // delaunator::Delaunator d(coords);

            // // calculates local coordinates (integer cell values) from global coordinates (meter)
            // for (auto c : d.coords)
            // {
            //     icoords.push_back((int)round(c/msg.info.resolution));
            // }

            // PLOGD.printf("Rasterize %d triangles", d.triangles.size());
            // for(std::size_t j = 0; j < d.triangles.size(); j+=3)
            // {
            //     // Checks if the calculated triangle is part of the polygon. For concave polygons,
            //     // the delaunator also returns triangles that lie outside the boundary
            //     // calculate center of gravity
            //     float sx = (d.coords[2*d.triangles[j]] + d.coords[2*d.triangles[j+1]] + d.coords[2*d.triangles[j+2]])/3.0F;
            //     float sy = (d.coords[2*d.triangles[j]+1] + d.coords[2*d.triangles[j+1]+1] + d.coords[2*d.triangles[j+2]+1])/3.0F;
            //     PLOGD.printf("Center of gravity for polygon (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f) ==> (%.2f, %.2f)",
            //         d.coords[2*d.triangles[j]], d.coords[2*d.triangles[j]+1], d.coords[2*d.triangles[j+1]], d.coords[2*d.triangles[j+1]+1],
            //         d.coords[2*d.triangles[j+2]], d.coords[2*d.triangles[j+2]+1], sx, sy);
            //     // Checks if the center of gravity of the triangle is inside the polygon boundary
            //     if (!is_point_inside_polygon(coords, sx, sy))    continue;

            //     // init triangle rasterizer with triangle points
            //     TriangleRasterizer btr(
            //         icoords[2 * d.triangles[j]], icoords[2 * d.triangles[j] + 1],       // x1, y1
            //         icoords[2 * d.triangles[j + 1]], icoords[2 * d.triangles[j + 1] + 1],   // x2, y2
            //         icoords[2 * d.triangles[j + 2]], icoords[2 * d.triangles[j + 2] + 1]);  // x3, y3

            //     // save to image array
            //     PLOGD.printf("TriangleRasterizer returned mins.size() = %d and maxs.size() = %d", btr.mins.size(), btr.maxs.size());
            //     for (int x = 0; x < btr.maxs.size(); x++)
            //     {
            //         for (int y = btr.mins[x]; y <= btr.maxs[x]; y++)
            //         {
            //             int pos = msg.info.width * y + x + btr.get_lmp();
            //             if (pos > msg.info.width * msg.info.height || pos < 0)
            //             {
            //                 PLOGW.printf("Cell (%d, %d) at array position %d is invalid.", x + btr.get_lmp(), y, pos);
            //                 continue;
            //             }
            //             msg.data[pos] = 100;
            //         }
            //     }
            // }
        } else
        {
            PLOGW << "Unknown svg attribute " << name;
        }
        elem = elem->NextSiblingElement();
    }
    PLOGD << "Copy data to message";
    if (msg.data.size() != rr.data.size())
    {
        std::cout << "msg.data.size() != rr.data.size()\n";
        PLOGW.printf("msg.data.size() = %d but rr.data.size() = %d", msg.data.size(), rr.data.size());
    }
    for (int j = 0; j < rr.data.size(); j++)
    {
        msg.data[j] = std::max(msg.data[j], static_cast<int8_t>(rr.data[j]));
    }
}

float
Map_IO::char2float(const char * attribute)
{
    std::string f(attribute);
    return std::stof(f);
}

bool
Map_IO::is_point_inside_polygon(std::vector<double> polygon_outline, float px, float py)
{
    PLOGD.printf("Check if point (%.2f, %.2f) is inside polygon", px, py);
    // Calculates the intersection points of a ray starting from the point (px, py) to the point (x_end, py)
    // with the polygon. If the number of intersection points is odd, the point is inside the polygon, otherwise outside.
    int intersections = 0;  /// @brief number of intersections
    
    // get last point from polygon outline vector
    double x0_ = polygon_outline[polygon_outline.size()-2];
    double y0_ = polygon_outline[polygon_outline.size()-1];
    // go through all polygon outline vertices
    for (int s = 1; s < polygon_outline.size(); s+=2)
    {
        // point (x0_, y0_) and (x1_, y1_) form a line
        double x1_ = polygon_outline[s-1];
        double y1_ = polygon_outline[s];

        // if both points are right from (px, py) or are above/below
        // the point continue, because the line cannot be intersected
        if ((x0_ > px && x1_ > px) ||
            ((y0_ > py && y1_ > py) || (y0_ < py && y1_ < py)))
        {
            x0_ = x1_;
            y0_ = y1_;
            continue;
        }
        
        // calculate intersection with polygon line
        float u = (x0_*(py-y1_) + x1_*(y0_-py) + px*(y1_-y0_))
                / (px*(y1_-y0_));
        float schnittx = px * (1.0F-u);

        // if x value of intersection point is larger than px continue, because it does
        // not intersect the ray
        if (schnittx > px)
        {
            x0_ = x1_;
            y0_ = y1_;
            continue;
        }

        // valid intersection
        intersections++;
        PLOGD.printf("Ray intersects line from (%.2f, %.2f) to (%.2f, %.2f)", x0_, y0_, x1_, y1_);
        // store last point from polygon for next iteration
        x0_ = x1_;
        y0_ = y1_;
    }

    // odd number of intersections required to continue with rasterization
    PLOGD << "Found " << intersections << " intersections -> " << ((intersections%2 != 0) ? "point is inside polygon" : "point is outside polygon");
    return (intersections%2 != 0);
}

// === Map output part ===

void
Map_IO::checkSaveParameters(SaveParameters &save_parameters)
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
Map_IO::tryWriteMapToFile(const SaveParameters &save_parameters)
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
Map_IO::saveMapToFile(const SaveParameters & save_parameters)
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