/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#include "sgd_costmap_plugins/static_vector_layer.hpp"
#include "sgd_costmap_plugins/delaunator.hpp"
#include "sgd_costmap_plugins/triangle_rasterizer.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace sgd_costmap_plugins
{

    StaticVectorLayer::StaticVectorLayer()
        //: map_buffer_(nullptr)
    {
        // initialize logging
        time_t now = time(0);
        tm *ltm = localtime(&now);

        char buf[24];
        std::sprintf(&buf[0], "%4d-%02d-%02d_%02d-%02d-%02d.csv", 1900+ltm->tm_year, 1+ltm->tm_mon, ltm->tm_mday,
                                ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
        std::string log_file_(buf);
        std::string log_file(".ros/log/vectormap_" + log_file_);

        plog::init(plog::severityFromString("D"), log_file.c_str());
        PLOGD << "Static Vector Layer";
    }

    StaticVectorLayer::~StaticVectorLayer()
    {
    }

    void
    StaticVectorLayer::onInitialize()
    {
        RCLCPP_INFO(node_->get_logger(), "Initialize StaticVectorLayer Costmap plugin");
        global_frame_ = layered_costmap_->getGlobalFrameID();

        getParameters();

        rclcpp::QoS map_qos(10); // initialize to default
        if (map_subscribe_transient_local_)
        {
            map_qos.transient_local();
            map_qos.reliable();
            map_qos.keep_last(1);
        }

        RCLCPP_INFO(node_->get_logger(), "Subscribing to the map topic (%s) with %s durability",
                    map_topic_.c_str(), map_subscribe_transient_local_ ? "transient local" : "volatile");

        // create subscribers
        map_sub_ = node_->create_subscription<sgd_msgs::msg::VecObstacleArray>(
            map_topic_, map_qos,
            std::bind(&StaticVectorLayer::incomingMap, this, std::placeholders::_1));

        // if (subscribe_to_updates_)
        // {
        //     RCLCPP_INFO(node_->get_logger(), "Subscribing to updates");
        //     map_update_sub_ = node_->create_subscription<sgd_msgs::msg::VecObstacleArray>(
        //         map_topic_ + "_updates",
        //         rclcpp::SystemDefaultsQoS(),
        //         std::bind(&StaticVectorLayer::incomingUpdate, this, std::placeholders::_1));
        // }
    }

    void
    StaticVectorLayer::activate()
    {
    }

    void
    StaticVectorLayer::deactivate()
    {
    }

    void
    StaticVectorLayer::reset()
    {
        RCLCPP_INFO(node_->get_logger(), "Reset StaticVectorLayer costmap plugin");
        has_updated_data_ = true;
    }

    void
    StaticVectorLayer::getParameters()
    {
        /*
        Parameter:
        -
        */

        int temp_lethal_threshold = 0;
        double temp_tf_tol = 0.0;

        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("subscribe_to_updates", rclcpp::ParameterValue(false));
        declareParameter("map_subscribe_transient_local", rclcpp::ParameterValue(true));
        declareParameter("transform_tolerance", rclcpp::ParameterValue(0.0));
        declareParameter("map_topic", rclcpp::ParameterValue(""));

        node_->get_parameter(name_ + "." + "enabled", enabled_);
        node_->get_parameter(name_ + "." + "subscribe_to_updates", subscribe_to_updates_);
        std::string private_map_topic, global_map_topic;
        node_->get_parameter(name_ + "." + "map_topic", private_map_topic);
        node_->get_parameter("map_topic", global_map_topic);
        if (!private_map_topic.empty())
        {
            map_topic_ = private_map_topic;
        }
        else
        {
            map_topic_ = global_map_topic;
        }
        node_->get_parameter(
            name_ + "." + "map_subscribe_transient_local",
            map_subscribe_transient_local_);
        node_->get_parameter("track_unknown_space", track_unknown_space_);
        node_->get_parameter("use_maximum", use_maximum_);
        node_->get_parameter("lethal_cost_threshold", temp_lethal_threshold);
        node_->get_parameter("unknown_cost_value", unknown_cost_value_);
        node_->get_parameter("trinary_costmap", trinary_costmap_);
        node_->get_parameter("transform_tolerance", temp_tf_tol);

        // Enforce bounds
        lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
        map_received_ = false;
        update_in_progress_.store(false);

        transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);
    }

    void
    StaticVectorLayer::processMap(const sgd_msgs::msg::VecObstacleArray &new_map)
    {
        if (!has_new_map)   return;

        RCLCPP_INFO(node_->get_logger(), "StaticVectorLayer: Process map");

        unsigned int size_x = new_map.info.width;
        unsigned int size_y = new_map.info.height;

        // vector map has coordinates in meters -> resizing not required
        RCLCPP_INFO(node_->get_logger(), "StaticVectorLayer: Received a map with size %d m x %d m and %d obstacles",
                size_x, size_y, new_map.obstacles.size());

        // create box outline
        int i = 0;
        box_outlines.clear();
        box_outlines.resize(new_map.obstacles.size());
        for (int i = 0; i < new_map.obstacles.size(); i++)
        {
            sgd_msgs::msg::VecObstacle o = new_map.obstacles.at(i);
            box_outline bo;
            if (o.vertices.size() > 1)
            {
                bo.min_x = o.vertices[0].x;
                bo.min_y = o.vertices[0].y;
                bo.max_x = o.vertices[0].x;
                bo.max_y = o.vertices[0].y;
                bo.obstacle = o;

                for (auto p : o.vertices)
                {
                    if (p.x > bo.max_x)     bo.max_x = p.x;
                    if (p.x < bo.min_x)     bo.min_x = p.x;
                    if (p.y > bo.max_y)     bo.max_y = p.y;
                    if (p.y < bo.min_y)     bo.min_y = p.y;
                }
            }
            else
            {
                bo.min_x = o.pose.position.x - o.radius;
                bo.min_y = o.pose.position.y - o.radius;
                bo.max_x = o.pose.position.x + o.radius;
                bo.max_y = o.pose.position.y + o.radius;
                bo.obstacle = o;
            }
            box_outlines[i] = bo;
        }

        // resize costmap if size, resolution or origin do not match
        Costmap2D *master = layered_costmap_->getCostmap();
        if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
                                               master->getSizeInCellsY() != size_y ||
                                               master->getResolution() != new_map.info.resolution ||
                                               master->getOriginX() != new_map.info.origin.position.x ||
                                               master->getOriginY() != new_map.info.origin.position.y ||
                                               !layered_costmap_->isSizeLocked()))
        {
            // Update the size of the layered costmap (and all layers, including this one)
            RCLCPP_INFO(
                node_->get_logger(),
                "StaticVectorLayer: Resizing costmap to %d X %d at %f m/pix", size_x, size_y,
                new_map.info.resolution);
            layered_costmap_->resizeMap(
                size_x, size_y, new_map.info.resolution,
                new_map.info.origin.position.x,
                new_map.info.origin.position.y,
                true);
        }
        else if (size_x_ != size_x || size_y_ != size_y || // NOLINT
                 resolution_ != new_map.info.resolution ||
                 origin_x_ != new_map.info.origin.position.x ||
                 origin_y_ != new_map.info.origin.position.y)
        {
            // only update the size of the costmap stored locally in this layer
            RCLCPP_INFO(
                node_->get_logger(),
                "StaticVectorLayer: Resizing static layer to %d X %d at %f m/pix", size_x, size_y,
                new_map.info.resolution);
            resizeMap(
                size_x, size_y, new_map.info.resolution,
                new_map.info.origin.position.x, new_map.info.origin.position.y);
        }

        unsigned int index = 0;

        // we have a new map, update full size of map
        //std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

        // initialize the costmap with static data
        // for (unsigned int i = 0; i < size_y; ++i) {
        //   for (unsigned int j = 0; j < size_x; ++j) {
        //     unsigned char value = new_map.data[index];
        //     costmap_[index] = interpretValue(value);
        //     ++index;
        //   }
        // }

        map_frame_ = new_map.header.frame_id;

        x_ = y_ = 0;
        width_ = size_x_;
        height_ = size_y_;
        has_updated_data_ = true;
        has_new_map = false;

        current_ = true;
    }

    void
    StaticVectorLayer::matchSize()
    {
        RCLCPP_INFO(node_->get_logger(), "StaticVectorLayer: matchSize()");
        // If we are using rolling costmap, the static map size is
        //   unrelated to the size of the layered costmap
        if (!layered_costmap_->isRolling())
        {
            Costmap2D *master = layered_costmap_->getCostmap();
            resizeMap(
                master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                master->getOriginX(), master->getOriginY());
        }
    }

    unsigned char
    StaticVectorLayer::interpretValue(unsigned char value)
    {
        // check if the static value is above the unknown or lethal thresholds
        if (track_unknown_space_ && value == unknown_cost_value_)
        {
            return NO_INFORMATION;
        }
        else if (!track_unknown_space_ && value == unknown_cost_value_)
        {
            return FREE_SPACE;
        }
        else if (value >= lethal_threshold_)
        {
            return LETHAL_OBSTACLE;
        }
        else if (trinary_costmap_)
        {
            return FREE_SPACE;
        }

        double scale = static_cast<double>(value) / lethal_threshold_;
        return scale * LETHAL_OBSTACLE;
    }

    void
    StaticVectorLayer::incomingMap(const sgd_msgs::msg::VecObstacleArray::SharedPtr new_map)
    {
        has_new_map = true;

        if (!update_in_progress_.load())
        {
            map_received_ = true;
            processMap(*new_map);
        }
    }

    void
    StaticVectorLayer::updateBounds(
        double robot_x, double robot_y, double robot_yaw, double *min_x,
        double *min_y, double *max_x, double *max_y)
    {
        if (!map_received_)
        {
            return;
        }

        if (!equals_eps(last_robot_x, robot_x) || !equals_eps(last_robot_y, robot_y))
        {
            // update bounds
            *min_x = robot_x - layered_costmap_->getCostmap()->getSizeInMetersX()/2;
            *max_x = robot_x + layered_costmap_->getCostmap()->getSizeInMetersX()/2;
            *min_y = robot_y - layered_costmap_->getCostmap()->getSizeInMetersY()/2;
            *max_y = robot_y + layered_costmap_->getCostmap()->getSizeInMetersY()/2;

            last_robot_x = robot_x;
            last_robot_y = robot_y;
        }
    }

    void
    StaticVectorLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
        {
            update_in_progress_.store(false);
            return;
        }

        if (!map_received_)
        {
            static int count = 0;
            // throttle warning down to only 1/10 message rate
            if (++count == 10)
            {
                RCLCPP_WARN(node_->get_logger(), "Can't update static vector costmap layer, no map received");
                count = 0;
            }
            update_in_progress_.store(false);
            return;
        }

        unsigned char * master_array = master_grid.getCharMap();
        unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

        // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
        // These variables are used to update the costmap only within this window
        // avoiding the updates of whole area.
        //
        // Fixing window coordinates with map size if necessary.
        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);

        // initialize costmap with free space
        for (int x_ = 0; x_ < layered_costmap_->getCostmap()->getSizeInCellsX(); x_++)
        {
            for (int y_ = 0; y_ < layered_costmap_->getCostmap()->getSizeInCellsY(); y_++)
            {
                uint index = master_grid.getIndex(x_, y_);
                master_array[index] = 0U;
            }
        }

        // get world frame
        double w_min_i, w_min_j, w_max_i, w_max_j;      // world coordinates
        layered_costmap_->getCostmap()->mapToWorld(min_i, min_j, w_min_i, w_min_j);
        layered_costmap_->getCostmap()->mapToWorld(max_i, max_j, w_max_i, w_max_j);

        // x origin in cells
        int resolution = 10;     // pixel per meter
        uint origin_cell_x = (int)round(w_min_i*resolution);
        uint origin_cell_y = (int)round(w_min_j*resolution);

        int k = 0;
        for (int i = 0; i < box_outlines.size(); i++)
        {
            if ((is_in_range(w_min_i, w_max_i, box_outlines[i].min_x) ||
                    is_in_range(w_min_i, w_max_i, box_outlines[i].max_x)) &&
                (is_in_range(w_min_j, w_max_j, box_outlines[i].min_y) ||
                    is_in_range(w_min_j, w_max_j, box_outlines[i].max_y)))
            {
                k++;
                if (!box_outlines[i].obstacle.vertices.empty())
                {
                    // triangulate polygon
                    std::vector<double> coords;
                    std::vector<int> icoords;
                    PLOGI << "Vertices:";
                    for (auto vert : box_outlines[i].obstacle.vertices)
                    {
                        coords.push_back(vert.x);
                        coords.push_back(vert.y);
                        PLOGI.printf("%.3f, %.3f", vert.x, vert.y);
                    }
                    //triangulation happens here
                    delaunator::Delaunator d(coords);

                    // calculate coords with resolution
                    for (auto c : d.coords)
                    {
                        icoords.push_back((int)round(c*resolution));
                    }

                    for(std::size_t j = 0; j < d.triangles.size(); j+=3)
                    {                    
                        // test if triangles are part of the polygon
                        // calculate schwerpunkt
                        float sx = (d.coords[2*d.triangles[j]] + d.coords[2*d.triangles[j+1]] + d.coords[2*d.triangles[j+2]])/3.0F;
                        float sy = (d.coords[2*d.triangles[j]+1] + d.coords[2*d.triangles[j+1]+1] + d.coords[2*d.triangles[j+2]+1])/3.0F;

                        // startpunkt: y = sy; x = box_outlines[i].min_x
                        // get ray intersections
                        int intersections = 0;
                        double x0_ = coords[coords.size()-2];
                        double y0_ = coords[coords.size()-1];
                        for (int s = 1; s < coords.size(); s+=2)
                        {
                            // immer zwei Punkte bilden eine Linie
                            // ein Punkt muss größer und einer kleiner als der Schwerpunkt sein, damit die Linie geschnitten werden kann
                            double x1_ = coords[s-1];
                            double y1_ = coords[s];

                            if (x0_ > sx && x1_ > sx)
                            {
                                x0_ = x1_;
                                y0_ = y1_;
                                continue;
                            }
                            if ((y0_ > sy && y1_ > sy) || (y0_ < sy && y1_ < sy))
                            {
                                x0_ = x1_;
                                y0_ = y1_;
                                continue;
                            }

                            float u = (x0_*(sy-y1_) + x1_*(y0_-sy) + sx*(y1_-y0_))
                                    / (x0_*(sy-sy) + x1_*(sy-sy) + sx*(y1_-y0_) + box_outlines[i].min_x*(y0_-y1_));
                            float schnittx = sx+u*(box_outlines[i].min_x-sx);
                            float schnitty = sy+u*(sy-sy);   // intersection point

                            if (schnittx > sx)
                            {
                                x0_ = x1_;
                                y0_ = y1_;
                                continue;
                            }

                            intersections++;

                            x0_ = x1_;
                            y0_ = y1_;
                        }
                        // odd number of intersections = inside polygon, else outside
                        if (intersections%2 == 0)   continue;   // odd number of intersections required to continue with rasterization

                        TriangleRasterizer btr(
                            icoords[2 * d.triangles[j]], icoords[2 * d.triangles[j] + 1],       // x1, y1
                            icoords[2 * d.triangles[j + 1]], icoords[2 * d.triangles[j + 1] + 1],   // x2, y2
                            icoords[2 * d.triangles[j + 2]], icoords[2 * d.triangles[j + 2] + 1]);  // x3, y3

                        // save to costmap
                        for (int x = 0; x < btr.mins.size(); x++)
                        {
                            // cell in x direction in map frame
                            int x_map = x+btr.get_lmp() - origin_cell_x;
                            if (x_map < 0 || x_map > layered_costmap_->getCostmap()->getSizeInCellsX())     continue;

                            for (int y = btr.mins[x]; y <= btr.maxs[x]; y++)
                            {
                                int y_map = y - origin_cell_y;
                                if (y_map < 0 || y_map > layered_costmap_->getCostmap()->getSizeInCellsY())     continue;
                                // calculate local costmap coordinates [0...100]
                                
                                uint index = master_grid.getIndex(x_map, y_map);
                                master_array[index] = 254U;

                            }
                        }
                    }
                }
                // else
                // {
                //     // rasterize circle
                // }

            }
        }
        RCLCPP_INFO(node_->get_logger(), "Found %d obstacles inside frame %.3f, %.3f, %.3f, %.3f",
                k, w_min_i, w_min_j, w_max_i, w_max_j);
    }

} // namespace sgd_costmap_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sgd_costmap_plugins::StaticVectorLayer, nav2_costmap_2d::Layer)
