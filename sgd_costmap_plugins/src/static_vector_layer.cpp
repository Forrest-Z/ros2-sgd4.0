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
#include "sgd_costmap_plugins/circle_rasterizer.hpp"

namespace sgd_costmap_plugins
{

StaticVectorLayer::StaticVectorLayer()  {}
StaticVectorLayer::~StaticVectorLayer() {}

void
StaticVectorLayer::onInitialize()
{
    RCLCPP_INFO(node_->get_logger(), "Initialize StaticVectorLayer Costmap plugin");
    // global_frame_ = layered_costmap_->getGlobalFrameID();

    initLogging();
    getParameters();

    rclcpp::QoS map_qos(10); // initialize to default
    if (map_subscribe_transient_local_)
    {
        map_qos.transient_local();
        map_qos.reliable();
        map_qos.keep_last(1);
    }

    // create subscribers
    RCLCPP_INFO(node_->get_logger(), "Subscribing to the map topic (%s) with %s durability",
                map_topic_.c_str(), map_subscribe_transient_local_ ? "transient local" : "volatile");
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
StaticVectorLayer::activate()   {}

void
StaticVectorLayer::deactivate() {}

void
StaticVectorLayer::reset()  {}

void
StaticVectorLayer::getParameters()
{
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
StaticVectorLayer::initLogging()
{
    // init logging
    declareParameter("log_dir", rclcpp::ParameterValue(".ros/log/"));
    declareParameter("log_severity", rclcpp::ParameterValue("I"));

    // initialize logging
    std::string log_dir_, log_sev_;
    node_->get_parameter(name_ + ".log_dir", log_dir_);
    node_->get_parameter(name_ + ".log_severity", log_sev_);

    std::string log_file(log_dir_ + "/static_vector_plugin.csv");
    RCLCPP_INFO(node_->get_logger(), "Save StaticVectorPlugin log file to: %s", log_file.c_str());

    plog::init(plog::severityFromString(log_sev_.c_str()), log_file.c_str());
    PLOGI.printf("Created StaticVectorLayer plugin. PLOG logging severity is %s", log_sev_.c_str());
}

void
StaticVectorLayer::processMap(const sgd_msgs::msg::VecObstacleArray &new_map)
{
    if (!has_new_map)   return;

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

    has_new_map = false;
    current_ = true;
}

void
StaticVectorLayer::matchSize()
{
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

    // only update bounds if robot has moved since last call to updateBounds
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

    PLOGI << "Update Costmap  -------------------";

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

    // get world frame
    double w_min_i, w_min_j, w_max_i, w_max_j;      // world coordinates
    layered_costmap_->getCostmap()->mapToWorld(min_i, min_j, w_min_i, w_min_j);
    layered_costmap_->getCostmap()->mapToWorld(max_i, max_j, w_max_i, w_max_j);

    double resolution = master_grid.getResolution();        /// @brief resolution of the costmap im meter/pixel
    uint origin_cell_x = (int)round(w_min_i/resolution);    /// @brief origin of the costmap in global costmap
    uint origin_cell_y = (int)round(w_min_j/resolution);    /// @brief origin of the costmap in global costmap

    int k = 0;  /// @brief counter for obstacles
    for (int i = 0; i < box_outlines.size(); i++)
    {
        // check if obstacle intersects costmap -> if yes display in costmap
        if ((is_in_range(box_outlines[i].min_x, box_outlines[i].max_x, w_min_i, w_max_i) &&
                is_in_range(box_outlines[i].min_y, box_outlines[i].max_y, w_min_j, w_max_j)))
        {
            k++;

            if (!box_outlines[i].obstacle.vertices.empty())
            {
                // triangulate polygon
                std::vector<double> coords;     /// @brief coordinates in meters
                std::vector<int> icoords;       /// @brief coordinates in cells

                PLOGD << "Vertices:";
                for (auto vert : box_outlines[i].obstacle.vertices)
                {
                    // add vertices to vector -> required for delauny triangulation with delaunator
                    coords.push_back(vert.x);
                    coords.push_back(vert.y);
                    PLOGD.printf("%.3f, %.3f", vert.x, vert.y);
                }
                // delauny triangulation happens here
                delaunator::Delaunator d(coords);

                // calculates local coordinates (integer celll values) from global coordinates (meter)
                for (auto c : d.coords)
                {
                    icoords.push_back((int)round(c/resolution));
                }

                for(std::size_t j = 0; j < d.triangles.size(); j+=3)
                {                    
                    // Checks if the calculated triangle is part of the polygon. For concave polygons,
                    // the delaunator also returns triangles that lie outside the boundary
                    // calculate center of gravity
                    float sx = (d.coords[2*d.triangles[j]] + d.coords[2*d.triangles[j+1]] + d.coords[2*d.triangles[j+2]])/3.0F;
                    float sy = (d.coords[2*d.triangles[j]+1] + d.coords[2*d.triangles[j+1]+1] + d.coords[2*d.triangles[j+2]+1])/3.0F;
                    // Checks if the center of gravity of the triangle is inside the polygon boundary
                    if (!is_point_inside_polygon(coords, sx, sy, box_outlines[i].min_x))    continue;

                    // init triangle rasterizer with triangle points
                    TriangleRasterizer btr(
                        icoords[2 * d.triangles[j]], icoords[2 * d.triangles[j] + 1],       // x1, y1
                        icoords[2 * d.triangles[j + 1]], icoords[2 * d.triangles[j + 1] + 1],   // x2, y2
                        icoords[2 * d.triangles[j + 2]], icoords[2 * d.triangles[j + 2] + 1]);  // x3, y3

                    // save to costmap
                    save_to_costmap(master_grid,
                            btr.get_lmp()-origin_cell_x, -origin_cell_y,
                            btr.mins, btr.maxs);
                }
            }
            else
            {
                PLOGD.printf("Rasterize circle: radius=%.3f -> integer radius=%d", box_outlines[i].obstacle.radius,
                        (int)round(box_outlines[i].obstacle.radius / resolution));
                // rasterize circle
                CircleRasterizer cr((int)round(box_outlines[i].obstacle.radius / resolution));

                save_to_costmap(master_grid,
                        round((box_outlines[i].obstacle.pose.position.x - box_outlines[i].obstacle.radius) / resolution) - origin_cell_x,
                        round((box_outlines[i].obstacle.pose.position.y - box_outlines[i].obstacle.radius) / resolution) - origin_cell_y,
                        cr.mins, cr.maxs);

                PLOGD << "Circle rasterization done.";
            }
        }
    }
    PLOGI.printf("Found %d obstacles inside frame %.3f, %.3f, %.3f, %.3f",
            k, w_min_i, w_min_j, w_max_i, w_max_j);
}

bool
StaticVectorLayer::is_point_inside_polygon(std::vector<double> polygon_outline, float px, float py, float x_end)
{
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
                / (px*(y1_-y0_) + x_end*(y0_-y1_));
        float schnittx = px+u*(x_end-px);

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
        // store last point from polygon for next iteration
        x0_ = x1_;
        y0_ = y1_;
    }

    // odd number of intersections required to continue with rasterization
    return (intersections%2 != 0);
}

void
StaticVectorLayer::save_to_costmap(
        nav2_costmap_2d::Costmap2D &master_grid,
        int x_start, int y_start,
        std::vector<int> lower_bound, std::vector<int> upper_bound)
{
    unsigned char * master_array = master_grid.getCharMap();

    // save to costmap
    for (int x = 0; x < lower_bound.size(); x++)
    {
        // cell in x direction in map frame
        int x_map = x_start + x;
        // if x value is outside costmap continue
        if (x_map < 0 || x_map >= layered_costmap_->getCostmap()->getSizeInCellsX())     continue;

        for (int y = lower_bound[x]; y <= upper_bound[x]; y++)
        {
            int y_map = y_start + y;
            // if y value is outside costmap continue
            if (y_map < 0 || y_map > layered_costmap_->getCostmap()->getSizeInCellsY())     continue;
            
            // calculate local costmap coordinates [0...100]
            uint index = master_grid.getIndex(x_map, y_map);
            master_array[index] = 254U;
        }
    }
}

} // namespace sgd_costmap_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sgd_costmap_plugins::StaticVectorLayer, nav2_costmap_2d::Layer)
