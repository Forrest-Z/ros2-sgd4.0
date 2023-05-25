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

#ifndef SGD_COSTMAP_PLUGINS__STATIC_VECTOR_LAYER_HPP_
#define SGD_COSTMAP_PLUGINS__STATIC_VECTOR_LAYER_HPP_

#include <mutex>
#include <string>
#include <algorithm>
#include <memory>

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "message_filters/subscriber.h"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "sgd_msgs/msg/vec_obstacle_array.hpp"
#include "sgd_rasterizer/circle_rasterizer.hpp"
#include "sgd_rasterizer/ray_rasterizer.hpp"

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

namespace sgd_costmap_plugins
{

class StaticVectorLayer : public nav2_costmap_2d::CostmapLayer
{

    struct box_outline
    {
        float min_x, min_y, max_x, max_y;
        sgd_msgs::msg::VecObstacle obstacle;
    };

public:
    StaticVectorLayer();
    virtual ~StaticVectorLayer();

    /**
     * @brief Method is called at the end of plugin initialization. There is usually
     * declarations of ROS parameters. This is where any required initialization should occur.
     */
    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();

    /**
     * @brief It may have any code to be executed during costmap reset.
     *
     */
    virtual void reset();

    /**
     * @brief Method is called to ask the plugin: which area of costmap layer it needs to
     * update. There are 3 input parameters of method: robot position and orientation and
     * 4 output parameters: pointers to window bounds. These bounds are used for performance
     * reasons: to update the area inside the window where is new info available, avoiding
     * updates of whole costmap on every iteration.
     *
     * @param robot_x
     * @param robot_y
     * @param robot_yaw
     * @param min_x
     * @param min_y
     * @param max_x
     * @param max_y
     */
    virtual void updateBounds(
        double robot_x, double robot_y, double robot_yaw, double *min_x,
        double *min_y, double *max_x, double *max_y);

    /**
     * @brief Method is called each time when costmap re-calculation is required. It updates
     * the costmap layer only within its bounds window. There are 4 input parameters of
     * method: calculation window bounds and 1 output parameter: reference to a resulting
     * costmap master_grid. The Layer class provides the plugin with an internal costmap,
     * costmap_, for updates. The master_grid should be updated with values within the window
     * bounds using one of the following update methods: updateWithAddition(), updateWithMax(),
     * updateWithOverwrite() or updateWithTrueOverwrite().
     *
     * @param master_grid
     * @param min_i
     * @param min_j
     * @param max_i
     * @param max_j
     */
    virtual void updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j, int max_i, int max_j);

    /**
     * @brief Method is called each time when map size was changed.
     *
     */
    virtual void matchSize();

private:
    /**
     * @brief Loads and saves the parameters for the plugin
     * 
     */
    void getParameters();

    /**
     * @brief Initialize plog from parameters
     * 
     */
    void initLogging();

    /**
     * @brief Processes the new map so that it can be used by the plugin. 
     * 
     * @param new_map 
     */
    void processMap(const sgd_msgs::msg::VecObstacleArray &new_map);

    /**
     * @brief  Callback to update the costmap's map from the map_server
     * @param new_map The map to put into the costmap. The origin of the new
     * map along with its size will determine what parts of the costmap's
     * static map are overwritten.
     */
    void incomingMap(const sgd_msgs::msg::VecObstacleArray::SharedPtr new_map);

    /**
     * @brief Checks if the obstacle outline intersects the costmap outline
     * (all values are given in world coordinates)
     *
     * @param min_obstacle lower bound of obstacle outline
     * @param max_obstacle upper bound of obstacle
     * @param min_costmap lower bound of costmap
     * @param max_costmap upper bound of costmap
     * @return true if obstacle outline intersects costmap outline
     */
    inline bool is_in_range(float min_obstacle, float max_obstacle, float min_costmap, float max_costmap)
    {
        // beide kleiner -> false
        // beide größer -> false
        if (min_obstacle < min_costmap && max_obstacle < min_costmap)
            return false;
        if (min_obstacle > max_costmap && max_obstacle > max_costmap)
            return false;
        return true;
    }

    /**
     * @brief Checks if the given point (px, py) is inside the outline given by polygon_outline.
     *
     * @param polygon_outline outline of the polygon
     * @param px x value of the point
     * @param py y value of the point
     * @return true if the point is inside the polygon otherwise false
     */
    bool is_point_inside_polygon(std::vector<double> polygon_outline, float px, float py, float x_end);

    /**
     * @brief Saves the calculated values in the costmap.
     *
     * @param master_grid costmap in which the values are to be saved
     * @param x_start cell where the obstacle starts in global coordinates (point with the smallest x value)
     * @param y_start cell where the obstacle starts in global coordinates (point with the smallest y value)
     * @param lower_bound lower boundary of the obstacle
     * @param upper_bound upper boundary of the obstacle
     */
    void save_to_costmap(nav2_costmap_2d::Costmap2D &master_grid, uint8_t confidence,
            int x_start, int y_start, std::vector<int> lower_bound, std::vector<int> upper_bound);

    // std::string global_frame_; /// @brief The global frame for the costmap
    // std::string map_frame_;    /// @brief frame that map is located in

    std::vector<box_outline> box_outlines; /// @brief contains outlines of the obstacles to check whether the obstacle should be displayed in the costmap or not

    rclcpp::Subscription<sgd_msgs::msg::VecObstacleArray>::SharedPtr map_sub_;  /// @brief subscriber for map topic
    // rclcpp::Subscription<sgd_msgs::msg::VecObstacleArray>::SharedPtr map_update_sub_;

    // Parameters
    std::string map_topic_;
    bool map_subscribe_transient_local_;
    bool subscribe_to_updates_;
    bool track_unknown_space_;
    bool use_maximum_;
    unsigned char lethal_threshold_;
    unsigned char unknown_cost_value_;
    bool trinary_costmap_;
    bool map_received_{false};
    tf2::Duration transform_tolerance_;
    
    std::atomic<bool> update_in_progress_;
    bool has_new_map{false};    /// @brief set to true if a new map was received 
    // sgd_msgs::msg::VecObstacleArray::SharedPtr map_buffer_;

    // sgd variables
    double last_robot_x = -1.0;     /// @brief x position of robot on last iteration
    double last_robot_y = -1.0;     /// @brief y position of robot on last iteration
    bool robo_has_moved = true;     /// @brief update costmap only if robot has moved

    std::unique_ptr<sgd_rasterizer::RayRasterizer> rr;

    /**
     * @brief Checks if d1 and d2 are equal in the range of 1 centimeter.
     *
     * @param d1
     * @param d2
     * @return true if d1 differs less than 1 centimeter from d2
     */
    inline bool equals_eps(double d1, double d2)
    {
        return (d1 > d2 - 10E-2 && d1 < d2 + 10E-2);
    }
};

} // namespace sgd_costmap_plugins

#endif // SGD_COSTMAP_PLUGINS__STATIC_VECTOR_LAYER_HPP_
