/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
//#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "sgd_msgs/msg/vec_obstacle_array.hpp"

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
  //int obstacle;
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
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y);

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
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief Method is called each time when map size was changed.
   * 
   */
  virtual void matchSize();

private:
  void getParameters();

  void processMap(const sgd_msgs::msg::VecObstacleArray & new_map);

  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void incomingMap(const sgd_msgs::msg::VecObstacleArray::SharedPtr new_map);

  unsigned char interpretValue(unsigned char value);

  /**
   * @brief 
   * 
   * @param lower 
   * @param upper 
   * @param value 
   * @return true 
   * @return false 
   */
  inline bool is_in_range(float min_obstacle, float max_obstacle, float min_costmap, float max_costmap)
  {
    // beide kleiner -> false
    // beide größer -> false
    if (min_obstacle < min_costmap && max_obstacle < min_costmap) return false;
    if (min_obstacle > max_costmap && max_obstacle > max_costmap) return false;
    return true;
  }

  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string map_frame_;  /// @brief frame that map is located in

  //std::vector<sgd_msgs::msg::VecObstacle> obstacles;
  std::vector<box_outline> box_outlines;

  bool has_updated_data_{false};
  bool has_new_map{false};

  unsigned int x_{0};
  unsigned int y_{0};
  unsigned int width_{0};
  unsigned int height_{0};

  rclcpp::Subscription<sgd_msgs::msg::VecObstacleArray>::SharedPtr map_sub_;
  rclcpp::Subscription<sgd_msgs::msg::VecObstacleArray>::SharedPtr map_update_sub_;

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
  // sgd_msgs::msg::VecObstacleArray::SharedPtr map_buffer_;

  // sgd variables
  double last_robot_x = -1.0, last_robot_y = -1.0;

  /**
   * @brief Checks if d1 and d2 are equal in the range of 1 centimeter.
   * 
   * @param d1 
   * @param d2 
   * @return true if d1 differs less than 1 centimeter from d2
   */
  inline bool equals_eps(double d1, double d2)
  {
    return (d1 > d2-10E-2 && d1 < d2+10E-2);
  }

};

}  // namespace sgd_costmap_plugins

#endif  // SGD_COSTMAP_PLUGINS__STATIC_VECTOR_LAYER_HPP_
