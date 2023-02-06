/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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


#ifndef SGD_RVIZ_PLUGINS__DISPLAYS__OBSTACLE_DISPLAY_HPP_
#define SGD_RVIZ_PLUGINS__DISPLAYS__OBSTACLE_DISPLAY_HPP_

#include "sgd_msgs/msg/vec_obstacle_array.hpp"
#include "tf2/utils.h"

#include "rviz_common/message_filter_display.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include <string>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/material_manager.hpp"

namespace Ogre
{
class ManualObject;
}

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace sgd_rviz_plugins
{

/**
 * \class ObstacleDisplay
 * \brief Displays a sgd_msgs::msg::VecObstacleArray message
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC ObstacleDisplay : public
  rviz_common::MessageFilterDisplay<sgd_msgs::msg::VecObstacleArray>
{
  Q_OBJECT

public:
  ObstacleDisplay();
  ~ObstacleDisplay() override;

  void onInitialize() override;

  void reset() override;

protected:
  void processMessage(sgd_msgs::msg::VecObstacleArray::ConstSharedPtr msg) override;

  Ogre::ManualObject * manual_object_;
  Ogre::MaterialPtr material_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
};

}  // namespace sgd_rviz_plugins

#endif  // SGD_RVIZ_PLUGINS__DISPLAYS__OBSTACLE_DISPLAY_HPP_