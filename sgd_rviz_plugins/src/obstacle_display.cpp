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

#include "sgd_rviz_plugins/obstacle_display.hpp"

namespace sgd_rviz_plugins
{

    ObstacleDisplay::ObstacleDisplay()
    {
        color_property_ = new rviz_common::properties::ColorProperty(
            "Color", QColor(25, 255, 0),
            "Color to draw the polygon.", this, SLOT(queueRender()));
        alpha_property_ = new rviz_common::properties::FloatProperty(
            "Alpha", 1.0f,
            "Amount of transparency to apply to the polygon.", this, SLOT(queueRender()));
        alpha_property_->setMin(0);
        alpha_property_->setMax(1);

        static int polygon_count = 0;
        std::string material_name = "ObstacleMaterial" + std::to_string(polygon_count++);
        material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);
    }

    ObstacleDisplay::~ObstacleDisplay()
    {
        if (initialized())
        {
            scene_manager_->destroyManualObject(manual_object_);
        }
    }

    void ObstacleDisplay::onInitialize()
    {
        MFDClass::onInitialize();

        manual_object_ = scene_manager_->createManualObject();
        manual_object_->setDynamic(true);
        scene_node_->attachObject(manual_object_);
    }

    void ObstacleDisplay::reset()
    {
        MFDClass::reset();
        manual_object_->clear();
    }

    bool validateFloats(sgd_msgs::msg::VecObstacleArray::ConstSharedPtr msg)
    {
        for (auto obst : msg->obstacles)
        {
            if(!rviz_common::validateFloats(obst.vertices))
            {
                return false;
            }
        }
        return true;
    }

    void ObstacleDisplay::processMessage(sgd_msgs::msg::VecObstacleArray::ConstSharedPtr msg)
    {
        if (msg->obstacles.size() < 1)  return;

        if (!validateFloats(msg))
        {
            setStatus(
                rviz_common::properties::StatusProperty::Error, "Topic",
                "Message contained invalid floating point values (nans or infs)");
            return;
        }

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
        {
            setMissingTransformToFixedFrame(msg->header.frame_id);
            return;
        }
        setTransformOk();

        scene_node_->setPosition(position);
        scene_node_->setOrientation(orientation);

        manual_object_->clear();

        Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property_->getColor());
        color.a = alpha_property_->getFloat();
        rviz_rendering::MaterialManager::enableAlphaBlending(material_, color.a);

        size_t num_points = msg->obstacles[0].vertices.size() * msg->obstacles.size();

        if (num_points > 0)
        {
            manual_object_->estimateVertexCount(num_points);
            manual_object_->begin(
                material_->getName(), Ogre::RenderOperation::OT_LINE_LIST, "rviz_rendering");

            // float yaw = tf2::getYaw(msg->pose.orientation);
            for (auto obstacle : msg->obstacles)
            {
                geometry_msgs::msg::Point32 &last_point = obstacle.vertices[0];
                // float x_ = cos(yaw)*last_point.x - sin(yaw)*last_point.y + msg->pose.position.x;
                // last_point.y = cos(yaw)*last_point.y + sin(yaw)*last_point.x + msg->pose.position.y;
                // last_point.x = x_;

                for (uint32_t i = 1; i < obstacle.vertices.size(); i++)
                {
                    geometry_msgs::msg::Point32 &msg_point = obstacle.vertices[i];
                    // float x = cos(yaw)*msg_point.x - sin(yaw)*msg_point.y + msg->pose.position.x;
                    // msg_point.y = cos(yaw)*msg_point.y + sin(yaw)*msg_point.x + msg->pose.position.y;
                    // msg_point.x = x;

                    // add two points per line
                    manual_object_->position(last_point.x, last_point.y, last_point.z);
                    manual_object_->colour(color);
                    manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
                    manual_object_->colour(color);

                    // set current point as last point
                    last_point = msg_point;
                }
            }
            manual_object_->end();
        }
    }

} // namespace sgd_rviz_plugins

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(sgd_rviz_plugins::ObstacleDisplay, rviz_common::Display)