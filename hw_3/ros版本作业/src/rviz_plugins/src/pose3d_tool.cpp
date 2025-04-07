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

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/render_panel.hpp"

#include "rviz_plugins/pose3d_tool.hpp"

namespace rviz_plugins {

Pose3DTool::Pose3DTool() : rviz_common::Tool(), arrow_(nullptr) {
    projection_finder_ =
        std::make_shared<rviz_rendering::ViewportProjectionFinder>();
}

Pose3DTool::~Pose3DTool() = default;

void Pose3DTool::onInitialize() {
    arrow_ = std::make_shared<rviz_rendering::Arrow>(scene_manager_, nullptr,
                                                     2.0f, 0.2f, 0.5f, 0.35f);
    arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
    arrow_->getSceneNode()->setVisible(false);
}

void Pose3DTool::activate() {
    setStatus("Click and drag mouse to set position/orientation.");
    state_ = Position;
}

void Pose3DTool::deactivate() { arrow_->getSceneNode()->setVisible(false); }

int Pose3DTool::processMouseEvent(rviz_common::ViewportMouseEvent &event) {
    int flags = 0;
    static Ogre::Vector3 ang_pos;
    static double initz;
    static double prevz;
    static double prevangle;
    const double z_scale = 50;
    const double z_interval = 0.5;
    Ogre::Quaternion orient_x = Ogre::Quaternion(
        Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);
    auto point_projection_on_xy_plane =
        projection_finder_->getViewportPointProjectionOnXYPlane(
            event.panel->getRenderWindow(), event.x, event.y);
    if (event.leftDown()) {
        assert(state_ == Position);
        if (point_projection_on_xy_plane.first) {
            pos_ = point_projection_on_xy_plane.second;
            arrow_->setPosition(pos_);
            state_ = Orientation;
            flags |= Render;
        }
    } else if (event.type == QEvent::MouseMove && event.left()) {
        if (state_ == Orientation) {
            // compute angle in x-y plane
            if (point_projection_on_xy_plane.first) {
                auto cur_pos = point_projection_on_xy_plane.second;
                double angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);
                arrow_->getSceneNode()->setVisible(true);
                arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle),
                                                        Ogre::Vector3::UNIT_Z) *
                                       orient_x);
                if (event.right())
                    state_ = Height;
                initz = pos_.z;
                prevz = event.y;
                prevangle = angle;
                flags |= Render;
            }
        }
        if (state_ == Height) {
            double z = event.y;
            double dz = z - prevz;
            prevz = z;
            pos_.z -= dz / z_scale;
            arrow_->setPosition(pos_);
            // Create a list of arrows
            for (size_t k = 0; k < arrow_array_.size(); k++)
                arrow_array_[k].reset();
            arrow_array_.clear();
            int cnt = ceil(fabs(initz - pos_.z) / z_interval);
            for (int k = 0; k < cnt; k++) {
                auto arrow = std::make_shared<rviz_rendering::Arrow>(
                    scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
                arrow->setColor(0.0f, 1.0f, 0.0f, 1.0f);
                arrow->getSceneNode()->setVisible(true);
                Ogre::Vector3 arr_pos = pos_;
                arr_pos.z =
                    initz - ((initz - pos_.z > 0) ? 1 : -1) * k * z_interval;
                arrow->setPosition(arr_pos);
                arrow->setOrientation(Ogre::Quaternion(Ogre::Radian(prevangle),
                                                       Ogre::Vector3::UNIT_Z) *
                                      orient_x);
                arrow_array_.push_back(arrow);
            }
            flags |= Render;
        }
    } else if (event.leftUp()) {
        if (state_ == Orientation || state_ == Height) {
            // Create a list of arrows
            for (size_t k = 0; k < arrow_array_.size(); k++) {
                arrow_array_[k].reset();
            }
            arrow_array_.clear();
            onPoseSet(pos_.x, pos_.y, pos_.z, prevangle);
            flags |= (Finished | Render);
        }
    }

    return flags;
}

geometry_msgs::msg::Quaternion Pose3DTool::orientationAroundZAxis(double angle)
{
  auto orientation = geometry_msgs::msg::Quaternion();
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = sin(angle) / (2 * cos(angle / 2));
  orientation.w = cos(angle / 2);
  return orientation;
}

} // namespace rviz_plugins
