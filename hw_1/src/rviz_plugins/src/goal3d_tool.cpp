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

#include "rviz_plugins/goal3d_tool.hpp"

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"

namespace rviz_plugins
{

Goal3DTool::Goal3DTool()
: rviz_plugins::Pose3DTool(), qos_profile_(5)
{
  shortcut_key_ = 'g';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "goal",
    "The topic on which to publish navigation goals.",
    getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

Goal3DTool::~Goal3DTool() = default;

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  setName("3D Nav Goal");
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
  publisher_ = raw_node->
    template create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), qos_profile_);
  clock_ = raw_node->get_clock();
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
{
  RVIZ_COMMON_LOG_INFO_STREAM("3D Goal Set");
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = clock_->now();
  goal.header.frame_id = fixed_frame;

  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  goal.pose.orientation = orientationAroundZAxis(theta);
  auto & position = goal.pose.position;
  auto & orientation = goal.pose.orientation;
  RVIZ_COMMON_LOG_INFO_STREAM(
    "Setting goal: Frame:" << fixed_frame << ", Position(" << position.x << ", " <<
      position.y << ", " << position.z << "), Orientation(" << orientation.x << ", " <<
      orientation.y << ", " << orientation.z << ", " << orientation.w <<
      ") = Angle: " << theta);
  publisher_->publish(goal);
}

} // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_plugins::Goal3DTool, rviz_common::Tool)
