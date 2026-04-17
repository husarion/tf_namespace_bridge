// Copyright 2026 Husarion sp. z o.o.
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

#include "tf_namespace_bridge/tf_namespace_bridge.hpp"

#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace tf_namespace_bridge {

namespace {

const rclcpp::QoS kTfQos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
const rclcpp::QoS kTfStaticQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

}  // namespace

TfNamespaceBridge::TfNamespaceBridge(const rclcpp::NodeOptions& options)
    : Node("tf_namespace_bridge", options) {
  // Derive frame prefix from node namespace: "/robot1" -> "robot1/"
  std::string ns = get_namespace();
  if (ns.size() > 1) {
    prefix_ = ns.substr(1) + "/";
  }

  if (prefix_.empty()) {
    throw std::invalid_argument(
        "tf_namespace_bridge requires a non-root namespace (e.g. --ros-args -r __ns:=/robot1). "
        "Running without a namespace would create a /tf feedback loop.");
  }

  RCLCPP_INFO(get_logger(), "Bridging TF with frame prefix: '%s'", prefix_.c_str());

  tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", kTfQos);
  tf_static_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", kTfStaticQos);

  // Relative topic names resolve to /<namespace>/tf and /<namespace>/tf_static
  tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "tf", kTfQos, [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { OnTf(msg); });

  tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "tf_static", kTfStaticQos,
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { OnTfStatic(msg); });
}

void TfNamespaceBridge::OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
  tf_pub_->publish(PrefixMessage(*msg));
}

void TfNamespaceBridge::OnTfStatic(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
  tf_static_pub_->publish(PrefixMessage(*msg));
}

tf2_msgs::msg::TFMessage TfNamespaceBridge::PrefixMessage(
    const tf2_msgs::msg::TFMessage& msg) const {
  tf2_msgs::msg::TFMessage prefixed = msg;
  for (auto& transform : prefixed.transforms) {
    transform.header.frame_id = prefix_ + transform.header.frame_id;
    transform.child_frame_id = prefix_ + transform.child_frame_id;
  }
  return prefixed;
}

}  // namespace tf_namespace_bridge
