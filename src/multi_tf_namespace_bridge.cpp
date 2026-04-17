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

#include "tf_namespace_bridge/multi_tf_namespace_bridge.hpp"

#include <string>
#include <unordered_set>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace tf_namespace_bridge {

namespace {

const rclcpp::QoS kTfSubQos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
const rclcpp::QoS kTfPubQos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
const rclcpp::QoS kTfStaticQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

}  // namespace

MultiTfNamespaceBridge::MultiTfNamespaceBridge(const rclcpp::NodeOptions& options)
    : Node("multi_tf_namespace_bridge", options) {
  declare_parameter<std::vector<std::string>>("namespaces", std::vector<std::string>{});

  tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", kTfPubQos);
  tf_static_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", kTfStaticQos);

  param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) { return OnSetParameters(params); });

  const auto initial = get_parameter("namespaces").as_string_array();
  if (initial.empty()) {
    RCLCPP_WARN(get_logger(),
                "No namespaces configured — node is idle. Set the 'namespaces' parameter to start "
                "bridging.");
  }
  UpdateSubscriptions(initial);
}

void MultiTfNamespaceBridge::UpdateSubscriptions(const std::vector<std::string>& namespaces) {
  const std::unordered_set<std::string> new_ns_set(namespaces.begin(), namespaces.end());

  for (auto it = subscriptions_.begin(); it != subscriptions_.end();) {
    if (new_ns_set.find(it->first) == new_ns_set.end()) {
      RCLCPP_INFO(get_logger(), "Removing bridge for namespace: '%s'", it->first.c_str());
      it = subscriptions_.erase(it);
    } else {
      ++it;
    }
  }

  for (const auto& ns : namespaces) {
    if (subscriptions_.count(ns) > 0) {
      continue;
    }

    RCLCPP_INFO(get_logger(), "Adding bridge for namespace: '%s'", ns.c_str());

    auto& subs = subscriptions_[ns];
    subs.tf_sub = create_subscription<tf2_msgs::msg::TFMessage>(
        "/" + ns + "/tf", kTfSubQos,
        [this, ns](const tf2_msgs::msg::TFMessage::SharedPtr msg) { OnTf(msg, ns); });

    subs.tf_static_sub = create_subscription<tf2_msgs::msg::TFMessage>(
        "/" + ns + "/tf_static", kTfStaticQos,
        [this, ns](const tf2_msgs::msg::TFMessage::SharedPtr msg) { OnTfStatic(msg, ns); });
  }
}

rcl_interfaces::msg::SetParametersResult MultiTfNamespaceBridge::OnSetParameters(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    if (param.get_name() == "namespaces") {
      UpdateSubscriptions(param.as_string_array());
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void MultiTfNamespaceBridge::OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg,
                                  const std::string& ns) {
  tf_pub_->publish(PrefixMessage(*msg, ns + "/"));
}

void MultiTfNamespaceBridge::OnTfStatic(const tf2_msgs::msg::TFMessage::SharedPtr msg,
                                        const std::string& ns) {
  tf_static_pub_->publish(PrefixMessage(*msg, ns + "/"));
}

tf2_msgs::msg::TFMessage MultiTfNamespaceBridge::PrefixMessage(const tf2_msgs::msg::TFMessage& msg,
                                                               const std::string& prefix) const {
  tf2_msgs::msg::TFMessage prefixed = msg;
  for (auto& transform : prefixed.transforms) {
    transform.header.frame_id = prefix + transform.header.frame_id;
    transform.child_frame_id = prefix + transform.child_frame_id;
  }
  return prefixed;
}

}  // namespace tf_namespace_bridge
