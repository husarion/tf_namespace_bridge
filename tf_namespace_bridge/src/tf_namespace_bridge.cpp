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

#include <chrono>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace tf_namespace_bridge {

namespace {

const rclcpp::QoS kTfSubQos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
const rclcpp::QoS kTfPubQos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
const rclcpp::QoS kTfStaticQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

constexpr auto kParamPollPeriod = std::chrono::milliseconds(200);
constexpr auto kSummaryPollPeriod = std::chrono::milliseconds(500);
constexpr auto kSummaryDebounce = std::chrono::seconds(3);

std::string Join(const std::vector<std::string>& items, const std::string& sep) {
  std::string out;
  for (size_t i = 0; i < items.size(); ++i) {
    if (i > 0) out += sep;
    out += items[i];
  }
  return out;
}

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

  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  if (!filter_.SetPatterns(params_.frame_filters)) {
    RCLCPP_ERROR(get_logger(),
                 "Invalid glob pattern(s) in frame_filters at startup; running with no filter "
                 "(pass-through).");
  } else {
    applied_filters_ = params_.frame_filters;
  }

  RCLCPP_INFO(get_logger(), "Bridging TF with frame prefix: '%s'", prefix_.c_str());
  if (filter_.active()) {
    RCLCPP_INFO(get_logger(), "Active frame_filters: [%s]", Join(applied_filters_, ", ").c_str());
  }

  tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", kTfPubQos);
  tf_static_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", kTfStaticQos);

  // Relative topic names resolve to /<namespace>/tf and /<namespace>/tf_static
  tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "tf", kTfSubQos, [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { OnTf(msg); });

  tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "tf_static", kTfStaticQos,
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { OnTfStatic(msg); });

  param_poll_timer_ = create_wall_timer(kParamPollPeriod, [this]() { OnParamPoll(); });
  summary_timer_ = create_wall_timer(kSummaryPollPeriod, [this]() { OnSummaryPoll(); });
}

void TfNamespaceBridge::OnParamPoll() {
  if (!param_listener_->is_old(params_)) return;
  auto new_params = param_listener_->get_params();

  if (new_params.frame_filters != applied_filters_) {
    if (filter_.SetPatterns(new_params.frame_filters)) {
      applied_filters_ = new_params.frame_filters;
      summary_pending_ = false;
      RCLCPP_INFO(get_logger(), "Applied new frame_filters: [%s]",
                  Join(applied_filters_, ", ").c_str());
    } else {
      RCLCPP_ERROR(get_logger(),
                   "Invalid glob pattern(s) in frame_filters; keeping previous filter [%s]",
                   Join(applied_filters_, ", ").c_str());
    }
  }

  params_ = new_params;
}

void TfNamespaceBridge::OnSummaryPoll() {
  if (!summary_pending_) return;
  if (std::chrono::steady_clock::now() - last_auto_include_change_ < kSummaryDebounce) return;

  const auto frames = filter_.AutoIncludedFrames();
  if (!frames.empty()) {
    RCLCPP_WARN(get_logger(),
                "Auto-included %zu parent frame(s) to keep TF tree connected: [%s]. "
                "Add them to 'frame_filters' to silence this warning.",
                frames.size(), Join(frames, ", ").c_str());
  }
  summary_pending_ = false;
}

void TfNamespaceBridge::OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
  ProcessAndPublish(*msg, tf_pub_);
}

void TfNamespaceBridge::OnTfStatic(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
  ProcessAndPublish(*msg, tf_static_pub_);
}

void TfNamespaceBridge::ProcessAndPublish(
    const tf2_msgs::msg::TFMessage& msg,
    const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr& publisher) {
  auto result = filter_.Apply(msg);

  for (const auto& f : result.newly_auto_included) {
    RCLCPP_INFO(get_logger(), "Frame '%s' is parent of a bridged frame; auto-included", f.c_str());
  }
  if (!result.newly_auto_included.empty()) {
    last_auto_include_change_ = std::chrono::steady_clock::now();
    summary_pending_ = true;
  }

  if (result.out_msg.transforms.empty()) return;
  publisher->publish(PrefixMessage(result.out_msg));
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
