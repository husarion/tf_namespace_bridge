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

#ifndef TF_NAMESPACE_BRIDGE__TF_NAMESPACE_BRIDGE_HPP_
#define TF_NAMESPACE_BRIDGE__TF_NAMESPACE_BRIDGE_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf_namespace_bridge/frame_filter.hpp"
#include "tf_namespace_bridge/tf_namespace_bridge_parameters.hpp"

namespace tf_namespace_bridge {

class TfNamespaceBridge : public rclcpp::Node {
 public:
  explicit TfNamespaceBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void OnParamPoll();
  void OnSummaryPoll();
  void OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void OnTfStatic(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  // Shared filter + auto-include-logging step for both /tf and /tf_static.
  FrameFilter::ApplyResult ApplyAndLog(const tf2_msgs::msg::TFMessage& msg);
  // (Re)create the /tf_static subscription. Used at startup and by the
  // reception watchdog to force a fresh transient_local query.
  void SubscribeStatic();
  // Reception watchdog: re-arm the /tf_static subscription until the upstream
  // latched static tree has been delivered (it is one-shot and never re-sent).
  void OnStaticWatchdog();
  // Publish the full accumulated static tree (so the latched snapshot is always
  // complete, regardless of how many messages it arrived across).
  void PublishStaticCache();
  tf2_msgs::msg::TFMessage PrefixMessage(const tf2_msgs::msg::TFMessage& msg) const;

  std::string prefix_;
  FrameFilter filter_;
  std::vector<std::string> applied_filters_;
  std::chrono::steady_clock::time_point last_auto_include_change_;
  bool summary_pending_ = false;

  // Accumulated, already-prefixed static transforms keyed by child_frame_id
  // (each frame has exactly one parent → last-wins is correct). Lets us always
  // re-publish the COMPLETE tree instead of just the last message received.
  std::map<std::string, geometry_msgs::msg::TransformStamped> static_cache_;
  bool static_received_ = false;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  rclcpp::TimerBase::SharedPtr param_poll_timer_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
  rclcpp::TimerBase::SharedPtr static_watchdog_timer_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;
};

}  // namespace tf_namespace_bridge

#endif  // TF_NAMESPACE_BRIDGE__TF_NAMESPACE_BRIDGE_HPP_
