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

#ifndef TF_NAMESPACE_BRIDGE__MULTI_TF_NAMESPACE_BRIDGE_HPP_
#define TF_NAMESPACE_BRIDGE__MULTI_TF_NAMESPACE_BRIDGE_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf_namespace_bridge/frame_filter.hpp"
#include "tf_namespace_bridge/multi_tf_namespace_bridge_parameters.hpp"

namespace tf_namespace_bridge {

class MultiTfNamespaceBridge : public rclcpp::Node {
 public:
  explicit MultiTfNamespaceBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  struct NamespaceState {
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub;
    FrameFilter filter;
    std::chrono::steady_clock::time_point last_auto_include_change;
    bool summary_pending = false;
    // Accumulated, already-prefixed static transforms keyed by child_frame_id,
    // so the latched /tf_static snapshot is always the COMPLETE tree.
    std::map<std::string, geometry_msgs::msg::TransformStamped> static_cache;
    bool static_received = false;
  };

  void UpdateSubscriptions(const std::vector<std::string>& namespaces);
  void OnParamPoll();
  void OnSummaryPoll();
  void OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, const std::string& ns);
  void OnTfStatic(const tf2_msgs::msg::TFMessage::SharedPtr msg, const std::string& ns);
  void ProcessAndPublish(NamespaceState& state, const std::string& ns,
                         const tf2_msgs::msg::TFMessage& msg,
                         const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr& publisher);
  // (Re)create one namespace's /tf_static subscription (startup + watchdog).
  void SubscribeStatic(const std::string& ns, NamespaceState& state);
  // Re-arm any namespace whose latched /tf_static hasn't been delivered yet.
  void OnStaticWatchdog();
  // Publish the merged accumulated static tree across ALL namespaces. All
  // namespaces share one latched (KeepLast(1)) publisher, so a per-namespace
  // publish would overwrite the others' history — every publish must carry
  // the complete cross-namespace snapshot.
  void PublishAllStaticCaches();
  tf2_msgs::msg::TFMessage PrefixMessage(const tf2_msgs::msg::TFMessage& msg,
                                         const std::string& prefix) const;

  std::shared_ptr<multi_tf_namespace_bridge::ParamListener> param_listener_;
  multi_tf_namespace_bridge::Params params_;
  rclcpp::TimerBase::SharedPtr param_poll_timer_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
  rclcpp::TimerBase::SharedPtr static_watchdog_timer_;
  std::vector<std::string> applied_filters_;
  std::unordered_map<std::string, NamespaceState> namespaces_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;
};

}  // namespace tf_namespace_bridge

#endif  // TF_NAMESPACE_BRIDGE__MULTI_TF_NAMESPACE_BRIDGE_HPP_
