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
#include <memory>
#include <string>
#include <vector>

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
  void ProcessAndPublish(const tf2_msgs::msg::TFMessage& msg,
                         const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr& publisher);
  tf2_msgs::msg::TFMessage PrefixMessage(const tf2_msgs::msg::TFMessage& msg) const;

  std::string prefix_;
  FrameFilter filter_;
  std::vector<std::string> applied_filters_;
  std::chrono::steady_clock::time_point last_auto_include_change_;
  bool summary_pending_ = false;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  rclcpp::TimerBase::SharedPtr param_poll_timer_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;
};

}  // namespace tf_namespace_bridge

#endif  // TF_NAMESPACE_BRIDGE__TF_NAMESPACE_BRIDGE_HPP_
