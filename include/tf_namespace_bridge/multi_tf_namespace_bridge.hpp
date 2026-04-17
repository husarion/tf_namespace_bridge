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

#include <string>
#include <unordered_map>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace tf_namespace_bridge {

class MultiTfNamespaceBridge : public rclcpp::Node {
 public:
  explicit MultiTfNamespaceBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  struct NamespaceSubscriptions {
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub;
  };

  void UpdateSubscriptions(const std::vector<std::string>& namespaces);
  rcl_interfaces::msg::SetParametersResult OnSetParameters(
      const std::vector<rclcpp::Parameter>& parameters);
  void OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg, const std::string& ns);
  void OnTfStatic(const tf2_msgs::msg::TFMessage::SharedPtr msg, const std::string& ns);
  tf2_msgs::msg::TFMessage PrefixMessage(const tf2_msgs::msg::TFMessage& msg,
                                         const std::string& prefix) const;

  std::unordered_map<std::string, NamespaceSubscriptions> subscriptions_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace tf_namespace_bridge

#endif  // TF_NAMESPACE_BRIDGE__MULTI_TF_NAMESPACE_BRIDGE_HPP_
