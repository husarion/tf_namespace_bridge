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

#ifndef TF_NAMESPACE_BRIDGE__NAMESPACE_TF_BRIDGE_HPP_
#define TF_NAMESPACE_BRIDGE__NAMESPACE_TF_BRIDGE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace tf_namespace_bridge {

class NamespaceTfBridge : public rclcpp::Node {
 public:
  explicit NamespaceTfBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void OnTfStatic(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  tf2_msgs::msg::TFMessage PrefixMessage(const tf2_msgs::msg::TFMessage& msg) const;

  std::string prefix_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;
};

}  // namespace tf_namespace_bridge

#endif  // TF_NAMESPACE_BRIDGE__NAMESPACE_TF_BRIDGE_HPP_
