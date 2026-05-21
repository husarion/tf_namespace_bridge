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

#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "tf_namespace_bridge/multi_tf_namespace_bridge.hpp"

namespace {
// See tf_namespace_bridge_node.cpp for rationale.
void QuietSignalHandler(int /*signum*/) { rclcpp::shutdown(); }
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, QuietSignalHandler);
  std::signal(SIGTERM, QuietSignalHandler);
  rclcpp::spin(std::make_shared<tf_namespace_bridge::MultiTfNamespaceBridge>());
  rclcpp::shutdown();
  return 0;
}
