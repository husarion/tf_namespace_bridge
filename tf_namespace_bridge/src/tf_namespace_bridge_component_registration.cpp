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

// Separate translation unit (not tf_namespace_bridge.cpp) so the plain
// tf_namespace_bridge executable doesn't need to link rclcpp_components. Only
// tf_namespace_bridge_component.so compiles this file. The CMake
// rclcpp_components_register_node() macro wires up ament_index/plugin
// metadata only — the class_loader export symbol still has to come from here.

#include "rclcpp_components/register_node_macro.hpp"
#include "tf_namespace_bridge/tf_namespace_bridge.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tf_namespace_bridge::TfNamespaceBridge)
