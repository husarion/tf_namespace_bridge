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

#include <chrono>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace tf_namespace_bridge {

namespace {

const rclcpp::QoS kTfSubQos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
const rclcpp::QoS kTfPubQos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
// See tf_namespace_bridge.cpp for the rationale: publisher latches the last
// (always-complete) snapshot; subscriber depth 100 matches the tf2_ros static
// listener convention.
const rclcpp::QoS kTfStaticPubQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
const rclcpp::QoS kTfStaticSubQos =
    rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();

constexpr auto kParamPollPeriod = std::chrono::milliseconds(200);
constexpr auto kSummaryPollPeriod = std::chrono::milliseconds(500);
constexpr auto kSummaryDebounce = std::chrono::seconds(3);
constexpr auto kStaticWatchdogPeriod = std::chrono::seconds(2);

std::string Join(const std::vector<std::string>& items, const std::string& sep) {
  std::string out;
  for (size_t i = 0; i < items.size(); ++i) {
    if (i > 0) out += sep;
    out += items[i];
  }
  return out;
}

}  // namespace

MultiTfNamespaceBridge::MultiTfNamespaceBridge(const rclcpp::NodeOptions& options)
    : Node("multi_tf_namespace_bridge", options) {
  param_listener_ =
      std::make_shared<multi_tf_namespace_bridge::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // Validate frame_filters once up-front so we know whether to keep an empty
  // applied set or accept the configured patterns.
  FrameFilter probe;
  if (probe.SetPatterns(params_.frame_filters)) {
    applied_filters_ = params_.frame_filters;
  } else {
    RCLCPP_ERROR(get_logger(),
                 "Invalid glob pattern(s) in frame_filters at startup; running with no filter "
                 "(pass-through).");
  }

  // Log only when the filter is actually active. After empty-entry skipping
  // (see FrameFilter::SetPatterns), inputs like [""] resolve to inactive.
  if (probe.active()) {
    RCLCPP_INFO(get_logger(), "Active frame_filters: [%s]", Join(applied_filters_, ", ").c_str());
  }

  tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", kTfPubQos);
  tf_static_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", kTfStaticPubQos);

  if (params_.namespaces.empty()) {
    RCLCPP_WARN(get_logger(),
                "No namespaces configured — node is idle. Set the 'namespaces' parameter to start "
                "bridging.");
  }
  UpdateSubscriptions(params_.namespaces);

  param_poll_timer_ = create_wall_timer(kParamPollPeriod, [this]() { OnParamPoll(); });
  summary_timer_ = create_wall_timer(kSummaryPollPeriod, [this]() { OnSummaryPoll(); });
  static_watchdog_timer_ =
      create_wall_timer(kStaticWatchdogPeriod, [this]() { OnStaticWatchdog(); });
}

void MultiTfNamespaceBridge::SubscribeStatic(const std::string& ns, NamespaceState& state) {
  state.tf_static_sub = create_subscription<tf2_msgs::msg::TFMessage>(
      "/" + ns + "/tf_static", kTfStaticSubQos,
      [this, ns](const tf2_msgs::msg::TFMessage::SharedPtr msg) { OnTfStatic(msg, ns); });
}

void MultiTfNamespaceBridge::OnStaticWatchdog() {
  // Per namespace: re-arm the /tf_static subscription until the upstream latched
  // tree arrives (one-shot topic; a lost transient_local delivery never
  // re-sends), then periodically re-publish the accumulated tree so downstream
  // consumers converge even if their late-join races. See tf_namespace_bridge.cpp.
  for (auto& [ns, state] : namespaces_) {
    if (!state.static_received) {
      RCLCPP_WARN_ONCE(get_logger(),
                       "[%s] No /tf_static received yet; re-arming subscription until the latched "
                       "static tree is delivered.",
                       ns.c_str());
      SubscribeStatic(ns, state);
    } else {
      PublishStaticCache(state);
    }
  }
}

void MultiTfNamespaceBridge::OnParamPoll() {
  if (!param_listener_->is_old(params_)) return;
  auto new_params = param_listener_->get_params();

  if (new_params.frame_filters != applied_filters_) {
    FrameFilter probe;
    if (probe.SetPatterns(new_params.frame_filters)) {
      for (auto& [ns, state] : namespaces_) {
        state.filter.SetPatterns(new_params.frame_filters);
        state.summary_pending = false;
      }
      applied_filters_ = new_params.frame_filters;
      if (probe.active()) {
        RCLCPP_INFO(get_logger(), "Applied new frame_filters: [%s]",
                    Join(applied_filters_, ", ").c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Cleared frame_filters (pass-through).");
      }
    } else {
      RCLCPP_ERROR(get_logger(),
                   "Invalid glob pattern(s) in frame_filters; keeping previous filter [%s]",
                   Join(applied_filters_, ", ").c_str());
    }
  }

  params_ = new_params;
  UpdateSubscriptions(new_params.namespaces);
}

void MultiTfNamespaceBridge::OnSummaryPoll() {
  const auto now = std::chrono::steady_clock::now();
  for (auto& [ns, state] : namespaces_) {
    if (!state.summary_pending) continue;
    if (now - state.last_auto_include_change < kSummaryDebounce) continue;

    const auto frames = state.filter.AutoIncludedFrames();
    if (!frames.empty()) {
      RCLCPP_WARN(get_logger(),
                  "[%s] Auto-included %zu parent frame(s) to keep TF tree connected: [%s]. "
                  "Add them to 'frame_filters' to silence this warning.",
                  ns.c_str(), frames.size(), Join(frames, ", ").c_str());
    }
    state.summary_pending = false;
  }
}

void MultiTfNamespaceBridge::UpdateSubscriptions(const std::vector<std::string>& namespaces) {
  const std::unordered_set<std::string> new_ns_set(namespaces.begin(), namespaces.end());

  for (auto it = namespaces_.begin(); it != namespaces_.end();) {
    if (new_ns_set.find(it->first) == new_ns_set.end()) {
      RCLCPP_INFO(get_logger(), "Removing bridge for namespace: '%s'", it->first.c_str());
      it = namespaces_.erase(it);
    } else {
      ++it;
    }
  }

  for (const auto& ns : namespaces) {
    if (namespaces_.count(ns) > 0) {
      continue;
    }

    RCLCPP_INFO(get_logger(), "Adding bridge for namespace: '%s'", ns.c_str());

    auto& state = namespaces_[ns];
    state.filter.SetPatterns(applied_filters_);  // applied_filters_ is pre-validated

    state.tf_sub = create_subscription<tf2_msgs::msg::TFMessage>(
        "/" + ns + "/tf", kTfSubQos,
        [this, ns](const tf2_msgs::msg::TFMessage::SharedPtr msg) { OnTf(msg, ns); });

    SubscribeStatic(ns, state);
  }
}

void MultiTfNamespaceBridge::OnTf(const tf2_msgs::msg::TFMessage::SharedPtr msg,
                                  const std::string& ns) {
  auto it = namespaces_.find(ns);
  if (it == namespaces_.end()) return;
  ProcessAndPublish(it->second, ns, *msg, tf_pub_);
}

void MultiTfNamespaceBridge::OnTfStatic(const tf2_msgs::msg::TFMessage::SharedPtr msg,
                                        const std::string& ns) {
  auto it = namespaces_.find(ns);
  if (it == namespaces_.end()) return;
  auto& state = it->second;

  auto result = state.filter.Apply(*msg);
  for (const auto& f : result.newly_auto_included) {
    RCLCPP_INFO(get_logger(), "[%s] Frame '%s' is parent of a bridged frame; auto-included",
                ns.c_str(), f.c_str());
  }
  if (!result.newly_auto_included.empty()) {
    state.last_auto_include_change = std::chrono::steady_clock::now();
    state.summary_pending = true;
  }
  if (result.out_msg.transforms.empty()) return;

  // Accumulate + re-publish the COMPLETE tree (see tf_namespace_bridge.cpp).
  for (const auto& t : PrefixMessage(result.out_msg, ns + "/").transforms) {
    state.static_cache[t.child_frame_id] = t;
  }
  state.static_received = true;
  PublishStaticCache(state);
}

void MultiTfNamespaceBridge::PublishStaticCache(NamespaceState& state) {
  if (state.static_cache.empty()) return;
  tf2_msgs::msg::TFMessage out;
  out.transforms.reserve(state.static_cache.size());
  for (const auto& [child, transform] : state.static_cache) out.transforms.push_back(transform);
  tf_static_pub_->publish(out);
}

void MultiTfNamespaceBridge::ProcessAndPublish(
    NamespaceState& state, const std::string& ns, const tf2_msgs::msg::TFMessage& msg,
    const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr& publisher) {
  auto result = state.filter.Apply(msg);

  for (const auto& f : result.newly_auto_included) {
    RCLCPP_INFO(get_logger(), "[%s] Frame '%s' is parent of a bridged frame; auto-included",
                ns.c_str(), f.c_str());
  }
  if (!result.newly_auto_included.empty()) {
    state.last_auto_include_change = std::chrono::steady_clock::now();
    state.summary_pending = true;
  }

  if (result.out_msg.transforms.empty()) return;
  publisher->publish(PrefixMessage(result.out_msg, ns + "/"));
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
