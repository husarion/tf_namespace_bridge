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

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf_namespace_bridge/tf_namespace_bridge.hpp"

using namespace std::chrono_literals;

namespace {

const rclcpp::QoS kTfQos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
const rclcpp::QoS kTfStaticQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

tf2_msgs::msg::TFMessage MakeMessage(
    const std::vector<std::pair<std::string, std::string>>& transforms) {
  tf2_msgs::msg::TFMessage msg;
  for (const auto& [parent, child] : transforms) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    msg.transforms.push_back(t);
  }
  return msg;
}

}  // namespace

class TfNamespaceBridgeTest : public ::testing::Test {
 protected:
  void SetUpWithNamespace(const std::string& ns) {
    rclcpp::NodeOptions opts;
    if (!ns.empty()) {
      opts.arguments({"--ros-args", "-r", "__ns:=/" + ns});
    }
    bridge_ = std::make_shared<tf_namespace_bridge::TfNamespaceBridge>(opts);
    test_node_ = rclcpp::Node::make_shared("test_node_ns");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(bridge_);
    executor_->add_node(test_node_);
  }

  void TearDown() override {
    executor_.reset();
    bridge_.reset();
    test_node_.reset();
  }

  bool WaitFor(std::chrono::milliseconds timeout, std::function<bool()> condition) {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some(5ms);
      if (condition()) return true;
    }
    return condition();
  }

  std::shared_ptr<tf_namespace_bridge::TfNamespaceBridge> bridge_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

TEST_F(TfNamespaceBridgeTest, PrefixesHeaderFrameIdAndChildFrameId) {
  SetUpWithNamespace("robot1");

  tf2_msgs::msg::TFMessage received;
  bool got = false;

  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", kTfQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        received = *msg;
        got = true;
      });
  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf", kTfQos);

  WaitFor(100ms, [] { return false; });
  pub->publish(MakeMessage({{"base_link", "imu_link"}}));

  ASSERT_TRUE(WaitFor(500ms, [&] { return got; })) << "No message received on /tf";
  ASSERT_EQ(received.transforms.size(), 1u);
  EXPECT_EQ(received.transforms[0].header.frame_id, "robot1/base_link");
  EXPECT_EQ(received.transforms[0].child_frame_id, "robot1/imu_link");
}

TEST_F(TfNamespaceBridgeTest, PrefixesAllTransformsInMessage) {
  SetUpWithNamespace("robot1");

  tf2_msgs::msg::TFMessage received;
  bool got = false;

  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", kTfQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        received = *msg;
        got = true;
      });
  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf", kTfQos);

  WaitFor(100ms, [] { return false; });
  pub->publish(MakeMessage(
      {{"base_link", "imu_link"}, {"base_link", "camera_link"}, {"base_link", "lidar_link"}}));

  ASSERT_TRUE(WaitFor(500ms, [&] { return got; }));
  ASSERT_EQ(received.transforms.size(), 3u);
  for (const auto& t : received.transforms) {
    EXPECT_EQ(t.header.frame_id, "robot1/base_link");
  }
  EXPECT_EQ(received.transforms[0].child_frame_id, "robot1/imu_link");
  EXPECT_EQ(received.transforms[1].child_frame_id, "robot1/camera_link");
  EXPECT_EQ(received.transforms[2].child_frame_id, "robot1/lidar_link");
}

TEST_F(TfNamespaceBridgeTest, PrefixesStaticTfFrames) {
  SetUpWithNamespace("robot1");

  tf2_msgs::msg::TFMessage received;
  bool got = false;

  auto pub =
      test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf_static", kTfStaticQos);
  WaitFor(100ms, [] { return false; });
  pub->publish(MakeMessage({{"base_link", "cover_link"}}));

  // Subscribe after publish — transient_local must deliver the latched message
  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", kTfStaticQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        received = *msg;
        got = true;
      });

  ASSERT_TRUE(WaitFor(500ms, [&] { return got; })) << "No message received on /tf_static";
  ASSERT_EQ(received.transforms.size(), 1u);
  EXPECT_EQ(received.transforms[0].header.frame_id, "robot1/base_link");
  EXPECT_EQ(received.transforms[0].child_frame_id, "robot1/cover_link");
}

TEST_F(TfNamespaceBridgeTest, AccumulatesStaticTfAcrossMessages) {
  // Regression: /tf_static can arrive across SEPARATE messages (multiple static
  // broadcasters, or RSP re-publishing a subset). The latched republish must
  // contain the COMPLETE accumulated tree, not just the last message — pre-fix,
  // the KeepLast(1) latched publisher kept only the final message and silently
  // dropped every other broadcaster's frames.
  SetUpWithNamespace("robot1");

  auto pub =
      test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf_static", kTfStaticQos);
  WaitFor(150ms, [] { return false; });  // let the bridge's sub match
  pub->publish(MakeMessage({{"base_link", "cover_link"}}));
  WaitFor(150ms, [] { return false; });
  pub->publish(MakeMessage({{"cover_link", "rplidar_link"}}));

  // Subscribe AFTER both publishes — transient_local must hand us the full set.
  tf2_msgs::msg::TFMessage received;
  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", kTfStaticQos,
      [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) { received = *msg; });

  ASSERT_TRUE(WaitFor(1500ms, [&] { return received.transforms.size() >= 2u; }))
      << "latched /tf_static did not carry both accumulated transforms";
  ASSERT_EQ(received.transforms.size(), 2u);
  std::vector<std::string> children{received.transforms[0].child_frame_id,
                                    received.transforms[1].child_frame_id};
  EXPECT_NE(std::find(children.begin(), children.end(), "robot1/cover_link"), children.end());
  EXPECT_NE(std::find(children.begin(), children.end(), "robot1/rplidar_link"), children.end());
}

TEST_F(TfNamespaceBridgeTest, WatchdogRearmSurvivesIntoLaterDelivery) {
  // Regression for the watchdog re-arm path itself (OnStaticWatchdog /
  // SubscribeStatic), not just the accumulation it enables. No /tf_static
  // publisher exists yet at startup, so the watchdog (2s cadence) re-arms the
  // subscription at least once with static_received_ still false. The point
  // is that the re-armed subscription must still be able to receive a message
  // that arrives afterwards — a broken re-arm (e.g. subscribing to the wrong
  // topic, or leaving a dangling callback) would silently never receive.
  SetUpWithNamespace("robot1");

  // Outlive at least one watchdog tick with nothing published.
  WaitFor(2500ms, [] { return false; });

  auto pub =
      test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf_static", kTfStaticQos);
  WaitFor(150ms, [] { return false; });  // let the re-armed sub match
  pub->publish(MakeMessage({{"base_link", "cover_link"}}));

  tf2_msgs::msg::TFMessage received;
  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", kTfStaticQos,
      [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) { received = *msg; });

  ASSERT_TRUE(WaitFor(1500ms, [&] { return !received.transforms.empty(); }))
      << "no /tf_static delivered after the watchdog re-armed the subscription";
  EXPECT_EQ(received.transforms[0].child_frame_id, "robot1/cover_link");
}

TEST_F(TfNamespaceBridgeTest, EmptyMessageIsNotRepublished) {
  // Bridge skips publishing when the post-filter message has no transforms,
  // including the trivial case of an empty input. The contract here is twofold:
  // the bridge does not crash on empty input, and it does not waste DDS
  // bandwidth forwarding empty TFMessage frames.
  SetUpWithNamespace("robot1");
  int count = 0;

  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", kTfQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr) { ++count; });
  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf", kTfQos);

  WaitFor(100ms, [] { return false; });
  pub->publish(tf2_msgs::msg::TFMessage{});

  WaitFor(300ms, [&] { return count > 0; });
  EXPECT_EQ(count, 0);
}

TEST_F(TfNamespaceBridgeTest, RootNamespaceThrowsToPreventFeedbackLoop) {
  // Running without a namespace would subscribe and publish to /tf simultaneously,
  // creating an infinite feedback loop. The node must refuse to start.
  EXPECT_THROW(SetUpWithNamespace(""), std::invalid_argument);
}

TEST_F(TfNamespaceBridgeTest, RemapCollisionThrowsToPreventFeedbackLoop) {
  // Regression for a production incident: a bringup-wide
  // "set_remap: from: /tf to: tf" (meant for robot_state_publisher/ekf_node/
  // controller_manager) also applied to this node, folding its absolute
  // "/tf" publisher onto the same resolved topic as its "tf" subscriber
  // (/robot1/tf). The namespace is non-root, so the existing guard misses
  // this — only comparing the post-remap resolved topic names catches it.
  // Unguarded, this ran for ~2 hours on hardware and reached >5 GB RSS.
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__ns:=/robot1", "-r", "/tf:=tf"});
  EXPECT_THROW(std::make_shared<tf_namespace_bridge::TfNamespaceBridge>(opts),
               std::invalid_argument);
}

TEST_F(TfNamespaceBridgeTest, StaticRemapCollisionThrowsToPreventFeedbackLoop) {
  // Same failure mode as above, for the /tf_static side.
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__ns:=/robot1", "-r", "/tf_static:=tf_static"});
  EXPECT_THROW(std::make_shared<tf_namespace_bridge::TfNamespaceBridge>(opts),
               std::invalid_argument);
}

// --- YAML params-file scenarios for empty / sentinel filters ---
//
// These tests document how rclcpp's --params-file YAML loader behaves for
// empty and "no filter" array values. The bridge guarantees pass-through for
// [""] and ["*"]; [] is fundamentally rejected by rclcpp before our code
// runs (the YAML loader cannot type-tag an empty sequence), so we assert
// the failure mode rather than silently work around it.

namespace {

std::string WriteParamsFile(const std::string& contents) {
  char tmpl[] = "/tmp/tf_namespace_bridge_test_XXXXXX.yaml";
  int fd = mkstemps(tmpl, 5);
  if (fd >= 0) {
    [[maybe_unused]] auto written = ::write(fd, contents.data(), contents.size());
    ::close(fd);
  }
  return tmpl;
}

rclcpp::NodeOptions OptionsForYaml(const std::string& ns, const std::string& yaml_path) {
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__ns:=/" + ns, "--params-file", yaml_path});
  return opts;
}

}  // namespace

TEST(TfNamespaceBridgeYamlConfig, EmptyArrayInYamlIsRejectedByRclcpp) {
  // rclcpp's YAML parameter loader cannot infer the element type of an empty
  // sequence, so it stores `frame_filters: []` as PARAMETER_NOT_SET and the
  // ParamListener then fails to convert. Documented limitation; users should
  // pass [""] or ["*"] instead. The throw originates from Node base class
  // construction, before our class body runs — we cannot catch it.
  const auto path = WriteParamsFile(
      "/**:\n  tf_namespace_bridge:\n    ros__parameters:\n      frame_filters: []\n");
  auto opts = OptionsForYaml("robot_yaml_empty", path);
  EXPECT_THROW(std::make_shared<tf_namespace_bridge::TfNamespaceBridge>(opts),
               rclcpp::exceptions::InvalidParameterValueException);
  std::remove(path.c_str());
}

TEST(TfNamespaceBridgeYamlConfig, EmptyStringSentinelInYamlIsAcceptedAsPassThrough) {
  // [""] is the recommended workaround for the empty-array limitation. The
  // FrameFilter silently skips empty patterns, leaving the filter inactive.
  const auto path = WriteParamsFile(
      "/**:\n  tf_namespace_bridge:\n    ros__parameters:\n      frame_filters: [\"\"]\n");
  auto opts = OptionsForYaml("robot_yaml_empty_str", path);
  EXPECT_NO_THROW(
      { auto bridge = std::make_shared<tf_namespace_bridge::TfNamespaceBridge>(opts); });
  std::remove(path.c_str());
}

TEST(TfNamespaceBridgeYamlConfig, StarPatternInYamlIsAcceptedAsPassThrough) {
  // ["*"] is the alternative workaround — the regex matches every frame, so
  // the bridge behaves as pass-through (with a tiny per-frame regex cost).
  const auto path = WriteParamsFile(
      "/**:\n  tf_namespace_bridge:\n    ros__parameters:\n      frame_filters: [\"*\"]\n");
  auto opts = OptionsForYaml("robot_yaml_star", path);
  EXPECT_NO_THROW(
      { auto bridge = std::make_shared<tf_namespace_bridge::TfNamespaceBridge>(opts); });
  std::remove(path.c_str());
}

// --- Frame filter integration ---

class TfNamespaceBridgeFilteredTest : public ::testing::Test {
 protected:
  void SetUpWithFilter(const std::string& ns, const std::vector<std::string>& filters) {
    rclcpp::NodeOptions opts;
    opts.arguments({"--ros-args", "-r", "__ns:=/" + ns});
    opts.parameter_overrides({rclcpp::Parameter("frame_filters", filters)});
    bridge_ = std::make_shared<tf_namespace_bridge::TfNamespaceBridge>(opts);
    test_node_ = rclcpp::Node::make_shared("test_node_filtered");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(bridge_);
    executor_->add_node(test_node_);
  }

  void TearDown() override {
    executor_.reset();
    bridge_.reset();
    test_node_.reset();
  }

  bool WaitFor(std::chrono::milliseconds timeout, std::function<bool()> condition) {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some(5ms);
      if (condition()) return true;
    }
    return condition();
  }

  std::shared_ptr<tf_namespace_bridge::TfNamespaceBridge> bridge_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

TEST_F(TfNamespaceBridgeFilteredTest, FilterBlocksNonMatchingChild) {
  SetUpWithFilter("robot1", {"base_link"});

  tf2_msgs::msg::TFMessage received;
  bool got = false;
  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", kTfQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        received = *msg;
        got = true;
      });
  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf", kTfQos);
  WaitFor(100ms, [] { return false; });
  pub->publish(MakeMessage({{"odom", "base_link"}, {"base_link", "imu_link"}}));

  ASSERT_TRUE(WaitFor(500ms, [&] { return got; })) << "No filtered message on /tf";
  ASSERT_EQ(received.transforms.size(), 1u);
  EXPECT_EQ(received.transforms[0].child_frame_id, "robot1/base_link");
}

TEST_F(TfNamespaceBridgeFilteredTest, GlobMatchesWildcardAndAutoIncludesParent) {
  SetUpWithFilter("robot1", {"wheel*"});

  tf2_msgs::msg::TFMessage received;
  bool got = false;
  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", kTfQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        received = *msg;
        got = true;
      });
  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf", kTfQos);
  WaitFor(100ms, [] { return false; });
  // odom→base_link must be in the same message so the auto-included parent's
  // own transform appears in the output (otherwise base_link would be flagged
  // for auto-include but its inbound edge is not in this message).
  pub->publish(
      MakeMessage({{"odom", "base_link"}, {"base_link", "wheel_fl"}, {"base_link", "imu_link"}}));

  ASSERT_TRUE(WaitFor(500ms, [&] { return got; }));
  // wheel_fl matches, imu_link rejected, base_link auto-included.
  ASSERT_EQ(received.transforms.size(), 2u);
  EXPECT_EQ(received.transforms[0].child_frame_id, "robot1/base_link");
  EXPECT_EQ(received.transforms[1].child_frame_id, "robot1/wheel_fl");
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
