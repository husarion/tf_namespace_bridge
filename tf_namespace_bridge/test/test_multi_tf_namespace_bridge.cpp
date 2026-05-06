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

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf_namespace_bridge/multi_tf_namespace_bridge.hpp"

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

class MultiTfNamespaceBridgeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("namespaces", std::vector<std::string>{"robot1"})});
    bridge_ = std::make_shared<tf_namespace_bridge::MultiTfNamespaceBridge>(opts);
    test_node_ = rclcpp::Node::make_shared("test_node");

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

  std::shared_ptr<tf_namespace_bridge::MultiTfNamespaceBridge> bridge_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

TEST_F(MultiTfNamespaceBridgeTest, PrefixesHeaderFrameIdAndChildFrameId) {
  tf2_msgs::msg::TFMessage received;
  bool got = false;

  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", kTfQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        received = *msg;
        got = true;
      });
  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf", kTfQos);

  WaitFor(100ms, [] { return false; });  // let discovery settle
  pub->publish(MakeMessage({{"base_link", "imu_link"}}));

  ASSERT_TRUE(WaitFor(500ms, [&] { return got; })) << "No message received on /tf";
  ASSERT_EQ(received.transforms.size(), 1u);
  EXPECT_EQ(received.transforms[0].header.frame_id, "robot1/base_link");
  EXPECT_EQ(received.transforms[0].child_frame_id, "robot1/imu_link");
}

TEST_F(MultiTfNamespaceBridgeTest, PrefixesAllTransformsInMessage) {
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

TEST_F(MultiTfNamespaceBridgeTest, PrefixesStaticTfFrames) {
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

TEST_F(MultiTfNamespaceBridgeTest, EmptyMessageIsNotRepublished) {
  // Bridge skips publishing when the post-filter message has no transforms,
  // including the trivial case of an empty input. The contract here is twofold:
  // the bridge does not crash on empty input, and it does not waste DDS
  // bandwidth forwarding empty TFMessage frames.
  int count = 0;

  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", kTfQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr) { ++count; });
  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf", kTfQos);

  WaitFor(100ms, [] { return false; });
  pub->publish(tf2_msgs::msg::TFMessage{});

  WaitFor(300ms, [&] { return count > 0; });
  EXPECT_EQ(count, 0);
}

TEST_F(MultiTfNamespaceBridgeTest, RuntimeAddNamespaceBridgesNewRobot) {
  tf2_msgs::msg::TFMessage received;
  bool got = false;

  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", kTfQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        received = *msg;
        got = true;
      });

  bridge_->set_parameter(
      rclcpp::Parameter("namespaces", std::vector<std::string>{"robot1", "robot2"}));

  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot2/tf", kTfQos);
  // ParamListener is polled every 200ms; wait for the bridge subscription to come up
  // before publishing — best_effort QoS does not buffer messages for late subscribers.
  ASSERT_TRUE(WaitFor(1000ms, [&] { return pub->get_subscription_count() > 0; }))
      << "Bridge did not subscribe after adding robot2";
  pub->publish(MakeMessage({{"base_link", "imu_link"}}));

  ASSERT_TRUE(WaitFor(500ms, [&] { return got; })) << "No message after adding robot2";
  EXPECT_EQ(received.transforms[0].header.frame_id, "robot2/base_link");
}

TEST_F(MultiTfNamespaceBridgeTest, RuntimeRemoveNamespaceDestroysSubscription) {
  // Verify subscription teardown via publisher's subscriber count — more reliable than
  // checking message delivery because DDS connection teardown is asynchronous.
  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf", kTfQos);

  ASSERT_TRUE(WaitFor(500ms, [&] { return pub->get_subscription_count() >= 1; }))
      << "Bridge subscription not established";

  bridge_->set_parameter(rclcpp::Parameter("namespaces", std::vector<std::string>{}));

  // ParamListener is polled every 200ms; allow extra margin on top of DDS teardown.
  EXPECT_TRUE(WaitFor(1500ms, [&] { return pub->get_subscription_count() == 0; }))
      << "Bridge subscription not removed after namespace was cleared";
}

// --- YAML params-file scenarios for empty / sentinel filters ---
//
// Mirror of the single-bridge YAML tests. See test_tf_namespace_bridge.cpp
// for the rationale.

namespace {

std::string WriteParamsFile(const std::string& contents) {
  char tmpl[] = "/tmp/multi_tf_namespace_bridge_test_XXXXXX.yaml";
  int fd = mkstemps(tmpl, 5);
  if (fd >= 0) {
    [[maybe_unused]] auto written = ::write(fd, contents.data(), contents.size());
    ::close(fd);
  }
  return tmpl;
}

rclcpp::NodeOptions OptionsForYaml(const std::string& yaml_path) {
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "--params-file", yaml_path});
  return opts;
}

}  // namespace

TEST(MultiTfNamespaceBridgeYamlConfig, EmptyArrayInYamlIsRejectedByRclcpp) {
  const auto path = WriteParamsFile(
      "/**:\n  multi_tf_namespace_bridge:\n    ros__parameters:\n      frame_filters: []\n");
  EXPECT_THROW(std::make_shared<tf_namespace_bridge::MultiTfNamespaceBridge>(OptionsForYaml(path)),
               rclcpp::exceptions::InvalidParameterValueException);
  std::remove(path.c_str());
}

TEST(MultiTfNamespaceBridgeYamlConfig, EmptyStringSentinelInYamlIsAcceptedAsPassThrough) {
  const auto path = WriteParamsFile(
      "/**:\n  multi_tf_namespace_bridge:\n    ros__parameters:\n      frame_filters: [\"\"]\n");
  EXPECT_NO_THROW({
    auto bridge =
        std::make_shared<tf_namespace_bridge::MultiTfNamespaceBridge>(OptionsForYaml(path));
  });
  std::remove(path.c_str());
}

TEST(MultiTfNamespaceBridgeYamlConfig, StarPatternInYamlIsAcceptedAsPassThrough) {
  const auto path = WriteParamsFile(
      "/**:\n  multi_tf_namespace_bridge:\n    ros__parameters:\n      frame_filters: [\"*\"]\n");
  EXPECT_NO_THROW({
    auto bridge =
        std::make_shared<tf_namespace_bridge::MultiTfNamespaceBridge>(OptionsForYaml(path));
  });
  std::remove(path.c_str());
}

// --- Frame filter integration ---

class MultiTfNamespaceBridgeFilteredTest : public ::testing::Test {
 protected:
  void SetUpWithFilter(const std::vector<std::string>& filters) {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("namespaces", std::vector<std::string>{"robot1"}),
                              rclcpp::Parameter("frame_filters", filters)});
    bridge_ = std::make_shared<tf_namespace_bridge::MultiTfNamespaceBridge>(opts);
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

  std::shared_ptr<tf_namespace_bridge::MultiTfNamespaceBridge> bridge_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

TEST_F(MultiTfNamespaceBridgeFilteredTest, FilterBlocksNonMatchingChild) {
  SetUpWithFilter({"base_link"});

  tf2_msgs::msg::TFMessage received;
  bool got = false;
  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", kTfQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        received = *msg;
        got = true;
      });
  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf", kTfQos);
  WaitFor(100ms, [] { return false; });
  // odom→base_link (matches) and base_link→imu_link (does not match): only the
  // first transform is bridged. base_link's parent (odom) is auto-included so
  // the global tree stays connected.
  pub->publish(MakeMessage({{"odom", "base_link"}, {"base_link", "imu_link"}}));

  ASSERT_TRUE(WaitFor(500ms, [&] { return got; })) << "No filtered message on /tf";
  ASSERT_EQ(received.transforms.size(), 1u);
  EXPECT_EQ(received.transforms[0].header.frame_id, "robot1/odom");
  EXPECT_EQ(received.transforms[0].child_frame_id, "robot1/base_link");
}

TEST_F(MultiTfNamespaceBridgeFilteredTest, GlobMatchesWildcardAndAutoIncludesParent) {
  SetUpWithFilter({"wheel*"});

  tf2_msgs::msg::TFMessage received;
  bool got = false;
  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", kTfQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        received = *msg;
        got = true;
      });
  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf", kTfQos);
  WaitFor(100ms, [] { return false; });
  // Both wheels match wheel*; base_link is auto-included as their parent.
  pub->publish(
      MakeMessage({{"odom", "base_link"}, {"base_link", "wheel_fl"}, {"base_link", "wheel_fr"}}));

  ASSERT_TRUE(WaitFor(500ms, [&] { return got; })) << "No filtered message on /tf";
  ASSERT_EQ(received.transforms.size(), 3u);
  EXPECT_EQ(received.transforms[0].child_frame_id, "robot1/base_link");
  EXPECT_EQ(received.transforms[1].child_frame_id, "robot1/wheel_fl");
  EXPECT_EQ(received.transforms[2].child_frame_id, "robot1/wheel_fr");
}

TEST_F(MultiTfNamespaceBridgeFilteredTest, FilterAppliesToTfStatic) {
  SetUpWithFilter({"wheel*"});

  tf2_msgs::msg::TFMessage received;
  bool got = false;

  auto pub =
      test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf_static", kTfStaticQos);
  WaitFor(100ms, [] { return false; });
  // Realistic /tf_static contents: base_link is the root and has no inbound
  // edge in this topic (it shows up on /tf via odom→base_link). The filter
  // drops cover_link and emits wheel_fl; base_link is added to the auto-include
  // set internally and would be republished when its inbound edge arrives.
  pub->publish(MakeMessage({{"base_link", "cover_link"}, {"base_link", "wheel_fl"}}));

  // Subscribe after publish to verify transient_local is preserved end-to-end.
  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", kTfStaticQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        received = *msg;
        got = true;
      });

  ASSERT_TRUE(WaitFor(500ms, [&] { return got; })) << "No filtered message on /tf_static";
  ASSERT_EQ(received.transforms.size(), 1u);
  EXPECT_EQ(received.transforms[0].child_frame_id, "robot1/wheel_fl");
}

TEST_F(MultiTfNamespaceBridgeFilteredTest, RuntimeFilterUpdate) {
  SetUpWithFilter({"base_link"});

  tf2_msgs::msg::TFMessage received;
  bool got = false;
  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", kTfQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        received = *msg;
        got = true;
      });
  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf", kTfQos);
  WaitFor(100ms, [] { return false; });

  // Switch the filter to wheel* at runtime.
  bridge_->set_parameter(rclcpp::Parameter("frame_filters", std::vector<std::string>{"wheel*"}));

  // Wait long enough for the 200ms parameter poll.
  WaitFor(500ms, [] { return false; });

  // Include odom→base_link in the same message so the auto-included parent's
  // inbound transform exists in the published output.
  pub->publish(
      MakeMessage({{"odom", "base_link"}, {"base_link", "wheel_fl"}, {"base_link", "imu_link"}}));
  ASSERT_TRUE(WaitFor(500ms, [&] { return got; })) << "No message after filter update";
  // wheel_fl now passes; imu_link still rejected. base_link auto-included.
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
