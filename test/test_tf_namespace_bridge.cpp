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

#include <chrono>
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
  const std::vector<std::pair<std::string, std::string>> & transforms) {
  tf2_msgs::msg::TFMessage msg;
  for (const auto & [parent, child] : transforms) {
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
  void SetUpWithNamespace(const std::string & ns) {
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
  for (const auto & t : received.transforms) {
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

TEST_F(TfNamespaceBridgeTest, EmptyMessageDoesNotCrash) {
  SetUpWithNamespace("robot1");
  int count = 0;

  auto sub = test_node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", kTfQos, [&](const tf2_msgs::msg::TFMessage::SharedPtr) { ++count; });
  auto pub = test_node_->create_publisher<tf2_msgs::msg::TFMessage>("/robot1/tf", kTfQos);

  WaitFor(100ms, [] { return false; });
  pub->publish(tf2_msgs::msg::TFMessage{});

  WaitFor(300ms, [&] { return count > 0; });
  EXPECT_EQ(count, 1);
}

TEST_F(TfNamespaceBridgeTest, RootNamespaceThrowsToPreventFeedbackLoop) {
  // Running without a namespace would subscribe and publish to /tf simultaneously,
  // creating an infinite feedback loop. The node must refuse to start.
  EXPECT_THROW(SetUpWithNamespace(""), std::invalid_argument);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
