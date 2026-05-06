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

#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf_namespace_bridge/frame_filter.hpp"

namespace tf_namespace_bridge {

namespace {

tf2_msgs::msg::TFMessage MakeMessage(
    std::initializer_list<std::pair<std::string, std::string>> edges) {
  tf2_msgs::msg::TFMessage msg;
  for (const auto& [parent, child] : edges) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    msg.transforms.push_back(t);
  }
  return msg;
}

// Returns the list of child_frame_ids in the message, for compact assertions.
std::vector<std::string> Children(const tf2_msgs::msg::TFMessage& msg) {
  std::vector<std::string> out;
  out.reserve(msg.transforms.size());
  for (const auto& t : msg.transforms) out.push_back(t.child_frame_id);
  return out;
}

}  // namespace

TEST(FrameFilterTest, EmptyPatternsAreInactive) {
  FrameFilter f;
  EXPECT_FALSE(f.active());
}

TEST(FrameFilterTest, SetPatternsSkipsEmptyStringsAlongsidePatterns) {
  // Empty entries are silently skipped so callers can use [""] as a "no filter"
  // sentinel without surfacing as an invalid-glob error.
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"", "wheel*"}));
  EXPECT_TRUE(f.active());
  auto result = f.Apply(MakeMessage({{"base_link", "wheel_fl"}, {"base_link", "imu_link"}}));
  EXPECT_EQ(Children(result.out_msg), (std::vector<std::string>{"wheel_fl"}));
}

TEST(FrameFilterTest, SetPatternsAllEmptyStringsBecomesInactive) {
  // [""] / ["", "", ""] collapse to no patterns -> inactive (pure pass-through).
  // This is the launch-friendly sentinel: launch YAML cannot pass an empty
  // string_array override, but [""] survives type-tagging and means the same.
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({""}));
  EXPECT_FALSE(f.active());
  auto result = f.Apply(MakeMessage({{"odom", "base_link"}, {"base_link", "imu_link"}}));
  EXPECT_EQ(Children(result.out_msg), (std::vector<std::string>{"base_link", "imu_link"}));
  EXPECT_TRUE(result.newly_auto_included.empty());
}

TEST(FrameFilterTest, InactiveFilterPassesEverythingUnchanged) {
  FrameFilter f;
  auto msg = MakeMessage({{"odom", "base_link"}, {"base_link", "imu_link"}});
  auto result = f.Apply(msg);
  EXPECT_EQ(Children(result.out_msg), (std::vector<std::string>{"base_link", "imu_link"}));
  EXPECT_TRUE(result.newly_auto_included.empty());
}

TEST(FrameFilterTest, MatchesExactName) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"base_link"}));
  auto result = f.Apply(MakeMessage({{"odom", "base_link"}, {"base_link", "imu_link"}}));
  EXPECT_EQ(Children(result.out_msg), (std::vector<std::string>{"base_link"}));
}

TEST(FrameFilterTest, GlobStarMatchesAnyTail) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"wheel*"}));
  auto result =
      f.Apply(MakeMessage({{"base_link", "wheel"},
                           {"base_link", "wheel_left"},
                           {"base_link", "wheel_right_front"},
                           {"base_link", "cwheel"},  // must NOT match (no anchor at start)
                           {"base_link", "imu_link"}}));
  EXPECT_EQ(Children(result.out_msg),
            (std::vector<std::string>{"wheel", "wheel_left", "wheel_right_front"}));
}

TEST(FrameFilterTest, GlobQuestionMarkMatchesSingleChar) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"link?"}));
  auto result =
      f.Apply(MakeMessage({{"x", "link1"}, {"x", "link2"}, {"x", "linkAB"}, {"x", "link"}}));
  EXPECT_EQ(Children(result.out_msg), (std::vector<std::string>{"link1", "link2"}));
}

TEST(FrameFilterTest, GlobEscapesRegexMetacharacters) {
  FrameFilter f;
  // The dot must be matched literally, not as "any char".
  ASSERT_TRUE(f.SetPatterns({"base.link"}));
  auto result = f.Apply(MakeMessage({{"x", "base.link"}, {"x", "baseXlink"}}));
  EXPECT_EQ(Children(result.out_msg), (std::vector<std::string>{"base.link"}));
}

TEST(FrameFilterTest, MultiplePatternsMatchUnion) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"odom", "base_link", "wheel*"}));
  auto result = f.Apply(MakeMessage({{"map", "odom"},
                                     {"odom", "base_link"},
                                     {"base_link", "wheel_fl"},
                                     {"base_link", "imu_link"}}));
  EXPECT_EQ(Children(result.out_msg), (std::vector<std::string>{"odom", "base_link", "wheel_fl"}));
}

TEST(FrameFilterTest, AutoIncludesMissingParent) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"wheel*"}));
  auto result = f.Apply(MakeMessage({{"base_link", "wheel_fl"}}));
  // wheel_fl matched directly; base_link auto-included.
  EXPECT_EQ(Children(result.out_msg), (std::vector<std::string>{"wheel_fl"}));
  EXPECT_EQ(result.newly_auto_included, (std::vector<std::string>{"base_link"}));
}

TEST(FrameFilterTest, AutoIncludeChainsThroughGrandparents) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"wheel*"}));
  // Single message contains the full chain odom → base_link → wheel_fl.
  auto result = f.Apply(MakeMessage({{"odom", "base_link"}, {"base_link", "wheel_fl"}}));
  // The chain walk auto-includes both base_link and odom (neither matches the filter).
  EXPECT_EQ(Children(result.out_msg), (std::vector<std::string>{"base_link", "wheel_fl"}));
  EXPECT_EQ(result.newly_auto_included, (std::vector<std::string>{"base_link", "odom"}));
}

TEST(FrameFilterTest, AutoIncludeReportsEachFrameOnce) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"wheel*"}));
  auto first = f.Apply(MakeMessage({{"base_link", "wheel_fl"}}));
  ASSERT_EQ(first.newly_auto_included, (std::vector<std::string>{"base_link"}));
  auto second = f.Apply(MakeMessage({{"base_link", "wheel_fr"}}));
  // base_link already auto-included; must not appear again.
  EXPECT_TRUE(second.newly_auto_included.empty());
  // wheel_fr passes (matches), base_link still gets re-published this round.
  EXPECT_EQ(Children(second.out_msg), (std::vector<std::string>{"wheel_fr"}));
}

TEST(FrameFilterTest, AutoIncludeRetroactivelyEmitsParentEdgeWhenItArrives) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"wheel*"}));

  // First message: only the wheel edge — base_link gets auto-included but its
  // own transform (odom → base_link) hasn't been seen yet.
  auto first = f.Apply(MakeMessage({{"base_link", "wheel_fl"}}));
  EXPECT_EQ(Children(first.out_msg), (std::vector<std::string>{"wheel_fl"}));
  ASSERT_EQ(first.newly_auto_included, (std::vector<std::string>{"base_link"}));

  // Second message: odom → base_link arrives. base_link's child_frame_id is
  // already in auto_include_, so this edge must now be passed through. Then
  // odom itself is also auto-included as the new parent.
  auto second = f.Apply(MakeMessage({{"odom", "base_link"}}));
  EXPECT_EQ(Children(second.out_msg), (std::vector<std::string>{"base_link"}));
  EXPECT_EQ(second.newly_auto_included, (std::vector<std::string>{"odom"}));
}

TEST(FrameFilterTest, MatchedFrameWithMatchedParentDoesNotAutoInclude) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"odom", "base_link"}));
  auto result = f.Apply(MakeMessage({{"odom", "base_link"}}));
  EXPECT_EQ(Children(result.out_msg), (std::vector<std::string>{"base_link"}));
  // odom is in the filter — must not be auto-included.
  EXPECT_TRUE(result.newly_auto_included.empty());
}

TEST(FrameFilterTest, AutoIncludedFramesIsSorted) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"wheel*"}));
  f.Apply(MakeMessage({{"odom", "base_link"}, {"base_link", "wheel_fl"}}));
  EXPECT_EQ(f.AutoIncludedFrames(), (std::vector<std::string>{"base_link", "odom"}));
}

TEST(FrameFilterTest, SetPatternsClearsAutoIncludeButPreservesParentGraph) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"wheel*"}));
  // Build up parent_of via two messages.
  f.Apply(MakeMessage({{"base_link", "wheel_fl"}}));
  f.Apply(MakeMessage({{"odom", "base_link"}}));
  ASSERT_EQ(f.AutoIncludedFrames(), (std::vector<std::string>{"base_link", "odom"}));

  // New filter — auto_include must reset.
  ASSERT_TRUE(f.SetPatterns({"base_link"}));
  EXPECT_TRUE(f.AutoIncludedFrames().empty());

  // The parent graph is preserved, so a new wheel message under the new filter
  // (which doesn't match wheel*) won't pass — but any matched frame walks the
  // existing graph immediately.
  auto result = f.Apply(MakeMessage({{"odom", "base_link"}}));
  EXPECT_EQ(Children(result.out_msg), (std::vector<std::string>{"base_link"}));
  // base_link's parent (odom) was already in parent_of_ and walks up cleanly.
  EXPECT_EQ(result.newly_auto_included, (std::vector<std::string>{"odom"}));
}

TEST(FrameFilterTest, ResetClearsEverything) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"wheel*"}));
  f.Apply(MakeMessage({{"base_link", "wheel_fl"}}));
  f.Reset();
  EXPECT_FALSE(f.active());
  EXPECT_TRUE(f.AutoIncludedFrames().empty());
  // After Reset, Apply with a fresh message just passes through (filter inactive).
  auto result = f.Apply(MakeMessage({{"x", "y"}}));
  EXPECT_EQ(Children(result.out_msg), (std::vector<std::string>{"y"}));
}

TEST(FrameFilterTest, EmptyMessageProducesEmptyOutput) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"wheel*"}));
  auto result = f.Apply(tf2_msgs::msg::TFMessage{});
  EXPECT_TRUE(result.out_msg.transforms.empty());
  EXPECT_TRUE(result.newly_auto_included.empty());
}

TEST(FrameFilterTest, ParentSelfLoopDoesNotInfiniteWalk) {
  FrameFilter f;
  ASSERT_TRUE(f.SetPatterns({"x"}));
  // Pathological: y is its own parent. Walk-up must terminate.
  auto result = f.Apply(MakeMessage({{"y", "x"}, {"y", "y"}}));
  // The contract of this test is that Apply terminates and emits the matched
  // frame; the self-loop edge itself is malformed and the bridge does not
  // suppress it (downstream tf2 rejects self-loops anyway).
  bool has_x = false;
  for (const auto& t : result.out_msg.transforms) {
    if (t.child_frame_id == "x") has_x = true;
  }
  EXPECT_TRUE(has_x);
  // y is auto-included exactly once; the self-loop guard prevents re-emitting
  // it on each parent-walk.
  EXPECT_EQ(result.newly_auto_included, (std::vector<std::string>{"y"}));
}

}  // namespace tf_namespace_bridge
