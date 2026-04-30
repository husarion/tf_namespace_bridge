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

#ifndef TF_NAMESPACE_BRIDGE__FRAME_FILTER_HPP_
#define TF_NAMESPACE_BRIDGE__FRAME_FILTER_HPP_

#include <regex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "tf2_msgs/msg/tf_message.hpp"

namespace tf_namespace_bridge {

// Whitelist filter for TF frames with auto-inclusion of missing parents.
//
// The filter matches glob patterns against child_frame_id of every transform.
// When a matched frame's parent does not match the filter, the parent is
// auto-included to keep the bridged TF tree connected; the caller is informed
// via ApplyResult::newly_auto_included so it can log a one-time INFO and
// emit a debounced summary warning.
//
// Empty pattern list => filter is inactive; Apply() returns input unchanged.
class FrameFilter {
 public:
  struct ApplyResult {
    tf2_msgs::msg::TFMessage out_msg;
    // Frames added to the auto-include set during this Apply() call.
    // Each frame appears at most once across the lifetime of FrameFilter
    // (between Reset/SetPatterns).
    std::vector<std::string> newly_auto_included;
  };

  // Replaces the active patterns. Returns false on invalid glob; in that case
  // existing patterns remain unchanged. Successful call clears the per-pattern
  // caches (match cache, auto-include set) but preserves the observed parent
  // graph.
  bool SetPatterns(const std::vector<std::string>& patterns);

  bool active() const { return !patterns_.empty(); }

  ApplyResult Apply(const tf2_msgs::msg::TFMessage& msg);

  // Sorted snapshot of frames currently auto-included (for summary warnings).
  std::vector<std::string> AutoIncludedFrames() const;

  // Hard reset: clears patterns, parent graph, caches, auto-include set.
  void Reset();

 private:
  bool Matches(const std::string& frame);
  void EnsureAutoInclude(const std::string& frame, std::vector<std::string>& newly_added);

  std::vector<std::regex> patterns_;
  std::unordered_map<std::string, bool> match_cache_;
  std::unordered_map<std::string, std::string> parent_of_;
  std::unordered_set<std::string> auto_include_;
};

}  // namespace tf_namespace_bridge

#endif  // TF_NAMESPACE_BRIDGE__FRAME_FILTER_HPP_
