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

#include "tf_namespace_bridge/frame_filter.hpp"

#include <algorithm>
#include <optional>
#include <regex>
#include <string>
#include <utility>
#include <vector>

namespace tf_namespace_bridge {

namespace {

// Bound on parent walks; protects against pathological cycles in the observed
// graph (TF should never have cycles, but a buggy publisher could produce one).
constexpr int kMaxParentWalkDepth = 64;

std::optional<std::regex> GlobToRegex(const std::string& glob) {
  if (glob.empty()) return std::nullopt;

  std::string regex_str;
  regex_str.reserve(glob.size() * 2 + 2);
  regex_str.push_back('^');
  for (const char c : glob) {
    switch (c) {
      case '*':
        regex_str += ".*";
        break;
      case '?':
        regex_str += '.';
        break;
      case '.':
      case '+':
      case '(':
      case ')':
      case '[':
      case ']':
      case '{':
      case '}':
      case '^':
      case '$':
      case '\\':
      case '|':
        regex_str.push_back('\\');
        regex_str.push_back(c);
        break;
      default:
        regex_str.push_back(c);
    }
  }
  regex_str.push_back('$');

  try {
    return std::regex(regex_str);
  } catch (const std::regex_error&) {
    return std::nullopt;
  }
}

}  // namespace

bool FrameFilter::SetPatterns(const std::vector<std::string>& patterns) {
  std::vector<std::regex> compiled;
  compiled.reserve(patterns.size());
  for (const auto& p : patterns) {
    auto re = GlobToRegex(p);
    if (!re) return false;
    compiled.push_back(std::move(*re));
  }
  patterns_ = std::move(compiled);
  // Pattern-dependent caches must be invalidated; parent graph is pattern-agnostic.
  match_cache_.clear();
  auto_include_.clear();
  return true;
}

FrameFilter::ApplyResult FrameFilter::Apply(const tf2_msgs::msg::TFMessage& msg) {
  ApplyResult result;
  if (!active()) {
    result.out_msg = msg;
    return result;
  }

  // Phase 1: refresh the observed parent graph from this message.
  for (const auto& t : msg.transforms) {
    parent_of_[t.child_frame_id] = t.header.frame_id;
  }

  // Phase 2: extend auto_include_ by walking up from every transform whose
  // child either matches the filter or was already auto-included. Done before
  // emission so that a single message containing both an ancestor edge and a
  // matched descendant edge correctly carries both through.
  for (const auto& t : msg.transforms) {
    const bool matched = Matches(t.child_frame_id);
    const bool auto_inc = auto_include_.count(t.child_frame_id) > 0;
    if (matched || auto_inc) {
      EnsureAutoInclude(t.header.frame_id, result.newly_auto_included);
    }
  }

  // Phase 3: emit transforms whose child is matched or auto-included.
  result.out_msg.transforms.reserve(msg.transforms.size());
  for (const auto& t : msg.transforms) {
    if (Matches(t.child_frame_id) || auto_include_.count(t.child_frame_id) > 0) {
      result.out_msg.transforms.push_back(t);
    }
  }
  return result;
}

std::vector<std::string> FrameFilter::AutoIncludedFrames() const {
  std::vector<std::string> out(auto_include_.begin(), auto_include_.end());
  std::sort(out.begin(), out.end());
  return out;
}

void FrameFilter::Reset() {
  patterns_.clear();
  match_cache_.clear();
  parent_of_.clear();
  auto_include_.clear();
}

bool FrameFilter::Matches(const std::string& frame) {
  auto it = match_cache_.find(frame);
  if (it != match_cache_.end()) return it->second;

  bool result = false;
  for (const auto& p : patterns_) {
    if (std::regex_match(frame, p)) {
      result = true;
      break;
    }
  }
  match_cache_.emplace(frame, result);
  return result;
}

void FrameFilter::EnsureAutoInclude(const std::string& frame,
                                    std::vector<std::string>& newly_added) {
  std::string current = frame;
  for (int depth = 0; depth < kMaxParentWalkDepth; ++depth) {
    if (Matches(current)) return;
    if (auto_include_.insert(current).second) {
      newly_added.push_back(current);
    }
    auto it = parent_of_.find(current);
    if (it == parent_of_.end()) return;
    if (it->second == current) return;  // self-loop guard
    current = it->second;
  }
}

}  // namespace tf_namespace_bridge
