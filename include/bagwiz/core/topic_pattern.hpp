// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TOPIC_PATTERN_HPP_
#define BAGWIZ__CORE__TOPIC_PATTERN_HPP_

#include <string>
#include <string_view>

namespace bagwiz::core
{

// Filter for topic / type names supplied by users on the command line.
//
// Pattern grammar (designed to be intuitive for the common cases):
//   * empty                  -> match every input
//   * contains '*' or '?'    -> shell-style glob, anchored at both ends
//                                 '*' matches any sequence (including '/')
//                                 '?' matches exactly one character
//   * otherwise              -> case-sensitive substring match
//
// Examples against {/sensing/lidar/front/points, /perception/object, /tf}:
//   sensing            -> substring, keeps /sensing/lidar/front/points
//   /sensing           -> substring, keeps /sensing/lidar/front/points
//   /sensing/*         -> glob,      keeps /sensing/lidar/front/points
//   */points           -> glob,      keeps /sensing/lidar/front/points
//   /sensing/*/points  -> glob,      keeps /sensing/lidar/front/points
class TopicPattern
{
public:
  // Construct a matcher from a user-supplied pattern. An empty pattern is
  // treated as "match everything".
  explicit TopicPattern(std::string_view pattern);

  // True iff this matcher was built from an empty pattern, in which case
  // matches() returns true for every input.
  bool match_all() const { return match_all_; }

  // True iff `topic` is kept by this filter.
  bool matches(const std::string & topic) const;

private:
  enum class Mode { kAll, kSubstring, kGlob };

  Mode mode_;
  bool match_all_;
  std::string pattern_;
};

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TOPIC_PATTERN_HPP_
