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

// Filter for topic names supplied by users on the command line.
//
// Pattern grammar (chosen to match the two common cases without surprise):
//   * empty               -> match every topic
//   * starts with '/'     -> prefix match on the topic name
//   * does not start '/'  -> substring match anywhere in the topic name
//
// Examples against a topic set like {/sensing/lidar/front/points,
// /perception/object}:
//   /sensing             -> prefix match, keeps /sensing/*
//   lidar/front          -> substring match
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
  enum class Mode { kAll, kPrefix, kSubstring };

  Mode mode_;
  bool match_all_;
  std::string pattern_;
};

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TOPIC_PATTERN_HPP_
