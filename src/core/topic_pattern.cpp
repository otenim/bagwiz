// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/topic_pattern.hpp"

#include <string>
#include <string_view>

namespace bagwiz::core
{

TopicPattern::TopicPattern(std::string_view pattern)
: mode_(pattern.empty() ? Mode::kAll : (pattern.front() == '/' ? Mode::kPrefix : Mode::kSubstring)),
  match_all_(mode_ == Mode::kAll),
  pattern_(pattern)
{
}

bool TopicPattern::matches(const std::string & topic) const
{
  switch (mode_) {
    case Mode::kAll:
      return true;
    case Mode::kPrefix:
      return topic.size() >= pattern_.size() && topic.compare(0, pattern_.size(), pattern_) == 0;
    case Mode::kSubstring:
      return topic.find(pattern_) != std::string::npos;
  }
  return false;
}

}  // namespace bagwiz::core
