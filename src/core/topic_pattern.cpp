// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/topic_pattern.hpp"

#include <cstddef>
#include <string>
#include <string_view>

namespace bagwiz::core
{

namespace
{

bool has_glob_metacharacter(std::string_view pattern)
{
  return pattern.find_first_of("*?") != std::string_view::npos;
}

// Iterative glob matcher with a single backtracking point. Anchored at both
// ends. '*' matches any sequence (including '/'); '?' matches exactly one
// character; every other byte must match literally.
bool glob_match(std::string_view pat, std::string_view str)
{
  std::size_t p = 0;
  std::size_t s = 0;
  std::size_t star = std::string_view::npos;
  std::size_t resume = 0;
  while (s < str.size()) {
    if (p < pat.size() && pat[p] == '*') {
      star = p++;
      resume = s;
    } else if (p < pat.size() && (pat[p] == '?' || pat[p] == str[s])) {
      ++p;
      ++s;
    } else if (star != std::string_view::npos) {
      p = star + 1;
      s = ++resume;
    } else {
      return false;
    }
  }
  while (p < pat.size() && pat[p] == '*') {
    ++p;
  }
  return p == pat.size();
}

}  // namespace

TopicPattern::TopicPattern(std::string_view pattern)
: mode_(
    pattern.empty() ? Mode::kAll
                    : (has_glob_metacharacter(pattern) ? Mode::kGlob : Mode::kSubstring)),
  match_all_(mode_ == Mode::kAll),
  pattern_(pattern)
{
}

bool TopicPattern::matches(const std::string & topic) const
{
  switch (mode_) {
    case Mode::kAll:
      return true;
    case Mode::kSubstring:
      return topic.find(pattern_) != std::string::npos;
    case Mode::kGlob:
      return glob_match(pattern_, topic);
  }
  return false;
}

}  // namespace bagwiz::core
