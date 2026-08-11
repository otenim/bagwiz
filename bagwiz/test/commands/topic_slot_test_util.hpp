// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__TOPIC_SLOT_TEST_UTIL_HPP_
#define COMMANDS__TOPIC_SLOT_TEST_UTIL_HPP_

#include "bagwiz/commands/topic_option.hpp"

#include <span>
#include <string>

namespace bagwiz::test
{

// The slot in `slots` whose option carries long name `long_name` (e.g. "topic"
// for "-t,--topic"), or nullptr when none matches. A wiring test should look a
// slot up this way rather than by index: an index shifts silently whenever a
// new slot is declared earlier in the same subcommand — PcdConcatCliWiring
// broke exactly this way when Task 8 added -t/--topic ahead of --pcd. Reserve
// an index-based `slots[N]` assertion for a test whose point IS declaration
// order (a `scope` must name an earlier slot — see topic_expand.cpp's
// resolve_context()), where the order is itself part of the contract under
// test, not incidental.
[[nodiscard]] inline const bagwiz::commands::TopicSlot * slot_for(
  std::span<const bagwiz::commands::TopicSlot> slots, const std::string & long_name)
{
  for (const auto & slot : slots) {
    for (const auto & name : slot.option->get_lnames()) {
      if (name == long_name) {
        return &slot;
      }
    }
  }
  return nullptr;
}

}  // namespace bagwiz::test

#endif  // COMMANDS__TOPIC_SLOT_TEST_UTIL_HPP_
