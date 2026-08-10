// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TOPIC_OPTION_HPP_
#define BAGWIZ__COMMANDS__TOPIC_OPTION_HPP_

#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace CLI
{
class App;
class Option;
}  // namespace CLI

// Declaration of topic-valued CLI options.
//
// Every option whose value names a topic is declared through add_topic_option()
// rather than CLI::App::add_option(), so the CLI knows which arguments are
// topic slots and what each one accepts. Two consumers read the result: the
// expansion pass in topic_expand.hpp, which turns '*' selectors into topic
// names before the command runs, and shell completion, which offers only
// topics a slot accepts. Declaring the slot is what earns a command both;
// there is no separate table to keep in sync.
namespace bagwiz::commands
{

enum class TopicSelectorMode {
  // The value may be a '*' glob, expanded against the bag before the command
  // runs. Only multi-value slots use this: expanding into a slot that holds one
  // topic would make the outcome depend on what the bag happens to contain.
  kGlob,
  // The value must be a literal topic name. A value containing '*' is rejected
  // with an error naming the flag and the reason.
  kLiteral,
};

struct TopicSlotSpec
{
  // Message types this slot accepts; empty accepts every type. Filters glob
  // expansion and completion candidates. Values come from topic_types.hpp.
  std::span<const std::string_view> allowed_types{};

  TopicSelectorMode mode{TopicSelectorMode::kGlob};

  // The value is `<topic>=<rhs>`. For a kGlob slot only the half before the
  // first '=' is a selector — the right half is a file path or a scalar, never
  // a topic. For a kLiteral slot the whole value is checked for '*'.
  bool pair_value{false};

  // When set, selectors resolve against this option's expanded result instead
  // of the bag's topic list. Used where a value must name one of another
  // flag's topics rather than any topic in the bag.
  const CLI::Option * scope{};

  // Replaces the default kLiteral rejection message where the reason is not
  // self-evident from the flag alone.
  std::string_view reject_reason{};
};

// A registered slot. Exactly one of `multi_target` / `single_target` is set.
struct TopicSlot
{
  const CLI::Option * option{};
  std::vector<std::string> * multi_target{};
  std::string * single_target{};
  TopicSlotSpec spec;
};

// Declare a topic-valued option and register it. Returns the CLI::Option so the
// caller can keep chaining ->required(), ->excludes(), ->check(), and so on.
CLI::Option * add_topic_option(
  CLI::App & app, std::string flags, std::string & target, std::string description,
  const TopicSlotSpec & spec);
CLI::Option * add_topic_option(
  CLI::App & app, std::string flags, std::vector<std::string> & target, std::string description,
  const TopicSlotSpec & spec);

// Name the option holding the bag this (sub)command's topic slots resolve
// against. Call once per subcommand that declares topic slots; without it the
// expansion pass has no bag to expand against and leaves the values verbatim.
void set_topic_input(CLI::App & app, std::filesystem::path & input_target);

// Slots registered on `app`, in declaration order. Empty when `app` declares
// none. Declaration order is the order the expansion pass processes them, which
// is what lets a slot's `scope` refer to an earlier slot's expanded result.
[[nodiscard]] std::span<const TopicSlot> topic_slots_of(const CLI::App & app);

// The input path registered by set_topic_input(), or nullptr when `app`
// registered none.
[[nodiscard]] const std::filesystem::path * topic_input_of(const CLI::App & app);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TOPIC_OPTION_HPP_
