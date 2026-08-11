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
#include <optional>
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

  // When true, every literal value this slot receives must name a topic in
  // the resolution universe; an absent one is an error in the same "selector
  // matched no topic" shape a non-matching glob produces. Applies in both
  // kGlob and kLiteral mode — a kGlob slot's glob-produced matches are
  // already guaranteed present by construction, so in practice this only
  // ever rejects a literal, whichever mode carried it. The check is
  // presence-only and ignores allowed_types, so a wrongly-typed literal still
  // reaches the command and still gets that command's type error.
  //
  // Best-effort, not absolute: it runs against the bag read at expansion
  // time, and expand_app() leaves every value untouched (skipping this check
  // along with everything else) when that read fails — see expand_app() in
  // topic_expand.cpp. In practice run() re-opens the same path with the same
  // io::open_read and fails too, but that is a property of the commands this
  // flag is set on, not a guarantee this flag itself makes.
  //
  // Set it ONLY on a slot whose command has no presence check of its own.
  // Most commands do (generate video --pcd reports "pcd topic '…' not found",
  // map slam --color reports "Topic '…' is not present in …"), and theirs are
  // more specific. The exceptions are topic drop -t, topic keep -t, and
  // trim --align, whose presence check WAS resolve_topic_patterns()'s
  // unmatched reporting — delete that without this flag and
  // `topic keep -i bag.mcap -t /typo` rewrites the bag in place with nothing
  // in it, exit 0, no warning.
  bool require_present{false};
};

// A registered slot. Exactly one of `multi_target` / `single_target` is set.
// All pointers here are non-owning: `option` is owned by the App it was
// added to, `multi_target`/`single_target` by whatever the caller passed to
// add_topic_option() — except the std::optional<std::string> overload, where
// `single_target` instead points at an internal proxy string owned by this
// same registration (see topic_option.cpp), since std::optional<std::string>
// has no stable std::string storage to point at until it is engaged — and
// `spec`'s span/string_view members by whatever the caller passed for
// `allowed_types`/`reject_reason`. Every one of them must outlive the App —
// true by construction for a Command's own configure(), which is the only
// place these are ever called.
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
// For a slot whose absence is meaningfully different from an empty value (the
// command auto-resolves the topic when the flag is omitted). Only
// TopicSelectorMode::kLiteral is supported — enforced with a thrown
// std::logic_error, not just documented, because a kGlob slot's expanded
// result gets written back through TopicSlot::single_target, which for this
// overload points at an internal proxy, not `target`; declaring one this way
// would silently strand the expansion there (the expansion pass's kLiteral
// branch, by contrast, only ever reads a slot's current value and never
// writes it back, so kLiteral needs no such sync).
//
// `target` holds a value whenever the flag was given a token at all —
// including an explicit empty string (`--cam-info ""`), which this overload
// treats as a real (empty) value rather than "absent". That differs from
// CLI::App::add_option's native std::optional<T> handling, whose
// lexical_assign specialization maps an empty input to nullopt instead; the
// new behavior is arguably more useful (an explicit "" stops being silently
// indistinguishable from omitting the flag), and no call site in this
// codebase relied on the old one. `target` is otherwise left untouched when
// the flag is omitted — except that a caller chaining ->default_val(...) or
// ->force_callback() onto the returned option would engage it even then (no
// call site does this today). ->capture_default_str() reads the internal
// proxy's value (always "" before parsing), not `target`'s.
CLI::Option * add_topic_option(
  CLI::App & app, std::string flags, std::optional<std::string> & target, std::string description,
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
