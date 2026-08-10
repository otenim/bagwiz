// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BASE__TOPIC_MATCH_HPP_
#define BAGWIZ__CORE__BASE__TOPIC_MATCH_HPP_

#include <span>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

// Glob matching for topic-name selectors. The only wildcard is '*', which
// matches any run of characters — including '/' and the empty string; every
// other character matches literally. A pattern without '*' is therefore an
// exact topic-name match. Used by `bagwiz topic drop` to expand the
// user-supplied selectors (e.g. "/sensing/*", "*/image_raw", "*") against a
// bag's topic list.
//
// Pure functions — no I/O. The caller owns reading the bag's topic list.
namespace bagwiz::core
{

// True when `pattern` matches `topic` under the '*'-only glob rules above.
[[nodiscard]] bool topic_glob_match(std::string_view pattern, std::string_view topic);

// Outcome of expanding a set of selector patterns against a topic list.
struct TopicPatternResolution
{
  // Topic names matched by at least one pattern (deduplicated across patterns).
  std::unordered_set<std::string> matched;
  // Patterns that matched no topic, preserved in input order (with duplicates).
  // Surfaced so the caller can fail fast on a typo'd selector instead of
  // silently rewriting a bag with nothing removed.
  std::vector<std::string> unmatched;
};

// Expand `patterns` against `topic_names`. A topic enters `matched` when any
// pattern matches it; a pattern enters `unmatched` when it matches no topic.
[[nodiscard]] TopicPatternResolution resolve_topic_patterns(
  std::span<const std::string> patterns, std::span<const std::string> topic_names);

// A topic name paired with its message type. resolve_topic_selectors() needs
// the type in order to honour a slot's accepted-type filter.
struct TopicEntry
{
  std::string name;
  std::string type;
};

// Outcome of resolving a slot's values against a bag's topic list.
struct SelectorResolution
{
  // The slot's resolved values, deduplicated, in resolution order: selectors in
  // argument order, and within one selector its matches sorted lexicographically
  // by topic name. The order is part of the contract because some slots are
  // order-sensitive (`pcd concat --pcd` concatenates in list order and takes the
  // first entry as its time-matching reference), and lexicographic is the order
  // `bagwiz ls` already prints.
  std::vector<std::string> matched;
  // Globs that matched no topic, in input order (with duplicates). A literal
  // never appears here: it is passed through for the command's own presence and
  // type checks to judge.
  std::vector<std::string> unmatched;
};

// Resolve `selectors` against `topics`.
//
// A selector without '*' is a literal and is copied to `matched` verbatim —
// unfiltered and unvalidated — so the command's existing "not present" and
// "wrong type" errors keep firing with their current wording. A selector
// containing '*' is matched against the entries whose type appears in
// `allowed_types` (an empty span accepts every type); matching nothing puts it
// in `unmatched`.
[[nodiscard]] SelectorResolution resolve_topic_selectors(
  std::span<const std::string> selectors, std::span<const TopicEntry> topics,
  std::span<const std::string_view> allowed_types);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BASE__TOPIC_MATCH_HPP_
