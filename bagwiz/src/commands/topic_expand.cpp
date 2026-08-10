// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/topic_expand.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/topic_match.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <cstddef>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.topic";

// The flag as the user typed it, e.g. "-t/--topics", for error messages.
std::string flag_label(const CLI::Option & option)
{
  std::string label;
  for (const auto & name : option.get_snames()) {
    label += label.empty() ? "-" : "/-";
    label += name;
  }
  for (const auto & name : option.get_lnames()) {
    label += label.empty() ? "--" : "/--";
    label += name;
  }
  return label;
}

// The command path, e.g. "pcd undistort", for error messages.
std::string command_label(const CLI::App & app)
{
  std::vector<std::string> parts;
  for (const CLI::App * cur = &app; cur != nullptr; cur = cur->get_parent()) {
    if (!cur->get_name().empty() && cur->get_parent() != nullptr) {
      parts.insert(parts.begin(), cur->get_name());
    }
  }
  std::string label;
  for (const auto & part : parts) {
    if (!label.empty()) {
      label += ' ';
    }
    label += part;
  }
  return label.empty() ? "bagwiz" : label;
}

// The bag's topics as name/type pairs, or nullopt when the path is not a
// readable bag (a calibration YAML, a missing file, an unsupported format).
std::optional<std::vector<core::TopicEntry>> read_topics(const std::filesystem::path & path)
{
  try {
    auto reader = io::open_read(path);
    if (!reader) {
      return std::nullopt;
    }
    std::vector<core::TopicEntry> entries;
    for (const auto & topic : reader->topics()) {
      entries.push_back(core::TopicEntry{topic.name, topic.type});
    }
    return entries;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool contains_glob(std::string_view value)
{
  return value.find('*') != std::string_view::npos;
}

// Split `<topic>=<rhs>` at the first '=', matching parse_stamp_offsets() and
// the `cam-info replace` parser. Returns {value, ""} when there is no '='.
std::pair<std::string, std::string> split_pair(const std::string & value)
{
  const auto eq = value.find('=');
  if (eq == std::string::npos) {
    return {value, {}};
  }
  return {value.substr(0, eq), value.substr(eq)};  // rhs keeps its leading '='
}

bool reject_glob(const CLI::App & app, const TopicSlot & slot, const std::string & value)
{
  const std::string label = flag_label(*slot.option);
  if (!slot.spec.reject_reason.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "%s: %s does not accept globs — %s (got '%s')", command_label(app).c_str(),
      label.c_str(), std::string{slot.spec.reject_reason}.c_str(), value.c_str());
  } else {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "%s: %s takes a single topic and does not accept globs; pass a literal topic "
      "name (got '%s')",
      command_label(app).c_str(), label.c_str(), value.c_str());
  }
  return false;
}

std::vector<std::string> & values_of(const TopicSlot & slot, std::vector<std::string> & scratch)
{
  if (slot.multi_target != nullptr) {
    return *slot.multi_target;
  }
  scratch.assign(1, *slot.single_target);
  return scratch;
}

// Resolution universe for a slot: another slot's expanded result when scoped,
// else the bag's own topic list. A scoped universe carries no type
// information (its entries come from another slot's plain names), so the
// caller must not apply a type filter to it — the governing slot already did.
// Nullopt means `slot.spec.scope` names an option that is not an earlier slot
// of this command; the caller treats that as failure.
std::optional<std::vector<core::TopicEntry>> resolution_universe(
  const CLI::App & app, const TopicSlot & slot, const std::vector<core::TopicEntry> & bag_topics,
  const std::unordered_map<const CLI::Option *, std::vector<std::string>> & expanded)
{
  if (slot.spec.scope == nullptr) {
    return bag_topics;
  }

  const auto it = expanded.find(slot.spec.scope);
  if (it == expanded.end()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "%s: internal error — %s is scoped to an option that is not an earlier topic "
      "slot of this command",
      command_label(app).c_str(), flag_label(*slot.option).c_str());
    return std::nullopt;
  }

  std::vector<core::TopicEntry> universe;
  universe.reserve(it->second.size());
  for (const auto & name : it->second) {
    universe.push_back(core::TopicEntry{name, {}});
  }
  return universe;
}

// Expand one slot's values against `universe`, filtered by `allowed`. Splits
// pair values at '=' before resolving so only the left half is a selector,
// and expands one value at a time so each keeps its own right half.
std::optional<std::vector<std::string>> resolve_values(
  const CLI::App & app, const TopicSlot & slot, const std::vector<std::string> & values,
  const std::vector<core::TopicEntry> & universe, std::span<const std::string_view> allowed)
{
  std::vector<std::string> selectors;
  std::vector<std::string> suffixes;
  selectors.reserve(values.size());
  suffixes.reserve(values.size());
  for (const auto & value : values) {
    if (slot.spec.pair_value) {
      auto [lhs, rhs] = split_pair(value);
      selectors.push_back(std::move(lhs));
      suffixes.push_back(std::move(rhs));
    } else {
      selectors.push_back(value);
      suffixes.emplace_back();
    }
  }

  std::vector<std::string> result;
  for (std::size_t i = 0; i < selectors.size(); ++i) {
    const std::vector<std::string> one{selectors[i]};
    const auto resolved = core::resolve_topic_selectors(one, universe, allowed);
    if (!resolved.unmatched.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "%s: %s selector '%s' matched no topic", command_label(app).c_str(),
        flag_label(*slot.option).c_str(), selectors[i].c_str());
      return std::nullopt;
    }
    for (const auto & name : resolved.matched) {
      result.push_back(name + suffixes[i]);
    }
  }
  return result;
}

// Deduplicate `values`, preserving first occurrence.
std::vector<std::string> dedupe(std::vector<std::string> values)
{
  std::vector<std::string> out;
  std::unordered_set<std::string> seen;
  for (auto & entry : values) {
    if (seen.insert(entry).second) {
      out.push_back(std::move(entry));
    }
  }
  return out;
}

// Expand one slot's values. `expanded` maps an already-processed slot's option
// to its result, so a slot with `scope` set can resolve against it.
bool expand_slot(
  const CLI::App & app, const TopicSlot & slot, const std::vector<core::TopicEntry> & bag_topics,
  const std::unordered_map<const CLI::Option *, std::vector<std::string>> & expanded)
{
  std::vector<std::string> scratch;
  const std::vector<std::string> & values = values_of(slot, scratch);
  if (values.empty()) {
    return true;
  }

  if (slot.spec.mode == TopicSelectorMode::kLiteral) {
    for (const auto & value : values) {
      if (contains_glob(value)) {
        return reject_glob(app, slot, value);
      }
    }
    return true;
  }

  const auto universe = resolution_universe(app, slot, bag_topics, expanded);
  if (!universe) {
    return false;
  }

  // A scoped universe carries no types, so the type filter must not be applied
  // to it — the governing slot already applied its own.
  const std::span<const std::string_view> allowed =
    slot.spec.scope != nullptr ? std::span<const std::string_view>{} : slot.spec.allowed_types;

  auto resolved = resolve_values(app, slot, values, *universe, allowed);
  if (!resolved) {
    return false;
  }
  std::vector<std::string> deduped = dedupe(std::move(*resolved));

  if (slot.multi_target != nullptr) {
    *slot.multi_target = std::move(deduped);
  } else {
    *slot.single_target = deduped.empty() ? std::string{} : deduped.front();
  }
  return true;
}

bool expand_app(const CLI::App & app)
{
  const auto slots = topic_slots_of(app);
  if (slots.empty()) {
    return true;
  }

  const std::filesystem::path * input = topic_input_of(app);
  if (input == nullptr || input->empty()) {
    return true;  // no bag to resolve against
  }

  auto topics = read_topics(*input);
  if (!topics) {
    // Not a readable bag. Leave every value verbatim so the command's own
    // validation reports it in its own terms.
    return true;
  }
  const std::vector<core::TopicEntry> bag_topics = std::move(*topics);

  std::unordered_map<const CLI::Option *, std::vector<std::string>> expanded;
  for (const auto & slot : slots) {
    if (!expand_slot(app, slot, bag_topics, expanded)) {
      return false;
    }
    if (slot.multi_target != nullptr) {
      expanded.emplace(slot.option, *slot.multi_target);
    } else {
      expanded.emplace(slot.option, std::vector<std::string>{*slot.single_target});
    }
  }
  return true;
}

}  // namespace

bool expand_topic_selectors(const CLI::App & root)
{
  if (!expand_app(root)) {
    return false;
  }
  // get_subcommands() returns the subcommands parsed on this invocation, so the
  // walk visits exactly the command the user ran.
  for (const CLI::App * sub : root.get_subcommands()) {
    if (!expand_topic_selectors(*sub)) {
      return false;
    }
  }
  return true;
}

}  // namespace bagwiz::commands
