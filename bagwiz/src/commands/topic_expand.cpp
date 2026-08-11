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

#include <algorithm>
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

// Comma-joined topic names for an error message.
std::string join_topics(const std::vector<std::string> & topics)
{
  std::string joined;
  for (const auto & topic : topics) {
    if (!joined.empty()) {
      joined += ", ";
    }
    joined += topic;
  }
  return joined;
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
  } catch (const std::exception & e) {
    // Not necessarily a calibration YAML — could equally be a corrupt or
    // truncated bag. Debug-only: an ordinary "not a bag" input (the common
    // case for e.g. `cam-info recompute-p`) would otherwise log on every run.
    BAGWIZ_LOG_DEBUG(
      kLogger, "topic expansion: could not open '%s' as a bag: %s", path.c_str(), e.what());
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

// A copy, not a reference: the multi-value case would otherwise alias
// `*slot.multi_target`, which assign_slot_result() below overwrites before
// the caller is done with the values it read out of it. Copying once here is
// cheap (CLI argument counts) and removes that aliasing hazard entirely
// rather than relying on call-order to keep it safe.
std::vector<std::string> values_of(const TopicSlot & slot)
{
  if (slot.multi_target != nullptr) {
    return *slot.multi_target;
  }
  return {*slot.single_target};
}

// Where a slot's selectors resolve against, and the type filter to apply
// while doing so. Bundled together because the two facts are one decision:
// a scoped universe's entries come from another slot's plain names and carry
// no type information, so the type filter must be empty exactly when the
// universe is scoped — expressing that as two independently-checked
// `scope != nullptr` tests invites them to drift apart.
struct ResolutionContext
{
  std::vector<core::TopicEntry> universe;
  std::span<const std::string_view> allowed;
};

// Nullopt means `slot.spec.scope` names an option that is not an earlier
// slot of this command; the caller treats that as failure.
std::optional<ResolutionContext> resolve_context(
  const CLI::App & app, const TopicSlot & slot, const std::vector<core::TopicEntry> & bag_topics,
  const std::unordered_map<const CLI::Option *, std::vector<std::string>> & expanded)
{
  if (slot.spec.scope == nullptr) {
    return ResolutionContext{bag_topics, slot.spec.allowed_types};
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
  return ResolutionContext{std::move(universe), {}};
}

// One value produced while expanding a slot, tagged with whether a glob
// selector produced it. The tag drives dedupe(): a glob-produced entry that
// duplicates an earlier one (glob- or literal-produced) is dropped, but a
// literal the user typed by hand is always kept, even if it duplicates
// something else in the list — the command's own "topic given more than
// once" checks run after expansion and must still see it.
struct ExpandedValue
{
  std::string value;
  bool from_glob{false};
};

// True when `name` names an entry in `universe`, ignoring type — the
// presence check TopicSlotSpec::require_present asks for is deliberately
// blind to allowed_types, so a wrongly-typed literal still reaches the
// command and gets that command's own (more specific) type error.
bool topic_is_present(const std::vector<core::TopicEntry> & universe, const std::string & name)
{
  return std::any_of(universe.begin(), universe.end(), [&name](const core::TopicEntry & entry) {
    return entry.name == name;
  });
}

// The one error shape for "this selector produced nothing": an unmatched
// glob and, on a require_present slot, an absent literal both funnel through
// here so there is exactly one place that could ever grow a second shape.
void log_selector_matched_nothing(
  const CLI::App & app, const TopicSlot & slot, const std::string & value)
{
  BAGWIZ_LOG_ERROR(
    kLogger, "%s: %s selector '%s' matched no topic", command_label(app).c_str(),
    flag_label(*slot.option).c_str(), value.c_str());
}

// Expand one slot's values against `ctx.universe`, filtered by `ctx.allowed`.
// Splits pair values at '=' before resolving so only the left half is a
// selector, and expands one value at a time so each keeps its own right
// half. CROSS-SELECTOR dedup is deliberately not delegated to a single
// batched call into core::resolve_topic_selectors() — that function already
// dedupes literals internally, but only within one call, and this loop calls
// it once per selector so that path is never exercised. The CLI layer owns
// the cross-selector rule (see dedupe()); do not "simplify" this into one
// batched call, which would silently reintroduce whole-list deduplication.
std::optional<std::vector<ExpandedValue>> resolve_values(
  const CLI::App & app, const TopicSlot & slot, const std::vector<std::string> & values,
  const ResolutionContext & ctx)
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

  std::vector<ExpandedValue> result;
  for (std::size_t i = 0; i < selectors.size(); ++i) {
    const bool from_glob = contains_glob(selectors[i]);
    const std::vector<std::string> one{selectors[i]};
    const auto resolved = core::resolve_topic_selectors(one, ctx.universe, ctx.allowed);
    // A glob that matched nothing is already in resolved.unmatched. A literal
    // is always copied through unvalidated by resolve_topic_selectors (see
    // topic_match.hpp), so require_present slots re-check its presence here —
    // same error, same shape, whether the selector was a literal or a glob.
    const bool missing =
      !resolved.unmatched.empty() ||
      (!from_glob && slot.spec.require_present && !topic_is_present(ctx.universe, selectors[i]));
    if (missing) {
      log_selector_matched_nothing(app, slot, selectors[i]);
      return std::nullopt;
    }
    for (const auto & name : resolved.matched) {
      result.push_back(ExpandedValue{name + suffixes[i], from_glob});
    }
  }
  return result;
}

// Deduplicate `values`, preserving first occurrence. A glob-produced entry
// is dropped when the list already carries it; a literal is always kept. See
// ExpandedValue for why.
std::vector<std::string> dedupe(std::vector<ExpandedValue> values)
{
  std::vector<std::string> out;
  std::unordered_set<std::string> seen;
  for (auto & entry : values) {
    if (entry.from_glob && seen.count(entry.value) != 0) {
      continue;
    }
    seen.insert(entry.value);
    out.push_back(std::move(entry.value));
  }
  return out;
}

// Writes a slot's deduplicated result back into its target. A single-target
// slot resolving to more than one topic is a caller bug — TopicSlotSpec::mode
// defaults to kGlob, so a single-target slot that omits `.mode = kLiteral`
// would otherwise silently keep only the first match — and is reported as an
// error naming the flag and every match instead of failing silently.
bool assign_slot_result(
  const CLI::App & app, const TopicSlot & slot, std::vector<std::string> deduped)
{
  if (slot.multi_target != nullptr) {
    *slot.multi_target = std::move(deduped);
    return true;
  }
  if (deduped.size() > 1) {
    BAGWIZ_LOG_ERROR(
      kLogger, "%s: %s takes a single topic but the selector matched %zu topics (%s)",
      command_label(app).c_str(), flag_label(*slot.option).c_str(), deduped.size(),
      join_topics(deduped).c_str());
    return false;
  }
  *slot.single_target = deduped.empty() ? std::string{} : deduped.front();
  return true;
}

// Expand one slot's values. `expanded` maps an already-processed slot's option
// to its result, so a slot with `scope` set can resolve against it.
bool expand_slot(
  const CLI::App & app, const TopicSlot & slot, const std::vector<core::TopicEntry> & bag_topics,
  const std::unordered_map<const CLI::Option *, std::vector<std::string>> & expanded)
{
  const std::vector<std::string> values = values_of(slot);
  if (values.empty()) {
    return true;
  }

  if (slot.spec.mode == TopicSelectorMode::kLiteral) {
    // require_present applies uniformly across both modes: a kLiteral slot's
    // value never goes through resolve_values() (there is nothing to expand),
    // so the presence check has to happen here instead. The universe is only
    // resolved when actually needed, since most kLiteral slots don't set the
    // flag and resolve_context() can fail on a scope-configuration bug.
    const std::optional<ResolutionContext> ctx =
      slot.spec.require_present ? resolve_context(app, slot, bag_topics, expanded)
                                : std::optional<ResolutionContext>{};
    if (slot.spec.require_present && !ctx) {
      return false;
    }
    for (const auto & value : values) {
      if (contains_glob(value)) {
        return reject_glob(app, slot, value);
      }
      if (ctx && !topic_is_present(ctx->universe, value)) {
        log_selector_matched_nothing(app, slot, value);
        return false;
      }
    }
    return true;
  }

  const auto ctx = resolve_context(app, slot, bag_topics, expanded);
  if (!ctx) {
    return false;
  }

  auto resolved = resolve_values(app, slot, values, *ctx);
  if (!resolved) {
    return false;
  }
  return assign_slot_result(app, slot, dedupe(std::move(*resolved)));
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
