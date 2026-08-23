// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/completion.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/commands/topic_types.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/topic_match.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.complete";
constexpr std::string_view kCompletionCommand = "__complete";
constexpr int kMinimumCompletionProbeArgc = 2;
constexpr int kCompletionCommandArg = 1;
constexpr int kMinimumCompletionArgc = 3;
constexpr int kCursorWordArg = 2;
constexpr int kCompletionWordsBeginArg = 3;
constexpr std::size_t kTopLevelCommandWord = 0;
constexpr std::size_t kFirstCommandArgWord = 1;
constexpr std::size_t kSecondCommandArgWord = 2;
constexpr std::size_t kThirdCommandArgWord = 3;
constexpr std::size_t kFourthCommandArgWord = 4;

// Required rosbag input is always behind -i/--input after the flag conversion.
constexpr std::array<std::string_view, 2> kInputFlags{{"-i", "--input"}};

// Single topic operand behind -t/--topic.
constexpr std::array<std::string_view, 2> kSingleTopicFlags{{"-t", "--topic"}};

enum class CompletionShell { Bash, Zsh, Fish };

struct CompletionRequest
{
  std::vector<std::string> words;
  std::size_t cursor_word = 0;
};

struct ShellDefinition
{
  CompletionShell shell{};
  std::string_view name{};
};

const std::vector<ShellDefinition> & shell_definitions()
{
  static const std::vector<ShellDefinition> kDefinitions{
    {CompletionShell::Bash, "bash"},
    {CompletionShell::Zsh, "zsh"},
    {CompletionShell::Fish, "fish"},
  };
  return kDefinitions;
}

std::vector<std::string> supported_shell_name_strings()
{
  std::vector<std::string> names;
  for (const auto & definition : shell_definitions()) {
    names.emplace_back(definition.name);
  }
  return names;
}

std::optional<CompletionShell> parse_shell(const std::string_view & name)
{
  for (const auto & definition : shell_definitions()) {
    if (definition.name == name) {
      return definition.shell;
    }
  }
  return std::nullopt;
}

bool starts_with(std::string_view value, std::string_view prefix)
{
  return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
}

// True when `word` equals one of `candidates`. Lets a branch test a whole flag
// alias set (e.g. kSingleTopicFlags) against the named constant instead of
// repeating the spellings as literals.
bool is_one_of(std::string_view word, std::span<const std::string_view> candidates)
{
  return std::find(candidates.begin(), candidates.end(), word) != candidates.end();
}

std::string basename(std::string_view path)
{
  const auto pos = path.find_last_of("/\\");
  if (pos == std::string_view::npos) {
    return std::string{path};
  }
  return std::string{path.substr(pos + 1)};
}

std::optional<std::size_t> parse_size(std::string_view text)
{
  try {
    std::size_t consumed = 0;
    const auto value = static_cast<std::size_t>(std::stoul(std::string{text}, &consumed));
    if (consumed != text.size()) {
      return std::nullopt;
    }
    return value;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::filesystem::path expand_current_user_home(const std::filesystem::path & path)
{
  const std::string text = path.string();
  if (text != "~" && !starts_with(text, "~/")) {
    return path;
  }

  const char * const home = std::getenv("HOME");
  if (home == nullptr || std::string_view{home}.empty()) {
    return path;
  }

  if (text == "~") {
    return std::filesystem::path{home};
  }
  return std::filesystem::path{home} / text.substr(2);
}

std::optional<std::string> env_var(const char * name)
{
  const char * const value = std::getenv(name);
  if (value == nullptr || std::string_view{value}.empty()) {
    return std::nullopt;
  }
  return std::string{value};
}

std::optional<std::filesystem::path> home_directory()
{
  const auto home = env_var("HOME");
  if (!home) {
    return std::nullopt;
  }
  return std::filesystem::path{*home};
}

std::optional<std::filesystem::path> install_path_for(CompletionShell shell)
{
  const auto home = home_directory();
  if (!home) {
    return std::nullopt;
  }

  switch (shell) {
    case CompletionShell::Bash: {
      const auto base = env_var("XDG_DATA_HOME").value_or((*home / ".local" / "share").string());
      return std::filesystem::path{base} / "bash-completion" / "completions" / "bagwiz";
    }
    case CompletionShell::Zsh:
      return *home / ".zsh" / "completions" / "_bagwiz";
    case CompletionShell::Fish: {
      const auto base = env_var("XDG_CONFIG_HOME").value_or((*home / ".config").string());
      return std::filesystem::path{base} / "fish" / "completions" / "bagwiz.fish";
    }
  }
  return std::nullopt;
}

bool write_script_to(
  const std::filesystem::path & target, const std::string_view & contents, bool overwrite)
{
  std::error_code ec;
  if (std::filesystem::exists(target, ec) && !overwrite) {
    BAGWIZ_LOG_ERROR(
      kLogger, "refusing to overwrite existing file: %s (pass -w/--overwrite to replace it)",
      target.string().c_str());
    return false;
  }

  const auto parent = target.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      BAGWIZ_LOG_ERROR(
        kLogger, "failed to create directory %s: %s", parent.string().c_str(),
        ec.message().c_str());
      return false;
    }
  }

  std::ofstream stream(target, std::ios::trunc);
  if (!stream) {
    BAGWIZ_LOG_ERROR(kLogger, "failed to open %s for writing", target.string().c_str());
    return false;
  }
  stream << contents;
  if (!stream) {
    BAGWIZ_LOG_ERROR(kLogger, "failed to write completion script to %s", target.string().c_str());
    return false;
  }
  return true;
}

CompletionRequest parse_request(int argc, char * const * argv)
{
  CompletionRequest request;
  if (argc < kMinimumCompletionArgc) {
    return request;
  }

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const auto cursor_word = parse_size(argv[kCursorWordArg]);
  if (!cursor_word) {
    return request;
  }

  request.cursor_word = *cursor_word;
  for (int i = kCompletionWordsBeginArg; i < argc; ++i) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    request.words.emplace_back(argv[i]);
  }

  if (!request.words.empty() && basename(request.words.front()) == "bagwiz") {
    request.words.erase(request.words.begin());
    if (request.cursor_word > 0) {
      --request.cursor_word;
    }
  }

  if (request.cursor_word > request.words.size()) {
    request.cursor_word = request.words.size();
  }
  return request;
}

std::string current_word(const CompletionRequest & request)
{
  if (request.cursor_word < request.words.size()) {
    return request.words[request.cursor_word];
  }
  return {};
}

std::vector<std::string> matching(
  const std::vector<std::string_view> & candidates, const std::string_view & prefix)
{
  std::vector<std::string> result;
  for (const auto candidate : candidates) {
    if (starts_with(candidate, prefix)) {
      result.emplace_back(candidate);
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

// Flags exposed by the top-level CLI::App. `--help` / `-h` are auto-added
// by CLI11 for every App, and `--version` is wired in main().
constexpr std::array<std::string_view, 3> kTopLevelFlags{
  "--help",
  "--version",
  "-h",
};

// `--help` / `-h` are auto-added by CLI11 for every App and subcommand. Each
// per-command flag table prepends these so the user always sees them on
// `-<TAB>`, even when the command/subcommand defines no other flags of its
// own.
constexpr std::array<std::string_view, 2> kCommonHelpFlags{
  "--help",
  "-h",
};

std::vector<std::string_view> with_help(std::initializer_list<std::string_view> flags)
{
  std::vector<std::string_view> result;
  result.reserve(flags.size() + kCommonHelpFlags.size());
  for (const auto & flag : kCommonHelpFlags) {
    result.push_back(flag);
  }
  for (const auto & flag : flags) {
    result.push_back(flag);
  }
  return result;
}

std::vector<std::string> top_level_candidates(const std::string_view & prefix)
{
  if (starts_with(prefix, "-")) {
    return matching({kTopLevelFlags.begin(), kTopLevelFlags.end()}, prefix);
  }

  std::vector<std::string> result;
  for (const auto & cmd : Registry::instance().all()) {
    // Hidden commands (e.g. the `joke` easter egg) are omitted from
    // completion just as they are from --help, so they never surface to a
    // user who does not already know they exist.
    if (cmd->hidden()) {
      continue;
    }
    if (starts_with(cmd->name(), prefix)) {
      result.emplace_back(cmd->name());
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

// Topic-name completion candidates from the bag at `input_path` whose names
// start with `prefix`. When `allowed_types` is non-empty, a topic is offered
// only if its type is one of the listed types (e.g. `tf tree`'s single
// TFMessage type, or `traj dump`'s supported set); an empty `allowed_types`
// offers every topic. When `static_only` is true a topic must additionally be
// a static TF topic (core::is_static_tf_topic — TFMessage type with
// `tf_static` as the name's final path segment, the same test every bagwiz
// static-TF reader applies), so it alone offers exactly the bag's static TF
// topics. Best-effort: a bag that fails to
// open yields no candidates and the shell's default file completion takes over.
std::vector<std::string> complete_topics(
  const std::filesystem::path & input_path, const std::string_view & prefix,
  std::span<const std::string_view> allowed_types, bool static_only = false)
{
  std::vector<std::string> result;
  const auto expanded = expand_current_user_home(input_path);

  // A bare single-file `.db3.zstd` envelope carries no metadata.yaml, so reading
  // its topic list forces a full decompress of the whole database to a temp file
  // — seconds of hang per TAB on a multi-GB bag. Offer nothing so the shell's
  // default file completion takes over instead of blocking. Directory bags (FILE-
  // or MESSAGE-mode) serve their topic list from metadata.yaml without touching
  // the envelope, so they stay fast and are deliberately not skipped here.
  std::error_code ec;
  if (!std::filesystem::is_directory(expanded, ec) && io::is_file_compressed_bag(expanded)) {
    return {};
  }

  try {
    const auto reader = io::open_read(expanded);
    for (const auto & topic : reader->topics()) {
      if (!starts_with(topic.name, prefix)) {
        continue;
      }
      if (
        !allowed_types.empty() &&
        std::find(allowed_types.begin(), allowed_types.end(), topic.type) == allowed_types.end()) {
        continue;
      }
      if (static_only && !core::is_static_tf_topic(topic)) {
        continue;
      }
      result.push_back(topic.name);
    }
  } catch (const std::exception &) {
    return {};
  }

  std::sort(result.begin(), result.end());
  return result;
}

// Soft cap on TF messages scanned for frame-id discovery. Static TF is
// usually one message; dynamic TF re-publishes the same edges, so the
// distinct frame-id set saturates well before this cap. The cap keeps
// per-keystroke completion latency bounded on multi-GB bags.
constexpr std::size_t kFrameIdScanMessageCap = 5000;

// Walks the bag's tf2_msgs/msg/TFMessage topics once and returns the
// sorted, deduplicated set of header.frame_id / child_frame_id values
// it observed. When `static_only` is true only */tf_static topics are
// scanned (for `tf static calc`, which resolves the static tree); otherwise
// every TF topic contributes. Reads at most `kFrameIdScanMessageCap`
// messages so completion stays responsive on large bags. Swallows every
// exception:
// completion is best-effort and a bag that fails to open should silently fall
// through to the shell's file-completion fallback rather than spew
// errors during TAB.
std::vector<std::string> collect_tf_frame_ids(
  const std::filesystem::path & bag_path, bool static_only = false)
{
  std::vector<std::string> frame_ids;
  const auto expanded = expand_current_user_home(bag_path);

  // Frame-id discovery iterates TF messages. For a FILE-mode zstd bag the first
  // read decompresses the whole shard to a temp .db3 up front — seconds of hang
  // per TAB on a multi-GB bag, regardless of the scan cap. Offer nothing instead.
  // MESSAGE-mode bags decompress per message (bounded by kFrameIdScanMessageCap)
  // and uncompressed bags are cheap, so both keep working.
  if (io::is_file_compressed_bag(expanded)) {
    return {};
  }

  try {
    auto reader = io::open_read(expanded);

    std::vector<std::string> tf_topic_names;
    for (const auto & t : reader->topics()) {
      if (t.type == core::kTfMessageTypeName && (!static_only || core::is_static_tf_topic(t))) {
        tf_topic_names.push_back(t.name);
      }
    }
    if (tf_topic_names.empty()) {
      return {};
    }

    io::ReadFilter filter;
    filter.topics = std::move(tf_topic_names);
    reader->set_filter(filter);

    std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoders;
    for (const auto & topic_info : reader->topics()) {
      if (topic_info.type != core::kTfMessageTypeName) {
        continue;
      }
      if (static_only && !core::is_static_tf_topic(topic_info)) {
        continue;
      }
      auto open = core::decoder::open_decoder(topic_info);
      if (!open.ok()) {
        continue;  // best-effort: skip undecodable topics rather than abort
      }
      decoders.emplace(topic_info.name, std::move(open.decoder));
    }
    if (decoders.empty()) {
      return {};
    }

    std::unordered_set<std::string> seen;
    io::RawMessage raw;
    std::size_t scanned = 0;
    while (scanned < kFrameIdScanMessageCap && reader->next(raw)) {
      ++scanned;
      auto it = decoders.find(raw.topic->name);
      if (it == decoders.end()) {
        continue;
      }
      const auto decoded = it->second->decode(raw.payload);
      if (!decoded.ok()) {
        continue;
      }
      for (const auto & t : core::extract_tf_message(*decoded.value)) {
        if (!t.header.frame_id.empty()) {
          seen.insert(t.header.frame_id);
        }
        if (!t.child_frame_id.empty()) {
          seen.insert(t.child_frame_id);
        }
      }
    }

    frame_ids.assign(seen.begin(), seen.end());
    std::sort(frame_ids.begin(), frame_ids.end());
  } catch (const std::exception &) {
    return {};
  }
  return frame_ids;
}

// Completion candidates for the value of `--of` / `--ref`. Returns the
// bag's TF frame ids filtered by `prefix`. When the bag yields no frame
// ids to suggest — whether it failed to open or opened cleanly but carries
// no TF data — we return an empty list so completion simply offers nothing
// and the shell's default file-completion fallback takes over.
std::vector<std::string> complete_frame_id_value(
  const std::filesystem::path & input_path, const std::string_view & prefix,
  bool static_only = false)
{
  std::vector<std::string> result;
  for (const auto & frame : collect_tf_frame_ids(input_path, static_only)) {
    if (starts_with(frame, prefix)) {
      result.push_back(frame);
    }
  }
  return result;
}

// Returns the value of the most recently occurring flag in `flags`, or nullopt
// if none is present before the cursor word.
std::optional<std::string_view> find_flag_value(
  const CompletionRequest & request, std::span<const std::string_view> flags)
{
  for (std::size_t i = request.cursor_word; i > kFirstCommandArgWord; --i) {
    const auto & prev = request.words[i - 1];
    for (const auto & flag : flags) {
      if (prev == flag) {
        if (i < request.words.size()) {
          return request.words[i];
        }
        return std::nullopt;
      }
    }
  }
  return std::nullopt;
}

// The bag path behind -i/--input, or nullopt when it is absent, empty, or
// itself a flag — all shapes of a half-typed command line that cannot name a
// bag. Every bag-backed completion needs exactly this check before opening the
// bag, and offers nothing when it fails so the shell's default file completion
// takes over.
std::optional<std::string_view> find_input_bag(const CompletionRequest & request)
{
  const auto bag_arg = find_flag_value(request, kInputFlags);
  if (!bag_arg || bag_arg->empty() || bag_arg->starts_with("-")) {
    return std::nullopt;
  }
  return bag_arg;
}

// Collect every value given to a variadic flag before the cursor: for each
// occurrence of `flag`, the run of non-option words following it. Multiple
// occurrences accumulate in command-line order. Words at or past the cursor
// are not yet committed input, so they are never seen. The `--flag=value`
// spelling is not recognized, matching the rest of this file.
std::vector<std::string_view> collect_flag_values(
  const CompletionRequest & request, const std::string_view & flag)
{
  std::vector<std::string_view> values;
  const std::size_t end = std::min(request.cursor_word, request.words.size());
  for (std::size_t i = kFirstCommandArgWord + 1; i < end; ++i) {
    if (request.words[i] != flag) {
      continue;
    }
    while (i + 1 < end && !request.words[i + 1].starts_with("-")) {
      values.push_back(request.words[++i]);
    }
  }
  return values;
}

// True when the cursor sits in a value slot owned by `flag`. Unlike a plain
// `words[cursor - 1] == flag` check this walks back over the flag's earlier
// values, so a variadic flag completes at its second and later value slots too.
// The walk stops at the top-level command word: nothing at or before it is a
// flag value.
bool is_value_slot_of(const CompletionRequest & request, const std::string_view & flag)
{
  for (std::size_t i = request.cursor_word; i > kFirstCommandArgWord; --i) {
    const auto & word = request.words[i - 1];
    if (word == flag) {
      return true;
    }
    if (word.starts_with("-")) {
      return false;  // some other flag owns this slot
    }
  }
  return false;
}

// The CLI::App for the command path the cursor sits in, or nullptr when the
// words do not name a registered command (e.g. the cursor is still on the
// top-level command word, or an earlier word is a flag rather than a
// subcommand name). Slots are declared per CLI::App (see topic_option.hpp),
// so finding this App is the registry-driven replacement for
// TopicArgBinding's hand-written command/subcommand fields.
const CLI::App * app_for_request(const CLI::App & root, const CompletionRequest & request)
{
  const CLI::App * current = &root;
  for (std::size_t i = kTopLevelCommandWord; i < request.cursor_word; ++i) {
    const auto & word = request.words[i];
    if (word.empty() || word.starts_with("-")) {
      break;
    }
    const CLI::App * next = nullptr;
    for (const auto * sub : current->get_subcommands({})) {
      if (sub->get_name() == word) {
        next = sub;
        break;
      }
    }
    if (next == nullptr) {
      break;
    }
    current = next;
  }
  return current == &root ? nullptr : current;
}

// Every flag spelling `option` was declared with, formatted the way a
// completion request spells it ("-t", "--topic", ...), so each can be tested
// with is_value_slot_of().
std::vector<std::string> option_flag_names(const CLI::Option & option)
{
  std::vector<std::string> names;
  for (const auto & name : option.get_snames()) {
    names.push_back("-" + name);
  }
  for (const auto & name : option.get_lnames()) {
    names.push_back("--" + name);
  }
  return names;
}

// The slot declared on `app` whose option owns the cursor's value position,
// or nullptr when the cursor is not sitting in any topic slot's value.
const TopicSlot * slot_for_cursor(const CLI::App & app, const CompletionRequest & request)
{
  for (const auto & slot : topic_slots_of(app)) {
    for (const auto & name : option_flag_names(*slot.option)) {
      if (is_value_slot_of(request, name)) {
        return &slot;
      }
    }
  }
  return nullptr;
}

// True when the registry alone cannot say what to offer for this slot's
// value, so a command-specific handler (or nothing, when none exists) takes
// over instead of try_topic_completion:
//  - a kLiteral, non-paired slot with a reject_reason exists to NAME
//    something new (an output topic via --as, a rename destination, an
//    embedding target) rather than to SELECT something already in the bag —
//    every such slot sets reject_reason to explain why a glob makes no sense
//    there (see TopicSlotSpec::reject_reason). `tf static update -t` is one: its
//    completion (the bag's static TF topics) is command-specific, a
//    distinction the registry does not encode.
//  - a scoped slot (TopicSlotSpec::scope) resolves against another option's
//    EXPANDED result, which exists only after CLI11 has parsed and
//    topic_expand.cpp has run — neither has happened yet during completion,
//    so there is nothing here to filter against. `pcd concat --stamp-offset`
//    is the one slot this applies to today (scoped to --pcd); its completion
//    stays command-specific for exactly this reason.
//  - a pair slot whose selector is the RIGHT half (TopicSlotSpec::
//    pair_selector_rhs) has a literal left half that must name one of another
//    flag's values and a topic right half — two candidate sets the generic
//    pair handling cannot express. `generate video cam --pcd` is the one slot
//    this applies to today; complete_generate() completes both halves.
//  - a pair-optional slot (TopicSlotSpec::pair_optional) accepts a bare value
//    alongside pairs, so the generic "append '=' to every candidate" pair
//    handling would mangle its bare-value completion. `generate video cam
//    --cam-info` is the one slot this applies to today.
bool completion_defers_to_command(const TopicSlotSpec & spec)
{
  if (spec.scope != nullptr || spec.pair_selector_rhs || spec.pair_optional) {
    return true;
  }
  return spec.mode == TopicSelectorMode::kLiteral && !spec.pair_value &&
         !spec.reject_reason.empty();
}

// Finds the topic slot the cursor sits in and, if one applies, dispatches to
// topic completion. Returns std::nullopt when no slot applies so the caller
// can fall through to per-command completion. Returning an empty vector means
// "a slot applies, but there are no candidates" (e.g. bad bag path) — the
// shell's default file completion then takes over.
std::optional<std::vector<std::string>> try_topic_completion(
  const CLI::App & root, const CompletionRequest & request)
{
  if (request.words.empty()) {
    return std::nullopt;
  }

  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return std::nullopt;
  }

  const CLI::App * app = app_for_request(root, request);
  if (app == nullptr) {
    return std::nullopt;
  }

  const auto * slot = slot_for_cursor(*app, request);
  if (slot == nullptr || completion_defers_to_command(slot->spec)) {
    return std::nullopt;
  }

  // On the <rhs> half of a `<topic>=<rhs>` value, offer nothing so the
  // shell's default file completion takes over (bash splits the typed value
  // at '=', leaving a bare '=' before the cursor; zsh/fish keep it unsplit,
  // showing up as a current word containing '=').
  if (
    slot->spec.pair_value &&
    (current.find('=') != std::string::npos ||
     (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "="))) {
    return std::vector<std::string>{};
  }

  const auto input_path = find_input_bag(request);
  if (!input_path) {
    return std::vector<std::string>{};
  }

  auto candidates =
    complete_topics(std::filesystem::path{*input_path}, current, slot->spec.allowed_types);

  // A kLiteral pair slot always requires both halves (map slam --cam-info
  // names one CameraInfo topic per --color camera); append '=' so the shell
  // scripts drop the auto-space and leave the cursor ready for <rhs>. A
  // kGlob pair slot may take a bare value instead (cam-info replace accepts
  // a lone <topic>, applying the shared --yaml), so its candidates are left
  // without the suffix.
  if (slot->spec.pair_value && slot->spec.mode == TopicSelectorMode::kLiteral) {
    for (auto & topic : candidates) {
      topic += '=';
    }
  }
  return candidates;
}

std::vector<std::string> complete_complete_command(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return matching(with_help({"--install", "--overwrite", "--shell", "-w"}), current);
  }
  return {};
}

std::vector<std::string> complete_convert(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"format"}, current);
  }

  const auto & mode = request.words[kFirstCommandArgWord];

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    if (mode == "format") {
      return matching(
        with_help({"--input", "--output", "--overwrite", "--storage", "-i", "-o", "-w"}), current);
    }
  }

  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--storage") {
    return matching({"mcap", "sqlite3"}, current);
  }
  return {};
}

std::vector<std::string> complete_traj(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"dump", "join"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & mode = request.words[kFirstCommandArgWord];
    if (mode == "dump") {
      return matching(
        with_help(
          {"--format", "--input", "--of", "--output", "--overwrite", "--ref", "--topic", "-f", "-i",
           "-o", "-t", "-w"}),
        current);
    }
    if (mode == "join") {
      return matching(
        with_help(
          {"--as", "--force", "--format", "--input", "--msg-type", "--of", "--output",
           "--overwrite", "--ref", "--traj", "-i", "-m", "-o", "-w"}),
        current);
    }
  }

  if (request.cursor_word > 0) {
    const auto & previous = request.words[request.cursor_word - 1];
    if (previous == "--format" || previous == "-f") {
      return matching({"tum"}, current);
    }
    if (previous == "--msg-type" || previous == "-m") {
      return matching({"tf"}, current);
    }
    if (previous == "--of" || previous == "--ref") {
      const auto bag_arg = find_input_bag(request);
      if (!bag_arg) {
        return {};
      }
      return complete_frame_id_value(*bag_arg, current);
    }
  }
  return {};
}

// `tf static` is a command group with six actions, `calc`, `cp`, `drop`,
// `dump`, `join`, and `update`. The action verb adds one positional slot,
// shifting every argument one word to the right of the flat `tf` subcommands.
//
//   calc:      `tf`(0) `static`(1) `calc`(2)      -i|--input <bag> --of <frame> --ref <frame>
//                                                  [--json]
//   cp:        `tf`(0) `static`(1) `cp`(2)        --src <bag> --dst <bag> [-o <out>] [--force]
//                                                  [-w|--overwrite]
//   drop:      `tf`(0) `static`(1) `drop`(2)      -i|--input <bag> --frame <frame>... [-o <out>]
//                                                  [-w|--overwrite]
//   dump:      `tf`(0) `static`(1) `dump`(2)      -i|--input <bag> [-o <out>] [-w|--overwrite]
//   join:      `tf`(0) `static`(1) `join`(2)      -i|--input <bag> --yaml <file> [-t <topic>]
//                                                  [-o <out>] [--force] [-w|--overwrite]
//   update:    `tf`(0) `static`(1) `update`(2)    -i|--input <bag> --yaml <file> [-t <topic>]
//                                                  [-o <out>] [-w|--overwrite]
//
// At the action slot (word 2) the candidates are `calc` / `cp` / `drop` /
// `dump` / `join` / `update`; past it each action completes on its own, in the
// `complete_tf_static_*` helpers below.

// A `tf static` action whose only candidates are its own flags: at a `-` word
// offer `flags` (plus the common help flags), anywhere else offer nothing so
// the shell's default completion takes over. `cp`, `dump`, and `join` are all
// of this shape — none of their value slots carries bagwiz candidates: they
// name bags, file paths, or (for `join`'s `--topic`) a topic being created.
std::vector<std::string> complete_tf_static_flags_only(
  const CompletionRequest & request, const std::string & current,
  std::initializer_list<std::string_view> flags)
{
  if (request.cursor_word >= kThirdCommandArgWord && current.starts_with("-")) {
    return matching(with_help(flags), current);
  }
  return {};
}

// `calc` surfaces `-i`/`--input`/`--json`/`--of`/`--ref` for any `-` word. Its
// `--of`/`--ref` value slots complete from the bag's static `*/tf_static` frame
// ids only, since `tf static calc` resolves the static tree.
std::vector<std::string> complete_tf_static_calc(
  const CompletionRequest & request, const std::string & current)
{
  if (request.cursor_word >= kThirdCommandArgWord && current.starts_with("-")) {
    return matching(with_help({"--input", "--json", "--of", "--ref", "-i"}), current);
  }
  if (request.cursor_word > 0) {
    const auto & previous = request.words[request.cursor_word - 1];
    if (previous == "--of" || previous == "--ref") {
      const auto bag_arg = find_input_bag(request);
      if (!bag_arg) {
        return {};
      }
      return complete_frame_id_value(*bag_arg, current, /*static_only=*/true);
    }
  }
  return {};
}

// `drop` surfaces `-i`/`-o`/`--overwrite` plus `--frame`. Its `--frame` value
// slot completes static frame ids like `calc`'s `--of`/`--ref`: it names a frame
// of the bag's static TF tree to remove.
std::vector<std::string> complete_tf_static_drop(
  const CompletionRequest & request, const std::string & current)
{
  if (request.cursor_word >= kThirdCommandArgWord && current.starts_with("-")) {
    return matching(
      with_help({"--frame", "--input", "--output", "--overwrite", "-i", "-o", "-w"}), current);
  }
  if (request.cursor_word > 0) {
    const auto & previous = request.words[request.cursor_word - 1];
    if (previous == "--frame") {
      const auto bag_arg = find_input_bag(request);
      if (!bag_arg) {
        return {};
      }
      return complete_frame_id_value(*bag_arg, current, /*static_only=*/true);
    }
  }
  return {};
}

// `update` surfaces the `join` flag set minus `--force`. Its `--yaml` is a file
// path and falls through to the shell. Unlike `join`'s, `update`'s `--topic`
// value slot does complete, from the bag's static TF topics only: it homes newly
// added edges, and an edge landing on a non-static TF topic would be invisible to
// every static-TF reader, so the dynamic `/tf` is deliberately not offered. The
// flag still accepts a brand-new topic name, which simply has no candidate to
// offer.
std::vector<std::string> complete_tf_static_update(
  const CompletionRequest & request, const std::string & current)
{
  if (request.cursor_word >= kThirdCommandArgWord && current.starts_with("-")) {
    return matching(
      with_help(
        {"--input", "--output", "--overwrite", "--topic", "--yaml", "-i", "-o", "-t", "-w"}),
      current);
  }
  if (request.cursor_word > 0) {
    const auto & previous = request.words[request.cursor_word - 1];
    if (is_one_of(previous, kSingleTopicFlags)) {
      const auto bag_arg = find_input_bag(request);
      if (!bag_arg) {
        return {};
      }
      return complete_topics(
        expand_current_user_home(*bag_arg), current, kTfMessageTypes, /*static_only=*/true);
    }
  }
  return {};
}

std::vector<std::string> complete_tf_static(
  const CompletionRequest & request, const std::string & current)
{
  if (request.cursor_word == kSecondCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"calc", "cp", "drop", "dump", "join", "update"}, current);
  }

  // Reaching here implies cursor_word > kSecondCommandArgWord, so words[2]
  // exists (parse_request clamps cursor_word to words.size()).
  const auto & action = request.words[kSecondCommandArgWord];

  if (action == "calc") {
    return complete_tf_static_calc(request, current);
  }
  if (action == "cp") {
    return complete_tf_static_flags_only(
      request, current, {"--dst", "--force", "--output", "--overwrite", "--src", "-o", "-w"});
  }
  if (action == "drop") {
    return complete_tf_static_drop(request, current);
  }
  if (action == "dump") {
    return complete_tf_static_flags_only(
      request, current, {"--input", "--output", "--overwrite", "-i", "-o", "-w"});
  }
  if (action == "join") {
    return complete_tf_static_flags_only(
      request, current,
      {"--as", "--force", "--input", "--output", "--overwrite", "--yaml", "-i", "-o", "-w"});
  }
  if (action == "update") {
    return complete_tf_static_update(request, current);
  }
  return {};
}

// `tf` has two subcommands: `tree`, and `static` (itself a nested command
// group, handled by complete_tf_static). At the subcommand slot (word 1) the
// candidates are `static` / `tree`.
//
//   tree: `tf`(0) `tree`(1) -i|--input <bag> [-t|--topics <topic-or-selector>...]
//
// `tree`'s -t/--topics is a declared topic slot (TFMessage topics only, at
// every value slot since the flag is variadic and optional), so
// try_topic_completion handles its values; here we surface only `tree`'s own
// flags for any `-` word.
std::vector<std::string> complete_tf(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"static", "tree"}, current);
  }

  const auto & mode = request.words[kFirstCommandArgWord];

  // `static` is a nested command group (`static calc`); its positional shape
  // differs from the flat `tree` subcommand, so it is handled apart.
  if (mode == "static") {
    return complete_tf_static(request, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    if (mode == "tree") {
      return matching(with_help({"--input", "--topics", "-i", "-t"}), current);
    }
    return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
  }

  return {};
}

// `calib` is a command group for sensor-extrinsic calibration. Its only
// subcommand is `cam-lidar`. At the subcommand slot (word 1) the only candidate
// is `cam-lidar`. `--pcd` (PointCloud2 topics), `--pose` (pose topics),
// `--cam` (image topics), and `--cam-info` (CameraInfo topics) are declared
// topic slots, so try_topic_completion handles their values before this
// function is reached. Here we surface `cam-lidar`'s flags for any `-` word,
// complete `--parent` / `--child` values from the bag's static-TF frame ids,
// and complete `--of` / `--ref` from its full frame-id set (like `pcd
// undistort`'s frame pair).
//
//   cam-lidar: `calib`(0) `cam-lidar`(1) -i|--input <bag> --pcd <topic>
//              --pose <topic> --cam <topic> [--of <frame>] [--ref <frame>]
//              --parent <frame> --child <frame> [--cam-info <topic>] [-o <out>]
//              [--samples <n>] [--fix <axes>] [--keyframe-dist <m>]
//              [--keyframe-rot <deg>] [--max-trans <m>] [--max-rot <deg>]
//              [--nid-bins <n>] [--min-depth <m>] [--max-depth <m>]
//              [--voxel <m>] [--skip-start <dur>] [--skip-end <dur>]
//              [--cam-offset <dur>|auto] [--imu <topic>] [-j|--threads <N>]
//              [--json] [-w|--overwrite]
std::vector<std::string> complete_calib(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"cam-lidar"}, current);
  }

  if (request.words.size() <= kFirstCommandArgWord) {
    return {};
  }
  const auto & sub = request.words[kFirstCommandArgWord];
  if (sub != "cam-lidar") {
    return {};
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    return matching(
      with_help(
        {"--cam",
         "--cam-info",
         "--cam-offset",
         "--child",
         "--fix",
         "--imu",
         "--input",
         "--json",
         "--keyframe-dist",
         "--keyframe-rot",
         "--max-depth",
         "--max-rot",
         "--max-trans",
         "--min-depth",
         "--nid-bins",
         "--of",
         "--output",
         "--overwrite",
         "--parent",
         "--pcd",
         "--pose",
         "--ref",
         "--samples",
         "--skip-end",
         "--skip-start",
         "--threads",
         "--voxel",
         "-i",
         "-j",
         "-o",
         "-w"}),
      current);
  }

  if (request.cursor_word > 0) {
    const auto & previous = request.words[request.cursor_word - 1];
    // --cam-offset takes a duration (free text) or the word `auto`; only the
    // word can be offered.
    if (previous == "--cam-offset") {
      return matching({"auto"}, current);
    }
    if (previous == "--parent" || previous == "--child") {
      const auto bag_arg = find_input_bag(request);
      if (!bag_arg) {
        return {};
      }
      return complete_frame_id_value(*bag_arg, current, /*static_only=*/true);
    }
    if (previous == "--of" || previous == "--ref") {
      const auto bag_arg = find_input_bag(request);
      if (!bag_arg) {
        return {};
      }
      return complete_frame_id_value(*bag_arg, current);
    }
  }
  return {};
}

// `topic` is a command group with three action verbs, `drop`, `keep`, and
// `rename`. At the action slot (word 1) the candidates are those verbs.
// `drop`/`keep`'s -t/--topics (every value slot) and `rename`'s --src are
// declared topic slots, so try_topic_completion handles their values;
// `rename`'s --dst names a new topic and offers nothing (a declared literal
// slot with a reject_reason). Here we surface each verb's own flags for any
// `-` word.
//
//   drop:   `topic`(0) `drop`(1)   -i|--input <bag> -t|--topics <selector>...
//           [-o <out>] [-w|--overwrite]
//   keep:   `topic`(0) `keep`(1)   -i|--input <bag> -t|--topics <selector>...
//           [-o <out>] [-w|--overwrite]
//   rename: `topic`(0) `rename`(1) -i|--input <bag> --src <topic>
//           --dst <topic> [-o <out>] [-w|--overwrite]
std::vector<std::string> complete_topic(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"drop", "keep", "rename"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & verb = request.words[kFirstCommandArgWord];
    if (verb == "drop" || verb == "keep") {
      return matching(
        with_help({"--input", "--output", "--overwrite", "--topics", "-i", "-o", "-t", "-w"}),
        current);
    }
    if (verb == "rename") {
      return matching(
        with_help({"--dst", "--input", "--output", "--overwrite", "--src", "-i", "-o", "-w"}),
        current);
    }
  }
  return {};
}

// `stamp` is a command group with one action verb, `sync`. At the action slot
// (word 1) the sole candidate is `sync`; past it we surface `sync`'s own flags
// for any `-` word. The `-i`/`-o` values are bag paths, so they carry no bagwiz
// candidates and fall through to the shell's own file completion.
//
//   sync: `stamp`(0) `sync`(1) -i|--input <bag> [-o <out>] [-w|--overwrite]
std::vector<std::string> complete_stamp(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"sync"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & verb = request.words[kFirstCommandArgWord];
    if (verb == "sync") {
      return matching(with_help({"--input", "--output", "--overwrite", "-i", "-o", "-w"}), current);
    }
    return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
  }
  return {};
}

// The -t/--topic values typed so far on a `generate video cam` command line
// (both spellings collect into one list), used to complete the <image_topic>
// left half of --pcd / --cam-info pair values. Glob values are dropped: a
// pair's left half must name one already-resolved view topic, which a glob
// string never does.
std::vector<std::string_view> collect_video_cam_image_topics(const CompletionRequest & request)
{
  auto values = collect_flag_values(request, "-t");
  const auto long_values = collect_flag_values(request, "--topic");
  values.insert(values.end(), long_values.begin(), long_values.end());
  std::erase_if(values, [](std::string_view v) { return v.find('*') != std::string_view::npos; });
  return values;
}

// Where the cursor sits within a possible "<lhs>=<rhs>" pair value: the
// "<lhs>=" prefix to re-attach to right-half candidates (empty when the
// cursor is not on a right half) and the typed right-half prefix to filter
// them with. The current word carries '=' when the shell keeps the value
// unsplit (zsh/fish); a bare '=' as the previous word is bash's split form.
struct PairRhsContext
{
  std::string lhs_prefix;
  std::string rhs_prefix;
};

PairRhsContext pair_rhs_context(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (const auto eq = current.find('='); eq != std::string::npos) {
    return PairRhsContext{current.substr(0, eq + 1), current.substr(eq + 1)};
  }
  if (
    request.cursor_word >= 2 && request.cursor_word <= request.words.size() &&
    request.words[request.cursor_word - 1] == "=") {
    return PairRhsContext{
      std::string{request.words[request.cursor_word - 2]} + "=", std::string{current}};
  }
  return PairRhsContext{{}, std::string{current}};
}

// Value completion for generate video cam's deferred pair slots (see
// completion_defers_to_command). A bare value completes topics of
// `rhs_types`; a pair value completes the -t topics on the left half ('='
// appended, so the shell drops the auto-space) and topics of `rhs_types` on
// the right half.
std::vector<std::string> complete_video_cam_pair_value(
  const CompletionRequest & request, std::span<const std::string_view> rhs_types)
{
  const auto input_path = find_input_bag(request);
  if (!input_path) {
    return {};
  }
  const auto rhs = pair_rhs_context(request);
  if (!rhs.lhs_prefix.empty()) {
    auto candidates =
      complete_topics(std::filesystem::path{*input_path}, rhs.rhs_prefix, rhs_types);
    for (auto & candidate : candidates) {
      candidate = rhs.lhs_prefix + candidate;
    }
    return candidates;
  }
  const auto current = current_word(request);
  auto candidates = complete_topics(std::filesystem::path{*input_path}, current, rhs_types);
  for (const auto & topic : collect_video_cam_image_topics(request)) {
    if (topic.starts_with(current)) {
      candidates.push_back(std::string{topic} + "=");
    }
  }
  std::sort(candidates.begin(), candidates.end());
  return candidates;
}

// `generate` is a command group for producing media from a rosbag; its sole
// subcommand is `video`, itself a nested command group whose only leaf is
// `cam` (like `tf static`). At the subcommand slot (word 1) the only candidate
// is `video`; at the leaf slot (word 2, under `video`) the only candidate is
// `cam`. `cam`'s `-t/--topic` (image topics) is a declared topic slot, so
// try_topic_completion handles its values before this function is reached;
// `--pcd` and `--cam-info` are pair-valued slots whose completion defers here
// (see completion_defers_to_command). Here we surface `cam`'s own flags for
// any `-` word, and the enum choices for `--field` and `--scheme`.
//
//   cam: `generate`(0) `video`(1) `cam`(2) -i|--input <bag>
//        -t|--topic <image_topic>... -o|--output <path> [--grid <cols>x<rows>]
//        [--cam-info <topic>|<image>=<info>] [--rectify] [--resize <s>]
//        [--width <px>]
//        [--pcd <topic>|<image>=<topic>...] [--field <f>]
//        [--min <v>] [--max <v>] [--scheme <s>] [--point-size <n>]
//        [--alpha <a>] [-w|--overwrite]
std::vector<std::string> complete_generate(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"video"}, current);
  }

  // Reaching here implies cursor_word > kFirstCommandArgWord, so words[1]
  // exists (parse_request clamps cursor_word to words.size()).
  const auto & group = request.words[kFirstCommandArgWord];
  if (group != "video") {
    return {};
  }

  if (request.cursor_word == kSecondCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"cam"}, current);
  }

  if (request.cursor_word >= kThirdCommandArgWord && current.starts_with("-")) {
    if (request.words[kSecondCommandArgWord] == "cam") {
      return matching(
        with_help({"--alpha",      "--cam-info", "--field",  "--grid",      "--input",
                   "--max",        "--min",      "--output", "--overwrite", "--pcd",
                   "--point-size", "--rectify",  "--resize", "--scheme",    "--topic",
                   "--width",      "-i",         "-o",       "-t",          "-w"}),
        current);
    }
  }

  if (
    request.cursor_word >= kThirdCommandArgWord && request.words[kSecondCommandArgWord] == "cam") {
    // The deferred pair slots' values: --pcd takes PointCloud2 topics (bare or
    // <image>=<pcd>), --cam-info CameraInfo topics (bare or <image>=<info>).
    if (is_value_slot_of(request, "--pcd")) {
      return complete_video_cam_pair_value(request, kPointCloud2Type);
    }
    if (is_value_slot_of(request, "--cam-info")) {
      return complete_video_cam_pair_value(request, kCameraInfoType);
    }
  }

  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--field") {
    return matching({"distance", "intensity", "x", "y", "z"}, current);
  }
  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--scheme") {
    return matching({"inferno", "jet", "magma", "plasma", "rainbow", "turbo", "viridis"}, current);
  }
  return {};
}

// `map` is a command group for map generation and post-processing. Its action
// verbs are `slam` and `viewer`. The verb adds one positional slot, shifting
// every argument one word to the right of a flat command.
//
//   slam:   `map`(0) `slam`(1) -i|--input <bag> --pcd <topic> -o|--output <root>
//           [--backend <cpu|cuda|auto>] [--frame <frame_id>] [--imu <topic>]
//           [--gnss <topic>] [--color <topic>...]
//           [--cam-info <image>=<info>...]
//           [--color-min-dist <m>] [--color-keyframe-blur]
//           [--input-res <m>] [--min-range <m>] [--max-range <m>]
//           [-j|--threads <N>] [--viewer] [-w|--overwrite]
//           [--no-progress] [--no-warmup-fill] [--no-cooldown-fill]
//           [--no-color-propagate] [--fill-min-inliers <f>] [--submap-keyframes <N>]
//           [--remove-outliers] [--outlier-r <m>] [--outlier-k <N>]
//           [--remove-dynamic] [--dynamic-method <dufomap|erasor2>]
//           [--dynamic-res <m>] [--dynamic-ds <m>] [--dynamic-dp <N>]
//           [--dynamic-sensor-height <m>] [--upsample <rate>]
//   viewer: `map`(0) `viewer`(1) -i|--input <map>
//
// At the action slot (word 1) the candidates are `slam` and `viewer` (or the
// help flags for a `-` word). Past it, `--pcd` (PointCloud2 topics), `--imu`
// (Imu topics), `--gnss` (NavSatFix topics), `--color` (image topics), and
// `--cam-info` (the `<image_topic>` half of each `<image>=<info>` pair,
// completed as "<topic>="; the `<info_topic>` half offers nothing) are all
// declared topic slots, so try_topic_completion handles their values before
// this function is reached. Here we surface `slam`'s flags for any `-` word
// and complete `--frame` (frame ids from the bag's static TF, not a topic
// slot). `viewer` has no value-bearing flags and its `--input` value is a
// path.

std::vector<std::string> complete_map(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"slam", "viewer"}, current);
  }

  // Reaching here implies cursor_word >= kSecondCommandArgWord, so words[1] exists.
  const auto & verb = request.words[kFirstCommandArgWord];
  if (verb == "viewer") {
    if (current.starts_with("-")) {
      return matching(with_help({"--input", "-i"}), current);
    }
    return {};
  }
  // Only `slam` has flags or a bag to complete from.
  if (verb != "slam") {
    return {};
  }

  if (current.starts_with("-")) {
    return matching(
      with_help(
        {"--backend",
         "--cam-info",
         "--color",
         "--color-keyframe-blur",
         "--color-min-dist",
         "--dynamic-dp",
         "--dynamic-ds",
         "--dynamic-method",
         "--dynamic-res",
         "--dynamic-sensor-height",
         "--fill-min-inliers",
         "--frame",
         "--gnss",
         "--imu",
         "--input",
         "--input-res",
         "--max-range",
         "--min-range",
         "--no-color-propagate",
         "--no-cooldown-fill",
         "--no-progress",
         "--no-warmup-fill",
         "--outlier-k",
         "--outlier-r",
         "--output",
         "--overwrite",
         "--pcd",
         "--remove-dynamic",
         "--remove-outliers",
         "--submap-keyframes",
         "--threads",
         "--upsample",
         "--viewer",
         "-i",
         "-j",
         "-o",
         "-w"}),
      current);
  }

  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--backend") {
    return matching({"auto", "cpu", "cuda"}, current);
  }

  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--dynamic-method") {
    return matching({"dufomap", "erasor2"}, current);
  }

  // --frame takes a frame id resolved through the bag's static TF, so it
  // completes from the bag's static-TF frame ids, like `tf static calc`.
  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--frame") {
    const auto bag_arg = find_input_bag(request);
    if (!bag_arg) {
      return {};
    }
    return complete_frame_id_value(*bag_arg, current, /*static_only=*/true);
  }

  return {};
}

// `ls -i|--input <bag>` lists topics. Its only flag is `-l/--long` (per-topic
// COUNT and HZ); <input> is a path that falls through to the shell's file
// completion. We surface `-i`/`--input` and `-l`/`--long` plus the implicit help
// flags for any `-` word.
//
//   ls: `ls`(0) -i|--input <bag> [-l|--long]
std::vector<std::string> complete_ls(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return matching(with_help({"--input", "--long", "-i", "-l"}), current);
  }
  return {};
}

// `trim -i|--input <bag>` copies only the messages inside a time window. All
// its flags are surfaced for any `-` word; <input> is a path that falls through
// to the shell's file completion. `--align`'s and `--keep`'s values are
// declared topic slots (any type, every value in their run — both are
// variadic), so try_topic_completion handles them before this function is
// reached; `--stamp` completes its two clock choices.
//
//   trim: `trim`(0) -i|--input <bag>
//         {[--start <off>] [--end <off>|--duration <len>] | --both <off> |
//          --align <topics>...} [--keep <topics>...] [--stamp header|recv]
//         [-o <out>] [-w]
std::vector<std::string> complete_trim(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return matching(
      with_help(
        {"--align", "--both", "--duration", "--end", "--input", "--keep", "--output", "--overwrite",
         "--stamp", "--start", "-i", "-o", "-w"}),
      current);
  }

  if (request.cursor_word > 0 && request.words[request.cursor_word - 1] == "--stamp") {
    return matching({"header", "recv"}, current);
  }

  return {};
}

// `walk -i|--input <bag> -t|--topic <topic>` walks a single topic's messages.
// Both `-t/--topic` (every topic in the bag) and `--cam-info` (CameraInfo
// topics only) are declared topic slots, so try_topic_completion handles their
// values before this function is reached. Here we surface walk's own flags
// (plus the implicit help flags) for any `-` word.
//
//   walk: `walk`(0) -i|--input <bag> -t|--topic <topic> [--cam-info <topic>]
std::vector<std::string> complete_walk(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (current.starts_with("-")) {
    return matching(with_help({"--cam-info", "--input", "--topic", "-i", "-t"}), current);
  }
  return {};
}

// `cam-info` is a command group for sensor_msgs/msg/CameraInfo operations. Its
// subcommands are `replace`, `recompute-p`, and `dump`. At the subcommand slot
// (word 1) those are the candidates (or the implicit help flags for a `-`
// word).
//
// All three subcommands' -t/--topic(s) are declared topic slots, so
// try_topic_completion handles their values and nothing in this function
// completes a topic value.
//
// <input> and <calib> are paths that fall through to the shell's file
// completion. `--frame-id`'s value is a free-form header override with nothing
// to suggest, `-o`/`--output`'s is an output path, and `-a`/`--alpha`'s is a
// free number in [0, 1], so none of those get value completion.
//
//   replace:     `cam-info`(0) `replace`(1) -i|--input <bag> --yaml <yaml>
//                -t|--topics <topic>... [--frame-id <id>] [-o <out>] [-w|--overwrite]
//   recompute-p: `cam-info`(0) `recompute-p`(1) -i|--input <bag>
//                [-t|--topics <topic>...] [-a|--alpha <a>] [-o <out>] [-w|--overwrite]
//   dump:        `cam-info`(0) `dump`(1) -i|--input <bag> -t|--topic <topic>
//                [-o <out>] [-w|--overwrite]
std::vector<std::string> complete_cam_info(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"replace", "recompute-p", "dump"}, current);
  }

  if (request.words.size() <= kFirstCommandArgWord) {
    return {};
  }
  const auto & sub = request.words[kFirstCommandArgWord];

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    if (sub == "replace") {
      return matching(
        with_help(
          {"--frame-id", "--input", "--output", "--overwrite", "--topics", "--yaml", "-i", "-o",
           "-t", "-w"}),
        current);
    }
    if (sub == "recompute-p") {
      return matching(
        with_help(
          {"--alpha", "--input", "--output", "--overwrite", "--topics", "-a", "-i", "-o", "-t",
           "-w"}),
        current);
    }
    if (sub == "dump") {
      return matching(
        with_help({"--input", "--output", "--overwrite", "--topic", "-i", "-o", "-t", "-w"}),
        current);
    }
    return {};
  }

  return {};
}

// `pcd` is a command group for PointCloud2 topic processing. Its subcommands are
// `concat` and `undistort`. At the subcommand slot (word 1) the candidates are
// those two (or the implicit help flags for a `-` word). `-i`/`--input` names a
// path that falls through to the shell's file completion. Past the subcommand we
// surface each subcommand's own flags for any `-` word.
//
// For `concat`, `--as` names a new topic to create — a declared
// literal slot with a reject_reason, so try_topic_completion leaves it alone
// and it offers nothing here either. `--pcd` is a declared topic slot
// (PointCloud2 topics), so try_topic_completion handles its values. Only
// `--stamp-offset` stays bespoke below: it is scoped to `--pcd` rather than
// carrying its own allowed_types, a relationship try_topic_completion does
// not resolve, so its `<topic>` half is completed here (as `<topic>=`) at
// every value in its run, until the cursor moves past `=` onto the `<value>`
// half. Its candidates mirror the command's resolution scope: once `--pcd`
// values are on the line, only the PointCloud2 topics those selectors match
// are offered; before any `--pcd` value, all of the bag's PointCloud2
// topics. `--frame`, `--tolerance`, and `-o`/`--output` take free-form /
// numeric / path values, so they get no value completion.
//
//   concat: `pcd`(0) `concat`(1) -i|--input <bag> --as <output_topic>
//           --pcd <t...> [--frame <f>] [--tolerance <val>]
//           [--stamp-offset <t=v>...]... [-o <out>] [--drop-inputs] [--force]
//           [-j|--threads <N>] [-w|--overwrite]
//
// For `undistort`, `--pose`, `--twist`, and `--pcd` are all declared topic
// slots (pose: TFMessage / Odometry / PoseStamped / PoseWithCovarianceStamped;
// twist: Twist / TwistStamped / TwistWithCovarianceStamped; pcd: PointCloud2 —
// the accepted sets validate_undistort_topics enforces), so
// try_topic_completion handles their values. `--ref`/`--of` complete the
// bag's TF frame ids, mirroring `traj dump`/`join`; they are not topic slots
// (a frame id is not a topic), so they stay here. `-o`/`--output` takes a
// path, `-j`/`--threads` takes a count, and `--max-extrap-duration` takes a
// free-form duration, so they get no value completion.
//
//   undistort: `pcd`(0) `undistort`(1) -i|--input <bag> (--pose|--twist) <topic>
//              --pcd <t...> [--ref <frame>] [--of <frame>] [-o <out>]
//              [-j|--threads <N>] [-w|--overwrite] [--no-extrap]
//              [--max-extrap-duration <dur>]
std::vector<std::string> complete_pcd(const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.cursor_word == kFirstCommandArgWord) {
    if (current.starts_with("-")) {
      return matching({kCommonHelpFlags.begin(), kCommonHelpFlags.end()}, current);
    }
    return matching({"concat", "undistort"}, current);
  }

  if (request.cursor_word >= kSecondCommandArgWord && current.starts_with("-")) {
    const auto & sub = request.words[kFirstCommandArgWord];
    if (sub == "concat") {
      return matching(
        with_help(
          {"--as", "--drop-inputs", "--force", "--frame", "--input", "--output", "--overwrite",
           "--pcd", "--stamp-offset", "--threads", "--tolerance", "-i", "-j", "-o", "-w"}),
        current);
    }
    if (sub == "undistort") {
      return matching(
        with_help(
          {"--compression", "--compression-level", "--input", "--max-extrap-duration",
           "--no-extrap", "--of", "--output", "--overwrite", "--pcd", "--pose", "--ref",
           "--threads", "--twist", "-i", "-j", "-o", "-w"}),
        current);
    }
  }

  // Fixed value sets of `undistort`'s compression flags (the --storage idiom).
  if (
    request.cursor_word > kFirstCommandArgWord &&
    request.words[kFirstCommandArgWord] == "undistort") {
    const auto & prev = request.words[request.cursor_word - 1];
    if (prev == "--compression") {
      return matching({"lz4", "none", "zstd"}, current);
    }
    if (prev == "--compression-level") {
      return matching({"default", "fast", "fastest", "slow", "slowest"}, current);
    }
  }

  // --stamp-offset consumes one or more <topic>=<value> values per occurrence
  // (the CLI option is a vector with no arity limit, like --pcd), so complete
  // the <topic> half for every value in its run, not just the word immediately
  // after the flag. Walk back from the cursor over the values already given
  // (bash splits a typed value at '=', and the resulting `topic`/`=`/`value`
  // fragments are all skipped here as non-option words); if the nearest option
  // word is --stamp-offset, the cursor is still consuming its values. Each
  // candidate carries a trailing '=' so the shell scripts drop the auto-space
  // and leave the cursor on the value.
  //
  // The <value> half (a duration) has nothing to suggest. zsh/fish keep a
  // typed topic=value unsplit, so a value in progress shows up as a current
  // word already containing '='; bash splits it, leaving a bare '=' as the
  // word right before the cursor. Both cases return no candidates.
  if (request.cursor_word > kFirstCommandArgWord && current.find('=') == std::string::npos) {
    const bool after_equals = request.words[request.cursor_word - 1] == "=";
    for (std::size_t w = request.cursor_word; w > kFirstCommandArgWord;) {
      const auto & word = request.words[--w];
      if (!word.starts_with("-")) {
        continue;  // a value already given to --stamp-offset; keep scanning
      }
      if (word == "--stamp-offset") {
        if (after_equals) {
          return {};  // on the <value> half (bash split at '='); nothing to offer
        }
        const auto bag_arg = find_flag_value(request, kInputFlags);
        if (bag_arg && !bag_arg->empty() && !bag_arg->starts_with("-")) {
          auto topics = complete_topics(expand_current_user_home(*bag_arg), "", kPointCloud2Type);
          // The command resolves --stamp-offset's <topic> half against the
          // resolved --pcd list (TopicSlotSpec::scope in configure_concat),
          // not the whole bag, so once --pcd values are on the line the
          // candidates narrow to the topics those selectors match. With no
          // --pcd value typed yet there is nothing to scope to, and the
          // bag-wide PointCloud2 list stays as the fallback.
          const auto selectors = collect_flag_values(request, "--pcd");
          if (!selectors.empty()) {
            std::vector<std::string> scoped;
            for (const auto & topic : topics) {
              const auto matches = [&topic](const std::string_view & selector) {
                // A selector without '*' is an exact topic-name match.
                return core::topic_glob_match(selector, topic);
              };
              if (std::any_of(selectors.begin(), selectors.end(), matches)) {
                scoped.push_back(topic);
              }
            }
            topics = std::move(scoped);
          }
          std::vector<std::string> result;
          for (const auto & topic : topics) {
            if (starts_with(topic, current)) {
              result.push_back(topic + '=');
            }
          }
          return result;  // stays sorted: complete_topics sorts, filtering keeps order
        }
      }
      break;  // the nearest option decides; a non-stamp-offset option ends the run
    }
  }

  // `--pcd` (both subcommands), `--pose`, and `--twist` are declared topic
  // slots, so try_topic_completion handles their values before this function
  // is reached. undistort's --of/--ref complete the bag's TF frame ids,
  // mirroring `traj dump`/`join`; they are not topic slots (a frame id is not
  // a topic), so they stay here.
  if (request.cursor_word > 0) {
    const auto & previous = request.words[request.cursor_word - 1];
    if (previous == "--of" || previous == "--ref") {
      const auto bag_arg = find_input_bag(request);
      if (!bag_arg) {
        return {};
      }
      return complete_frame_id_value(*bag_arg, current);
    }
  }
  return {};
}

std::vector<std::string> complete_request(const CLI::App & app, const CompletionRequest & request)
{
  const auto current = current_word(request);
  if (request.words.empty() || request.cursor_word == kTopLevelCommandWord) {
    return top_level_candidates(current);
  }

  if (auto topic_candidates = try_topic_completion(app, request)) {
    return std::move(*topic_candidates);
  }

  const auto & command = request.words.front();
  if (command == "complete") {
    return complete_complete_command(request);
  }
  if (command == "convert") {
    return complete_convert(request);
  }
  if (command == "traj") {
    return complete_traj(request);
  }
  if (command == "tf") {
    return complete_tf(request);
  }
  if (command == "topic") {
    return complete_topic(request);
  }
  if (command == "stamp") {
    return complete_stamp(request);
  }
  if (command == "generate") {
    return complete_generate(request);
  }
  if (command == "map") {
    return complete_map(request);
  }
  if (command == "calib") {
    return complete_calib(request);
  }
  if (command == "cam-info") {
    return complete_cam_info(request);
  }
  if (command == "pcd") {
    return complete_pcd(request);
  }
  if (command == "walk") {
    return complete_walk(request);
  }
  if (command == "ls") {
    return complete_ls(request);
  }
  if (command == "trim") {
    return complete_trim(request);
  }
  return {};
}

void print_candidates(const std::vector<std::string> & candidates)
{
  for (const auto & candidate : candidates) {
    std::cout << candidate << '\n';
  }
}

const char * bash_completion_script()
{
  return R"BWCOMP(# bash completion for bagwiz.
# Install with:
#   bagwiz complete bash > ~/.local/share/bash-completion/completions/bagwiz

_bagwiz_completion()
{
  local cur out
  cur="${COMP_WORDS[COMP_CWORD]}"

  if ! out="$(bagwiz __complete "$COMP_CWORD" "${COMP_WORDS[@]}" 2>/dev/null)"; then
    return 0
  fi

  if [[ -z "${out}" ]]; then
    return 0
  fi

  local IFS=$'\n'
  COMPREPLY=($(compgen -W "${out}" -- "${cur}"))

  # `<topic>=` candidates (e.g. `pcd concat --stamp-offset <topic>=<value>`) must
  # not receive the default trailing space, so the value can be typed right after
  # the `=`. Suppress it only when every candidate ends with `=`.
  if [[ ${#COMPREPLY[@]} -gt 0 ]]; then
    local __bw_all_eq=1 __bw_c
    for __bw_c in "${COMPREPLY[@]}"; do
      [[ "${__bw_c}" == *= ]] || { __bw_all_eq=0; break; }
    done
    [[ ${__bw_all_eq} -eq 1 ]] && compopt -o nospace
  fi
}

complete -o default -F _bagwiz_completion bagwiz
)BWCOMP";
}

const char * zsh_completion_script()
{
  return R"BWCOMP(#compdef bagwiz
# zsh completion for bagwiz.
# Install with:
#   mkdir -p ~/.zsh/completions
#   bagwiz complete zsh > ~/.zsh/completions/_bagwiz
# Then ensure ~/.zsh/completions is in $fpath before `compinit` in ~/.zshrc:
#   fpath=(~/.zsh/completions $fpath)
#   autoload -Uz compinit && compinit

_bagwiz()
{
  local -a candidates
  local out

  if ! out="$(bagwiz __complete $((CURRENT - 1)) "${words[@]}" 2>/dev/null)"; then
    _files
    return 0
  fi

  if [[ -z "${out}" ]]; then
    _files
    return 0
  fi

  local IFS=$'\n'
  candidates=(${(f)out})

  if (( ${#candidates} == 0 )); then
    _files
    return 0
  fi

  # `<topic>=` candidates (e.g. `pcd concat --stamp-offset <topic>=<value>`) must
  # keep the cursor on the value, so add them with an empty suffix (no trailing
  # space) instead of via _describe. Only when every candidate ends with `=`.
  local __bw_all_eq=1 __bw_c
  for __bw_c in "${candidates[@]}"; do
    [[ "${__bw_c}" == *= ]] || { __bw_all_eq=0; break; }
  done
  if (( __bw_all_eq )); then
    compadd -S '' -- "${candidates[@]}" && return 0
    _files
    return 0
  fi

  _describe -t bagwiz 'bagwiz' candidates && return 0
  _files
}

compdef _bagwiz bagwiz

if [ "${funcstack[1]}" = "_bagwiz" ]; then
  _bagwiz "$@"
fi
)BWCOMP";
}

const char * fish_completion_script()
{
  return R"BWCOMP(# fish completion for bagwiz.
# Install with:
#   bagwiz complete fish > ~/.config/fish/completions/bagwiz.fish

function __bagwiz_complete
    set -l tokens (commandline -opc)
    set -l current (commandline -ct)
    set -l cursor (count $tokens)
    bagwiz __complete $cursor $tokens $current 2>/dev/null
end

function __bagwiz_no_candidates
    set -l result (__bagwiz_complete)
    test -z "$result"
end

# Show bagwiz-supplied candidates when the helper returns any; otherwise fall
# back to the shell's default file completion (matches bash's `complete -o
# default` behavior).
complete -c bagwiz -f -a '(__bagwiz_complete)'
complete -c bagwiz -F -n __bagwiz_no_candidates
)BWCOMP";
}

const char * completion_script(CompletionShell shell)
{
  switch (shell) {
    case CompletionShell::Bash:
      return bash_completion_script();
    case CompletionShell::Zsh:
      return zsh_completion_script();
    case CompletionShell::Fish:
      return fish_completion_script();
  }
  return "";
}

// The command that loads `target` into the current shell session. All three
// shells autoload the script from the standard completion directory on the next
// startup, so the only thing needed to activate it immediately is to source the
// file we just wrote.
std::string activate_command(CompletionShell shell, const std::filesystem::path & target)
{
  switch (shell) {
    case CompletionShell::Bash:
    case CompletionShell::Zsh:
    case CompletionShell::Fish:
      return "source " + target.string();
  }
  return {};
}

class CompleteCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "complete"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Generate shell completion scripts";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("--shell", shell_, "Shell to generate completions for")
      ->required()
      ->check(CLI::IsMember(supported_shell_name_strings()));
    app.add_flag(
      "--install", install_,
      "Write the script to the shell's standard completion directory instead of stdout");
    app.add_flag(
      "-w,--overwrite", overwrite_, "Overwrite an existing file when used with --install");
  }

  int run() override
  {
    const auto shell = parse_shell(shell_);
    if (!shell) {
      BAGWIZ_LOG_ERROR(kLogger, "unsupported shell: %s", shell_.c_str());
      return 1;
    }

    if (!install_) {
      std::cout << completion_script(*shell);
      return 0;
    }

    const auto target = install_path_for(*shell);
    if (!target) {
      BAGWIZ_LOG_ERROR(kLogger, "cannot determine install path: HOME is not set");
      return 1;
    }

    if (!write_script_to(*target, completion_script(*shell), overwrite_)) {
      return 1;
    }
    std::cout << "installed: " << target->string() << '\n'
              << "Completion will be active in new terminal sessions.\n"
              << "To enable it in the current shell now, run:\n"
              << "  " << activate_command(*shell, *target) << '\n';
    return 0;
  }

private:
  std::string shell_;
  bool install_ = false;
  bool overwrite_ = false;
};

}  // namespace

std::vector<std::string> supported_shells()
{
  return supported_shell_name_strings();
}

std::optional<std::string> completion_script_for(const std::string_view & shell)
{
  const auto parsed = parse_shell(shell);
  if (!parsed) {
    return std::nullopt;
  }
  return std::string{completion_script(*parsed)};
}

std::optional<std::filesystem::path> default_install_path_for(const std::string_view & shell)
{
  const auto parsed = parse_shell(shell);
  if (!parsed) {
    return std::nullopt;
  }
  return install_path_for(*parsed);
}

std::optional<std::string> activate_command_for(
  const std::string_view & shell, const std::filesystem::path & target)
{
  const auto parsed = parse_shell(shell);
  if (!parsed) {
    return std::nullopt;
  }
  return activate_command(*parsed, target);
}

bool install_completion_script(
  const std::string_view & shell, const std::filesystem::path & target, bool overwrite)
{
  const auto parsed = parse_shell(shell);
  if (!parsed) {
    BAGWIZ_LOG_ERROR(kLogger, "unsupported shell: %s", std::string(shell).c_str());
    return false;
  }
  return write_script_to(target, completion_script(*parsed), overwrite);
}

bool is_completion_request(int argc, char * const * argv)
{
  if (argc < kMinimumCompletionProbeArgc) {
    return false;
  }
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const char * const command = argv[kCompletionCommandArg];
  return command != nullptr && std::string_view{command} == kCompletionCommand;
}

int run_completion_request(int argc, char * const * argv, const CLI::App & app)
{
  const auto request = parse_request(argc, argv);
  print_candidates(complete_request(app, request));
  return 0;
}

BAGWIZ_REGISTER_COMMAND(CompleteCommand)

}  // namespace bagwiz::commands
