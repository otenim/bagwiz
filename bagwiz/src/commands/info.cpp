// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/info.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/str_utils.hpp"
#include "bagwiz/io/bag_describe.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "format_units.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <fmt/core.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.info";

// Key column width: "Compression:" is the longest key, so every value starts
// in the same column.
constexpr int kKeyWidth = 13;

// A field the bag's own summary does not state. Nothing is scanned to fill
// it in, so the text says where the answer would have had to come from.
constexpr const char * kUnknownSummary = "unknown (no summary in the bag)";
constexpr const char * kUnknown = "unknown";
// A field that has no value for this bag (the extent of an empty bag).
constexpr const char * kNotApplicable = "-";

// Minimum widths of the `-l` table's columns, so a short listing's header is
// not cramped. Actual widths grow with the data.
constexpr int kMinSizeWidth = 4;      // "SIZE"
constexpr int kMinMessagesWidth = 8;  // "MESSAGES"
constexpr int kMinStartWidth = 5;     // "START"
constexpr int kMinDurationWidth = 8;  // "DURATION"

std::string layout_text(io::Layout layout)
{
  switch (layout) {
    case io::Layout::Directory:
      return "directory";
    case io::Layout::SingleFile:
      return "single-file";
    case io::Layout::Auto:
      break;
  }
  return kUnknown;
}

std::string storage_text(io::Format format)
{
  switch (format) {
    case io::Format::Mcap:
      return "mcap";
    case io::Format::Sqlite3:
      return "sqlite3";
    case io::Format::Auto:
      break;
  }
  return kUnknown;
}

// "zstd", or "lz4+zstd" for an MCAP whose chunks mix codecs.
std::string join_codecs(const std::vector<std::string> & codecs)
{
  std::string joined;
  for (const auto & codec : codecs) {
    if (!joined.empty()) {
      joined += '+';
    }
    joined += codec;
  }
  return joined;
}

// `<codec> <what>`, or just `<what>` when the declaration named no codec.
std::string with_codec(const std::vector<std::string> & codecs, std::string_view what)
{
  if (codecs.empty()) {
    return std::string(what);
  }
  return join_codecs(codecs) + " " + std::string(what);
}

// The vocabulary of `bagwiz compress`: MCAP chunk compression with the ratio
// its chunk index states, or the rosbag2 mode a metadata.yaml declares.
std::string compression_text(const io::BagCompression & compression)
{
  if (compression.mode == "none") {
    return "none";
  }
  if (compression.mode == "chunk") {
    std::string text = with_codec(compression.codecs, "chunks");
    if (
      compression.compressed_bytes.has_value() && compression.uncompressed_bytes.has_value() &&
      *compression.compressed_bytes > 0) {
      const double ratio = static_cast<double>(*compression.uncompressed_bytes) /
                           static_cast<double>(*compression.compressed_bytes);
      text += fmt::format(" ({:.1f}x)", ratio);
    }
    return text;
  }
  if (compression.mode == "message") {
    return with_codec(compression.codecs, "per-message (rosbag2 MESSAGE mode)");
  }
  if (compression.mode == "file") {
    return with_codec(compression.codecs, "whole-file envelope (rosbag2 FILE mode)");
  }
  return kUnknownSummary;
}

std::string metadata_text(const io::BagDescription & description)
{
  if (description.layout != io::Layout::Directory) {
    return "none (single file)";
  }
  if (!description.has_metadata_yaml) {
    return "metadata.yaml missing (reconstructed from the directory)";
  }
  std::string text = "metadata.yaml";
  if (description.metadata_version.has_value()) {
    text += fmt::format(" version {}", *description.metadata_version);
  }
  if (!description.ros_distro.empty()) {
    text += fmt::format(", ros_distro {}", description.ros_distro);
  }
  return text;
}

std::string topics_text(const io::BagDescription & description)
{
  if (!description.topics.has_value()) {
    return kUnknown;
  }
  std::set<std::string> types;
  for (const auto & topic : *description.topics) {
    types.insert(topic.type);
  }
  return fmt::format(
    "{} ({} {})", description.topics->size(), types.size(), types.size() == 1 ? "type" : "types");
}

std::string count_text(std::optional<std::int64_t> count)
{
  return count.has_value() ? fmt::format("{}", *count) : std::string(kUnknownSummary);
}

// Start, end and duration follow one rule: rendered when the summary states
// the extent, "-" for a bag known to hold no messages, unknown otherwise.
struct ExtentText
{
  std::string start;
  std::string end;
  std::string duration;
};

ExtentText extent_text(
  std::optional<std::int64_t> count, std::optional<std::int64_t> start_ns,
  std::optional<std::int64_t> end_ns, std::string_view unknown)
{
  if (start_ns.has_value() && end_ns.has_value()) {
    return {
      core::format_timestamp(*start_ns), core::format_timestamp(*end_ns),
      format_duration(*end_ns - *start_ns)};
  }
  if (count.has_value() && *count == 0) {
    return {kNotApplicable, kNotApplicable, kNotApplicable};
  }
  return {std::string(unknown), std::string(unknown), std::string(unknown)};
}

void print_summary(const io::BagDescription & description, bool human_sizes)
{
  const auto extent =
    extent_text(description.message_count, description.start_ns, description.end_ns, kUnknown);
  const std::vector<std::pair<std::string, std::string>> rows = {
    {"Path", description.path.string()},
    {"Layout", layout_text(description.layout)},
    {"Storage", storage_text(description.format)},
    {"Compression", compression_text(description.compression)},
    {"Files", fmt::format("{}", description.files.size())},
    {"Size", format_size(description.size_bytes, human_sizes)},
    {"Metadata", metadata_text(description)},
    {"Topics", topics_text(description)},
    {"Messages", count_text(description.message_count)},
    {"Start", extent.start},
    {"End", extent.end},
    {"Duration", extent.duration},
  };
  for (const auto & row : rows) {
    fmt::print(stdout, "{:<{}} {}\n", row.first + ":", kKeyWidth, row.second);
  }
}

struct FileRow
{
  std::string size;
  std::string messages;
  std::string start;
  std::string duration;
  std::string path;
};

// One row per storage file, after a blank line: what each shard holds, so a
// split recording can be read shard by shard. A value the shard's summary
// does not state prints "-".
void print_files(const io::BagDescription & description, bool human_sizes)
{
  std::vector<FileRow> rows;
  rows.reserve(description.files.size());
  for (const auto & file : description.files) {
    const auto extent = extent_text(file.message_count, file.start_ns, file.end_ns, kNotApplicable);
    rows.push_back({
      file.size_bytes.has_value() ? format_size(*file.size_bytes, human_sizes)
                                  : std::string(kNotApplicable),
      file.message_count.has_value() ? fmt::format("{}", *file.message_count)
                                     : std::string(kNotApplicable),
      extent.start,
      extent.duration,
      file.path.string(),
    });
  }

  int size_w = kMinSizeWidth;
  int messages_w = kMinMessagesWidth;
  int start_w = kMinStartWidth;
  int duration_w = kMinDurationWidth;
  for (const auto & row : rows) {
    size_w = std::max(size_w, static_cast<int>(row.size.size()));
    messages_w = std::max(messages_w, static_cast<int>(row.messages.size()));
    start_w = std::max(start_w, static_cast<int>(row.start.size()));
    duration_w = std::max(duration_w, static_cast<int>(row.duration.size()));
  }

  fmt::print(
    stdout, "\n{:>{}} {:>{}} {:<{}} {:>{}} {}\n", "SIZE", size_w, "MESSAGES", messages_w, "START",
    start_w, "DURATION", duration_w, "PATH");
  for (const auto & row : rows) {
    fmt::print(
      stdout, "{:>{}} {:>{}} {:<{}} {:>{}} {}\n", row.size, size_w, row.messages, messages_w,
      row.start, start_w, row.duration, duration_w, row.path);
  }
}

}  // namespace

int run_info(const InfoArgs & args)
{
  io::BagDescription description;
  try {
    description = io::describe_bag(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
    return 1;
  }

  print_summary(description, !args.bytes);
  if (args.long_listing) {
    print_files(description, !args.bytes);
  }
  return 0;
}

// `bagwiz info -i <input>` prints a rosbag's bag-level metadata as one
// `Key: value` line per field: where it is and how it is laid out (layout,
// storage backend, compression, file count, on-disk size, metadata.yaml),
// and what it holds (topic count, message count, start, end, duration).
// Everything comes from the bag's own summaries — metadata.yaml, the MCAP
// summary section, a .db3's `metadata` row and timestamp index — and from
// stat(2), so the command answers in the time it takes to open the files.
// A bag that does not summarise a field reports it as unknown rather than
// scanning its messages for it. Per-topic detail belongs to `ls` and `du`.
class InfoCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "info"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Show bag-level metadata of a rosbag";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("-i,--input", args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    app.add_flag(
      "-l,--long", args_.long_listing,
      "Also list every storage file (shard) of the bag with its size, message count, start "
      "time and duration.");
    app.add_flag(
      "-b,--bytes", args_.bytes,
      "Print sizes as raw byte counts instead of human-readable units (the default, e.g. "
      "4.0K, 1.2M).");
  }

  int run() override { return run_info(args_); }

private:
  InfoArgs args_;
};

BAGWIZ_REGISTER_COMMAND(InfoCommand)

}  // namespace bagwiz::commands
