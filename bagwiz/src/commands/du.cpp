// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/du.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.du";

// Minimum widths so the header never looks cramped for a bag of tiny topics.
// Actual widths are computed from the data so long sizes / topic names do not
// push later columns out of alignment.
constexpr int kMinSizeWidth = 4;   // "SIZE"
constexpr int kMinTopicWidth = 5;  // "TOPIC"

// Raw byte count, or a 1024-based human-readable rendering in the style of
// `du -h`: values below 1 KiB stay raw bytes, everything above prints one
// decimal and a K/M/G/T/P/E suffix ("4.0K", "1.2M").
std::string format_size(std::uint64_t bytes, bool human)
{
  if (!human) {
    return fmt::format("{}", bytes);
  }
  constexpr std::array<char, 6> kSuffixes{'K', 'M', 'G', 'T', 'P', 'E'};
  double value = static_cast<double>(bytes);
  std::size_t idx = 0;
  while (value >= 1024.0 && idx < kSuffixes.size()) {
    value /= 1024.0;
    ++idx;
  }
  if (idx == 0) {
    return fmt::format("{}", bytes);
  }
  return fmt::format("{:.1f}{}", value, kSuffixes[idx - 1]);
}

struct Row
{
  std::string topic;
  std::uint64_t bytes = 0;
};

}  // namespace

int run_du(const DuArgs & args)
{
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }

  // Restrict the scan when the caller selected topics. The presence check
  // backstops the CLI's require_present slot spec (du has no presence check
  // of its own otherwise): run_du is also called directly from tests, and a
  // name that matches no real topic must error rather than silently report
  // an empty listing.
  std::vector<std::string> selected;
  if (!args.topics.empty()) {
    for (const auto & name : args.topics) {
      const auto found = std::find_if(
        reader->topics().begin(), reader->topics().end(),
        [&name](const auto & t) { return t.name == name; });
      if (found == reader->topics().end()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "du: topic '%s' is not present in %s.", name.c_str(), args.input_path.c_str());
        return 1;
      }
    }
    selected = args.topics;
    io::ReadFilter filter;
    filter.topics = args.topics;
    reader->set_filter(filter);
  } else {
    for (const auto & t : reader->topics()) {
      selected.push_back(t.name);
    }
  }

  // Full-message scan: every topic's size is the sum of its serialized
  // payload bytes. There is no summary shortcut — neither the MCAP summary
  // nor metadata.yaml records per-topic byte totals.
  std::unordered_map<std::string, std::uint64_t> sizes;
  io::RawMessage msg;
  try {
    while (reader->next(msg)) {
      sizes[msg.topic->name] += msg.payload.size();
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed while scanning %s: %s", args.input_path.c_str(), e.what());
    return 1;
  }

  std::vector<Row> rows;
  rows.reserve(selected.size());
  std::uint64_t total = 0;
  for (const auto & name : selected) {
    const std::uint64_t bytes = sizes.count(name) != 0 ? sizes.at(name) : 0;
    rows.push_back({name, bytes});
    total += bytes;
  }

  // Size descending — du's whole point is finding what is big — with topic
  // name ascending as the tie-breaker so the output stays stable.
  std::sort(rows.begin(), rows.end(), [](const Row & a, const Row & b) {
    return a.bytes != b.bytes ? a.bytes > b.bytes : a.topic < b.topic;
  });

  int size_w = kMinSizeWidth;
  int topic_w = kMinTopicWidth;
  for (const auto & row : rows) {
    size_w = std::max(size_w, static_cast<int>(format_size(row.bytes, args.human).size()));
    topic_w = std::max(topic_w, static_cast<int>(row.topic.size()));
  }
  size_w = std::max(size_w, static_cast<int>(format_size(total, args.human).size()));
  topic_w = std::max(topic_w, 5);  // "total"

  fmt::print(stdout, "{:>{}} {:<{}}\n", "SIZE", size_w, "TOPIC", topic_w);
  for (const auto & row : rows) {
    fmt::print(
      stdout, "{:>{}} {:<{}}\n", format_size(row.bytes, args.human), size_w, row.topic, topic_w);
  }
  fmt::print(stdout, "{:>{}} {:<{}}\n", format_size(total, args.human), size_w, "total", topic_w);
  return 0;
}

// `bagwiz du -i <input>` reports each topic's total serialized payload size,
// in the spirit of du(1): a size column first, the topic name after it, rows
// sorted by size descending, and a closing `total` row. The size is the sum
// of the uncompressed serialized payload bytes (the logical message size),
// not the on-disk footprint — per-topic chunk compression makes the latter
// unrecoverable — so a compressed bag's reported total can exceed its file
// size. Computing it requires a full scan of the bag's messages, on every
// storage format.
class DuCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "du"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Report per-topic serialized payload sizes in a rosbag";
  }

  void configure(CLI::App & app) override
  {
    // Drop -h from the help flag so it can carry du(1)'s meaning instead:
    // human-readable sizes. --help keeps working.
    app.set_help_flag("--help", "Show this help message and exit");
    app.add_option("-i,--input", args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    set_topic_input(app, args_.input_path);
    add_topic_option(
      app, "-t,--topics", args_.topics,
      "Topic selector(s) to report. A literal topic name or a '*' glob. Repeat for several. "
      "Omit to report every topic.",
      TopicSlotSpec{.require_present = true});
    app.add_flag(
      "-h,--human", args_.human,
      "Print sizes in human-readable units (1024-based, e.g. 4.0K, 1.2M) instead of raw bytes.");
  }

  int run() override { return run_du(args_); }

private:
  DuArgs args_;
};

BAGWIZ_REGISTER_COMMAND(DuCommand)

}  // namespace bagwiz::commands
