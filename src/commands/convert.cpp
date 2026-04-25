// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/ros1_to_cdr.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/rosbag1_reader.hpp"

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.convert";

struct PerConn
{
  std::string topic;
  std::string ros1_type;
  std::string ros2_type;  // empty if topic is being skipped
  bool keep = false;      // false → drop messages on this conn
  uint64_t written = 0;
  uint64_t failures = 0;
};

}  // namespace

// `bagwiz convert` is a command group for cross-format bag conversion.
// Phase 1 ships only `1to2` (ROS 1 -> ROS 2); the structure leaves
// room for a future `2to1` to slot in alongside.
class ConvertCommand : public Command
{
public:
  std::string_view name() const override { return "convert"; }
  std::string_view description() const override { return "Convert between bag formats"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_1to2(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::k1to2:
        return run_1to2();
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, k1to2 };
  Subcommand selected_ = Subcommand::kNone;

  struct OneToTwoArgs
  {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    std::string storage = "mcap";
  } r1_to_r2_args_;

  void configure_1to2(CLI::App & app)
  {
    auto * sub = app.add_subcommand("1to2", "Convert a ROS 1 .bag to a ROS 2 rosbag");
    sub->add_option("input", r1_to_r2_args_.input_path, "ROS 1 .bag file")
      ->required()
      ->check(CLI::ExistingFile);
    sub
      ->add_option(
        "output", r1_to_r2_args_.output_path, "Output rosbag2 directory (or .mcap/.db3 file)")
      ->required();
    sub->add_option("-s,--storage", r1_to_r2_args_.storage, "Output storage backend")
      ->default_val("mcap")
      ->check(CLI::IsMember({"mcap", "sqlite3"}));
    sub->footer(
      "Only standard message types from the built-in whitelist are converted.\n"
      "Topics with unsupported types are skipped with a warning.");
    sub->callback([this]() { selected_ = Subcommand::k1to2; });
  }

  int run_1to2()
  {
    const auto & args = r1_to_r2_args_;

    std::unique_ptr<io::Rosbag1Reader> reader;
    try {
      reader = std::make_unique<io::Rosbag1Reader>(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    // The reader populates connections lazily as chunks are walked; pull
    // one message just to start the chunk loop so we can enumerate the
    // very first chunk's connections before declaring topics. We then
    // re-process the message after declaring the topic. Simpler
    // alternative: declare topics on-the-fly the first time we see a
    // conn_id we have not seen yet — that's what we do below.

    io::CreateOptions copts;
    copts.format = (args.storage == "sqlite3") ? io::Format::Sqlite3 : io::Format::Mcap;
    copts.layout = io::Layout::Auto;  // factory picks Directory unless path ends in .mcap/.db3
    // Disable mcap chunk compression by default — conversion output is
    // typically a re-record, callers can recompress later if they want.
    copts.mcap_compression = "none";

    std::unique_ptr<io::BagWriter> writer;
    try {
      writer = io::open_write(args.output_path, copts);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open output %s: %s", args.output_path.c_str(), e.what());
      return 1;
    }

    std::unordered_map<uint32_t, PerConn> per_conn;
    std::unordered_set<std::string> declared_topics;

    auto ensure_declared = [&](uint32_t conn_id) -> PerConn * {
      auto it = per_conn.find(conn_id);
      if (it != per_conn.end()) {
        return &it->second;
      }

      // Find the source connection metadata.
      const io::Ros1Connection * src = nullptr;
      for (const auto & c : reader->connections()) {
        if (c.conn_id == conn_id) {
          src = &c;
          break;
        }
      }
      if (src == nullptr) {
        // Should not happen for a well-formed bag; log once and remember
        // as skipped so we don't keep searching.
        BAGWIZ_LOG_WARN(kLogger, "Unknown conn_id %u in message stream; skipping", conn_id);
        PerConn pc;
        pc.keep = false;
        return &per_conn.emplace(conn_id, std::move(pc)).first->second;
      }

      PerConn pc;
      pc.topic = src->topic;
      pc.ros1_type = src->type;

      const auto mapped = core::map_ros1_type(src->type);
      if (!mapped) {
        BAGWIZ_LOG_WARN(
          kLogger, "Skipping topic '%s' (type '%s' not in whitelist)", pc.topic.c_str(),
          pc.ros1_type.c_str());
        pc.keep = false;
      } else {
        pc.ros2_type = *mapped;
        // Same topic may appear under multiple conn_ids in ROS 1 bags
        // (e.g. one publisher per chunk). Declare the topic once with
        // BagWriter; subsequent writes to the same topic are accepted.
        if (!declared_topics.contains(pc.topic)) {
          io::TopicInfo t;
          t.name = pc.topic;
          t.type = pc.ros2_type;
          t.serialization_format = "cdr";
          t.offered_qos_profiles = "";  // ROS 1 has no equivalent
          try {
            writer->declare_topic(t);
            declared_topics.insert(pc.topic);
            BAGWIZ_LOG_INFO(
              kLogger, "Mapped '%s': %s -> %s", pc.topic.c_str(), pc.ros1_type.c_str(),
              pc.ros2_type.c_str());
          } catch (const std::exception & e) {
            BAGWIZ_LOG_WARN(
              kLogger, "declare_topic failed for '%s': %s; skipping topic", pc.topic.c_str(),
              e.what());
            pc.ros2_type.clear();
          }
        }
        pc.keep = !pc.ros2_type.empty();
      }

      return &per_conn.emplace(conn_id, std::move(pc)).first->second;
    };

    uint64_t total_in = 0;
    uint64_t total_out = 0;
    io::Ros1Message msg;
    while (true) {
      try {
        if (!reader->next(msg)) {
          break;
        }
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "ros1 read error: %s", e.what());
        return 1;
      }
      ++total_in;

      auto * pc = ensure_declared(msg.conn_id);
      if (pc == nullptr || !pc->keep) {
        continue;
      }

      auto result = core::convert_ros1_to_cdr(pc->ros2_type, msg.payload);
      if (!result.ok) {
        ++pc->failures;
        if (pc->failures <= 3) {
          BAGWIZ_LOG_WARN(
            kLogger, "convert failed on '%s' (type %s): %s", pc->topic.c_str(),
            pc->ros2_type.c_str(), result.error.c_str());
        }
        continue;
      }

      try {
        writer->write(pc->topic, msg.timestamp_ns, std::span<const std::byte>(result.cdr));
        ++pc->written;
        ++total_out;
      } catch (const std::exception & e) {
        // Per-message write failures are downgraded to a (rate-limited)
        // warning so a single bad message cannot abort the entire bag.
        // Hard storage errors (disk full, etc.) will keep firing here
        // and the user will see the per-topic failure tally at the end.
        ++pc->failures;
        if (pc->failures <= 3) {
          BAGWIZ_LOG_WARN(
            kLogger, "writer->write failed on '%s': %s; skipping message", pc->topic.c_str(),
            e.what());
        }
      }
    }

    try {
      writer->close();
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "writer->close failed: %s", e.what());
      return 1;
    }

    BAGWIZ_LOG_INFO(
      kLogger, "Conversion done: %" PRIu64 "/%" PRIu64 " messages written across %zu topic(s)",
      total_out, total_in, declared_topics.size());
    for (const auto & [conn_id, pc] : per_conn) {
      if (pc.keep && pc.failures > 0) {
        BAGWIZ_LOG_WARN(
          kLogger, "Topic '%s': %" PRIu64 " message(s) failed to convert", pc.topic.c_str(),
          pc.failures);
      }
    }

    return 0;
  }
};

BAGWIZ_REGISTER_COMMAND(ConvertCommand)

}  // namespace bagwiz::commands
