// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/cdr_to_ros1.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/ros1_message_definitions.hpp"
#include "bagwiz/core/ros1_to_cdr.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"
#include "bagwiz/io/rosbag1_reader.hpp"
#include "bagwiz/io/rosbag1_writer.hpp"

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

// Per-topic state used by 2to1, indexed by ROS 2 topic name.
struct TwoToOnePerTopic
{
  std::string topic;
  std::string ros2_type;
  std::string ros1_type;
  bool keep = false;
  uint32_t conn_id = 0;
  uint64_t written = 0;
  uint64_t failures = 0;
};

}  // namespace

// `bagwiz convert` is a command group for cross-format bag conversion.
// Ships `1to2` (ROS 1 -> ROS 2), `2to1` (ROS 2 -> ROS 1), and
// `storage` (ROS 2 mcap <-> sqlite3 repack).
class ConvertCommand : public Command
{
public:
  std::string_view name() const override { return "convert"; }
  std::string_view description() const override { return "Convert between bag formats"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_1to2(app);
    configure_2to1(app);
    configure_storage(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::k1to2:
        return run_1to2();
      case Subcommand::k2to1:
        return run_2to1();
      case Subcommand::kStorage:
        return run_storage();
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, k1to2, k2to1, kStorage };
  Subcommand selected_ = Subcommand::kNone;

  struct OneToTwoArgs
  {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    std::string storage = "mcap";
  } r1_to_r2_args_;

  struct TwoToOneArgs
  {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
  } r2_to_r1_args_;

  struct StorageArgs
  {
    std::filesystem::path input_path;
    std::string storage;  // "mcap" or "sqlite3"
    std::filesystem::path output_path;
  } storage_args_;

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

  void configure_2to1(CLI::App & app)
  {
    auto * sub = app.add_subcommand("2to1", "Convert a ROS 2 rosbag to a ROS 1 .bag file");
    sub->add_option("input", r2_to_r1_args_.input_path, "ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("output", r2_to_r1_args_.output_path, "Output ROS 1 .bag file")->required();
    sub->footer(
      "Only standard message types from the built-in whitelist are converted.\n"
      "Topics with unsupported types are skipped with a warning.\n"
      "Output is a non-compressed ROS 1 bag v2.0; rosbag2-layer compression\n"
      "(compression_mode: FILE / MESSAGE) on the input is not supported.");
    sub->callback([this]() { selected_ = Subcommand::k2to1; });
  }

  // Inspect metadata.yaml of a directory-layout input to detect rosbag2's
  // generic compression layer (which we don't decompress). Single-file
  // inputs have no metadata.yaml; mcap chunk-level compression there is
  // handled transparently by libmcap.
  static int check_input_compression(const std::filesystem::path & input)
  {
    std::error_code ec;
    if (!std::filesystem::is_directory(input, ec)) {
      return 0;
    }
    const auto metadata_path = input / "metadata.yaml";
    if (!std::filesystem::exists(metadata_path)) {
      return 0;
    }
    io::BagMetadata md;
    try {
      md = io::load_metadata_yaml(metadata_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_WARN(
        kLogger, "Could not parse metadata.yaml (%s); proceeding without compression check",
        e.what());
      return 0;
    }
    // rosbag2 emits "NONE" or omits the field for non-compressed bags;
    // anything else is a hard fail.
    if (!md.compression_mode.empty() && md.compression_mode != "NONE") {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "input bag uses rosbag2-layer compression (compression_mode='%s', format='%s'); "
        "decompress with `ros2 bag convert` first",
        md.compression_mode.c_str(), md.compression_format.c_str());
      return 1;
    }
    return 0;
  }

  int run_2to1()
  {
    const auto & args = r2_to_r1_args_;

    if (const int rc = check_input_compression(args.input_path); rc != 0) {
      return rc;
    }

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    // Build per-topic state from the reader's topic list. The reader
    // returns TopicInfo entries up front (no message scan needed); we
    // declare connections in the writer eagerly so any
    // unresolvable-type warnings surface before the message loop runs.
    std::unique_ptr<io::Rosbag1Writer> writer;
    try {
      writer = std::make_unique<io::Rosbag1Writer>(args.output_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open output %s: %s", args.output_path.c_str(), e.what());
      return 1;
    }

    std::unordered_map<std::string, TwoToOnePerTopic> per_topic;
    for (const auto & t : reader->topics()) {
      TwoToOnePerTopic state;
      state.topic = t.name;
      state.ros2_type = t.type;

      auto mapped = core::map_ros2_type(t.type);
      if (!mapped) {
        BAGWIZ_LOG_WARN(
          kLogger, "Skipping topic '%s' (type '%s' not in whitelist)", state.topic.c_str(),
          state.ros2_type.c_str());
        per_topic.emplace(state.topic, std::move(state));
        continue;
      }
      state.ros1_type = *mapped;

      const auto * meta = core::find_ros1_meta(state.ros1_type);
      if (meta == nullptr) {
        BAGWIZ_LOG_WARN(
          kLogger, "Skipping topic '%s': no ROS 1 message_definition for '%s' (whitelist mismatch)",
          state.topic.c_str(), state.ros1_type.c_str());
        per_topic.emplace(state.topic, std::move(state));
        continue;
      }

      try {
        state.conn_id = writer->declare_connection(
          state.topic, state.ros1_type, meta->md5sum, meta->message_definition);
        state.keep = true;
        BAGWIZ_LOG_INFO(
          kLogger, "Mapped '%s': %s -> %s", state.topic.c_str(), state.ros2_type.c_str(),
          state.ros1_type.c_str());
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(
          kLogger, "declare_connection failed for '%s': %s; skipping topic", state.topic.c_str(),
          e.what());
      }
      per_topic.emplace(state.topic, std::move(state));
    }

    uint64_t total_in = 0;
    uint64_t total_out = 0;
    io::RawMessage msg;
    while (true) {
      try {
        if (!reader->next(msg)) {
          break;
        }
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "ros2 read error: %s", e.what());
        return 1;
      }
      ++total_in;

      if (msg.topic == nullptr) {
        continue;
      }
      auto it = per_topic.find(msg.topic->name);
      if (it == per_topic.end() || !it->second.keep) {
        continue;
      }
      auto & st = it->second;

      auto result = core::convert_cdr_to_ros1(st.ros2_type, msg.payload);
      if (!result.ok) {
        ++st.failures;
        if (st.failures <= 3) {
          BAGWIZ_LOG_WARN(
            kLogger, "convert failed on '%s' (type %s): %s", st.topic.c_str(), st.ros2_type.c_str(),
            result.error.c_str());
        }
        continue;
      }

      try {
        writer->write(st.conn_id, msg.timestamp_ns, std::span<const std::byte>(result.ros1));
        ++st.written;
        ++total_out;
      } catch (const std::exception & e) {
        ++st.failures;
        if (st.failures <= 3) {
          BAGWIZ_LOG_WARN(
            kLogger, "writer->write failed on '%s': %s; skipping message", st.topic.c_str(),
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

    std::size_t kept_topics = 0;
    for (const auto & entry : per_topic) {
      if (entry.second.keep) {
        ++kept_topics;
      }
    }
    BAGWIZ_LOG_INFO(
      kLogger, "Conversion done: %" PRIu64 "/%" PRIu64 " messages written across %zu topic(s)",
      total_out, total_in, kept_topics);
    for (const auto & entry : per_topic) {
      const auto & st = entry.second;
      if (st.keep && st.failures > 0) {
        BAGWIZ_LOG_WARN(
          kLogger, "Topic '%s': %" PRIu64 " message(s) failed to convert", st.topic.c_str(),
          st.failures);
      }
    }

    return 0;
  }

  void configure_storage(CLI::App & app)
  {
    auto * sub =
      app.add_subcommand("storage", "Repack a ROS 2 rosbag into a different storage backend");
    sub->add_option("input", storage_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("storage", storage_args_.storage, "Target storage backend")
      ->required()
      ->check(CLI::IsMember({"mcap", "sqlite3"}));
    sub
      ->add_option(
        "output", storage_args_.output_path, "Output rosbag2 directory (or .mcap/.db3 file)")
      ->required();
    sub->footer(
      "Messages are copied verbatim — only the storage backend changes; no\n"
      "deserialization or type conversion is performed.\n"
      "Inputs that use rosbag2-layer compression (compression_mode != NONE)\n"
      "are rejected; decompress with `ros2 bag convert` first.");
    sub->callback([this]() { selected_ = Subcommand::kStorage; });
  }

  int run_storage()
  {
    const auto & args = storage_args_;

    if (const int rc = check_input_compression(args.input_path); rc != 0) {
      return rc;
    }

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    const io::Format target_format =
      (args.storage == "sqlite3") ? io::Format::Sqlite3 : io::Format::Mcap;

    // Reject same-storage repack: it's almost always a user mistake (and a
    // plain copy is what they actually want). Detection is by magic bytes
    // (single-file inputs) or metadata.yaml (directory layouts) — never
    // by extension — so renamed files are still classified correctly.
    const auto source_format = io::detect_format(args.input_path);
    if (source_format == target_format) {
      BAGWIZ_LOG_ERROR(
        kLogger, "input is already in '%s' storage; nothing to convert", args.storage.c_str());
      return 1;
    }

    io::CreateOptions copts;
    copts.format = target_format;
    copts.layout = io::Layout::Auto;  // factory picks SingleFile if extension matches
    // Mirror 1to2: leave compression off so the output is predictable;
    // callers can recompress with `ros2 bag convert` if they want.
    copts.mcap_compression = "none";

    std::unique_ptr<io::BagWriter> writer;
    try {
      writer = io::open_write(args.output_path, copts);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open output %s: %s", args.output_path.c_str(), e.what());
      return 1;
    }

    std::size_t declared = 0;
    for (const auto & t : reader->topics()) {
      try {
        writer->declare_topic(t);
        ++declared;
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(
          kLogger, "declare_topic failed for '%s': %s; skipping topic", t.name.c_str(), e.what());
      }
    }

    uint64_t total_in = 0;
    uint64_t total_out = 0;
    uint64_t total_failed = 0;
    io::RawMessage msg;
    while (true) {
      try {
        if (!reader->next(msg)) {
          break;
        }
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "ros2 read error: %s", e.what());
        return 1;
      }
      ++total_in;

      if (msg.topic == nullptr) {
        continue;
      }

      try {
        writer->write(msg.topic->name, msg.timestamp_ns, msg.payload);
        ++total_out;
      } catch (const std::exception & e) {
        ++total_failed;
        if (total_failed <= 3) {
          BAGWIZ_LOG_WARN(
            kLogger, "writer->write failed on '%s': %s; skipping message", msg.topic->name.c_str(),
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
      kLogger, "Repack done: %" PRIu64 "/%" PRIu64 " messages written across %zu topic(s)",
      total_out, total_in, declared);
    if (total_failed > 0) {
      BAGWIZ_LOG_WARN(kLogger, "%" PRIu64 " message(s) failed to write", total_failed);
    }

    return 0;
  }
};

BAGWIZ_REGISTER_COMMAND(ConvertCommand)

}  // namespace bagwiz::commands
