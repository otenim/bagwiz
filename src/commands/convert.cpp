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
#include "bagwiz/core/msg_definition_resolver.hpp"
#include "bagwiz/core/ros1_message_definitions.hpp"
#include "bagwiz/core/ros1_meta_synthesizer.hpp"
#include "bagwiz/core/ros1_to_cdr.hpp"
#include "bagwiz/core/schema_resolver.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"
#include "bagwiz/io/rosbag1_reader.hpp"
#include "bagwiz/io/rosbag1_writer.hpp"

#include <algorithm>
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
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.convert";

// Resolve the target storage backend for a write. The CLI takes `--storage`
// optionally; when omitted, we fall back to inferring from the output path's
// extension (`.mcap` / `.db3`). If neither is conclusive — typically a
// directory output without an explicit flag — we surface a clear error
// instead of silently picking a default, since the user has not actually
// chosen one. `storage_flag` is the CLI string value (empty when the user
// did not pass `--storage`); the returned format is never `Format::Auto`.
io::Format resolve_target_storage(
  const std::string & storage_flag, const std::filesystem::path & output_path,
  std::string & error_out)
{
  if (!storage_flag.empty()) {
    return (storage_flag == "sqlite3") ? io::Format::Sqlite3 : io::Format::Mcap;
  }
  const auto inferred = io::infer_format_from_extension(output_path);
  if (inferred != io::Format::Auto) {
    return inferred;
  }
  error_out = "cannot determine target storage from output path '" + output_path.string() +
              "'; pass --storage <mcap|sqlite3> or use a .mcap/.db3 extension";
  return io::Format::Auto;
}

struct PerConn
{
  std::string topic;
  std::string ros1_type;
  std::string ros2_type;  // empty if topic is being skipped
  bool keep = false;      // false → drop messages on this conn
  uint64_t written = 0;
  uint64_t failures = 0;
};

// Reason a topic was excluded from 1to2 output. Topic-level errors are
// the granularity at which project decision 8/B operates: any of these
// causes the affected topic to be dropped, the run to exit non-zero, and
// the topic to appear in the end-of-run summary.
enum class TopicSkipReason : int {
  // Source ROS 1 type does not parse as `pkg/Type` and has no rename
  // override entry. Catches typos and accidental ROS 2-shaped names in
  // ROS 1 inputs.
  TypeNameInvalid,
  // No source (bag-embedded / AMENT / introspection) could produce a
  // ROS 2 .msg text. Usually means the matching ROS 2 distro is not
  // sourced.
  SchemaUnresolvable,
  // ROS 2 schema text resolved, but `synthesize_ros1_meta()` refused
  // to canonicalise it (project decision 10/B — wstring or other ROS
  // 1-incompatible construct).
  CanonicalisationRefused,
  // The MD5 we computed from the ROS 2 schema does not match the bag's
  // ROS 1 connection md5sum. The default "silent corruption guard" path
  // (project decision 3/D); upgraded/downgraded by --strict /
  // --allow-md5-mismatch in a later commit.
  Md5Mismatch,
  // Writer rejected the topic declaration (storage backend error).
  WriterDeclareFailed,
};

const char * topic_skip_reason_label(TopicSkipReason r)
{
  switch (r) {
    case TopicSkipReason::TypeNameInvalid:
      return "invalid ROS 1 type name";
    case TopicSkipReason::SchemaUnresolvable:
      return "ROS 2 schema not resolvable from any source";
    case TopicSkipReason::CanonicalisationRefused:
      return "ROS 2 schema cannot be canonicalised to ROS 1 form";
    case TopicSkipReason::Md5Mismatch:
      return "ROS 1 md5sum does not match ROS 2 schema";
    case TopicSkipReason::WriterDeclareFailed:
      return "writer rejected topic declaration";
  }
  return "unknown";
}

// Aggregated record emitted at end-of-run so users see exactly which
// topics were dropped and why. Carries the reason category plus a
// human-readable detail string (md5 hex pair, dlerror text, etc.).
struct SkippedTopic
{
  std::string topic;
  std::string ros1_type;
  std::string ros2_type;
  TopicSkipReason reason;
  std::string detail;
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
    std::string storage;  // empty when --storage not passed; resolved at run time
  } r1_to_r2_args_;

  struct TwoToOneArgs
  {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
  } r2_to_r1_args_;

  struct StorageArgs
  {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    std::string storage;  // empty when --storage not passed; resolved at run time
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
    sub
      ->add_option(
        "-s,--storage", r1_to_r2_args_.storage,
        "Output storage backend (default: inferred from output extension)")
      ->check(CLI::IsMember({"mcap", "sqlite3"}));
    sub->footer(
      "Only standard message types from the built-in whitelist are converted.\n"
      "Topics with unsupported types are skipped with a warning.\n"
      "If --storage is omitted, the backend is inferred from the output path's\n"
      "extension (.mcap or .db3); other paths (e.g. a directory) require --storage.");
    sub->callback([this]() { selected_ = Subcommand::k1to2; });
  }

  int run_1to2()
  {
    const auto & args = r1_to_r2_args_;

    std::string err;
    const io::Format target_format = resolve_target_storage(args.storage, args.output_path, err);
    if (target_format == io::Format::Auto) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err.c_str());
      return 1;
    }

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
    copts.format = target_format;
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

    // Track schema resolution outcomes so we can summarise at the end of
    // the run. Without this, an MCAP output that silently lost
    // self-description for half its topics would surface only as a
    // Foxglove "schema encoding '' is not supported" error downstream.
    std::size_t resolved_defs = 0;
    std::vector<SkippedTopic> skipped_topics;

    // Categorise + record one topic-level skip. Caller is expected to
    // also flip `pc.keep = false` so the message loop stops admitting
    // payloads for the affected conn_id.
    const auto record_skip = [&skipped_topics](
                               const PerConn & pc, TopicSkipReason reason, std::string detail) {
      SkippedTopic s;
      s.topic = pc.topic;
      s.ros1_type = pc.ros1_type;
      s.ros2_type = pc.ros2_type;
      s.reason = reason;
      s.detail = std::move(detail);
      BAGWIZ_LOG_WARN(
        kLogger, "Skipping topic '%s' (%s -> %s): %s%s%s", s.topic.c_str(), s.ros1_type.c_str(),
        s.ros2_type.empty() ? "?" : s.ros2_type.c_str(), topic_skip_reason_label(reason),
        s.detail.empty() ? "" : " — ", s.detail.c_str());
      skipped_topics.push_back(std::move(s));
    };

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
        record_skip(pc, TopicSkipReason::TypeNameInvalid, "");
        pc.keep = false;
        return &per_conn.emplace(conn_id, std::move(pc)).first->second;
      }
      pc.ros2_type = *mapped;

      // Same topic may appear under multiple conn_ids in ROS 1 bags
      // (e.g. one publisher per chunk). Declare the topic once with
      // BagWriter; subsequent writes to the same topic are accepted.
      if (!declared_topics.contains(pc.topic)) {
        // Resolve the ROS 2 schema. ROS 1 bags have no embedded ROS 2
        // schema, so the bag-embedded path is unused; AMENT and
        // introspection are the candidate sources.
        core::ResolveSchemaInput resolve_in;
        resolve_in.ros2_type = pc.ros2_type;
        const auto resolved = core::resolve_schema(resolve_in);
        if (!resolved.ok) {
          // None of the three sources produced a schema. Most failure
          // detail lives on the AMENT / introspection candidates; pull
          // them into one short string for the summary.
          std::string detail;
          for (const auto & c : resolved.candidates) {
            if (!c.error.empty()) {
              if (!detail.empty()) {
                detail += "; ";
              }
              detail += c.error;
            }
          }
          record_skip(pc, TopicSkipReason::SchemaUnresolvable, std::move(detail));
          pc.keep = false;
          return &per_conn.emplace(conn_id, std::move(pc)).first->second;
        }

        // Synthesise the canonical ROS 1 form to compare md5 against
        // the bag-recorded md5sum (project decision 3/D + 4/C). The
        // canonicalisation can refuse outright (e.g. wstring per
        // decision 10/B) — surface that as a topic-level skip.
        const auto meta = core::synthesize_ros1_meta(pc.ros2_type, resolved.text);
        if (!meta.ok) {
          record_skip(pc, TopicSkipReason::CanonicalisationRefused, meta.error);
          pc.keep = false;
          return &per_conn.emplace(conn_id, std::move(pc)).first->second;
        }

        if (meta.meta.md5sum != src->md5sum) {
          // Silent-corruption guard: producer and our schema disagree
          // on the wire shape. Default per decision 3/D is topic-skip
          // + non-zero exit; --strict / --allow-md5-mismatch (commit
          // #7) will let users reshape this policy.
          std::string detail = "bag md5=" + src->md5sum + " synthesised=" + meta.meta.md5sum +
                               " (source=" + std::string(core::to_string(resolved.source)) + ")";
          record_skip(pc, TopicSkipReason::Md5Mismatch, std::move(detail));
          pc.keep = false;
          return &per_conn.emplace(conn_id, std::move(pc)).first->second;
        }

        io::TopicInfo t;
        t.name = pc.topic;
        t.type = pc.ros2_type;
        t.serialization_format = "cdr";
        t.offered_qos_profiles = "";  // ROS 1 has no equivalent
        t.schema_text = resolved.text;
        t.schema_encoding = resolved.encoding;
        ++resolved_defs;

        try {
          writer->declare_topic(t);
          declared_topics.insert(pc.topic);
          BAGWIZ_LOG_INFO(
            kLogger, "Mapped '%s': %s -> %s [md5 ok via %s]", pc.topic.c_str(),
            pc.ros1_type.c_str(), pc.ros2_type.c_str(),
            std::string(core::to_string(resolved.source)).c_str());
        } catch (const std::exception & e) {
          record_skip(pc, TopicSkipReason::WriterDeclareFailed, e.what());
          pc.keep = false;
          return &per_conn.emplace(conn_id, std::move(pc)).first->second;
        }
      }

      pc.keep = !pc.ros2_type.empty();
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
    if (resolved_defs > 0) {
      BAGWIZ_LOG_INFO(
        kLogger, "resolved %zu message definition(s) with md5 verified against bag", resolved_defs);
    }
    for (const auto & [conn_id, pc] : per_conn) {
      if (pc.keep && pc.failures > 0) {
        BAGWIZ_LOG_WARN(
          kLogger, "Topic '%s': %" PRIu64 " message(s) failed to convert", pc.topic.c_str(),
          pc.failures);
      }
    }

    // Topic-skip summary (project decision 3/D + 8/B). Exit code
    // reflects whether any topic was dropped: zero only when every
    // topic in the input was emitted.
    if (!skipped_topics.empty()) {
      BAGWIZ_LOG_WARN(kLogger, "Skipped %zu topic(s):", skipped_topics.size());
      // Show at most the first 5 in detail to keep the trailing summary
      // tractable on bags with many divergent topics; tally the rest by
      // reason category so users still see scope.
      constexpr std::size_t kMaxDetailRows = 5;
      const std::size_t shown = std::min(kMaxDetailRows, skipped_topics.size());
      for (std::size_t i = 0; i < shown; ++i) {
        const auto & s = skipped_topics[i];
        BAGWIZ_LOG_WARN(
          kLogger, "  - '%s' (%s -> %s): %s%s%s", s.topic.c_str(), s.ros1_type.c_str(),
          s.ros2_type.empty() ? "?" : s.ros2_type.c_str(), topic_skip_reason_label(s.reason),
          s.detail.empty() ? "" : " — ", s.detail.c_str());
      }
      if (skipped_topics.size() > kMaxDetailRows) {
        BAGWIZ_LOG_WARN(
          kLogger, "  (... %zu more skipped topic(s) not shown)",
          skipped_topics.size() - kMaxDetailRows);
      }
      return 2;  // non-zero exit so callers can detect partial conversion
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
    sub
      ->add_option(
        "output", storage_args_.output_path, "Output rosbag2 directory (or .mcap/.db3 file)")
      ->required();
    sub
      ->add_option(
        "-s,--storage", storage_args_.storage,
        "Target storage backend (default: inferred from output extension)")
      ->check(CLI::IsMember({"mcap", "sqlite3"}));
    sub->footer(
      "Messages are copied verbatim — only the storage backend changes; no\n"
      "deserialization or type conversion is performed.\n"
      "If --storage is omitted, the backend is inferred from the output path's\n"
      "extension (.mcap or .db3); other paths (e.g. a directory) require --storage.\n"
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

    std::string err;
    const io::Format target_format = resolve_target_storage(args.storage, args.output_path, err);
    if (target_format == io::Format::Auto) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err.c_str());
      return 1;
    }

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    // Reject same-storage repack: it's almost always a user mistake (and a
    // plain copy is what they actually want). Detection is by magic bytes
    // (single-file inputs) or metadata.yaml (directory layouts) — never
    // by extension — so renamed files are still classified correctly.
    const auto source_format = io::detect_format(args.input_path);
    if (source_format == target_format) {
      const char * fmt_name = (target_format == io::Format::Sqlite3) ? "sqlite3" : "mcap";
      BAGWIZ_LOG_ERROR(kLogger, "input is already in '%s' storage; nothing to convert", fmt_name);
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

    // Force schema bytes onto the topic list before declaring so MCAP
    // outputs preserve self-description across a repack (one-shot shard
    // open for multi-shard MCAP inputs; no-op for single-file MCAP and
    // SQLite3 where schemas are either already loaded or not embedded).
    reader->populate_schemas();

    // SQLite3 storage in Humble (and earlier) does not embed message
    // definitions, so reader->topics() comes back with empty
    // schema_text. Resolve each missing definition from
    // $AMENT_PREFIX_PATH/share/<pkg>/msg/<Type>.msg before declaring
    // the topic — otherwise the resulting MCAP loses self-description
    // and breaks strict downstream readers like rosbags-convert.
    std::size_t declared = 0;
    std::size_t resolved_defs = 0;
    std::size_t unresolved_defs = 0;
    for (const auto & t : reader->topics()) {
      io::TopicInfo augmented = t;
      if (augmented.schema_text.empty()) {
        auto resolved = core::resolve_message_definition(augmented.type);
        if (!resolved.text.empty()) {
          augmented.schema_text = std::move(resolved.text);
          augmented.schema_encoding = std::move(resolved.encoding);
          ++resolved_defs;
        } else {
          ++unresolved_defs;
          if (unresolved_defs <= 5) {
            BAGWIZ_LOG_WARN(
              kLogger,
              "no .msg on disk for type '%s' (topic '%s'); writing MCAP without "
              "self-description for this topic",
              augmented.type.c_str(), augmented.name.c_str());
          }
        }
      }
      try {
        writer->declare_topic(augmented);
        ++declared;
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(
          kLogger, "declare_topic failed for '%s': %s; skipping topic", t.name.c_str(), e.what());
      }
    }
    if (resolved_defs > 0) {
      BAGWIZ_LOG_INFO(
        kLogger, "resolved %zu missing message definition(s) from $AMENT_PREFIX_PATH",
        resolved_defs);
    }
    if (unresolved_defs > 5) {
      BAGWIZ_LOG_WARN(
        kLogger, "(plus %zu more topic(s) without resolvable .msg)", unresolved_defs - 5);
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
