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
#include "bagwiz/core/msg_definition_resolver.hpp"
#include "bagwiz/core/msg_schema/parser.hpp"
#include "bagwiz/core/ros2_yaml_to_cdr.hpp"
#include "bagwiz/core/schema_resolver.hpp"
#include "bagwiz/core/yaml_msg_stamp_sync.hpp"
#include "bagwiz/core/yaml_msg_validate.hpp"
#include "bagwiz/io/atomic_replace.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <fmt/core.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.join";

void ascii_trim_inplace(std::string & s)
{
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
    s.erase(s.begin());
  }
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) {
    s.pop_back();
  }
}

void ascii_tolower_inplace(std::string & s)
{
  for (char & c : s) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
}

std::optional<int64_t> parse_join_stamp_ns(
  std::string_view raw, bool bag_has_messages, int64_t start_ns, int64_t end_ns)
{
  std::string s(raw);
  ascii_trim_inplace(s);
  ascii_tolower_inplace(s);
  if (s == "head") {
    return bag_has_messages ? start_ns : 0LL;
  }
  if (s == "tail") {
    return bag_has_messages ? end_ns : 0LL;
  }

  errno = 0;
  char * end = nullptr;
  const double sec = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || errno == ERANGE) {
    return std::nullopt;
  }
  while (*end == ' ' || *end == '\t') {
    ++end;
  }
  if (*end != '\0') {
    return std::nullopt;
  }
  constexpr double max_abs = static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9;
  constexpr double min_abs = -static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9;
  if (sec > max_abs || sec < min_abs) {
    return std::nullopt;
  }
  return static_cast<int64_t>(sec * 1e9);
}

// Choose a sibling path next to `target` that does not yet exist, so the
// in-place flow can stage the rewritten bag before swapping it over the
// original. Uses pid + steady_clock nanos to avoid collisions across
// concurrent invocations on the same input.
std::filesystem::path make_staging_path(const std::filesystem::path & target)
{
  std::filesystem::path parent = target.parent_path();
  if (parent.empty()) {
    parent = std::filesystem::current_path();
  }
  const auto stem = target.filename().string();
  const auto pid = static_cast<int64_t>(::getpid());
  for (int attempt = 0; attempt < 16; ++attempt) {
    const auto ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count();
    auto leaf = fmt::format(".{}.bagwiz-staged-{}-{}-{}", stem, pid, ts, attempt);
    auto candidate = parent / leaf;
    std::error_code ec;
    if (!std::filesystem::exists(candidate, ec)) {
      return candidate;
    }
  }
  throw std::runtime_error(
    "join: could not allocate a unique staging path next to " + target.string());
}

// RAII guard that removes the staged path if the in-place swap never
// happens. Clear `path` to disarm after a successful swap.
struct StagedCleanup
{
  std::filesystem::path path;

  StagedCleanup() = default;
  StagedCleanup(const StagedCleanup &) = delete;
  StagedCleanup & operator=(const StagedCleanup &) = delete;
  StagedCleanup(StagedCleanup &&) = delete;
  StagedCleanup & operator=(StagedCleanup &&) = delete;

  ~StagedCleanup()
  {
    if (path.empty()) {
      return;
    }
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
};

}  // namespace

class JoinCommand : public Command
{
public:
  std::string_view name() const override { return "join"; }
  std::string_view description() const override
  {
    return "Insert a YAML-encoded message into an existing rosbag (in place by default, "
           "or into a copy when -o is given)";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("input", input_path_, "Source bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    app
      .add_option(
        "-o,--output", output_path_,
        "Destination bag path. When given, <input> is left untouched and the modified bag is "
        "written here (must not exist yet). When omitted, <input> is edited in place "
        "(written to a sibling temp path, then atomically swapped over <input>).")
      ->check([](const std::string & p) -> std::string {
        if (std::filesystem::exists(std::filesystem::path(p))) {
          return std::string("output path already exists: ") + p;
        }
        return {};
      });
    app
      .add_option(
        "topic", topic_,
        "Topic name. If absent from the input bag, pass -t <ros2_type> to create it.")
      ->required();
    app.add_option("msg", msg_path_, "YAML file describing the inserted message")
      ->required()
      ->check(CLI::ExistingPath);
    app
      .add_option(
        "at", stamp_arg_,
        "Receive-time at: head | tail | or POSIX epoch seconds (optionally fractional)")
      ->required();
    app.add_option(
      "-t,--type", type_arg_,
      "ROS 2 message type (e.g. std_msgs/msg/String) used to create <topic> when it is not "
      "already present in the input bag. Ignored when the topic already exists, except that a "
      "mismatch with the bag's recorded type is reported.");
    app.add_flag(
      "--sync-msg-stamp", sync_msg_stamp_,
      "Sync top-level header.stamp in YAML to the resolved receive-time <at>");
  }

  int run() override
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(input_path_);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", input_path_.c_str(), e.what());
      return 1;
    }

    const io::TopicInfo * topic_info = nullptr;
    for (const auto & t : reader->topics()) {
      if (t.name == topic_) {
        topic_info = &t;
        break;
      }
    }

    // When the topic does not exist in the input bag, create it on the fly
    // from -t/--type. The synthetic TopicInfo carries only the bare minimum
    // (name + type + cdr); the schema text is filled below by the same
    // resolver chain used for existing topics, and the writer's declare_topic
    // copies are also routed through that chain.
    io::TopicInfo synthetic_topic;
    const bool created_topic = (topic_info == nullptr);
    if (created_topic) {
      if (type_arg_.empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "topic '%s' is not present in bag %s; pass -t <ros2_type> to create it (e.g. "
          "-t std_msgs/msg/String)",
          topic_.c_str(), input_path_.c_str());
        return 1;
      }
      synthetic_topic.name = topic_;
      synthetic_topic.type = type_arg_;
      synthetic_topic.serialization_format = "cdr";
      topic_info = &synthetic_topic;
    } else if (!type_arg_.empty() && type_arg_ != topic_info->type) {
      BAGWIZ_LOG_WARN(
        kLogger, "ignoring -t '%s': topic '%s' already exists in bag with type '%s'",
        type_arg_.c_str(), topic_.c_str(), topic_info->type.c_str());
    }

    reader->populate_schemas();

    std::string schema_text;
    core::ResolveSchemaInput rs_in;
    rs_in.ros2_type = topic_info->type;
    rs_in.bag_embedded_text = topic_info->schema_text;
    rs_in.bag_embedded_encoding = topic_info->schema_encoding;
    const auto rs_out = core::resolve_schema(rs_in);
    if (rs_out.ok && !rs_out.text.empty()) {
      schema_text = rs_out.text;
    }
    if (schema_text.empty()) {
      const auto amd = core::resolve_message_definition(topic_info->type);
      schema_text = amd.text;
      if (!amd.encoding.empty() && amd.encoding != "ros2msg") {
        BAGWIZ_LOG_ERROR(kLogger, "resolved encoding '%s' is not supported", amd.encoding.c_str());
        return 1;
      }
    }
    if (schema_text.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "cannot locate .msg/schema text for type '%s' (populate AMENT_PREFIX_PATH or "
        "embed schema in MCAP)",
        topic_info->type.c_str());
      return 1;
    }

    auto pr = core::msg_schema::parse_schema(topic_info->type, schema_text);
    if (!pr.ok() || !pr.schema.has_value()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "schema parse failed for '%s': %s", topic_info->type.c_str(), pr.error.c_str());
      return 1;
    }
    core::msg_schema::SchemaModel model = std::move(*pr.schema);

    YAML::Node yaml_root;
    try {
      yaml_root = YAML::LoadFile(msg_path_);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "YAML load failed for %s: %s", msg_path_.c_str(), e.what());
      return 1;
    }

    const auto stats = reader->compute_stats();
    const bool has_msgs = stats.total_messages > 0;
    auto insert_ns = parse_join_stamp_ns(stamp_arg_, has_msgs, stats.start_ns, stats.end_ns);
    if (!insert_ns.has_value()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "invalid at %s (use head | tail | epoch seconds)", stamp_arg_.c_str());
      return 1;
    }

    if (sync_msg_stamp_) {
      const auto sync = core::sync_top_level_header_stamp_to_time(model, yaml_root, *insert_ns);
      if (!sync.ok) {
        BAGWIZ_LOG_ERROR(
          kLogger, "failed to sync YAML header.stamp to <at>: %s", sync.error.c_str());
        return 1;
      }
    }

    const auto vz = core::validate_ros2_yaml_for_message_schema(model, yaml_root);
    if (!vz.ok) {
      BAGWIZ_LOG_ERROR(
        kLogger, "YAML is incompatible with type '%s': %s", topic_info->type.c_str(),
        vz.error.c_str());
      return 1;
    }

    const auto ser = core::ros2_yaml_to_cdr_bytes(topic_info->type, model, yaml_root);
    if (!ser.ok) {
      BAGWIZ_LOG_ERROR(
        kLogger, "YAML to CDR serialization failed for '%s': %s", topic_info->type.c_str(),
        ser.error.c_str());
      return 1;
    }

    io::CreateOptions wc;
    wc.format = io::detect_format(input_path_);
    wc.layout = io::Layout::Auto;

    const bool inplace = output_path_.empty();
    StagedCleanup staged;
    std::filesystem::path write_path;
    try {
      write_path = inplace ? make_staging_path(input_path_) : output_path_;
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", e.what());
      return 1;
    }
    if (inplace) {
      staged.path = write_path;
    }

    std::unique_ptr<io::BagWriter> writer;
    try {
      writer = io::open_write(write_path, wc);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "failed to create output bag %s: %s", write_path.c_str(), e.what());
      return 1;
    }

    for (const auto & t : reader->topics()) {
      io::TopicInfo augmented = t;
      if (augmented.schema_text.empty()) {
        const auto amd = core::resolve_message_definition(augmented.type);
        if (!amd.text.empty()) {
          augmented.schema_text = amd.text;
          augmented.schema_encoding = amd.encoding;
        }
      }
      try {
        writer->declare_topic(augmented);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(
          kLogger, "declare_topic failed for '%s': %s; skipping topic entry", t.name.c_str(),
          e.what());
      }
    }

    // The synthetic topic is not in reader->topics(), so it needs its own
    // declare. Schema text reuses the one resolved above for validation /
    // serialization — by this point it is non-empty (an empty schema bails
    // out earlier with a clear error).
    if (created_topic) {
      io::TopicInfo declared = synthetic_topic;
      declared.schema_text = schema_text;
      if (declared.schema_encoding.empty()) {
        declared.schema_encoding = "ros2msg";
      }
      try {
        writer->declare_topic(declared);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(
          kLogger, "declare_topic failed for new topic '%s' (type '%s'): %s", topic_.c_str(),
          type_arg_.c_str(), e.what());
        return 1;
      }
    }

    const std::span<const std::byte> payload(
      reinterpret_cast<const std::byte *>(ser.cdr.data()), ser.cdr.size());

    bool injected = false;
    io::RawMessage msg;
    while (true) {
      bool more = false;
      try {
        more = reader->next(msg);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "read failed: %s", e.what());
        return 1;
      }

      if (more && msg.topic == nullptr) {
        continue;
      }

      const int64_t next_ts = more ? msg.timestamp_ns : std::numeric_limits<int64_t>::max();

      if (!injected && insert_ns.value() <= next_ts) {
        try {
          writer->write(topic_, insert_ns.value(), payload);
          injected = true;
        } catch (const std::exception & e) {
          BAGWIZ_LOG_ERROR(kLogger, "writing inserted message failed: %s", e.what());
          return 1;
        }
      }

      if (!more) {
        break;
      }

      try {
        writer->write(msg.topic->name, msg.timestamp_ns, msg.payload);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(kLogger, "rewrite of '%s' failed: %s", msg.topic->name.c_str(), e.what());
      }
    }

    try {
      writer->close();
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "writer->close failed: %s", e.what());
      return 1;
    }
    writer.reset();
    // Release the input reader before swapping in-place: on some platforms
    // an open handle on a path being replaced is a hazard. The reader holds
    // no further messages we need after the write loop.
    reader.reset();

    if (inplace) {
      try {
        io::atomic_replace(write_path, input_path_);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(
          kLogger, "failed to swap staged bag into place at %s: %s", input_path_.c_str(), e.what());
        return 1;
      }
      staged.path.clear();
      fmt::print(
        stdout, "joined 1 message on '{}'{} -> {} (in place)\n", topic_,
        created_topic ? fmt::format(" (new topic of type {})", type_arg_) : std::string{},
        input_path_.string());
    } else {
      fmt::print(
        stdout, "joined 1 message on '{}'{} -> {}\n", topic_,
        created_topic ? fmt::format(" (new topic of type {})", type_arg_) : std::string{},
        output_path_.string());
    }
    return 0;
  }

private:
  std::filesystem::path input_path_;
  std::filesystem::path output_path_;
  std::string topic_;
  std::string msg_path_;
  std::string stamp_arg_;
  std::string type_arg_;
  bool sync_msg_stamp_ = false;
};

BAGWIZ_REGISTER_COMMAND(JoinCommand)

}  // namespace bagwiz::commands
