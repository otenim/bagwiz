// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/introspection_loader.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/message_deserializer.hpp"
#include "bagwiz/core/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <fmt/core.h>

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.traj";
constexpr const char * kFormatTum = "tum";

// Declarative catalogue of the ROS 2 message types `traj` accepts in
// v1. Every entry is a scalar (one message == one sample); array-shaped
// types (TFMessage, Path, PoseArray) are tracked for v2 and explicitly
// rejected below until that work lands.
struct SupportedType
{
  std::string_view name;
  bool has_header_stamp;  // false => fall back to bag log time
};

constexpr std::array<SupportedType, 6> kSupportedTypes = {{
  {"geometry_msgs/msg/PoseStamped", true},
  {"geometry_msgs/msg/PoseWithCovarianceStamped", true},
  {"geometry_msgs/msg/TransformStamped", true},
  {"nav_msgs/msg/Odometry", true},
  {"geometry_msgs/msg/Pose", false},
  {"geometry_msgs/msg/Transform", false},
}};

const SupportedType * lookup_supported(std::string_view type_name)
{
  for (const auto & t : kSupportedTypes) {
    if (t.name == type_name) {
      return &t;
    }
  }
  return nullptr;
}

// Human-readable block appended to help output and to the "unsupported
// type" error so users can always see what's allowed.
constexpr const char * kSupportedTypesHelp =
  "Supported topic types:\n"
  "  Stamped (header.stamp used as TUM timestamp):\n"
  "    - geometry_msgs/msg/PoseStamped\n"
  "    - geometry_msgs/msg/PoseWithCovarianceStamped\n"
  "    - geometry_msgs/msg/TransformStamped\n"
  "    - nav_msgs/msg/Odometry\n"
  "  Unstamped (bag log time used as TUM timestamp):\n"
  "    - geometry_msgs/msg/Pose\n"
  "    - geometry_msgs/msg/Transform";

}  // namespace

// `bagwiz traj <input> <topic> <output> [-f tum] [--frame-id <id>]`
// extracts a topic's pose trajectory and writes it to disk. v1 accepts
// only the scalar types listed in kSupportedTypes; multi-sample types
// (TFMessage, Path, PoseArray) are tracked for a future PR and rejected
// with a clear error today. `--frame-id` is reserved for that follow-up
// and silently ignored when specified on v1 types.
class TrajCommand : public Command
{
public:
  std::string_view name() const override { return "traj"; }
  std::string_view description() const override
  {
    return "Extract a topic's pose trajectory and save it (currently TUM format)";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("input", input_path_, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    app.add_option("topic", topic_, "Topic name to extract poses from")->required();
    app.add_option("output", output_path_, "Output file path")->required();
    app.add_option("-f,--format", format_, "Output format")
      ->default_val(kFormatTum)
      ->check(CLI::IsMember({kFormatTum}));
    app.add_option(
      "--frame-id", frame_id_,
      "Frame identifier. Reserved for multi-sample types (TFMessage, Path); "
      "ignored for the single-sample types supported today.");
    app.footer(kSupportedTypesHelp);
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
    if (topic_info == nullptr) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is not present in %s", topic_.c_str(), input_path_.c_str());
      return 1;
    }
    const std::string type_name = topic_info->type;

    const SupportedType * spec = lookup_supported(type_name);
    if (spec == nullptr) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' has type '%s', which is not supported by `traj`.\n%s", topic_.c_str(),
        type_name.c_str(), kSupportedTypesHelp);
      return 1;
    }

    io::ReadFilter filter;
    filter.topics.push_back(topic_);
    reader->set_filter(filter);

    const core::IntrospectionLoad introspection = core::load_introspection(type_name);
    if (!introspection.ok()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Message type '%s' could not be loaded (tried %s; error: %s). Source a workspace that "
        "provides the package and re-run.",
        type_name.c_str(), introspection.library_name.c_str(), introspection.error.c_str());
      return 1;
    }

    std::vector<core::TrajectoryPose> poses;
    std::int64_t decoded = 0;
    bool warned_bag_log_time = false;

    io::RawMessage raw;
    while (true) {
      try {
        if (!reader->next(raw)) {
          break;
        }
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "read error: %s", e.what());
        return 1;
      }
      ++decoded;
      try {
        const core::DeserializedMessage msg(introspection, raw.payload);
        const auto extraction = core::extract_pose(msg.members(), msg.data(), raw.timestamp_ns);
        if (!extraction) {
          // Should not be reachable for whitelisted types (the shape
          // check inside extract_pose matches by field names) but keep
          // the fast-fail in case a non-std variant of a supported name
          // slips through.
          BAGWIZ_LOG_ERROR(
            kLogger, "Failed to extract pose from message #%ld of type '%s' (unexpected schema).",
            decoded, type_name.c_str());
          return 1;
        }
        if (!extraction->used_header_stamp && !warned_bag_log_time) {
          BAGWIZ_LOG_WARN(
            kLogger,
            "Type '%s' has no header; using bag log time (recorder receive time) for TUM "
            "timestamps.",
            type_name.c_str());
          warned_bag_log_time = true;
        }
        poses.push_back(extraction->pose);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "decode error on message #%ld: %s", decoded, e.what());
        return 1;
      }
    }

    if (!frame_id_.empty()) {
      // v1 silently ignores --frame-id; leave a debug-level trace so
      // anyone debugging knows it was received but unused.
      BAGWIZ_LOG_DEBUG(
        kLogger, "--frame-id='%s' ignored for scalar type '%s'", frame_id_.c_str(),
        type_name.c_str());
    }

    if (poses.empty()) {
      BAGWIZ_LOG_WARN(
        kLogger, "No messages found on topic '%s'; writing an empty trajectory.", topic_.c_str());
    }

    if (format_ != kFormatTum) {
      BAGWIZ_LOG_ERROR(kLogger, "Unsupported format '%s'", format_.c_str());
      return 1;
    }

    std::ofstream out(output_path_, std::ios::out | std::ios::trunc);
    if (!out) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open output path %s for writing", output_path_.c_str());
      return 1;
    }
    core::write_tum(out, poses);
    out.close();

    BAGWIZ_LOG_INFO(
      kLogger, "Wrote %zu poses (from %ld messages) to %s in %s format", poses.size(), decoded,
      output_path_.c_str(), format_.c_str());
    return 0;
  }

private:
  std::filesystem::path input_path_;
  std::string topic_;
  std::filesystem::path output_path_;
  std::string format_;
  std::string frame_id_;
};

BAGWIZ_REGISTER_COMMAND(TrajCommand)

}  // namespace bagwiz::commands
