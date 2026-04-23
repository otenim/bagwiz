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

// Declarative catalogue of the ROS 2 message types `traj export` accepts.
// `kind` drives the per-message extraction path: scalar (one message ==
// one sample) vs multi-sample (one message -> zero-or-more samples via
// an array field plus a `--base-frame` selector).
enum class TypeKind {
  kScalarStamped,    // header + pose/transform, single sample
  kScalarUnstamped,  // bare Pose / Transform, timestamp from bag log time
  kTfMessage,        // array of TransformStamped, filter by child_frame_id
  kPath,             // Header + array of PoseStamped, validate header.frame_id
};

struct SupportedType
{
  std::string_view name;
  TypeKind kind;
};

constexpr std::array<SupportedType, 8> kSupportedTypes = {{
  {"geometry_msgs/msg/PoseStamped", TypeKind::kScalarStamped},
  {"geometry_msgs/msg/PoseWithCovarianceStamped", TypeKind::kScalarStamped},
  {"geometry_msgs/msg/TransformStamped", TypeKind::kScalarStamped},
  {"nav_msgs/msg/Odometry", TypeKind::kScalarStamped},
  {"geometry_msgs/msg/Pose", TypeKind::kScalarUnstamped},
  {"geometry_msgs/msg/Transform", TypeKind::kScalarUnstamped},
  {"tf2_msgs/msg/TFMessage", TypeKind::kTfMessage},
  {"nav_msgs/msg/Path", TypeKind::kPath},
}};

bool kind_requires_base_frame(TypeKind k)
{
  return k == TypeKind::kTfMessage || k == TypeKind::kPath;
}

const SupportedType * lookup_supported(std::string_view type_name)
{
  for (const auto & t : kSupportedTypes) {
    if (t.name == type_name) {
      return &t;
    }
  }
  return nullptr;
}

// Human-readable block appended to the `traj export --help` output and
// to the "unsupported type" error so users can always see what's
// allowed.
constexpr const char * kSupportedTypesHelp =
  "Supported topic types:\n"
  "  Stamped scalar (timestamp from header.stamp):\n"
  "    - geometry_msgs/msg/PoseStamped\n"
  "    - geometry_msgs/msg/PoseWithCovarianceStamped\n"
  "    - geometry_msgs/msg/TransformStamped\n"
  "    - nav_msgs/msg/Odometry\n"
  "  Unstamped scalar (timestamp from bag log time):\n"
  "    - geometry_msgs/msg/Pose\n"
  "    - geometry_msgs/msg/Transform\n"
  "  Multi-sample (requires --base-frame):\n"
  "    - tf2_msgs/msg/TFMessage     (filter: child_frame_id == --base-frame)\n"
  "    - nav_msgs/msg/Path          (validate: header.frame_id == --base-frame)";

}  // namespace

// `bagwiz traj` is a command group for trajectory-shaped operations.
// The group is wired up as a CLI11 subcommand so operations (convert,
// align, compare, ...) can be added by dropping a new `configure_*` +
// `run_*` pair next to the existing one.
//
// Subcommands
// -----------
//   export    Extract a topic's pose trajectory and save it.
//             Accepted shapes: scalar stamped (header.stamp timestamps),
//             scalar unstamped (bag log time timestamps, one-shot warning),
//             and multi-sample (TFMessage / Path with `--base-frame`).
class TrajCommand : public Command
{
public:
  std::string_view name() const override { return "traj"; }
  std::string_view description() const override { return "Trajectory operations"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_export(app);
    // New subcommands go here: configure_convert(app), configure_align(app), ...
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kExport:
        return run_export();
      case Subcommand::kNone:
        // Unreachable: require_subcommand(1) forces a selection and main
        // only calls run() after a successful parse. Kept for safety.
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kExport };
  Subcommand selected_ = Subcommand::kNone;

  struct ExportArgs
  {
    std::filesystem::path input_path;
    std::string topic;
    std::filesystem::path output_path;
    std::string format;
    std::string base_frame;
  } export_args_;

  void configure_export(CLI::App & app)
  {
    auto * sub = app.add_subcommand("export", "Extract a topic's pose trajectory and save it");
    sub->add_option("input", export_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("topic", export_args_.topic, "Topic name to extract poses from")->required();
    sub->add_option("output", export_args_.output_path, "Output file path")->required();
    sub->add_option("-f,--format", export_args_.format, "Output format")
      ->default_val(kFormatTum)
      ->check(CLI::IsMember({kFormatTum}));
    sub->add_option(
      "--base-frame", export_args_.base_frame,
      "Frame identifier. Required for multi-sample types: filters by child_frame_id on "
      "tf2_msgs/msg/TFMessage, validates header.frame_id on nav_msgs/msg/Path. "
      "Ignored for single-sample (scalar) types.");
    sub->footer(kSupportedTypesHelp);
    sub->callback([this]() { selected_ = Subcommand::kExport; });
  }

  int run_export()
  {
    const auto & args = export_args_;

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    const io::TopicInfo * topic_info = nullptr;
    for (const auto & t : reader->topics()) {
      if (t.name == args.topic) {
        topic_info = &t;
        break;
      }
    }
    if (topic_info == nullptr) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is not present in %s", args.topic.c_str(), args.input_path.c_str());
      return 1;
    }
    const std::string type_name = topic_info->type;

    const SupportedType * spec = lookup_supported(type_name);
    if (spec == nullptr) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' has type '%s', which is not supported by `traj export`.\n%s",
        args.topic.c_str(), type_name.c_str(), kSupportedTypesHelp);
      return 1;
    }

    if (kind_requires_base_frame(spec->kind) && args.base_frame.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Type '%s' is multi-sample; --base-frame is required (filter for TFMessage, validator "
        "for Path).",
        type_name.c_str());
      return 1;
    }

    io::ReadFilter filter;
    filter.topics.push_back(args.topic);
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
        switch (spec->kind) {
          case TypeKind::kScalarStamped:
          case TypeKind::kScalarUnstamped: {
            const auto extraction = core::extract_pose(msg.members(), msg.data(), raw.timestamp_ns);
            if (!extraction) {
              BAGWIZ_LOG_ERROR(
                kLogger,
                "Failed to extract pose from message #%ld of type '%s' (unexpected schema).",
                decoded, type_name.c_str());
              return 1;
            }
            if (!extraction->used_header_stamp && !warned_bag_log_time) {
              BAGWIZ_LOG_WARN(
                kLogger, "Type '%s' has no header; using bag log time (recorder receive time).",
                type_name.c_str());
              warned_bag_log_time = true;
            }
            poses.push_back(extraction->pose);
            break;
          }
          case TypeKind::kTfMessage: {
            auto tf = core::extract_tf_message_poses(msg.members(), msg.data(), args.base_frame);
            if (!tf.ok()) {
              BAGWIZ_LOG_ERROR(
                kLogger, "TFMessage extraction failed at message #%ld: %s", decoded,
                tf.error.c_str());
              return 1;
            }
            poses.insert(poses.end(), tf.poses.begin(), tf.poses.end());
            break;
          }
          case TypeKind::kPath: {
            auto path = core::extract_path_poses(msg.members(), msg.data(), args.base_frame);
            if (!path.ok()) {
              BAGWIZ_LOG_ERROR(
                kLogger, "Path validation failed at message #%ld: %s", decoded, path.error.c_str());
              return 1;
            }
            poses.insert(poses.end(), path.poses.begin(), path.poses.end());
            break;
          }
        }
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "decode error on message #%ld: %s", decoded, e.what());
        return 1;
      }
    }

    if (!args.base_frame.empty() && !kind_requires_base_frame(spec->kind)) {
      // Scalar types don't need --base-frame; leave a debug trace so
      // anyone troubleshooting knows the flag was observed but unused.
      BAGWIZ_LOG_DEBUG(
        kLogger, "--base-frame='%s' ignored for scalar type '%s'", args.base_frame.c_str(),
        type_name.c_str());
    }

    if (spec->kind == TypeKind::kTfMessage && poses.empty() && decoded > 0) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "No TransformStamped in '%s' matched child_frame_id='%s'; writing an empty trajectory.",
        args.topic.c_str(), args.base_frame.c_str());
    }

    if (poses.empty()) {
      BAGWIZ_LOG_WARN(
        kLogger, "No messages found on topic '%s'; writing an empty trajectory.",
        args.topic.c_str());
    }

    if (args.format != kFormatTum) {
      BAGWIZ_LOG_ERROR(kLogger, "Unsupported format '%s'", args.format.c_str());
      return 1;
    }

    std::ofstream out(args.output_path, std::ios::out | std::ios::trunc);
    if (!out) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Failed to open output path %s for writing", args.output_path.c_str());
      return 1;
    }
    core::write_tum(out, poses);
    out.close();

    BAGWIZ_LOG_INFO(
      kLogger, "Wrote %zu poses (from %ld messages) to %s in %s format", poses.size(), decoded,
      args.output_path.c_str(), args.format.c_str());
    return 0;
  }
};

BAGWIZ_REGISTER_COMMAND(TrajCommand)

}  // namespace bagwiz::commands
