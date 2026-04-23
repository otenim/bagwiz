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

// Declarative list of ROS 2 message types `traj export` accepts. Every
// entry is scalar (one message == one sample); extract_pose works the
// same way for all of them and falls back to bag log time when the
// message has no header.
constexpr std::array<std::string_view, 6> kSupportedTypes = {{
  "geometry_msgs/msg/PoseStamped",
  "geometry_msgs/msg/PoseWithCovarianceStamped",
  "geometry_msgs/msg/TransformStamped",
  "nav_msgs/msg/Odometry",
  "geometry_msgs/msg/Pose",
  "geometry_msgs/msg/Transform",
}};

bool is_supported(std::string_view type_name)
{
  for (const auto & t : kSupportedTypes) {
    if (t == type_name) {
      return true;
    }
  }
  return false;
}

// Human-readable block appended to the `traj export --help` output and
// to the "unsupported type" error so users can always see what's
// allowed.
constexpr const char * kSupportedTypesHelp =
  "Supported message types:\n"
  "  Stamped (timestamp from header.stamp):\n"
  "    - geometry_msgs/msg/PoseStamped\n"
  "    - geometry_msgs/msg/PoseWithCovarianceStamped\n"
  "    - geometry_msgs/msg/TransformStamped\n"
  "    - nav_msgs/msg/Odometry\n"
  "  Unstamped (timestamp from bag log time):\n"
  "    - geometry_msgs/msg/Pose\n"
  "    - geometry_msgs/msg/Transform";

}  // namespace

// `bagwiz traj` is a command group for trajectory-shaped operations.
// The group is wired up as a CLI11 subcommand so operations (convert,
// align, compare, ...) can be added by dropping a new `configure_*` +
// `run_*` pair next to the existing one.
//
// Subcommands
// -----------
//   export    Extract a topic's pose trajectory and save it.
//             Supported types: PoseStamped, PoseWithCovarianceStamped,
//             TransformStamped, Odometry (header.stamp timestamps) and
//             Pose, Transform (bag log time timestamps, one-shot warning).
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

    if (!is_supported(type_name)) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' has type '%s', which is not supported by `traj export`.\n%s",
        args.topic.c_str(), type_name.c_str(), kSupportedTypesHelp);
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
        const auto extraction = core::extract_pose(msg.members(), msg.data(), raw.timestamp_ns);
        if (!extraction) {
          BAGWIZ_LOG_ERROR(
            kLogger, "Failed to extract pose from message #%ld of type '%s' (unexpected schema).",
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
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "decode error on message #%ld: %s", decoded, e.what());
        return 1;
      }
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
