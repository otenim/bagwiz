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

// Declarative catalogue of the ROS 2 message types `traj` accepts.
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

// Human-readable block appended to help output and to the "unsupported
// type" error so users can always see what's allowed.
constexpr const char * kSupportedTypesHelp =
  "Supported topic types:\n"
  "  Stamped scalar (header.stamp used as TUM timestamp):\n"
  "    - geometry_msgs/msg/PoseStamped\n"
  "    - geometry_msgs/msg/PoseWithCovarianceStamped\n"
  "    - geometry_msgs/msg/TransformStamped\n"
  "    - nav_msgs/msg/Odometry\n"
  "  Unstamped scalar (bag log time used as TUM timestamp):\n"
  "    - geometry_msgs/msg/Pose\n"
  "    - geometry_msgs/msg/Transform\n"
  "  Multi-sample (requires --base-frame):\n"
  "    - tf2_msgs/msg/TFMessage     (filter: child_frame_id == --base-frame)\n"
  "    - nav_msgs/msg/Path          (validate: header.frame_id == --base-frame)";

}  // namespace

// `bagwiz traj <input> <topic> <output> [-f tum] [--base-frame <id>]`
// extracts a topic's pose trajectory and writes it to disk.
//
// Accepted types fall into three kinds (see kSupportedTypes):
//   * scalar stamped    -- one sample per message, stamp from header.stamp
//                          (PoseStamped, PoseWithCovarianceStamped,
//                          TransformStamped, Odometry)
//   * scalar unstamped  -- one sample per message, stamp from bag log time
//                          (Pose, Transform); emits a one-shot warning
//   * multi-sample      -- zero or more samples per message via an array
//                          field, with `--base-frame` as the selector:
//                            TFMessage: filter by child_frame_id
//                            Path     : validate top-level header.frame_id
//
// For multi-sample types `--base-frame` is mandatory and the command
// fails fast if it is missing. For scalar types it is silently ignored.
// PoseArray is deliberately out of scope: every pose in one message
// shares the same stamp, which yields duplicate-timestamp rows that
// are awkward for TUM consumers.
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
      "--base-frame", base_frame_,
      "Frame identifier. Required for multi-sample types: filters by child_frame_id on "
      "tf2_msgs/msg/TFMessage, validates header.frame_id on nav_msgs/msg/Path. "
      "Ignored for single-sample (scalar) types.");
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

    if (kind_requires_base_frame(spec->kind) && base_frame_.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Type '%s' is multi-sample; --base-frame is required (filter for TFMessage, validator "
        "for Path).",
        type_name.c_str());
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
                kLogger,
                "Type '%s' has no header; using bag log time (recorder receive time) for TUM "
                "timestamps.",
                type_name.c_str());
              warned_bag_log_time = true;
            }
            poses.push_back(extraction->pose);
            break;
          }
          case TypeKind::kTfMessage: {
            auto tf = core::extract_tf_message_poses(msg.members(), msg.data(), base_frame_);
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
            auto path = core::extract_path_poses(msg.members(), msg.data(), base_frame_);
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

    if (!base_frame_.empty() && !kind_requires_base_frame(spec->kind)) {
      // Scalar types don't need --base-frame; leave a debug trace so
      // anyone troubleshooting knows the flag was observed but unused.
      BAGWIZ_LOG_DEBUG(
        kLogger, "--base-frame='%s' ignored for scalar type '%s'", base_frame_.c_str(),
        type_name.c_str());
    }

    if (spec->kind == TypeKind::kTfMessage && poses.empty() && decoded > 0) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "No TransformStamped in '%s' matched child_frame_id='%s'; writing an empty trajectory.",
        topic_.c_str(), base_frame_.c_str());
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
  std::string base_frame_;
};

BAGWIZ_REGISTER_COMMAND(TrajCommand)

}  // namespace bagwiz::commands
