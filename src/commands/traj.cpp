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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <fmt/core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.traj";
constexpr const char * kFormatTum = "tum";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr std::string_view kTfStaticSuffix = "tf_static";

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

// cppcheck-suppress passedByValue
bool is_supported(std::string_view type_name)
{
  for (const auto & t : kSupportedTypes) {
    if (t == type_name) {
      return true;
    }
  }
  return false;
}

// cppcheck-suppress passedByValue
bool is_unstamped_type(std::string_view type_name)
{
  return type_name == "geometry_msgs/msg/Pose" || type_name == "geometry_msgs/msg/Transform";
}

// Only Odometry and TransformStamped carry a top-level child_frame_id
// that names the moving frame. `--of` (tracked frame) requires one of
// these so the "retarget to a different rigid body" composition is well
// defined.
// cppcheck-suppress passedByValue
bool type_has_child_frame(std::string_view type_name)
{
  return type_name == "nav_msgs/msg/Odometry" || type_name == "geometry_msgs/msg/TransformStamped";
}

// /tf_static and any topic whose name terminates in "tf_static" use the
// transient_local durability and carry one-shot, time-independent
// transforms. Everything else carrying TFMessage is treated as dynamic
// and stored in the time-indexed history.
bool is_static_tf_topic(std::string_view topic_name)
{
  if (topic_name.size() < kTfStaticSuffix.size()) {
    return false;
  }
  return topic_name.compare(
           topic_name.size() - kTfStaticSuffix.size(), kTfStaticSuffix.size(), kTfStaticSuffix) ==
         0;
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
  "    - geometry_msgs/msg/Transform\n"
  "\n"
  "Frame selection\n"
  "  --in <frame>  Reference (fixed) frame the output is expressed in.\n"
  "                Defaults to the topic's header.frame_id.\n"
  "  --of <frame>  Tracked (moving) frame whose pose each sample\n"
  "                represents. Defaults to the topic's child_frame_id\n"
  "                (Odometry, TransformStamped); `--of` is rejected for\n"
  "                types that do not carry child_frame_id.\n"
  "  Either flag pulls TF from the bag (any tf2_msgs/msg/TFMessage\n"
  "  topic; names ending in `tf_static` are treated as static). Fails\n"
  "  fast if the bag has no TF or a lookup is out of range.";

struct TfTopic
{
  std::string name;
  bool is_static;
};

std::vector<TfTopic> collect_tf_topics(const io::BagReader & reader)
{
  std::vector<TfTopic> topics;
  for (const auto & t : reader.topics()) {
    if (t.type == kTfMessageType) {
      topics.push_back({t.name, is_static_tf_topic(t.name)});
    }
  }
  return topics;
}

// Read every TFMessage from `tf_topics` through a fresh BagReader pass
// and feed each contained TransformStamped into `buffer`. Dynamic and
// static transforms are routed via the `is_static` flag so tf2's lookup
// semantics (interpolated history vs. time-independent) match the
// source topic.
void load_tf_buffer(
  const std::filesystem::path & bag_path, const std::vector<TfTopic> & tf_topics,
  tf2::BufferCore & buffer)
{
  auto tf_reader = io::open_read(bag_path);
  io::ReadFilter filter;
  for (const auto & t : tf_topics) {
    filter.topics.push_back(t.name);
  }
  tf_reader->set_filter(filter);

  std::unordered_map<std::string, bool> is_static_by_topic;
  for (const auto & t : tf_topics) {
    is_static_by_topic[t.name] = t.is_static;
  }

  const core::IntrospectionLoad introspection = core::load_introspection(kTfMessageType);
  if (!introspection.ok()) {
    throw std::runtime_error(
      "Could not load introspection for tf2_msgs/msg/TFMessage: " + introspection.error);
  }

  io::RawMessage raw;
  while (tf_reader->next(raw)) {
    const core::DeserializedMessage msg(introspection, raw.payload);
    const auto * tf_msg = static_cast<const tf2_msgs::msg::TFMessage *>(msg.data());
    const bool is_static = is_static_by_topic.at(raw.topic->name);
    for (const auto & t : tf_msg->transforms) {
      buffer.setTransform(t, "bagwiz", is_static);
    }
  }
}

// Pack a TrajectoryPose into a tf2::Transform and back, so the two
// composition directions below share the same conversion code path.
tf2::Transform to_tf2(const core::TrajectoryPose & p)
{
  tf2::Transform t;
  t.setOrigin(tf2::Vector3(p.tx, p.ty, p.tz));
  t.setRotation(tf2::Quaternion(p.qx, p.qy, p.qz, p.qw));
  return t;
}

void from_tf2(const tf2::Transform & t, core::TrajectoryPose & p)
{
  const auto o = t.getOrigin();
  const auto r = t.getRotation();
  p.tx = o.x();
  p.ty = o.y();
  p.tz = o.z();
  p.qx = r.x();
  p.qy = r.y();
  p.qz = r.z();
  p.qw = r.w();
}

tf2::Transform to_tf2(const geometry_msgs::msg::TransformStamped & ts)
{
  tf2::Transform t;
  t.setOrigin(
    tf2::Vector3(
      ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z));
  t.setRotation(
    tf2::Quaternion(
      ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z,
      ts.transform.rotation.w));
  return t;
}

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
//             `--in <frame>` picks the output reference frame;
//             `--of <frame>` picks the tracked body. Either pulls TF
//             from the bag.
class TrajCommand : public Command
{
public:
  std::string_view name() const override { return "traj"; }
  std::string_view description() const override { return "Trajectory operations"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_export(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kExport:
        return run_export();
      case Subcommand::kNone:
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
    std::string in_frame;
    std::string of_frame;
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
      "--in", export_args_.in_frame,
      "Reference (fixed) frame the output trajectory is expressed in. "
      "Defaults to the topic's header.frame_id. Requires TF in the bag when non-default.");
    sub->add_option(
      "--of", export_args_.of_frame,
      "Tracked (moving) frame whose pose each sample represents. Defaults to the topic's "
      "child_frame_id. Requires a type with child_frame_id (Odometry, TransformStamped).");
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

    const bool in_set = !args.in_frame.empty();
    const bool of_set = !args.of_frame.empty();
    const bool use_tf = in_set || of_set;

    if (use_tf && is_unstamped_type(type_name)) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "--in / --of cannot be used with unstamped type '%s' (no header.frame_id to start "
        "from).",
        type_name.c_str());
      return 1;
    }
    if (of_set && !type_has_child_frame(type_name)) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "--of requires a type with child_frame_id (nav_msgs/msg/Odometry or "
        "geometry_msgs/msg/TransformStamped); got '%s'.",
        type_name.c_str());
      return 1;
    }

    // Optional TF buffer populated only when --in or --of is given.
    tf2::BufferCore tf_buffer;
    if (use_tf) {
      const auto tf_topics = collect_tf_topics(*reader);
      if (tf_topics.empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "--in / --of specified but the bag has no tf2_msgs/msg/TFMessage topic; TF is "
          "required to change the reference or tracked frame.");
        return 1;
      }
      try {
        load_tf_buffer(args.input_path, tf_topics, tf_buffer);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "Failed to load TF from the bag: %s", e.what());
        return 1;
      }
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

        // Compose: start with message_pose (= T_source_tracked), retarget
        // tracked -> --of on the right, then shift the coordinate system
        // source -> --in on the left. Skipping either half is a no-op when
        // the requested frame already matches.
        tf2::Transform P = to_tf2(extraction->pose);
        const tf2::TimePoint tp(std::chrono::nanoseconds(extraction->pose.timestamp_ns));

        if (of_set && args.of_frame != extraction->child_frame_id) {
          try {
            const auto tf =
              tf_buffer.lookupTransform(extraction->child_frame_id, args.of_frame, tp);
            P = P * to_tf2(tf);
          } catch (const tf2::TransformException & e) {
            BAGWIZ_LOG_ERROR(
              kLogger,
              "TF lookup failed at message #%ld (--of: target='%s' source='%s' t=%ld ns): %s",
              decoded, extraction->child_frame_id.c_str(), args.of_frame.c_str(),
              extraction->pose.timestamp_ns, e.what());
            return 1;
          }
        }

        if (in_set && args.in_frame != extraction->frame_id) {
          try {
            const auto tf = tf_buffer.lookupTransform(args.in_frame, extraction->frame_id, tp);
            P = to_tf2(tf) * P;
          } catch (const tf2::TransformException & e) {
            BAGWIZ_LOG_ERROR(
              kLogger,
              "TF lookup failed at message #%ld (--in: target='%s' source='%s' t=%ld ns): %s",
              decoded, args.in_frame.c_str(), extraction->frame_id.c_str(),
              extraction->pose.timestamp_ns, e.what());
            return 1;
          }
        }

        core::TrajectoryPose out_pose = extraction->pose;
        from_tf2(P, out_pose);
        poses.push_back(out_pose);
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
