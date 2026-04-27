// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/core/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fmt/core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <algorithm>
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
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.traj";
constexpr const char * kFormatTum = "tum";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr std::string_view kTfStaticSuffix = "tf_static";

// Declarative list of ROS 2 message types `traj export` accepts. The
// first six are scalar (one message == one sample); extract_pose works
// the same way for all of them and falls back to bag log time when the
// message has no header. `tf2_msgs/msg/TFMessage` carries N
// TransformStamped edges per message; the user picks one with
// --edge SRC:DST, after which extraction behaves the same as
// TransformStamped (single labeled edge per message, --from / --to
// compose via the TF buffer when they differ from the edge's
// frames).
constexpr std::array<std::string_view, 7> kSupportedTypes = {{
  "geometry_msgs/msg/PoseStamped",
  "geometry_msgs/msg/PoseWithCovarianceStamped",
  "geometry_msgs/msg/TransformStamped",
  "nav_msgs/msg/Odometry",
  "geometry_msgs/msg/Pose",
  "geometry_msgs/msg/Transform",
  "tf2_msgs/msg/TFMessage",
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

// cppcheck-suppress passedByValue
bool is_tf_message_type(std::string_view type_name)
{
  return type_name == kTfMessageType;
}

// Types where `--to` (tracked frame) is meaningful. Odometry and
// TransformStamped carry a top-level child_frame_id, so the override
// composes against that. TFMessage candidates each carry their own
// child_frame_id once `--edge SRC:DST` has selected the labeled edge,
// so `--to` is meaningful for it as well.
// cppcheck-suppress passedByValue
bool type_has_child_frame(std::string_view type_name)
{
  return type_name == "nav_msgs/msg/Odometry" ||
         type_name == "geometry_msgs/msg/TransformStamped" || is_tf_message_type(type_name);
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
  "  Multi-edge (requires --edge SRC:DST):\n"
  "    - tf2_msgs/msg/TFMessage  (the chosen edge becomes the input\n"
  "                               topic; --from / --to then apply\n"
  "                               the same way as for TransformStamped)\n"
  "\n"
  "Frame selection\n"
  "  --edge <SRC:DST>  Required for tf2_msgs/msg/TFMessage. Picks which\n"
  "                    labeled edge to extract (frame_id=SRC,\n"
  "                    child_frame_id=DST). Rejected for other types.\n"
  "  --from <frame>    Reference (fixed) frame the output is expressed\n"
  "                    in. Defaults to the topic's header.frame_id (or\n"
  "                    SRC for TFMessage with --edge).\n"
  "  --to   <frame>    Tracked (moving) frame whose pose each sample\n"
  "                    represents. Defaults to the topic's\n"
  "                    child_frame_id (or DST for TFMessage with\n"
  "                    --edge). Rejected for types that do not carry\n"
  "                    child_frame_id.\n"
  "  Setting --from or --to to a non-default frame pulls TF from the\n"
  "  bag (any tf2_msgs/msg/TFMessage topic; names ending in `tf_static`\n"
  "  are treated as static). Messages whose lookup is out of range are\n"
  "  skipped and counted in the summary.";

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

  // One decoder per TF topic. The factory picks the schema-driven
  // backend when the MCAP shard carries the embedded ros2msg schema for
  // tf2_msgs/msg/TFMessage and falls back to introspection otherwise.
  std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoder_by_topic;
  for (const auto & topic_info : tf_reader->topics()) {
    if (topic_info.type != kTfMessageType) {
      continue;
    }
    if (is_static_by_topic.find(topic_info.name) == is_static_by_topic.end()) {
      continue;
    }
    auto open = core::decoder::open_decoder(topic_info);
    if (!open.ok()) {
      throw std::runtime_error(
        "Could not open decoder for TF topic '" + topic_info.name + "': " + open.error);
    }
    decoder_by_topic.emplace(topic_info.name, std::move(open.decoder));
  }

  io::RawMessage raw;
  while (tf_reader->next(raw)) {
    auto it = decoder_by_topic.find(raw.topic->name);
    if (it == decoder_by_topic.end()) {
      continue;
    }
    const auto decoded = it->second->decode(raw.payload);
    if (!decoded.ok()) {
      throw std::runtime_error(
        "Failed to decode TF message on '" + raw.topic->name + "': " + decoded.error);
    }
    const auto transforms = core::extract_tf_message(*decoded.value);
    const bool is_static = is_static_by_topic.at(raw.topic->name);
    for (const auto & t : transforms) {
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
//             `--from <frame>` picks the output reference frame;
//             `--to <frame>` picks the tracked body. Either pulls TF
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
    std::string from_frame;
    std::string to_frame;
    // Raw "SRC:DST" string when --edge is given. Parsed into edge_src
    // / edge_dst after CLI parse so we can validate the syntax.
    std::string edge_spec;
    std::string edge_src;
    std::string edge_dst;
  } export_args_;

  void configure_export(CLI::App & app)
  {
    auto * sub = app.add_subcommand("export", "Extract a topic's pose trajectory and save it");
    sub->add_option("input", export_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("output", export_args_.output_path, "Output file path")->required();
    sub->add_option("topic", export_args_.topic, "Topic name to extract poses from")->required();
    sub->add_option("-f,--format", export_args_.format, "Output format")
      ->default_val(kFormatTum)
      ->check(CLI::IsMember({kFormatTum}));
    sub->add_option(
      "--from", export_args_.from_frame,
      "Reference (fixed) frame the output trajectory is expressed in. "
      "Defaults to the topic's header.frame_id. Requires TF in the bag when non-default.");
    sub->add_option(
      "--to", export_args_.to_frame,
      "Tracked (moving) frame whose pose each sample represents. Defaults to the topic's "
      "child_frame_id. Requires a type with child_frame_id (Odometry, TransformStamped).");
    sub->add_option(
      "--edge", export_args_.edge_spec,
      "TF edge to extract from a tf2_msgs/msg/TFMessage input, in 'SRC:DST' form. "
      "Required for TFMessage; rejected for other types. "
      "After filtering, --from/--to apply uniformly (same as TransformStamped).");
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

    const bool from_set = !args.from_frame.empty();
    const bool to_set = !args.to_frame.empty();
    const bool edge_set = !export_args_.edge_spec.empty();
    const bool use_tf = from_set || to_set;
    const bool tfmsg_input = is_tf_message_type(type_name);

    // --edge is the way to point at a specific labeled edge inside a
    // multi-edge TFMessage. For other input types the message itself
    // already carries (frame_id, child_frame_id), so the flag would
    // just confuse semantics.
    if (edge_set && !tfmsg_input) {
      BAGWIZ_LOG_ERROR(
        kLogger, "--edge applies only to tf2_msgs/msg/TFMessage input; topic '%s' has type '%s'.",
        args.topic.c_str(), type_name.c_str());
      return 1;
    }
    if (tfmsg_input && !edge_set) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Topic '%s' has type 'tf2_msgs/msg/TFMessage'; --edge SRC:DST is required to select "
        "which edge of the TF tree to extract.",
        args.topic.c_str());
      return 1;
    }
    if (edge_set) {
      const auto colon = export_args_.edge_spec.find(':');
      if (colon == std::string::npos || colon == 0 || colon + 1 == export_args_.edge_spec.size()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "--edge expects 'SRC:DST' with non-empty frame names; got '%s'.",
          export_args_.edge_spec.c_str());
        return 1;
      }
      export_args_.edge_src = export_args_.edge_spec.substr(0, colon);
      export_args_.edge_dst = export_args_.edge_spec.substr(colon + 1);
    }

    if (use_tf && is_unstamped_type(type_name)) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "--from / --to cannot be used with unstamped type '%s' (no header.frame_id to start "
        "from).",
        type_name.c_str());
      return 1;
    }
    if (to_set && !type_has_child_frame(type_name)) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "--to requires a type with child_frame_id (nav_msgs/msg/Odometry, "
        "geometry_msgs/msg/TransformStamped, or tf2_msgs/msg/TFMessage); got '%s'.",
        type_name.c_str());
      return 1;
    }

    // TF buffer is consulted only when we need to compose a candidate
    // through the TF tree to reach (--from, --to). When TFMessage
    // input is given with --edge alone (no --from / --to), the
    // pre-filtered candidates already have the desired (frame_id,
    // child_frame_id) and no compose is needed, so use_tf is false
    // and we skip the buffer pass.
    tf2::BufferCore tf_buffer;
    if (use_tf) {
      const auto tf_topics = collect_tf_topics(*reader);
      if (tf_topics.empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "--from / --to specified but the bag has no tf2_msgs/msg/TFMessage topic; TF is "
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

    auto open_decoder = core::decoder::open_decoder(*topic_info);
    if (!open_decoder.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open decoder: %s", open_decoder.error.c_str());
      return 1;
    }
    const auto & decoder = *open_decoder.decoder;

    std::vector<core::TrajectoryPose> poses;
    std::int64_t decoded = 0;
    std::int64_t skipped = 0;
    bool warned_bag_log_time = false;
    std::string last_skip_reason;

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
        const auto decode_result = decoder.decode(raw.payload);
        if (!decode_result.ok()) {
          BAGWIZ_LOG_ERROR(
            kLogger, "Failed to decode message #%ld of type '%s': %s", decoded, type_name.c_str(),
            decode_result.error.c_str());
          return 1;
        }

        // Unified extraction: every supported message yields one or
        // more (frame_id, child_frame_id, pose, stamp) candidates.
        // Most types produce a single candidate; tf2_msgs/msg/TFMessage
        // produces one candidate per contained edge.
        //
        // For TFMessage input, --edge SRC:DST pre-filters candidates
        // down to the labeled edge the user asked for, after which
        // every type goes through the same --to / --from compose
        // path: there is no input-type-specific branch here.
        const auto candidates =
          core::extract_pose_candidates(*decode_result.value, raw.timestamp_ns);
        if (candidates.empty()) {
          ++skipped;
          continue;
        }

        bool emitted_for_message = false;
        for (const auto & extraction : candidates) {
          if (
            edge_set &&
            (extraction.frame_id != args.edge_src || extraction.child_frame_id != args.edge_dst)) {
            // TFMessage edge that the user did not request. Skip
            // silently — the same bag message commonly carries
            // unrelated edges (e.g. map->ndt_base_link alongside
            // map->base_link) and we would not want to emit them.
            continue;
          }

          if (!extraction.used_header_stamp && !warned_bag_log_time) {
            BAGWIZ_LOG_WARN(
              kLogger, "Type '%s' has no header; using bag log time (recorder receive time).",
              type_name.c_str());
            warned_bag_log_time = true;
          }

          // Compose: start with message_pose (= T_source_tracked),
          // retarget tracked -> --to on the right, then shift the
          // coordinate system source -> --from on the left. Skipping
          // either half is a no-op when the requested frame already
          // matches (typical when --edge is given without --from /
          // --to: the candidate's frames already equal the
          // requested ones, so no TF lookup happens).
          tf2::Transform P = to_tf2(extraction.pose);
          const tf2::TimePoint tp(std::chrono::nanoseconds(extraction.pose.timestamp_ns));

          if (to_set && args.to_frame != extraction.child_frame_id) {
            if (extraction.child_frame_id.empty()) {
              continue;
            }
            try {
              const auto tf =
                tf_buffer.lookupTransform(extraction.child_frame_id, args.to_frame, tp);
              P = P * to_tf2(tf);
            } catch (const tf2::TransformException & e) {
              last_skip_reason = std::string("--to: ") + e.what();
              continue;
            }
          }

          if (from_set && args.from_frame != extraction.frame_id) {
            if (extraction.frame_id.empty()) {
              continue;
            }
            try {
              const auto tf = tf_buffer.lookupTransform(args.from_frame, extraction.frame_id, tp);
              P = to_tf2(tf) * P;
            } catch (const tf2::TransformException & e) {
              last_skip_reason = std::string("--from: ") + e.what();
              continue;
            }
          }

          core::TrajectoryPose out_pose = extraction.pose;
          from_tf2(P, out_pose);
          poses.push_back(out_pose);
          emitted_for_message = true;
          break;
        }

        if (!emitted_for_message) {
          ++skipped;
        }
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "decode error on message #%ld: %s", decoded, e.what());
        return 1;
      }
    }

    if (poses.empty()) {
      // Output is empty: either the topic carried no messages, or
      // every message failed to compose to (--from, --to). The two
      // cases ask for very different fixes, so distinguish them.
      if (decoded == 0) {
        BAGWIZ_LOG_WARN(
          kLogger, "No messages found on topic '%s'; writing an empty trajectory.",
          args.topic.c_str());
      } else {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "All %ld messages on topic '%s' were skipped (no candidate composed to "
          "(--from='%s', --to='%s')). Last reason: %s",
          decoded, args.topic.c_str(), args.from_frame.c_str(), args.to_frame.c_str(),
          last_skip_reason.empty() ? "(none recorded)" : last_skip_reason.c_str());
        return 1;
      }
    }

    if (args.format != kFormatTum) {
      BAGWIZ_LOG_ERROR(kLogger, "Unsupported format '%s'", args.format.c_str());
      return 1;
    }

    // Bag log time and header.stamp generally agree, but transient
    // system load can swap a few adjacent samples; downstream tools
    // (e.g. evo) require a non-decreasing time axis, so always sort
    // before writing. stable_sort is essentially free on
    // already-sorted input and preserves bag order for equal stamps.
    std::stable_sort(
      poses.begin(), poses.end(),
      [](const core::TrajectoryPose & a, const core::TrajectoryPose & b) {
        return a.timestamp_ns < b.timestamp_ns;
      });

    std::ofstream out(args.output_path, std::ios::out | std::ios::trunc);
    if (!out) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Failed to open output path %s for writing", args.output_path.c_str());
      return 1;
    }
    core::write_tum(out, poses);
    out.close();

    BAGWIZ_LOG_INFO(
      kLogger, "Wrote %zu poses (from %ld messages, %ld skipped) to %s in %s format", poses.size(),
      decoded, skipped, args.output_path.c_str(), args.format.c_str());
    return 0;
  }
};

BAGWIZ_REGISTER_COMMAND(TrajCommand)

}  // namespace bagwiz::commands
