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
#include "bagwiz/core/tf_chain.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/core/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fmt/core.h>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
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

constexpr const char * kLogger = "bagwiz.cmd.traj";
constexpr const char * kFormatTum = "tum";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr std::string_view kTfStaticSuffix = "tf_static";

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

// One observed edge from the input topic. Stored separately from the
// TF buffer so we can filter by chain-edge membership after the chain
// has been resolved.
struct InputEdge
{
  std::string frame_id;
  std::string child_frame_id;
  std::int64_t stamp_ns;
};

// Walk every TF topic once: insert each contained TransformStamped into
// `buffer` (static or dynamic per topic name) and, for messages on
// `input_topic`, record the (frame_id, child_frame_id, stamp_ns) so the
// caller can later filter by chain-edge membership without a second
// bag pass.
void load_tf_buffer_and_input_edges(
  const std::filesystem::path & bag_path, const std::vector<TfTopic> & tf_topics,
  const std::string & input_topic, tf2::BufferCore & buffer, std::vector<InputEdge> & input_edges)
{
  auto reader = io::open_read(bag_path);
  io::ReadFilter filter;
  for (const auto & t : tf_topics) {
    filter.topics.push_back(t.name);
  }
  reader->set_filter(filter);

  std::unordered_map<std::string, bool> is_static_by_topic;
  for (const auto & t : tf_topics) {
    is_static_by_topic[t.name] = t.is_static;
  }

  std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoder_by_topic;
  for (const auto & topic_info : reader->topics()) {
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
  while (reader->next(raw)) {
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
    const bool is_input = (raw.topic->name == input_topic);
    for (const auto & t : transforms) {
      buffer.setTransform(t, "bagwiz", is_static);
      if (is_input) {
        const std::int64_t ns = static_cast<std::int64_t>(t.header.stamp.sec) * 1'000'000'000LL +
                                static_cast<std::int64_t>(t.header.stamp.nanosec);
        input_edges.push_back({t.header.frame_id, t.child_frame_id, ns});
      }
    }
  }
}

}  // namespace

// `bagwiz traj` is a command group for trajectory-shaped operations.
//
// Subcommands
// -----------
//   export    Export the trajectory of `--to` expressed in `--from`
//             from a tf2_msgs/msg/TFMessage topic. Sample timestamps
//             come from updates of any chain edge between --from and
//             --to that arrive on the input topic; each sample is the
//             composed transform from the (static + dynamic) TF
//             buffer at that timestamp.
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
  } export_args_;

  void configure_export(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "export", "Export the --to → --from trajectory from a tf2_msgs/msg/TFMessage topic");
    sub->add_option("input", export_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("output", export_args_.output_path, "Output file path")->required();
    sub
      ->add_option(
        "topic", export_args_.topic,
        "Dynamic tf2_msgs/msg/TFMessage topic (e.g. /tf). Static counterparts "
        "(*tf_static) in the same bag are picked up automatically.")
      ->required();
    sub->add_option("-f,--format", export_args_.format, "Output format")
      ->default_val(kFormatTum)
      ->check(CLI::IsMember({kFormatTum}));
    sub
      ->add_option(
        "--from", export_args_.from_frame,
        "Reference (fixed) frame the output trajectory is expressed in.")
      ->required();
    sub
      ->add_option(
        "--to", export_args_.to_frame, "Tracked (moving) frame whose pose each sample represents.")
      ->required();
    sub->callback([this]() { selected_ = Subcommand::kExport; });
  }

  int run_export()
  {
    const auto & args = export_args_;

    if (args.from_frame == args.to_frame) {
      BAGWIZ_LOG_ERROR(
        kLogger, "--from and --to must be distinct frames; both were '%s'.",
        args.from_frame.c_str());
      return 1;
    }

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
    if (topic_info->type != kTfMessageType) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' has type '%s'; traj export only supports tf2_msgs/msg/TFMessage.",
        args.topic.c_str(), topic_info->type.c_str());
      return 1;
    }
    // /tf_static is one-shot and not a sensible sampling source. The
    // assumption is "1 dynamic + 1 static topic per bag", so the user
    // is expected to point at the dynamic side.
    if (is_static_tf_topic(args.topic)) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Topic '%s' looks like a static TF topic (name ends with 'tf_static'). "
        "Pass the dynamic /tf-style topic instead; static TF in the bag is loaded automatically.",
        args.topic.c_str());
      return 1;
    }

    const auto tf_topics = collect_tf_topics(*reader);
    if (tf_topics.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "Bag has no tf2_msgs/msg/TFMessage topics; nothing to export.");
      return 1;
    }

    // Single bag pass: populate the TF buffer with everything (static +
    // dynamic across all TF topics) and stash the per-edge tuples that
    // arrived on the input topic for later chain-edge filtering.
    //
    // Cache window: tf2::BufferCore defaults to 10 s, which silently
    // ages out older transforms as the bag streams in. Use a very large
    // window so the entire bag fits.
    tf2::BufferCore tf_buffer{std::chrono::hours(24 * 365)};
    std::vector<InputEdge> input_edges;
    try {
      load_tf_buffer_and_input_edges(
        args.input_path, tf_topics, args.topic, tf_buffer, input_edges);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to load TF from the bag: %s", e.what());
      return 1;
    }

    if (input_edges.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' carried no TransformStamped entries; nothing to export.",
        args.topic.c_str());
      return 1;
    }

    // Resolve the chain once at the first observed dynamic stamp. The
    // assumption (stable TF topology) makes a single resolution
    // representative for the whole bag.
    const tf2::TimePoint resolve_tp{std::chrono::nanoseconds(input_edges.front().stamp_ns)};
    const auto chain = core::resolve_chain(tf_buffer, args.from_frame, args.to_frame, resolve_tp);
    if (chain.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "No TF path between '%s' and '%s' in the bag (different connected components, "
        "or one of the frames is absent).",
        args.from_frame.c_str(), args.to_frame.c_str());
      return 1;
    }
    const auto path_edges = core::chain_to_edges(tf_buffer, chain, resolve_tp);

    // (frame_id, child_frame_id) -> on-path? membership test.
    auto edge_key = [](const std::string & a, const std::string & b) { return a + '\0' + b; };
    std::unordered_set<std::string> path_edge_set;
    path_edge_set.reserve(path_edges.size());
    for (const auto & e : path_edges) {
      path_edge_set.insert(edge_key(e.first, e.second));
    }

    std::vector<std::int64_t> sample_stamps;
    sample_stamps.reserve(input_edges.size());
    for (const auto & ie : input_edges) {
      if (path_edge_set.count(edge_key(ie.frame_id, ie.child_frame_id)) != 0) {
        sample_stamps.push_back(ie.stamp_ns);
      }
    }

    if (sample_stamps.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "No chain edges between '%s' and '%s' are published on '%s'. "
        "The whole chain may live on /tf_static (a fully-static path is rejected by "
        "traj export — there is no time axis to sample), or the input topic does not "
        "carry the relevant edge.",
        args.from_frame.c_str(), args.to_frame.c_str(), args.topic.c_str());
      return 1;
    }

    std::sort(sample_stamps.begin(), sample_stamps.end());
    sample_stamps.erase(
      std::unique(sample_stamps.begin(), sample_stamps.end()), sample_stamps.end());

    std::vector<core::TrajectoryPose> poses;
    poses.reserve(sample_stamps.size());
    std::int64_t skipped = 0;
    std::string last_skip_reason;
    for (const std::int64_t ns : sample_stamps) {
      const tf2::TimePoint tp{std::chrono::nanoseconds(ns)};
      try {
        const auto tf = tf_buffer.lookupTransform(args.from_frame, args.to_frame, tp);
        core::TrajectoryPose p;
        p.timestamp_ns = ns;
        p.tx = tf.transform.translation.x;
        p.ty = tf.transform.translation.y;
        p.tz = tf.transform.translation.z;
        p.qx = tf.transform.rotation.x;
        p.qy = tf.transform.rotation.y;
        p.qz = tf.transform.rotation.z;
        p.qw = tf.transform.rotation.w;
        poses.push_back(p);
      } catch (const tf2::TransformException & e) {
        ++skipped;
        last_skip_reason = e.what();
      }
    }

    if (poses.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "All %zu sample stamps failed to resolve via lookupTransform. Last reason: %s",
        sample_stamps.size(),
        last_skip_reason.empty() ? "(none recorded)" : last_skip_reason.c_str());
      return 1;
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
      kLogger, "Wrote %zu poses (from %zu sample stamps, %ld skipped) to %s in %s format",
      poses.size(), sample_stamps.size(), skipped, args.output_path.c_str(), args.format.c_str());
    return 0;
  }
};

BAGWIZ_REGISTER_COMMAND(TrajCommand)

}  // namespace bagwiz::commands
