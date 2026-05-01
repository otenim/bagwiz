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
#include "bagwiz/core/terminal_input.hpp"
#include "bagwiz/core/tf_rotation.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <fmt/core.h>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <exception>
#include <filesystem>
#include <map>
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

constexpr const char * kLogger = "bagwiz.cmd.tf";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr std::string_view kTfStaticSuffix = "tf_static";

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

// Walk the TF topics once: feed every contained TransformStamped into
// `buffer` with the correct static/dynamic flag, and collect the set of
// distinct timestamps emitted by dynamic /tf messages. Those timestamps
// become the sample points of the `tf walk` timeline -- each one is a
// moment at which the TF tree observably changed.
//
// Decoding goes through the unified open_decoder() path so for MCAP
// inputs the schema-driven backend handles the work and tf2_msgs no
// longer needs to be on AMENT_PREFIX_PATH at runtime; only its
// header-only struct definition is required at build time (via
// extract_tf_message → geometry_msgs::msg::TransformStamped).
void load_tf_and_timeline(
  const std::filesystem::path & bag_path, const std::vector<TfTopic> & tf_topics,
  tf2::BufferCore & buffer, std::vector<std::int64_t> & sorted_timestamps)
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

  // One decoder per TF topic so the schema_text differences across
  // shards / topics are handled by the factory rather than us.
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

  // Use a sorted vector-then-unique pattern; std::set has worse locality
  // for the inner-loop insert of every TransformStamped timestamp on a
  // large bag.
  std::vector<std::int64_t> all_dynamic_ts;

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
      if (!is_static) {
        const std::int64_t ns = static_cast<std::int64_t>(t.header.stamp.sec) * 1'000'000'000LL +
                                static_cast<std::int64_t>(t.header.stamp.nanosec);
        all_dynamic_ts.push_back(ns);
      }
    }
  }

  std::sort(all_dynamic_ts.begin(), all_dynamic_ts.end());
  all_dynamic_ts.erase(
    std::unique(all_dynamic_ts.begin(), all_dynamic_ts.end()), all_dynamic_ts.end());
  sorted_timestamps = std::move(all_dynamic_ts);
}

std::string format_timestamp(std::int64_t ns)
{
  const auto seconds = static_cast<std::time_t>(ns / 1'000'000'000);
  const auto nanos = static_cast<std::int64_t>(ns % 1'000'000'000);
  std::tm tm_utc{};
  ::gmtime_r(&seconds, &tm_utc);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_utc);
  return fmt::format("{}.{:09d} UTC ({}.{:09d} s)", buf, nanos, seconds, nanos);
}

// tf2::ExtrapolationException::what() carries the cache boundary in
// the form "earliest data is at time X". We parse it back out so the
// init-time probe can crop the timeline to the chain's published
// range. Only the "into the past" boundary is used today; the
// "into the future" wording is rare enough at timeline.front() that
// we simply ignore it and let the per-step renderer surface it.
struct ExtrapolationBoundary
{
  bool past = false;     // requested time is BEFORE the available range
  double stamp_s = 0.0;  // the boundary stamp parsed from the message
  bool stamp_parsed = false;
};

ExtrapolationBoundary parse_extrapolation(const std::string & what)
{
  ExtrapolationBoundary out;
  out.past = what.find("into the past") != std::string::npos;

  static constexpr std::string_view kMarker = "earliest data is at time ";
  const auto pos = what.find(kMarker);
  if (pos != std::string::npos) {
    const char * begin = what.c_str() + pos + kMarker.size();
    char * end = nullptr;
    const double v = std::strtod(begin, &end);
    if (end != begin) {
      out.stamp_s = v;
      out.stamp_parsed = true;
    }
  }
  return out;
}

}  // namespace

// `bagwiz tf` is a command group for TF inspection. Today it carries a
// single `walk` subcommand that steps through the TF tree between two
// frames chronologically; future verbs (echo, list, ...) drop in as
// peers.
//
// Subcommands
// -----------
//   walk    Step one-at-a-time through the TF between <from> and <to>
//           at each dynamic /tf update in the bag. Uses the same key
//           scheme as `bagwiz walk`: right/Space = next, left/b = prev,
//           g = first, G = last, q / Ctrl-C = quit.
class TfCommand : public Command
{
public:
  std::string_view name() const override { return "tf"; }
  std::string_view description() const override { return "TF inspection"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_walk(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kWalk:
        return run_walk();
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kWalk };
  Subcommand selected_ = Subcommand::kNone;

  enum class RotationFormat { kQuaternion, kEulerRad, kEulerDeg };

  struct WalkArgs
  {
    std::filesystem::path input_path;
    std::string from_frame;
    std::string to_frame;
    RotationFormat rot = RotationFormat::kQuaternion;
  } walk_args_;

  void configure_walk(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "walk", "Step through the TF between two frames at each dynamic /tf update");
    sub->add_option("input", walk_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "from", walk_args_.from_frame,
        "Reference (fixed) frame -- the output expresses <to> in this frame's coordinates")
      ->required();
    sub->add_option("to", walk_args_.to_frame, "Tracked (moving) frame to sample")->required();
    sub
      ->add_option(
        "-r,--rot", walk_args_.rot,
        "Rotation format: quat (default) | euler | euler_rad | euler_deg "
        "(euler is an alias for euler_rad)")
      ->transform(
        CLI::CheckedTransformer(
          std::map<std::string, RotationFormat>{
            {"quat", RotationFormat::kQuaternion},
            {"euler", RotationFormat::kEulerRad},
            {"euler_rad", RotationFormat::kEulerRad},
            {"euler_deg", RotationFormat::kEulerDeg},
          },
          CLI::ignore_case));
    sub->callback([this]() { selected_ = Subcommand::kWalk; });
  }

  int run_walk()
  {
    const auto & args = walk_args_;

    if (!::isatty(STDIN_FILENO) || !::isatty(STDOUT_FILENO)) {
      BAGWIZ_LOG_ERROR(
        kLogger, "tf walk requires an interactive terminal (stdin+stdout must be TTY)");
      return 1;
    }

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    const auto tf_topics = collect_tf_topics(*reader);
    if (tf_topics.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "Bag has no tf2_msgs/msg/TFMessage topic; nothing to walk.");
      return 1;
    }

    // Default tf2::BufferCore cache is 10 s, which silently ages out
    // older transforms while we replay an entire bag into it: by the
    // time loading finishes the buffer only has the last 10 s. Use a
    // very large window so every transform from the bag stays
    // available for lookup.
    tf2::BufferCore tf_buffer{std::chrono::hours(24 * 365)};
    std::vector<std::int64_t> timeline;
    try {
      load_tf_and_timeline(args.input_path, tf_topics, tf_buffer, timeline);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to load TF from the bag: %s", e.what());
      return 1;
    }

    // /tf_static-only bags are still walkable: tf2::BufferCore returns
    // static transforms for any query stamp, so we synthesize a single
    // step at t=0 and let the existing UI render it. Useful for sensor
    // calibration bags or any case where the user wants to verify a
    // fully-static from->to chain.
    const bool static_only = timeline.empty();
    if (static_only) {
      const bool has_static = std::any_of(
        tf_topics.begin(), tf_topics.end(), [](const TfTopic & t) { return t.is_static; });
      if (!has_static) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Bag has TFMessage topics but no transforms were decoded; nothing to do.");
        return 1;
      }
      timeline.push_back(0);
    }

    // Probe the chain at timeline.front() to (a) fail fast on chain
    // errors (frame not in the tree at all, missing bridge) and (b)
    // crop the warm-up region: when the chain is not yet established
    // at the bag's first dynamic stamp (typical when sensor /tf
    // precedes the localizer), tf2 reports the earliest stamp the
    // chain *is* queryable from. Drop everything before that so the
    // walk only ever shows valid steps.
    try {
      (void)tf_buffer.lookupTransform(
        args.from_frame, args.to_frame, tf2::TimePoint(std::chrono::nanoseconds(timeline.front())));
    } catch (const tf2::LookupException & e) {
      BAGWIZ_LOG_ERROR(kLogger, "TF chain error: %s", e.what());
      return 1;
    } catch (const tf2::ConnectivityException & e) {
      BAGWIZ_LOG_ERROR(kLogger, "TF chain error: %s", e.what());
      return 1;
    } catch (const tf2::ExtrapolationException & e) {
      if (static_only) {
        // The chain has at least one segment that needs a dynamic /tf
        // stamp this bag does not provide; cropping a synthetic timeline
        // would just leave it empty, so error out with a specific message.
        BAGWIZ_LOG_ERROR(
          kLogger,
          "TF chain '%s' -> '%s' is not fully static and this bag has no dynamic /tf updates "
          "to satisfy the missing segment(s): %s",
          args.from_frame.c_str(), args.to_frame.c_str(), e.what());
        return 1;
      }
      const auto info = parse_extrapolation(e.what());
      if (info.past && info.stamp_parsed) {
        const std::int64_t boundary_ns = static_cast<std::int64_t>(info.stamp_s * 1e9);
        timeline.erase(
          timeline.begin(), std::lower_bound(timeline.begin(), timeline.end(), boundary_ns));
      }
      if (timeline.empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "TF chain '%s' -> '%s' is never resolvable in this bag (no dynamic /tf stamps fall "
          "within the chain's published range).",
          args.from_frame.c_str(), args.to_frame.c_str());
        return 1;
      }
    } catch (const tf2::TransformException & e) {
      BAGWIZ_LOG_ERROR(kLogger, "TF error: %s", e.what());
      return 1;
    }

    core::TerminalRawMode raw_mode;
    if (!raw_mode.active()) {
      BAGWIZ_LOG_ERROR(kLogger, "failed to enter raw terminal mode");
      return 1;
    }

    std::size_t index = 0;
    std::string status;

    auto render = [&]() {
      fmt::print(stdout, "\x1b[2J\x1b[H");
      const std::int64_t ts = timeline[index];
      if (static_only) {
        fmt::print(stdout, "[STATIC TF]  (no dynamic /tf in bag)\n");
      } else {
        fmt::print(
          stdout, "[STEP {} / {}]  {}\n", index + 1, timeline.size(), format_timestamp(ts));
      }
      fmt::print(stdout, "TF: {}  ->  {}\n", args.from_frame, args.to_frame);

      // Show the resolved chain so the user can see *how* the composed
      // transform was computed. tf2::_chainAsVector populates the path
      // through the tree between source and target via their common
      // ancestor; calling it with target=to, source=from yields a vector
      // ordered as `from -> ... -> to`, matching the header above.
      // Failures here are silenced because the main lookupTransform
      // below will surface any chain error in a more useful form.
      const auto query_tp = tf2::TimePoint(std::chrono::nanoseconds(ts));
      try {
        std::vector<std::string> chain;
        tf_buffer._chainAsVector(
          args.to_frame, query_tp, args.from_frame, query_tp, args.from_frame, chain);
        if (!chain.empty()) {
          fmt::print(stdout, "chain: ");
          for (std::size_t i = 0; i < chain.size(); ++i) {
            if (i > 0) {
              fmt::print(stdout, " -> ");
            }
            fmt::print(stdout, "{}", chain[i]);
          }
          fmt::print(stdout, "\n");
        }
      } catch (const tf2::TransformException &) {
        // chain extraction failed; fall through and let lookupTransform error
      }
      fmt::print(stdout, "\n");

      try {
        const auto tf = tf_buffer.lookupTransform(args.from_frame, args.to_frame, query_tp);
        fmt::print(stdout, "translation:\n");
        fmt::print(stdout, "  x: {:.15g}\n", tf.transform.translation.x);
        fmt::print(stdout, "  y: {:.15g}\n", tf.transform.translation.y);
        fmt::print(stdout, "  z: {:.15g}\n", tf.transform.translation.z);
        if (args.rot == RotationFormat::kQuaternion) {
          fmt::print(stdout, "rotation (quaternion):\n");
          fmt::print(stdout, "  x: {:.15g}\n", tf.transform.rotation.x);
          fmt::print(stdout, "  y: {:.15g}\n", tf.transform.rotation.y);
          fmt::print(stdout, "  z: {:.15g}\n", tf.transform.rotation.z);
          fmt::print(stdout, "  w: {:.15g}\n", tf.transform.rotation.w);
        } else {
          auto rpy = core::quat_to_euler_rad(
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
            tf.transform.rotation.w);
          const bool deg = args.rot == RotationFormat::kEulerDeg;
          if (deg) {
            rpy = core::euler_rad_to_euler_deg(rpy);
          }
          fmt::print(stdout, "rotation (euler, {}):\n", deg ? "deg" : "rad");
          fmt::print(stdout, "  roll:  {:.15g}\n", rpy.roll);
          fmt::print(stdout, "  pitch: {:.15g}\n", rpy.pitch);
          fmt::print(stdout, "  yaw:   {:.15g}\n", rpy.yaw);
        }
      } catch (const tf2::TransformException & e) {
        // Past extrapolation has been cropped at init, so the only
        // failures expected here are mid-bag gaps or the chain
        // ceasing to publish before the bag ends. Surface tf2's
        // text directly; rare enough that we do not need
        // hand-formatted versions.
        fmt::print(stdout, "⚠  Lookup failed at this step: {}\n", e.what());
      }

      fmt::print(stdout, "\n  [→/Space] next   [←/b] prev   [g] first   [G] last   [q] quit\n");
      if (!status.empty()) {
        fmt::print(stdout, "  {}\n", status);
      }
      std::fflush(stdout);
    };

    render();

    while (true) {
      const core::KeyEvent ev = core::read_key_event();
      status.clear();
      switch (ev) {
        case core::KeyEvent::kNext:
          if (index + 1 < timeline.size()) {
            ++index;
          } else {
            index = 0;
            status = "(wrapped to first)";
          }
          break;
        case core::KeyEvent::kPrev:
          if (index > 0) {
            --index;
          } else {
            status = "(at first step)";
          }
          break;
        case core::KeyEvent::kFirst:
          index = 0;
          break;
        case core::KeyEvent::kLast:
          index = timeline.size() - 1;
          break;
        case core::KeyEvent::kScrollUp:
        case core::KeyEvent::kScrollDown:
        case core::KeyEvent::kScrollHead:
        case core::KeyEvent::kScrollTail:
          // Body fits on a normal terminal; ignore scroll keys.
          continue;
        case core::KeyEvent::kQuit:
          fmt::print(stdout, "\n");
          return 0;
        case core::KeyEvent::kUnknown:
          continue;
      }
      render();
    }
  }
};

BAGWIZ_REGISTER_COMMAND(TfCommand)

}  // namespace bagwiz::commands
