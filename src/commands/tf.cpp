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
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <fmt/core.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
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
#include <optional>
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
// the form "earliest data is at time X" or "latest data is at time
// X". We parse it back out so we can render a friendlier message
// (delta from the current step, target step index in the timeline)
// rather than dumping the raw multi-line error text.
struct ExtrapolationBoundary
{
  bool past = false;     // requested time is BEFORE the available range
  bool future = false;   // requested time is AFTER the available range
  double stamp_s = 0.0;  // the boundary stamp parsed from the message
  bool stamp_parsed = false;
};

ExtrapolationBoundary parse_extrapolation(const std::string & what)
{
  ExtrapolationBoundary out;
  out.past = what.find("into the past") != std::string::npos;
  out.future = what.find("into the future") != std::string::npos;

  for (const std::string_view marker :
       {std::string_view{"earliest data is at time "},
        std::string_view{"latest data is at time "}}) {
    const auto pos = what.find(marker);
    if (pos == std::string::npos) {
      continue;
    }
    const char * begin = what.c_str() + pos + marker.size();
    char * end = nullptr;
    const double v = std::strtod(begin, &end);
    if (end != begin) {
      out.stamp_s = v;
      out.stamp_parsed = true;
      break;
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

  enum class RotationFormat { kQuaternion, kEuler };

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
        "Rotation format: q=quaternion (default), e=euler (roll/pitch/yaw in radians)")
      ->transform(
        CLI::CheckedTransformer(
          std::map<std::string, RotationFormat>{
            {"q", RotationFormat::kQuaternion},
            {"e", RotationFormat::kEuler},
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

    if (timeline.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Bag has TFMessage topics but no dynamic /tf updates (only /tf_static). A walk with one "
        "step is not interesting; nothing to do.");
      return 1;
    }

    // Sanity-check the chain once at the first timestamp so chain errors
    // (frame not in the tree at all, missing bridge) fail fast instead
    // of producing N pages of the same error.
    try {
      (void)tf_buffer.lookupTransform(
        args.from_frame, args.to_frame, tf2::TimePoint(std::chrono::nanoseconds(timeline.front())));
    } catch (const tf2::LookupException & e) {
      BAGWIZ_LOG_ERROR(kLogger, "TF chain error: %s", e.what());
      return 1;
    } catch (const tf2::ConnectivityException & e) {
      BAGWIZ_LOG_ERROR(kLogger, "TF chain error: %s", e.what());
      return 1;
    } catch (const tf2::ExtrapolationException &) {
      // Expected at chain edges; the per-step renderer reports this
      // inline so the user can navigate past it.
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
    // When the current step's lookup fails with extrapolation, the
    // renderer fills this with the timeline index of the first/last
    // valid step so the [s] handler below can jump there directly.
    // Cleared on entry to every render so a successful lookup
    // disables the jump key.
    std::optional<std::size_t> jump_target;

    auto render = [&]() {
      jump_target.reset();
      fmt::print(stdout, "\x1b[2J\x1b[H");
      const std::int64_t ts = timeline[index];
      fmt::print(stdout, "[STEP {} / {}]  {}\n", index + 1, timeline.size(), format_timestamp(ts));
      fmt::print(stdout, "TF: {}  ->  {}\n\n", args.from_frame, args.to_frame);

      try {
        const auto tf = tf_buffer.lookupTransform(
          args.from_frame, args.to_frame, tf2::TimePoint(std::chrono::nanoseconds(ts)));
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
          tf2::Quaternion q(
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
            tf.transform.rotation.w);
          double roll = 0.0;
          double pitch = 0.0;
          double yaw = 0.0;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
          fmt::print(stdout, "rotation (euler, rad):\n");
          fmt::print(stdout, "  roll:  {:.15g}\n", roll);
          fmt::print(stdout, "  pitch: {:.15g}\n", pitch);
          fmt::print(stdout, "  yaw:   {:.15g}\n", yaw);
        }
      } catch (const tf2::ExtrapolationException & e) {
        // The TF chain exists in this bag but is not yet (or no
        // longer) being published at the current step's stamp. Tell
        // the user where the chain *is* available and which key to
        // press to get there, instead of dumping tf2's two-line
        // error text.
        const auto info = parse_extrapolation(e.what());
        const double current_s = static_cast<double>(ts) / 1e9;

        if (info.past) {
          fmt::print(
            stdout, "⚠  TF '{}' -> '{}' has no data yet at this step.\n", args.from_frame,
            args.to_frame);
        } else if (info.future) {
          fmt::print(
            stdout, "⚠  TF '{}' -> '{}' is no longer published past this step.\n", args.from_frame,
            args.to_frame);
        } else {
          fmt::print(
            stdout, "⚠  TF '{}' -> '{}' is unavailable at this step.\n", args.from_frame,
            args.to_frame);
        }

        if (info.stamp_parsed) {
          const double delta = info.stamp_s - current_s;
          fmt::print(
            stdout, "   {} publication: {:+.3f}s ({:.9f})\n",
            info.past     ? "first"
            : info.future ? "last"
                          : "boundary",
            delta, info.stamp_s);

          // Locate the timeline step that brackets the boundary so
          // the [s] key can jump there in one keystroke. lower_bound
          // for "past" = first step at-or-after the earliest
          // available stamp; upper_bound-1 for "future" = last step
          // at-or-before the latest available stamp.
          const std::int64_t boundary_ns = static_cast<std::int64_t>(info.stamp_s * 1e9);
          if (info.past) {
            const auto it = std::lower_bound(timeline.begin(), timeline.end(), boundary_ns);
            if (it != timeline.end()) {
              jump_target = static_cast<std::size_t>(std::distance(timeline.begin(), it));
              fmt::print(
                stdout, "   Press [s] to jump to step {} (first valid).\n", *jump_target + 1);
            }
          } else if (info.future) {
            const auto it = std::upper_bound(timeline.begin(), timeline.end(), boundary_ns);
            if (it != timeline.begin()) {
              jump_target = static_cast<std::size_t>(std::distance(timeline.begin(), it)) - 1;
              fmt::print(
                stdout, "   Press [s] to jump to step {} (last valid).\n", *jump_target + 1);
            }
          }
        }
      } catch (const tf2::LookupException & e) {
        fmt::print(stdout, "⚠  Frame is not present in this bag's TF tree: {}\n", e.what());
      } catch (const tf2::ConnectivityException & e) {
        fmt::print(
          stdout, "⚠  No path between '{}' and '{}' in this bag's TF tree: {}\n", args.from_frame,
          args.to_frame, e.what());
      } catch (const tf2::TransformException & e) {
        fmt::print(stdout, "⚠  Lookup failed at this step: {}\n", e.what());
      }

      fmt::print(
        stdout,
        "\n  [→/Space] next   [←/b] prev   [g] first   [G] last   [s] skip-to-valid   [q] quit\n");
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
        case core::KeyEvent::kJumpToValid:
          // Jump to whichever step the renderer flagged as the
          // boundary of valid data. Disabled on successful lookups
          // because there is nothing meaningful to jump to.
          if (jump_target) {
            index = *jump_target;
          } else {
            status = "(this step is already valid; nothing to jump to)";
          }
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
