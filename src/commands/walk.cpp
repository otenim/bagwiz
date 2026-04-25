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
#include "bagwiz/core/message_formatter.hpp"
#include "bagwiz/core/terminal_input.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <fmt/core.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <exception>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.walk";

// Cached owning copy of a single bag message. RawMessage's span is
// invalidated by the next BagReader::next() call, so walk must take a
// copy to allow backward navigation.
struct OwnedMessage
{
  int64_t timestamp_ns = 0;
  std::vector<std::byte> payload;
};

std::string format_timestamp(int64_t ns)
{
  const auto seconds = static_cast<std::time_t>(ns / 1'000'000'000);
  const auto nanos = static_cast<int64_t>(ns % 1'000'000'000);
  std::tm tm_utc{};
  ::gmtime_r(&seconds, &tm_utc);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_utc);
  return fmt::format("{}.{:09d} UTC ({}.{:09d})", buf, nanos, seconds, nanos);
}

std::vector<std::byte> copy_payload(std::span<const std::byte> src)
{
  return std::vector<std::byte>(src.begin(), src.end());
}

// Query the terminal's current row count. Falls back to a sane default so
// the pager still works on pipes or ioctl-hostile environments.
int terminal_rows()
{
  struct winsize ws{};
  if (::ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_row > 0) {
    return static_cast<int>(ws.ws_row);
  }
  return 24;
}

// Split a '\n'-delimited string into line views. A trailing '\n' does not
// produce an empty final element; a missing trailing '\n' keeps the tail.
std::vector<std::string_view> split_lines(const std::string & s)
{
  std::vector<std::string_view> out;
  std::size_t start = 0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '\n') {
      out.emplace_back(s.data() + start, i - start);
      start = i + 1;
    }
  }
  if (start < s.size()) {
    out.emplace_back(s.data() + start, s.size() - start);
  }
  return out;
}

}  // namespace

// `bagwiz walk <input> <topic>` walks the messages of a single topic one
// at a time and renders each payload as YAML, mirroring what `ros2 topic
// echo` produces. Decoding relies on the rosidl introspection typesupport
// library for the topic's message type; if that library is not installed
// the command exits with a pointer to the missing package.
//
// The view is a pager: long messages stay anchored at the top and the
// body can be scrolled within the current terminal window.
// Keys:
//   right / Space : next message (wraps from last back to first)
//   left / b      : previous message
//   up / k        : scroll body up one line
//   down / j      : scroll body down one line
//   Home / H      : jump body scroll to the head
//   End / T       : jump body scroll to the tail
//   g / G         : jump to first / last message (G forces a full scan)
//   q / Ctrl-C    : quit
// Messages are cached lazily so `prev` stays O(1) for anything already
// seen and `G` is the only key that can trigger a full-remaining scan.
class WalkCommand : public Command
{
public:
  std::string_view name() const override { return "walk"; }
  std::string_view description() const override
  {
    return "Walk messages of a topic as decoded YAML";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("input", input_path_, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    app.add_option("topic", topic_, "Topic name to inspect")->required();
  }

  int run() override
  {
    if (!::isatty(STDIN_FILENO) || !::isatty(STDOUT_FILENO)) {
      BAGWIZ_LOG_ERROR(kLogger, "walk requires an interactive terminal (stdin+stdout must be TTY)");
      return 1;
    }

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

    io::ReadFilter read_filter;
    read_filter.topics.push_back(topic_);
    reader->set_filter(read_filter);

    // Copy the fields we need off of the TopicInfo before iteration; its
    // backing span lives as long as the reader but referencing it via
    // pointer alongside owned state is easy to get wrong.
    const std::string topic_name = topic_info->name;
    const std::string type_name = topic_info->type;

    // Decoding is mandatory: bail out before entering the TUI if the
    // introspection typesupport library is not installed. The user can
    // source the right workspace and re-run.
    const core::IntrospectionLoad introspection = core::load_introspection(type_name);
    if (!introspection.ok()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Message type '%s' could not be loaded (tried %s; error: %s). Source a workspace that "
        "provides the package and re-run.",
        type_name.c_str(), introspection.library_name.c_str(), introspection.error.c_str());
      return 1;
    }

    std::vector<OwnedMessage> cache;
    bool exhausted = false;

    auto load_next = [&]() -> bool {
      if (exhausted) {
        return false;
      }
      io::RawMessage raw;
      try {
        if (!reader->next(raw)) {
          exhausted = true;
          return false;
        }
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "read error: %s", e.what());
        exhausted = true;
        return false;
      }
      cache.push_back({raw.timestamp_ns, copy_payload(raw.payload)});
      return true;
    };

    if (!load_next()) {
      fmt::print(
        stdout, "No messages found for topic '{}' in {}.\n", topic_name, input_path_.string());
      return 0;
    }

    core::TerminalRawMode raw_mode;
    if (!raw_mode.active()) {
      BAGWIZ_LOG_ERROR(kLogger, "failed to enter raw terminal mode");
      return 1;
    }

    std::size_t index = 0;
    std::size_t scroll = 0;
    std::string status;
    // Reserve rows for: 3-line header + blank + blank + 2 footer lines
    // (+1 status). 7 covers the worst case; min of 1 keeps tiny terminals
    // showing at least one body line rather than zero.
    constexpr int kOverheadRows = 7;

    auto render = [&]() {
      fmt::print(stdout, "\x1b[2J\x1b[H");
      const auto & msg = cache[index];
      const char * total_suffix = exhausted ? "" : "+";
      fmt::print(
        stdout, "[{} / {}{}]  {}  {}\n", index + 1, cache.size(), total_suffix, topic_name,
        type_name);
      fmt::print(stdout, "timestamp: {}\n", format_timestamp(msg.timestamp_ns));
      fmt::print(stdout, "size:      {} bytes\n\n", msg.payload.size());

      const auto formatted = core::format_message(introspection, msg.payload);

      const int rows = std::max(1, terminal_rows() - kOverheadRows);
      std::string scroll_hint;

      if (formatted.ok()) {
        const auto lines = split_lines(formatted.text);
        const std::size_t total_body_lines = lines.size();
        const std::size_t max_scroll = total_body_lines > static_cast<std::size_t>(rows)
                                         ? total_body_lines - static_cast<std::size_t>(rows)
                                         : 0;
        if (scroll > max_scroll) {
          scroll = max_scroll;
        }
        const std::size_t end = std::min(scroll + static_cast<std::size_t>(rows), total_body_lines);
        for (std::size_t i = scroll; i < end; ++i) {
          fmt::print(stdout, "{}\n", lines[i]);
        }
        if (total_body_lines > static_cast<std::size_t>(rows)) {
          scroll_hint = fmt::format("lines {}-{} of {}", scroll + 1, end, total_body_lines);
        }
      } else {
        fmt::print(stdout, "⚠  Could not decode this message: {}\n", formatted.error);
      }

      // Footer: repeated index so it stays visible on long messages, a
      // scroll indicator when applicable, the key legend, and any
      // transient status from the last action.
      fmt::print(stdout, "\n  [{} / {}{}]  {}", index + 1, cache.size(), total_suffix, topic_name);
      if (!scroll_hint.empty()) {
        fmt::print(stdout, "    {}", scroll_hint);
      }
      fmt::print(stdout, "\n");
      fmt::print(
        stdout,
        "  [→/Space] next   [←/b] prev   [↑/k] up   [↓/j] down   "
        "[Home/H] head   [End/T] tail   [g] first   [G] last   [q] quit\n");
      if (!status.empty()) {
        fmt::print(stdout, "  {}\n", status);
      }
      std::fflush(stdout);
    };

    status.clear();
    render();

    while (true) {
      const core::KeyEvent ev = core::read_key_event();
      status.clear();
      switch (ev) {
        case core::KeyEvent::kNext:
          if (index + 1 < cache.size()) {
            ++index;
          } else if (load_next()) {
            index = cache.size() - 1;
          } else {
            // Past the last message: wrap back to the top. cache is
            // always non-empty here because we required >= 1 message
            // before entering the TUI.
            index = 0;
            status = "(wrapped to first)";
          }
          scroll = 0;
          break;
        case core::KeyEvent::kPrev:
          if (index > 0) {
            --index;
            scroll = 0;
          } else {
            status = "(at first message)";
          }
          break;
        case core::KeyEvent::kFirst:
          index = 0;
          scroll = 0;
          break;
        case core::KeyEvent::kLast: {
          std::size_t loaded = 0;
          while (load_next()) {
            ++loaded;
          }
          index = cache.size() - 1;
          scroll = 0;
          if (loaded == 0 && exhausted) {
            status = "(already at last message)";
          }
          break;
        }
        case core::KeyEvent::kScrollUp:
          if (scroll > 0) {
            --scroll;
          }
          break;
        case core::KeyEvent::kScrollDown:
          // render() clamps scroll against the message's line count, so
          // blindly incrementing here is safe.
          ++scroll;
          break;
        case core::KeyEvent::kScrollHead:
          scroll = 0;
          break;
        case core::KeyEvent::kScrollTail:
          // Saturate to the maximum value; render() clamps it down to the
          // current message's last possible scroll offset on the next draw.
          scroll = std::numeric_limits<std::size_t>::max();
          break;
        case core::KeyEvent::kQuit:
          fmt::print(stdout, "\n");
          return 0;
        case core::KeyEvent::kUnknown:
          continue;  // ignore without redraw
      }
      render();
    }
  }

private:
  std::filesystem::path input_path_;
  std::string topic_;
};

BAGWIZ_REGISTER_COMMAND(WalkCommand)

}  // namespace bagwiz::commands
