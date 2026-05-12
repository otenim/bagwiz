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
#include "bagwiz/core/message_formatter.hpp"
#include "bagwiz/core/terminal_input.hpp"
#include "bagwiz/core/tui/pager.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <unistd.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <istream>
#include <memory>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.walk";

// header: 2 content lines (timestamp, size) + 1 blank separator.
constexpr int kHeaderRows = 3;
// footer: 1 blank separator above + (index/scroll-hint, key legend,
// status row reserved even when empty).
constexpr int kFooterRows = 4;

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

// Split a '\n'-delimited string into owned lines so the body vector
// can outlive the source string. A trailing '\n' does not produce an
// empty final element; a missing trailing '\n' keeps the tail.
std::vector<std::string> split_lines(const std::string & s)
{
  std::vector<std::string> out;
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

// ROS topic names use `/`; replace each `/` with `__` so path separators
// do not collide with underscores that appear inside topic name segments.
std::string topic_for_filename(std::string_view topic)
{
  std::string out;
  out.reserve(topic.size() * 2);
  for (unsigned char uc : topic) {
    const char c = static_cast<char>(uc);
    if (c == '/') {
      out += "__";
    } else {
      out.push_back(c);
    }
  }
  return out;
}

std::filesystem::path resolve_yaml_save_path(
  const std::string & line_from_stdin, const std::filesystem::path & cwd,
  const std::string & default_filename)
{
  std::string trimmed = line_from_stdin;
  while (!trimmed.empty() && (trimmed.back() == ' ' || trimmed.back() == '\t')) {
    trimmed.pop_back();
  }
  if (trimmed.empty()) {
    return cwd / default_filename;
  }

  std::filesystem::path user_path(trimmed);
  std::error_code ec;
  if (std::filesystem::exists(user_path, ec) && std::filesystem::is_directory(user_path, ec)) {
    return user_path / default_filename;
  }
  const char last = trimmed.back();
  if (last == '/' || last == '\\') {
    return std::filesystem::path(trimmed) / default_filename;
  }
  return user_path;
}

}  // namespace

// `bagwiz walk <input> <topic>` walks the messages of a single topic
// one at a time and renders each payload as YAML, mirroring what
// `ros2 topic echo` produces. Decoding relies on the rosidl
// introspection typesupport library (or the schema-driven path when
// the MCAP carries a `ros2msg` schema).
//
// The view is a pager driven by `bagwiz::core::tui::ScrollablePager`:
// the header (`[i/n+] topic type`, timestamp, size) and footer
// (index + scroll hint, key legend, status row) are pinned in place,
// and only the body region scrolls.
//
// Keys:
//   right / Space : next message (wraps from last back to first)
//   left / b      : previous message
//   up / k        : scroll body up one line
//   down / j      : scroll body down one line
//   Home / H      : jump body scroll to the head
//   End / T       : jump body scroll to the tail
//   g / G         : jump to first / last message (G forces a full scan)
//   s             : save current message as yaml (prompts for output path)
//   a             : toggle full-expansion of long primitive arrays
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

    const std::string topic_name = topic_info->name;
    const std::string type_name = topic_info->type;

    auto open_decoder = core::decoder::open_decoder(*topic_info);
    if (!open_decoder.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open decoder: %s", open_decoder.error.c_str());
      return 1;
    }
    const auto & decoder = *open_decoder.decoder;

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

    std::size_t index = 0;
    bool expand_arrays = false;
    std::string status;

    core::tui::PagerConfig pager_cfg;
    core::tui::ScrollablePager pager(pager_cfg);
    pager.set_layout(kHeaderRows, kFooterRows);

    // Cache the most recently rendered body so the scroll-hint in the
    // footer can reflect the visible lines without re-running the
    // formatter for every footer redraw.
    std::size_t last_body_size = 0;
    int last_body_rows = 0;

    auto build_frame = [&](std::size_t scroll, int body_rows) -> core::tui::Frame {
      core::tui::Frame frame;
      frame.header.reserve(kHeaderRows);
      frame.footer.reserve(kFooterRows);

      const auto & msg = cache[index];
      const char * total_suffix = exhausted ? "" : "+";
      const std::size_t last_loaded_index = cache.size() - 1;
      frame.header.push_back(fmt::format("timestamp: {}", format_timestamp(msg.timestamp_ns)));
      frame.header.push_back(fmt::format("size:      {} bytes", msg.payload.size()));
      frame.header.emplace_back();  // blank separator

      core::FormatOptions fmt_opts;
      fmt_opts.expand_long_arrays = expand_arrays;
      const auto decoded = decoder.decode(msg.payload);
      const auto formatted = decoded.ok() ? core::format_message(*decoded.value, fmt_opts)
                                          : core::FormatResult{"", decoded.error};
      if (formatted.ok()) {
        frame.body = split_lines(formatted.text);
      } else {
        frame.body.push_back(fmt::format("⚠  Could not decode this message: {}", formatted.error));
      }
      last_body_size = frame.body.size();
      last_body_rows = body_rows;

      std::string scroll_hint;
      if (last_body_size > static_cast<std::size_t>(body_rows)) {
        const std::size_t end =
          std::min(scroll + static_cast<std::size_t>(body_rows), last_body_size);
        scroll_hint = fmt::format("    lines {}-{} of {}", scroll + 1, end, last_body_size);
      }

      frame.footer.emplace_back();  // blank separator
      frame.footer.push_back(
        fmt::format(
          "  [{} / {}{}]  {}  {}{}", index, last_loaded_index, total_suffix, topic_name, type_name,
          scroll_hint));
      frame.footer.emplace_back(
        "  [→/Space] next   [←/b] prev   [↑/k] up   [↓/j] down   "
        "[Home/H] head   [End/T] tail   [g] first   [G] last   [s] save as yaml   "
        "[a] expand arrays   [q] quit");
      frame.footer.push_back(status.empty() ? std::string{} : fmt::format("  {}", status));
      return frame;
    };

    auto on_nav = [&](core::tui::NavKey nav) -> core::tui::AppKeyResult {
      status.clear();
      switch (nav) {
        case core::tui::NavKey::kNext:
          if (index + 1 < cache.size()) {
            ++index;
          } else if (load_next()) {
            index = cache.size() - 1;
          } else {
            index = 0;
            status = "(wrapped to first)";
          }
          pager.set_scroll_offset(0);
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kPrev:
          if (index > 0) {
            --index;
            pager.set_scroll_offset(0);
          } else {
            status = "(at first message)";
          }
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kFirst:
          index = 0;
          pager.set_scroll_offset(0);
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kLast: {
          std::size_t loaded = 0;
          while (load_next()) {
            ++loaded;
          }
          index = cache.size() - 1;
          pager.set_scroll_offset(0);
          if (loaded == 0 && exhausted) {
            status = "(already at last message)";
          }
          return core::tui::AppKeyResult::kHandled;
        }
        case core::tui::NavKey::kResize:
          return core::tui::AppKeyResult::kHandled;
        default:
          return core::tui::AppKeyResult::kIgnored;
      }
    };

    auto on_app_key = [&](core::KeyEvent ev) -> core::tui::AppKeyResult {
      status.clear();
      switch (ev) {
        case core::KeyEvent::kToggleArrayExpand:
          expand_arrays = !expand_arrays;
          if (expand_arrays) {
            status = "(arrays: expanded)";
          }
          return core::tui::AppKeyResult::kHandled;
        case core::KeyEvent::kSaveYaml: {
          const auto & cur = cache[index];
          core::FormatOptions save_opts;
          save_opts.expand_long_arrays = expand_arrays;
          const auto decoded = decoder.decode(cur.payload);
          const auto formatted = decoded.ok() ? core::format_message(*decoded.value, save_opts)
                                              : core::FormatResult{"", decoded.error};
          if (!formatted.ok()) {
            status = fmt::format("cannot save: {}", formatted.error);
            return core::tui::AppKeyResult::kHandled;
          }
          const std::string default_base =
            fmt::format("{}_{}.yaml", topic_for_filename(topic_name), index);
          std::filesystem::path cwd;
          try {
            cwd = std::filesystem::current_path();
          } catch (const std::exception & e) {
            status = fmt::format("cannot resolve working directory: {}", e.what());
            return core::tui::AppKeyResult::kHandled;
          }
          const std::filesystem::path default_full = cwd / default_base;

          std::filesystem::path out_path;
          bool save_ok = false;
          std::string failure_status;
          pager.with_line_input([&](std::istream & in, std::ostream & out) {
            out << fmt::format("Save YAML path (Enter for {}):\n", default_full.string());
            out.flush();
            std::string line;
            if (!std::getline(in, line)) {
              failure_status = "(save cancelled)";
              return;
            }
            out_path = resolve_yaml_save_path(line, cwd, default_base);
            std::error_code mk_ec;
            const auto parent = out_path.parent_path();
            if (!parent.empty()) {
              std::filesystem::create_directories(parent, mk_ec);
            }
            std::ofstream of(out_path, std::ios::binary);
            if (!of) {
              failure_status = fmt::format("could not open {} for writing", out_path.string());
              return;
            }
            of << formatted.text;
            if (!of.good()) {
              failure_status = fmt::format("write failed: {}", out_path.string());
              return;
            }
            save_ok = true;
          });
          if (save_ok) {
            status = fmt::format("saved {}", out_path.string());
          } else {
            status = failure_status;
          }
          return core::tui::AppKeyResult::kHandled;
        }
        default:
          return core::tui::AppKeyResult::kIgnored;
      }
    };

    return pager.run(build_frame, on_nav, on_app_key);
  }

private:
  std::filesystem::path input_path_;
  std::string topic_;
};

BAGWIZ_REGISTER_COMMAND(WalkCommand)

}  // namespace bagwiz::commands
