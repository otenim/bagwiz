// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/commands/topic_types.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/terminal_input.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/msg_yaml/message_formatter.hpp"
#include "bagwiz/core/tui/image/terminal_image_caps.hpp"
#include "bagwiz/core/tui/layout.hpp"
#include "bagwiz/core/tui/pager.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "walk_bag.hpp"      // NOLINT(build/include_subdir) src-local shared header
#include "walk_cursor.hpp"   // NOLINT(build/include_subdir) src-local shared header
#include "walk_frame.hpp"    // NOLINT(build/include_subdir) src-local shared header
#include "walk_help.hpp"     // NOLINT(build/include_subdir) src-local shared header
#include "walk_overlay.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "walk_preview.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "walk_save.hpp"     // NOLINT(build/include_subdir) src-local shared header

#include <fmt/core.h>
#include <unistd.h>

#include <cstddef>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.walk";

}  // namespace

// `bagwiz walk -i <input> -t <topic>` walks the messages of a single topic
// one at a time and renders each payload as YAML, mirroring what
// `ros2 topic echo` produces. Decoding prefers the schema-driven path when
// the MCAP carries a usable `ros2msg` schema, falling back to the rosidl
// introspection typesupport library otherwise.
//
// The view is a pager driven by `bagwiz::core::tui::ScrollablePager`:
// the header (timestamp, size) and footer (index + topic/type + scroll
// hint, key legend, status row) are pinned in place,
// and only the body region scrolls.
//
// Keys (the footer shows the current working set; '?' opens the full
// reference overlay in both the YAML view and the image preview):
//   right / Space : next message (wraps from last back to first)
//   left / b      : previous message
//   , . < >       : jump ~1 second backward/forward (~10 seconds for < >)
//   up/k down/j   : scroll the body; Home/H and End/T jump to head/tail
//   g / G         : jump to first / last message (G forces a full scan)
//   S             : save current message as yaml; inside the image preview,
//                   save the displayed frame as a PNG (both prompt for a path)
//   a             : toggle full-expansion of long primitive arrays
//   i             : toggle in-terminal image preview (image topics on a
//                   Kitty/Sixel-capable terminal; absent otherwise)
//   Esc           : back out one level — close the help, leave the
//                   preview; absorbed at the YAML view
//   q / Q         : quit walk (bound in the YAML view only; inert in the
//                   preview, the pickers and the help overlays)
//   Ctrl-C / Ctrl-D : terminate walk outright, from any screen
// Inside the image preview u/p/t (rectify, pcd overlay, topic picker) and
// the overlay adjusters f/c/r/=/-/]/[ apply.
// Messages are cached lazily so `prev` stays O(1) for anything already
// seen and `G` is the only key that always triggers a full-remaining scan
// (the forward time steps `.` / `>` read ahead only as far as the target
// timestamp, which drains the source when no later message exists).
class WalkCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "walk"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Walk messages of a topic as decoded YAML";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("-i,--input", input_path_, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    set_topic_input(app, input_path_);
    add_topic_option(
      app, "-t,--topic", topic_, "Topic name to inspect",
      TopicSlotSpec{.mode = TopicSelectorMode::kLiteral})
      ->required();
    add_topic_option(
      app, "--cam-info", camera_info_topic_,
      "Explicit CameraInfo topic for the rectify preview and the point-cloud projection "
      "overlay",
      TopicSlotSpec{.allowed_types = kCameraInfoType, .mode = TopicSelectorMode::kLiteral});
  }

  int run() override
  {
    if (!::isatty(STDIN_FILENO) || !::isatty(STDOUT_FILENO)) {
      BAGWIZ_LOG_ERROR(kLogger, "walk requires an interactive terminal (stdin+stdout must be TTY)");
      return 1;
    }

    auto opened = open_bag_and_find_topic(input_path_, topic_, kLogger);
    if (!opened.has_value()) {
      return 1;
    }
    auto & reader = *opened->reader;

    const std::vector<std::string> pcd_topics = collect_pcd_topics(reader);

    const auto camera =
      resolve_walk_camera_info(input_path_, topic_, camera_info_topic_, reader.topics());
    const auto & camera_info = camera.info;
    const auto & camera_info_error = camera.error;

    io::ReadFilter read_filter;
    read_filter.topics.push_back(topic_);
    reader.set_filter(read_filter);

    const std::string topic_name = opened->topic_info->name;
    const std::string type_name = opened->topic_info->type;

    auto open_decoder = core::decoder::open_decoder(*opened->topic_info);
    if (!open_decoder.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open decoder: %s", open_decoder.error.c_str());
      return 1;
    }
    const auto & decoder = *open_decoder.decoder;

    std::string status;
    // The cursor owns the lazy message cache; its source pulls from the
    // filtered reader and logs read errors at the point they happen.
    MessageCursor cursor(
      [&reader](OwnedMessage & msg) {
        io::RawMessage raw;
        try {
          if (!reader.next(raw)) {
            return false;
          }
        } catch (const std::exception & e) {
          BAGWIZ_LOG_ERROR(kLogger, "read error: %s", e.what());
          return false;
        }
        msg.timestamp_ns = raw.timestamp_ns;
        msg.payload = copy_payload(raw.payload);
        return true;
      },
      status);

    if (!cursor.load_next()) {
      BAGWIZ_LOG_INFO(
        kLogger, "No messages found for topic '%s' in %s.", topic_name.c_str(),
        input_path_.string().c_str());
      return 0;
    }

    // Image preview is offered only for topics to_packed_raster can decode and
    // only on terminals that speak a graphics protocol. Probe the terminal once,
    // before the pager owns stdin; the probe briefly enters raw mode so the
    // reply bytes arrive immediately instead of being held by line buffering.
    const bool is_image_topic = core::image::is_supported_image_type(type_name);
    core::tui::image::TerminalImageCaps image_caps;
    if (is_image_topic) {
      core::TerminalRawMode probe_raw;
      image_caps = core::tui::image::detect_terminal_image_caps(
        std::cout, STDIN_FILENO, core::tui::query_terminal_size());
    }
    const bool preview_available = is_image_topic && image_caps.can_render();

    bool expand_arrays = false;
    // The '?' overlay replaces the frame with the key reference; the body
    // scroll it hijacks is restored on close so the YAML view resumes where
    // it was.
    bool show_help = false;
    std::size_t help_saved_scroll = 0;

    core::tui::PagerConfig pager_cfg;
    core::tui::ScrollablePager pager(pager_cfg);

    PcdOverlayController overlay(input_path_, reader, pcd_topics, status);
    ImagePreviewSession preview(
      cursor, overlay, pager, status, topic_name, type_name, image_caps, camera_info,
      camera_info_error);

    auto build_frame = [&](std::size_t scroll, core::tui::Size term) -> core::tui::Frame {
      if (show_help) {
        return build_help_frame(term, yaml_help_lines());
      }
      return build_yaml_frame(
        scroll, term, cursor, decoder, expand_arrays, topic_name, type_name, status,
        preview_available);
    };

    const auto close_help = [&] {
      show_help = false;
      pager.set_scroll_offset(help_saved_scroll);
    };

    auto on_nav = [&](core::tui::NavKey nav) -> core::tui::AppKeyResult {
      status.clear();
      if (show_help) {
        // The overlay accepts only its own keys: Esc closes it, resize
        // repaints it, the scroll keys (handled inside the pager) move it,
        // and every other key — q included — is swallowed so a reference
        // lookup can neither act behind the card nor end the session.
        // (Ctrl-C / Ctrl-D never reaches this callback: the pager exits on
        // kTerminate before offering the key here.)
        switch (nav) {
          case core::tui::NavKey::kBack:
            close_help();
            return core::tui::AppKeyResult::kHandled;
          case core::tui::NavKey::kResize:
          case core::tui::NavKey::kQuit:  // kHandled keeps the pager alive
            return core::tui::AppKeyResult::kHandled;
          default:
            return core::tui::AppKeyResult::kIgnored;
        }
      }
      if (nav == core::tui::NavKey::kBack) {
        // Nothing to back out of at the top level: absorb Esc so a reflexive
        // press cannot end the session (quitting is q / Ctrl-C / Ctrl-D).
        return core::tui::AppKeyResult::kHandled;
      }
      switch (nav) {
        case core::tui::NavKey::kNext:
          cursor.navigate(MsgNav::kNext);
          pager.set_scroll_offset(0);
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kPrev:
          // Preserve the YAML view's behaviour: only reset the body scroll when
          // the cursor actually moved (pressing prev at the first message keeps
          // the current scroll position).
          if (cursor.navigate(MsgNav::kPrev)) {
            pager.set_scroll_offset(0);
          }
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kFirst:
          cursor.navigate(MsgNav::kFirst);
          pager.set_scroll_offset(0);
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kLast:
          cursor.navigate(MsgNav::kLast);
          pager.set_scroll_offset(0);
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kStepForward1s:
          cursor.navigate(MsgNav::kStepForward1s);
          pager.set_scroll_offset(0);
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kStepForward10s:
          cursor.navigate(MsgNav::kStepForward10s);
          pager.set_scroll_offset(0);
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kStepBackward1s:
          // Like prev, preserve body scroll when already at the boundary.
          if (cursor.navigate(MsgNav::kStepBackward1s)) {
            pager.set_scroll_offset(0);
          }
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kStepBackward10s:
          if (cursor.navigate(MsgNav::kStepBackward10s)) {
            pager.set_scroll_offset(0);
          }
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kResize:
          return core::tui::AppKeyResult::kHandled;
        default:
          return core::tui::AppKeyResult::kIgnored;
      }
    };

    auto on_app_key = [&](core::KeyEvent ev) -> core::tui::AppKeyResult {
      status.clear();
      if (ev == core::KeyEvent::kHelp && !show_help) {
        show_help = true;
        help_saved_scroll = pager.scroll_offset();
        pager.set_scroll_offset(0);
        return core::tui::AppKeyResult::kHandled;
      }
      if (show_help) {
        // Same swallow rule as the navigation keys above: Esc alone leaves
        // the reference ('?' only opens it).
        return core::tui::AppKeyResult::kIgnored;
      }
      switch (ev) {
        case core::KeyEvent::kToggleArrayExpand:
          expand_arrays = !expand_arrays;
          if (expand_arrays) {
            status = "(arrays: expanded)";
          }
          return core::tui::AppKeyResult::kHandled;
        case core::KeyEvent::kSaveYaml: {
          const auto & cur = cursor.cache()[cursor.index()];
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
            fmt::format("{}_{}.yaml", topic_for_filename(topic_name), cursor.index());
          std::filesystem::path cwd;
          try {
            cwd = std::filesystem::current_path();
          } catch (const std::exception & e) {
            status = fmt::format("cannot resolve working directory: {}", e.what());
            return core::tui::AppKeyResult::kHandled;
          }
          save_bytes_with_prompt(
            pager, "Save YAML path", cwd, default_base, std::as_bytes(std::span{formatted.text}),
            status);
          return core::tui::AppKeyResult::kHandled;
        }
        case core::KeyEvent::kTogglePreview:
          if (!is_image_topic) {
            status = "(not an image topic)";
            return core::tui::AppKeyResult::kHandled;
          }
          if (!image_caps.can_render()) {
            status = "(image preview not supported in this terminal)";
            return core::tui::AppKeyResult::kHandled;
          }
          if (preview.run() == ImagePreviewSession::Exit::kTerminate) {
            // Ctrl-C / Ctrl-D inside the preview ends the whole session.
            return core::tui::AppKeyResult::kQuit;
          }
          return core::tui::AppKeyResult::kHandled;
        default:
          return core::tui::AppKeyResult::kIgnored;
      }
    };

    const int exit_code = pager.run(build_frame, on_nav, on_app_key);

    return exit_code;
  }

private:
  std::filesystem::path input_path_;
  std::string topic_;
  std::optional<std::string> camera_info_topic_;
};

BAGWIZ_REGISTER_COMMAND(WalkCommand)

}  // namespace bagwiz::commands
