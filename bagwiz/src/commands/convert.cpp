// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/bag/bag_copy.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "topic_declare.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <cinttypes>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.convert";

// Resolve the target storage backend for a write. Precedence (first match
// wins):
//   1. Explicit `--storage <mcap|sqlite3>` from the CLI.
//   2. Output path's extension (`.mcap` → mcap, `.db3` → sqlite3) — only
//      applies to single-file outputs that carry one of those extensions.
//   3. Input bag's detected storage backend. This is the fallback for
//      directory-layout outputs (no extension to infer from) when the user
//      did not pass `--storage`: a pure layout change should not require
//      restating the storage backend.
// If none of the three resolves, `error_out` is set and `Format::Auto` is
// returned. `storage_flag` is the CLI string value (empty when the user did
// not pass `--storage`); `input_format` is the input's detected storage
// (`Format::Auto` when detection failed). The returned format is never
// `Format::Auto` on success.
io::Format resolve_target_storage(
  const std::string & storage_flag, const std::filesystem::path & output_path,
  io::Format input_format, std::string & error_out)
{
  if (!storage_flag.empty()) {
    return (storage_flag == "sqlite3") ? io::Format::Sqlite3 : io::Format::Mcap;
  }
  const auto inferred = io::infer_format_from_extension(output_path);
  if (inferred != io::Format::Auto) {
    return inferred;
  }
  if (input_format != io::Format::Auto) {
    return input_format;
  }
  error_out = "cannot determine target storage for output '" + output_path.string() +
              "': pass --storage <mcap|sqlite3>, use an output extension (.mcap or .db3), "
              "or supply an input whose storage backend can be auto-detected";
  return io::Format::Auto;
}

}  // namespace

// `bagwiz convert` is a command group for cross-format bag conversion.
// Ships `format` (ROS 2 mcap <-> sqlite3 repack, plus file <-> directory
// layout transitions inferred from the output path). It stays a group rather
// than collapsing into a flat `convert` so further conversion families can be
// added without reshaping the existing CLI surface.
class ConvertCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "convert"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Convert bag storage formats";
  }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_format(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kFormat:
        return run_format();
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kFormat };
  Subcommand selected_ = Subcommand::kNone;

  struct FormatArgs
  {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    std::string storage;     // empty when --storage not passed; resolved at run time
    bool overwrite = false;  // replace any pre-existing output_path
  } format_args_;

  void configure_format(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "format",
      "Repack a ROS 2 rosbag, converting between storage backends and/or "
      "file/directory layouts");
    sub->add_option("-i,--input", format_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "-o,--output", format_args_.output_path, "Output rosbag2 directory (or .mcap/.db3 file)")
      ->required();
    sub
      ->add_option(
        "--storage", format_args_.storage,
        "Target storage backend (default: inferred from the output extension when it "
        "is .mcap or .db3; otherwise the input bag's storage backend is reused)")
      ->check(CLI::IsMember({"mcap", "sqlite3"}));
    sub->add_flag(
      "-w,--overwrite", format_args_.overwrite,
      "Replace <output> if it already exists. Without this flag, an "
      "existing output path stops the run.");
    sub->callback([this]() { selected_ = Subcommand::kFormat; });
  }

  int run_format()
  {
    const auto & args = format_args_;

    // Detect the input's storage backend up-front so it can (a) feed
    // resolve_target_storage as the fallback for directory-layout outputs
    // without --storage, and (b) anchor the same-storage repack check
    // below. Magic-byte / metadata.yaml based, so renamed .mcap / .db3
    // inputs still classify correctly; the extension is consulted only
    // to resolve the inner storage of a single-file .zstd envelope,
    // where the magic sniff cannot see past the compression.
    const auto source_format = io::detect_format(args.input_path);

    std::string err;
    const io::Format target_format =
      resolve_target_storage(args.storage, args.output_path, source_format, err);
    if (target_format == io::Format::Auto) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err.c_str());
      return 1;
    }

    // Refuse an occupied output path before opening the input. The check
    // removes nothing, so the repack rejection below can still bail without
    // having destroyed the user's file; prepare_output_path() does the removal
    // once the run is committed to writing.
    if (const auto r = core::check_output_path_free(args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }

    auto reader = io::open_read_or_log(args.input_path, kLogger);
    if (!reader) {
      return 1;
    }

    // Reject same-storage + same-layout repacks: a plain `cp` is what the
    // user actually wants. When the layouts differ (e.g. file → directory
    // on the same backend) the run is allowed since the output shape
    // genuinely changes. Input layout is read from the filesystem (input
    // is guaranteed to exist by CLI::ExistingPath); output layout is read
    // from the path's extension (no `.mcap`/`.db3` → directory layout).
    if (source_format == target_format) {
      std::error_code ec;
      const bool input_is_directory = std::filesystem::is_directory(args.input_path, ec);
      const bool output_is_directory =
        io::infer_format_from_extension(args.output_path) == io::Format::Auto;
      if (input_is_directory == output_is_directory) {
        const char * fmt_name = (target_format == io::Format::Sqlite3) ? "sqlite3" : "mcap";
        BAGWIZ_LOG_ERROR(
          kLogger,
          "input is already in '%s' storage with the same layout; nothing to convert "
          "(use `cp -r` for a verbatim copy)",
          fmt_name);
        return 1;
      }
    }

    // The run is committed now: claim the output path for real, clearing any
    // pre-existing entry under -w/--overwrite.
    if (const auto r = core::prepare_output_path(args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }

    io::CreateOptions copts;
    copts.format = target_format;
    copts.layout = io::Layout::Auto;  // factory picks SingleFile if extension matches
    // Leave compression off so the output is predictable; callers can
    // recompress with `bagwiz compress` if they want.
    copts.mcap_compression = "none";

    std::unique_ptr<io::BagWriter> writer;
    try {
      writer = io::open_write(args.output_path, copts);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open output %s: %s", args.output_path.c_str(), e.what());
      return 1;
    }

    // Declare every input topic with schema backfill so the output keeps
    // self-description across the repack.
    const std::size_t declared = declare_reader_topics(*reader, *writer, kLogger);

    // convert re-encodes nothing (only the storage container changes), so it runs
    // through the shared rewrite seam with an empty suppress set on the threaded
    // backend. Note this is the decoded pipeline, not the mcap chunk
    // pass-through — convert never calls try_bag_passthrough_rewrite. A
    // read/write error now aborts the run (fail-fast) instead of silently
    // skipping messages, which could mask partial output corruption.
    core::BagCopyCounts counts;
    try {
      const std::unordered_set<std::string> none;
      counts = core::bag_copy_filtered(
        *reader, *writer, none, "convert", core::pipeline::BackendKind::Pipelined);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "convert read/write failed: %s", e.what());
      return 1;
    }

    try {
      writer->close();
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "writer->close failed: %s", e.what());
      return 1;
    }

    BAGWIZ_LOG_INFO(
      kLogger, "Repack done: %" PRIu64 " message(s) written across %zu topic(s)", counts.copied,
      declared);

    return 0;
  }
};

BAGWIZ_REGISTER_COMMAND(ConvertCommand)

}  // namespace bagwiz::commands
