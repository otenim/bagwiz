// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/compress.hpp"

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

constexpr const char * kLogger = "bagwiz.cmd.compress";

// Resolve the target storage backend for the write. Same precedence as
// `convert format` (explicit --storage, then the output path's extension,
// then the input's detected storage) so the two repack commands agree on
// what an extension-less output means. Returns Format::Auto and sets
// `error_out` when nothing resolves; the returned format is never
// Format::Auto on success.
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

int run_compress(const CompressArgs & args)
{
  // Detect the input's storage backend up-front so it can feed
  // resolve_target_storage as the fallback for directory-layout outputs
  // without --storage. Magic-byte / metadata.yaml based, so renamed
  // .mcap / .db3 inputs still classify correctly.
  const auto source_format = io::detect_format(args.input_path);

  std::string err;
  const io::Format target_format =
    resolve_target_storage(args.storage, args.output_path, source_format, err);
  if (target_format == io::Format::Auto) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", err.c_str());
    return 1;
  }

  // Resolve the effective mode and validate the mode/codec/storage
  // combination before touching the filesystem, so a flag mistake never
  // costs a partial output.
  std::string mode = args.mode;
  if (mode == "auto") {
    // MCAP's only compression shape is its storage-native chunk compression,
    // which `--mode file` names here even though it stays out of the
    // metadata. For SQLite3 the per-message mode is the default because it
    // leaves the shard readable in place; a FILE-mode envelope has to be
    // expanded to a temporary .db3 before anything can be read from it.
    mode = (target_format == io::Format::Mcap) ? "file" : "message";
  }
  if (mode == "none" && args.codec != "zstd") {
    BAGWIZ_LOG_ERROR(kLogger, "--codec %s has no effect with --mode none", args.codec.c_str());
    return 1;
  }
  if (target_format == io::Format::Mcap) {
    if (mode == "message") {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "rosbag2 defines no per-message compression for MCAP storage; use --mode file "
        "(chunk compression) or --storage sqlite3");
      return 1;
    }
  } else {
    if (mode != "none" && args.codec != "zstd") {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "sqlite3 storage supports only the 'zstd' codec (rosbag2 defines no other "
        "compression format for it); --codec lz4 is valid for MCAP outputs only");
      return 1;
    }
  }

  io::CreateOptions copts;
  copts.format = target_format;
  copts.layout = io::Layout::Auto;  // factory picks SingleFile if extension matches
  if (target_format == io::Format::Mcap) {
    copts.mcap_compression = (mode == "none") ? "none" : args.codec;
    copts.mcap_compression_level = args.level;
  } else {
    copts.sqlite3_compression_mode = mode;
    copts.sqlite3_compression_format = (mode == "none") ? "none" : "zstd";
    copts.sqlite3_compression_level = args.level;
  }

  // The sqlite3 writer factory refuses compression on a single-file output,
  // but it only gets to say so from inside open_write() — after
  // prepare_output_path() has already cleared whatever -w/--overwrite was
  // pointed at, which would destroy an existing bag and put nothing in its
  // place. Ask resolve_write_layout() for the same decision here instead, so
  // the run stops while the output is still untouched. The factory keeps its
  // own throw as the backstop for callers that do not pre-check.
  if (
    target_format == io::Format::Sqlite3 && mode != "none" &&
    io::resolve_write_layout(args.output_path, copts).layout == io::Layout::SingleFile) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "--mode %s needs a directory output: rosbag2 only decompresses when a "
      "metadata.yaml declares the mode, so the single sqlite3 file '%s' would read "
      "back as raw zstd frames with no error reported. Drop the .db3 extension to "
      "write a directory-layout bag, or pass --mode none",
      mode.c_str(), args.output_path.c_str());
    return 1;
  }

  // Refuse an occupied output path before opening the input. The check
  // removes nothing; prepare_output_path() does the removal once the run is
  // committed to writing.
  if (const auto r = core::check_output_path_free(args.output_path, args.overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    return 1;
  }

  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }

  // The run is committed now: claim the output path for real, clearing any
  // pre-existing entry under -w/--overwrite.
  if (const auto r = core::prepare_output_path(args.output_path, args.overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    return 1;
  }

  std::unique_ptr<io::BagWriter> writer;
  try {
    writer = io::open_write(args.output_path, copts);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open output %s: %s", args.output_path.c_str(), e.what());
    return 1;
  }

  // Declare every input topic with schema backfill so the output keeps
  // self-description across the re-encode.
  const std::size_t declared = declare_reader_topics(*reader, *writer, kLogger);

  // compress re-encodes every message, so it runs through the shared rewrite
  // seam with an empty suppress set on the threaded backend — the decoded
  // pipeline, never the mcap chunk pass-through (which would preserve the
  // input's chunk compression, the exact thing this command exists to
  // change). A read/write error aborts the run (fail-fast) instead of
  // silently skipping messages, which could mask partial output corruption.
  core::BagCopyCounts counts;
  try {
    const std::unordered_set<std::string> none;
    counts = core::bag_copy_filtered(
      *reader, *writer, none, "compress", core::pipeline::BackendKind::Pipelined);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "compress read/write failed: %s", e.what());
    return 1;
  }

  try {
    writer->close();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "writer->close failed: %s", e.what());
    return 1;
  }

  BAGWIZ_LOG_INFO(
    kLogger, "Compression done: %" PRIu64 " message(s) written across %zu topic(s)", counts.copied,
    declared);

  return 0;
}

// `bagwiz compress` re-encodes a rosbag with a different compression setup:
// MCAP chunk compression (zstd/lz4) for MCAP outputs, rosbag2 MESSAGE-mode
// (per-message zstd) or FILE-mode (.db3.zstd envelope) for SQLite3 directory
// outputs, and `--mode none` to decompress back to plain storage.
class CompressCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "compress"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Compress or decompress a rosbag";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("-i,--input", args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    app.add_option("-o,--output", args_.output_path, "Output rosbag (file or directory)")
      ->required();
    app
      .add_option(
        "--mode", args_.mode,
        "Compression mode: 'file' (MCAP chunk compression, or the whole-shard .db3.zstd "
        "envelope for sqlite3), 'message' (per-message zstd, sqlite3 only), 'none' "
        "(decompress), or 'auto' (file for mcap, message for sqlite3)")
      ->check(CLI::IsMember({"auto", "file", "message", "none"}))
      ->capture_default_str();
    app
      .add_option(
        "--codec", args_.codec,
        "Compression codec (lz4 is valid for MCAP chunk compression only; sqlite3 "
        "storage is always zstd)")
      ->check(CLI::IsMember({"zstd", "lz4"}))
      ->capture_default_str();
    app
      .add_option(
        "--level", args_.level,
        "Encoder effort: fastest, fast, default, slow, or slowest (default: the codec's "
        "own default)")
      ->check(CLI::IsMember({"fastest", "fast", "default", "slow", "slowest"}));
    app
      .add_option(
        "--storage", args_.storage,
        "Target storage backend (default: inferred from the output extension when it "
        "is .mcap or .db3; otherwise the input bag's storage backend is reused)")
      ->check(CLI::IsMember({"mcap", "sqlite3"}));
    app.add_flag(
      "-w,--overwrite", args_.overwrite,
      "Replace <output> if it already exists. Without this flag, an "
      "existing output path stops the run.");
  }

  int run() override { return run_compress(args_); }

private:
  CompressArgs args_;
};

BAGWIZ_REGISTER_COMMAND(CompressCommand)

}  // namespace bagwiz::commands
