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
#include "bagwiz/core/bag/rewrite.hpp"
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

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.compress";

// The backend an explicit --storage names. CLI::IsMember has already
// restricted the value to "mcap" / "sqlite3" by the time this is reached.
io::Format format_from_storage_flag(const std::string & storage_flag)
{
  return (storage_flag == "sqlite3") ? io::Format::Sqlite3 : io::Format::Mcap;
}

const char * format_name(io::Format format)
{
  return (format == io::Format::Sqlite3) ? "sqlite3" : "mcap";
}

// Resolve the target storage backend for a -o write. Same precedence as
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
    return format_from_storage_flag(storage_flag);
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

// The backend an in-place run writes back in: the input's own. Logs and
// returns Format::Auto when the input cannot be classified, or when --storage
// asks for a different backend than the bag already uses.
io::Format resolve_inplace_storage(
  const std::filesystem::path & input_path, const std::string & storage_flag,
  io::Format source_format)
{
  if (source_format == io::Format::Auto) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "cannot determine the storage backend of input '%s', and an in-place run has to "
      "write it back in its own backend: pass -o <output> (with --storage <mcap|sqlite3> "
      "or a .mcap / .db3 extension) to write a new bag instead",
      input_path.c_str());
    return io::Format::Auto;
  }
  // In place the bag keeps its identity and only its compression changes.
  // Re-encoding it into the other backend would leave the path — and, for a
  // single-file bag, its extension — naming storage the bytes no longer are,
  // so --storage may only restate what the input already is.
  if (!storage_flag.empty() && format_from_storage_flag(storage_flag) != source_format) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "--storage %s cannot be applied in place: '%s' is a %s bag and an in-place run "
      "preserves its storage backend. Pass -o <output> to write the converted bag to a "
      "new path",
      storage_flag.c_str(), input_path.c_str(), format_name(source_format));
    return io::Format::Auto;
  }
  return source_format;
}

}  // namespace

int run_compress(const CompressArgs & args)
{
  // Detect the input's storage backend up-front. It is the last fallback when
  // a -o output names no backend, and in in-place mode it *is* the target:
  // the rewrite writes the bag back over itself, so its storage identity has
  // to survive. Magic-byte / metadata.yaml based, so renamed .mcap / .db3
  // inputs still classify correctly.
  const auto source_format = io::detect_format(args.input_path);
  const bool in_place = !args.output_path.has_value();

  const io::Format target_format = [&]() {
    if (in_place) {
      return resolve_inplace_storage(args.input_path, args.storage, source_format);
    }
    std::string err;
    const auto resolved =
      resolve_target_storage(args.storage, *args.output_path, source_format, err);
    if (resolved == io::Format::Auto) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err.c_str());
    }
    return resolved;
  }();
  if (target_format == io::Format::Auto) {
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

  // The sqlite3 writer factory refuses compression on a single-file output,
  // but it only gets to say so from inside open_write() — after the dispatch
  // below has already cleared whatever -w/--overwrite was pointed at (or, in
  // place, started staging a replacement for <input>), which would destroy an
  // existing bag and put nothing in its place. Resolve the layout the write
  // will use here instead, so the run stops while everything is still
  // untouched. The factory keeps its own throw as the backstop for callers
  // that do not pre-check.
  if (target_format == io::Format::Sqlite3 && mode != "none") {
    io::CreateOptions probe;
    probe.format = target_format;
    probe.layout = io::Layout::Auto;  // factory picks SingleFile if extension matches
    // In place the layout is the input's own, exactly as the dispatch pins it.
    const io::Layout layout = in_place
                                ? io::create_options_preserving_storage(args.input_path).layout
                                : io::resolve_write_layout(*args.output_path, probe).layout;
    if (layout == io::Layout::SingleFile) {
      const auto & target = in_place ? args.input_path : *args.output_path;
      BAGWIZ_LOG_ERROR(
        kLogger,
        "--mode %s needs a directory output: rosbag2 only decompresses when a "
        "metadata.yaml declares the mode, so the single sqlite3 file '%s' would read "
        "back as raw zstd frames with no error reported. %s, or pass --mode none",
        mode.c_str(), target.c_str(),
        in_place ? "Pass -o <directory> to write a directory-layout bag instead"
                 : "Drop the .db3 extension to write a directory-layout bag");
      return 1;
    }
  }

  // Refuse an occupied -o path before opening the input. The check removes
  // nothing — the dispatch does the removal once the run is committed to
  // writing — so a collision costs one stat instead of a full open of an
  // input that may be a FILE-mode bag (which expands the whole database to a
  // temporary .db3 first).
  if (!in_place) {
    if (const auto r = core::check_output_path_free(*args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
  }

  // Open the input before the dispatch so a bag that cannot be read costs
  // nothing: once the dispatch runs, -w/--overwrite has already cleared the
  // -o path and the in-place branch is staging <input>'s replacement.
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }

  // -o vs in-place dispatch, shared with the other rewrite-style commands:
  // -o writes a fresh bag in the backend resolved above and leaves <input>
  // untouched; otherwise <input> is rewritten atomically via a sibling tmp,
  // preserving its storage format and layout. The compression knobs are
  // carried by the options so both branches encode identically.
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "compress: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "compress: pass failed; aborting in-place swap";
  rewrite_opts.output_format = target_format;
  if (target_format == io::Format::Mcap) {
    rewrite_opts.mcap_compression = (mode == "none") ? "none" : args.codec;
    rewrite_opts.mcap_compression_level = args.level;
  } else {
    // mcap_compression keeps the dispatch's "none" default and is simply
    // inert here — only the mcap writer reads it. The sqlite3 triple is what
    // configures this write.
    rewrite_opts.sqlite3_compression_mode = mode;
    rewrite_opts.sqlite3_compression_format = (mode == "none") ? "none" : "zstd";
    rewrite_opts.sqlite3_compression_level = args.level;
  }

  return core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & open_writer, const core::RewriteTarget & target) {
      std::unique_ptr<io::BagWriter> writer;
      try {
        writer = open_writer();
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "Failed to open output %s: %s", target.path.c_str(), e.what());
        return 1;
      }

      // Declare every input topic with schema backfill so the output keeps
      // self-description across the re-encode.
      const std::size_t declared = declare_reader_topics(*reader, *writer, kLogger);

      // compress re-encodes every message, so it runs through the shared
      // rewrite seam with an empty suppress set on the threaded backend — the
      // decoded pipeline, never the mcap chunk pass-through (which would
      // preserve the input's chunk compression, the exact thing this command
      // exists to change). A read/write error aborts the run (fail-fast)
      // instead of silently skipping messages, which could mask partial
      // output corruption.
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

      // Release the input before returning: in-place mode swaps the staged
      // tmp over <input> the moment this pass succeeds, and holding the bag
      // it is about to unlink open is a trap even where the OS tolerates it.
      reader.reset();

      BAGWIZ_LOG_INFO(
        kLogger, "Compression done: %" PRIu64 " message(s) written across %zu topic(s)",
        counts.copied, declared);
      return 0;
    });
}

// `bagwiz compress` re-encodes a rosbag with a different compression setup:
// MCAP chunk compression (zstd/lz4) for MCAP outputs, rosbag2 MESSAGE-mode
// (per-message zstd) or FILE-mode (.db3.zstd envelope) for SQLite3 directory
// outputs, and `--mode none` to decompress back to plain storage. With -o the
// result lands in a new bag; without it <input> is rewritten in place,
// keeping its storage backend and layout.
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
    app.add_option(
      "-o,--output", args_.output_path,
      "Write the result to this new bag (file or directory) instead of rewriting <input> "
      "in place.");
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
        "Target storage backend (default: inferred from the -o extension when it is "
        ".mcap or .db3; otherwise the input bag's storage backend is reused). In-place "
        "runs preserve the input's backend, so only a value naming it is accepted there")
      ->check(CLI::IsMember({"mcap", "sqlite3"}));
    app.add_flag(
      "-w,--overwrite", args_.overwrite,
      "Replace an existing -o/--output path. Without it, an existing output path stops the "
      "run. Has no effect in in-place mode (when -o is omitted, <input> is replaced "
      "atomically by design).");
  }

  int run() override { return run_compress(args_); }

private:
  CompressArgs args_;
};

BAGWIZ_REGISTER_COMMAND(CompressCommand)

}  // namespace bagwiz::commands
