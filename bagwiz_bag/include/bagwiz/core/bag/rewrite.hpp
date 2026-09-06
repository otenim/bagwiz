// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BAG__REWRITE_HPP_
#define BAGWIZ__CORE__BAG__REWRITE_HPP_

#include "bagwiz/io/bag_open.hpp"

#include <filesystem>
#include <functional>
#include <optional>
#include <string>

// The shared "-o vs in-place" dispatch every rewrite-style command — topic
// drop/keep/rename, trim, traj join, tf static cp/join/drop/update, cam-info
// replace/recompute-p, stamp sync, pcd concat/undistort/compress/decompress,
// video encode/decode, compress — runs after validating its arguments:
//
//   * with -o: guard the output path (prepare_output_path), then run the
//     command's pass with a writer factory targeting the output path.
//   * without -o: rewrite <input> atomically via write_bag_inplace, running
//     the same pass against the staged tmp path with the input's storage
//     format and layout pinned (a directory bag's path carries no extension
//     for Auto to resolve from, and Auto would fall through to the
//     Directory + Mcap default), and abort the swap when the pass reports
//     failure.
//
// Both branches give the output the same shape, which is what makes every
// bagwiz rewrite look alike to its user: the layout follows the -o path
// (a `.mcap` / `.db3` extension names a single file, anything else a
// directory), the storage format follows that extension and otherwise the
// input's, and the compression is carried over from the input, translated
// to the output storage (io::create_options_inheriting_compression). A
// command overrides only what it exists to change — `compress` its codec,
// `pcd undistort --compression` likewise — through BagRewriteOptions.
//
// Centralising the dispatch keeps the clobber policy, the Format::Auto
// guard, that output-shape policy, and the pass-status-to-exception
// translation identical across commands. The pass itself stays with the
// command.
namespace bagwiz::core
{

// Knobs and message texts for run_bag_rewrite. The messages are parameters
// (not fixed strings) because each command has always emitted its own
// wording; they must stay byte-identical to what the command printed before
// the dispatch was shared.
struct BagRewriteOptions
{
  // Logger name used for every message the dispatch emits.
  const char * logger = nullptr;

  // Logged (ERROR) in in-place mode when the input's storage format cannot be
  // detected. printf-style format with exactly one "%s", filled with the
  // input path.
  const char * format_unknown_error = nullptr;

  // Text of the std::runtime_error thrown to abort the in-place swap when the
  // pass returns non-zero. Never printed: the catch path returns the pass's
  // exit code, since the pass has already logged the specific error.
  const char * pass_failed_error = nullptr;

  // -o mode only: pin the output's storage format outright, leaving only the
  // layout to the output path's extension. For commands that resolve the
  // target backend themselves and must not have that decision re-derived
  // here — `compress --storage` outranks both the output extension and the
  // input's format, an order create_options_inheriting_format cannot express.
  // Format::Auto (the default) resolves the format from the output path's
  // extension and otherwise inherits the input's, through
  // io::create_options_inheriting_format. Ignored in in-place mode, where the
  // input's own storage is preserved and the command is expected to have
  // rejected a conflicting request already.
  io::Format output_format = io::Format::Auto;

  // Overrides for the writer's compression, applied in both modes. Empty
  // (the default) carries the input's compression over to the output —
  // the storage-appropriate translation io::create_options_inheriting_compression
  // makes — so a rewrite never silently strips or adds compression. A
  // non-empty value pins that knob instead: `compress` names the codec it
  // was asked for, `pcd undistort --compression` forwards the user's choice.
  //
  // mcap_compression is the chunk codec of an mcap output ("zstd", "lz4",
  // "none") and mcap_compression_level its encoder effort ("fastest",
  // "fast", "default", "slow", "slowest"); the sqlite3 triple is rosbag2's
  // mode ("none", "message", "file"), format ("none", "zstd") and the same
  // effort names for a sqlite3 output. A target's compression counts as
  // pinned when its codec or mode knob is non-empty; a level on its own
  // still inherits the codec and only sets the effort (no bag records the
  // level it was written with, so it is never inherited).
  //
  // These knobs govern only the decoded rewrite pipeline: the chunk
  // pass-through never opens a writer through these options and preserves
  // the input's chunk compression by copying the chunks themselves.
  std::string mcap_compression;
  std::string mcap_compression_level;
  std::string sqlite3_compression_mode;
  std::string sqlite3_compression_format;
  std::string sqlite3_compression_level;
};

// The command's rewrite pass. Receives the writer factory chosen by the
// dispatch (output path in -o mode, sibling tmp path in-place) and returns a
// process exit code: 0 on success; non-zero on failure after logging the
// specific error itself.
using BagRewritePass = std::function<int(const io::WriterFactory & open_writer)>;

// The resolved write target of the dispatch branch that runs the pass: the
// concrete path being written (the output path in -o mode, the sibling tmp
// path in-place) and the exact CreateOptions the writer factory hands to
// io::open_write. Exposed so a pass can offer the target to a path-level
// fast path — the mcap chunk pass-through — before opening a writer through
// the factory.
struct RewriteTarget
{
  std::filesystem::path path;
  io::CreateOptions create_options;
};

// Pass variant for commands that try the chunk pass-through first: same
// contract as BagRewritePass, with the resolved target alongside the
// factory.
using BagRewritePassWithTarget =
  std::function<int(const io::WriterFactory & open_writer, const RewriteTarget & target)>;

// Run the -o / in-place dispatch for a rewrite-style command.
//
// `input_path` is the bag read by the pass and the path rewritten in place
// when `output_path` is nullopt (for tf-static-cp-style commands it is the
// destination bag). `overwrite` only governs the -o branch's clobber policy;
// in-place mode always replaces <input>.
//
// Returns the pass's exit code, or 1 when the dispatch itself fails (output
// collision, undetectable input format, in-place swap error).
int run_bag_rewrite(
  const std::filesystem::path & input_path,
  const std::optional<std::filesystem::path> & output_path, bool overwrite,
  const BagRewriteOptions & options, const BagRewritePass & pass);

// Overload handing the pass the resolved RewriteTarget as well. The
// BagRewritePass overload delegates here; both run the identical dispatch.
int run_bag_rewrite(
  const std::filesystem::path & input_path,
  const std::optional<std::filesystem::path> & output_path, bool overwrite,
  const BagRewriteOptions & options, const BagRewritePassWithTarget & pass);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BAG__REWRITE_HPP_
