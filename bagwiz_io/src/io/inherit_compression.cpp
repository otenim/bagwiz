// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "inherit_compression.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_describe.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <algorithm>
#include <exception>
#include <filesystem>
#include <optional>
#include <string>

namespace bagwiz::io
{

namespace
{
constexpr const char * kLogger = "bagwiz.io";

// The rosbag2 compression modes: metadata.yaml declarations rather than a
// storage's own compression. rosbag2 defines exactly one format for them.
bool is_rosbag2_mode(const std::string & mode)
{
  return mode == "message" || mode == "file";
}

// The one codec the output carries. "" when the input is plain.
std::string pick_codec(const BagCompression & compression)
{
  if (compression.mode == "none") {
    return {};
  }
  if (compression.codecs.empty()) {
    // A rosbag2 mode named without its format: zstd is the only format
    // rosbag2 defines for MESSAGE and FILE mode, so that is what it means.
    return is_rosbag2_mode(compression.mode) ? std::string("zstd") : std::string();
  }
  if (compression.codecs.size() == 1) {
    return compression.codecs.front();
  }
  // Several codecs across one MCAP's chunks. One writer takes one codec;
  // zstd is bagwiz's own default and the better ratio.
  const bool has_zstd = std::find(compression.codecs.begin(), compression.codecs.end(), "zstd") !=
                        compression.codecs.end();
  return has_zstd ? std::string("zstd") : compression.codecs.front();
}

// The input's compression in the words `bagwiz info` uses, for the notes.
std::string describe_input(const BagCompression & compression, const std::string & codec)
{
  if (compression.mode == "message") {
    return codec + " per-message (rosbag2 MESSAGE mode)";
  }
  if (compression.mode == "file") {
    return codec + " whole-file envelope (rosbag2 FILE mode)";
  }
  return codec + " chunks";
}

// The input's compression in one phrase, for the log line that records an
// exact carry-over.
std::string describe_compression(const BagCompression & compression)
{
  const std::string codec = pick_codec(compression);
  if (codec.empty()) {
    return "none";
  }
  return describe_input(compression, codec);
}

}  // namespace

namespace detail
{

std::optional<InheritedCompression> translate_compression(
  const BagCompression & compression, ResolvedWriteLayout target)
{
  if (compression.mode == "unknown") {
    return std::nullopt;
  }
  InheritedCompression out;
  const std::string codec = pick_codec(compression);
  const std::string input = describe_input(compression, codec);

  if (target.format == Format::Mcap) {
    if (codec.empty()) {
      out.mcap_compression = "none";
      return out;
    }
    if (compression.codecs.size() > 1) {
      // Only an MCAP input mixes codecs, so the chunk vocabulary fits.
      out.mcap_compression = "zstd";
      out.note = "the input mixes lz4 and zstd chunks; the output uses zstd chunks throughout";
      return out;
    }
    out.mcap_compression = codec;
    if (is_rosbag2_mode(compression.mode)) {
      out.note = "carrying the input's " + input + " over as " + codec + " chunk compression";
    }
    return out;
  }

  // SQLite3: rosbag2's own compression layer, declared in metadata.yaml.
  if (codec.empty()) {
    out.sqlite3_compression_mode = "none";
    out.sqlite3_compression_format = "none";
    return out;
  }
  if (target.layout == Layout::SingleFile) {
    // rosbag2 reads the mode from metadata.yaml alone, which only a
    // directory bag has; the single-file writer refuses compression for the
    // same reason (see create_sqlite3_file).
    out.sqlite3_compression_mode = "none";
    out.sqlite3_compression_format = "none";
    out.dropped = true;
    out.note =
      "a single-file .db3 cannot carry compression (rosbag2 learns the mode from "
      "metadata.yaml, which only a directory bag has), so the input's " +
      input + " is not kept: the output is written plain. Name a directory output " + "to keep it";
    return out;
  }
  out.sqlite3_compression_format = "zstd";
  if (is_rosbag2_mode(compression.mode)) {
    out.sqlite3_compression_mode = compression.mode;
    return out;
  }
  // MCAP chunk compression has no sqlite3 counterpart. MESSAGE mode is the
  // one that keeps the shard readable in place, which is also what
  // `bagwiz compress --mode auto` picks for sqlite3.
  out.sqlite3_compression_mode = "message";
  out.note = "carrying the input's " + input + " over as sqlite3 MESSAGE-mode zstd";
  if (codec != "zstd") {
    out.note += " (sqlite3 storage has no " + codec + " mode)";
  }
  return out;
}

}  // namespace detail

namespace
{

// The knobs for a plain output on `target`'s storage: what a rewrite writes
// when the reference's compression cannot be read. A rewrite must never add
// compression its input did not have, and plain is the one answer that
// cannot; the CreateOptions default (zstd chunks) would.
void set_plain(CreateOptions & options, ResolvedWriteLayout target)
{
  if (target.format == Format::Mcap) {
    options.mcap_compression = "none";
  } else {
    options.sqlite3_compression_mode = "none";
    options.sqlite3_compression_format = "none";
  }
}

}  // namespace

CreateOptions create_options_inheriting_compression(
  const std::filesystem::path & reference_path, const std::filesystem::path & output_path,
  CreateOptions options) noexcept
{
  try {
    const auto target = resolve_write_layout(output_path, options);

    BagDescription description;
    try {
      description = describe_bag(reference_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_WARN(
        kLogger, "%s: compression not carried over to the output (%s); the output is written plain",
        reference_path.c_str(), e.what());
      set_plain(options, target);
      return options;
    }

    const auto inherited = detail::translate_compression(description.compression, target);
    if (!inherited.has_value()) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "%s: compression unknown (no readable MCAP summary section), so it is not "
        "carried over; the output is written plain",
        reference_path.c_str());
      set_plain(options, target);
      return options;
    }

    if (target.format == Format::Mcap) {
      options.mcap_compression = inherited->mcap_compression;
    } else {
      options.sqlite3_compression_mode = inherited->sqlite3_compression_mode;
      options.sqlite3_compression_format = inherited->sqlite3_compression_format;
    }

    if (inherited->dropped) {
      BAGWIZ_LOG_WARN(kLogger, "%s: %s", reference_path.c_str(), inherited->note.c_str());
    } else if (!inherited->note.empty()) {
      BAGWIZ_LOG_INFO(kLogger, "%s: %s", reference_path.c_str(), inherited->note.c_str());
    } else {
      BAGWIZ_LOG_DEBUG(
        kLogger, "%s: the output carries the input's compression (%s)", reference_path.c_str(),
        describe_compression(description.compression).c_str());
    }
    return options;
  } catch (...) {
    // noexcept: a failure past the describe step (resolving the layout,
    // logging) leaves the caller's options as they were.
    return options;
  }
}

}  // namespace bagwiz::io
