// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__INHERIT_COMPRESSION_HPP_
#define IO__INHERIT_COMPRESSION_HPP_

#include "bagwiz/io/bag_describe.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <optional>
#include <string>

// The pure half of create_options_inheriting_compression: the translation
// from what a bag's summaries say about its compression to the CreateOptions
// knobs that reproduce it on a given output storage and layout. Split off so
// the table can be tested cell by cell without a bag on disk.
namespace bagwiz::io::detail
{

// The compression knobs a write needs to carry a bag's compression over,
// plus what the carry-over had to change on the way. Only the knobs of the
// target storage are filled; the others stay empty.
struct InheritedCompression
{
  // MCAP outputs: the chunk codec — "zstd", "lz4", or "none".
  std::string mcap_compression;
  // SQLite3 outputs: rosbag2's mode / format pair — "none" / "none",
  // "message" / "zstd", or "file" / "zstd".
  std::string sqlite3_compression_mode;
  std::string sqlite3_compression_format;
  // What the translation had to change, in words the caller logs after the
  // bag's path: the input codec has no counterpart on the target storage, the
  // input mixes codecs, or the target cannot carry compression at all. Empty
  // when the carry-over is exact.
  std::string note;
  // True when the output ends up uncompressed although the input was not —
  // the one case worth a warning rather than an info line.
  bool dropped = false;
};

// Translate `compression`, as describe_bag reports it, into the knobs a
// write resolved as `target` needs. Pure: no I/O, no logging. Returns
// nullopt for "unknown" (an mcap without a summary section), where there is
// nothing to carry over and the caller keeps whatever it already had.
std::optional<InheritedCompression> translate_compression(
  const BagCompression & compression, ResolvedWriteLayout target);

}  // namespace bagwiz::io::detail

#endif  // IO__INHERIT_COMPRESSION_HPP_
