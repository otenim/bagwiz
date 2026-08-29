// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__MCAP_INDEX_SIZES_HPP_
#define IO__MCAP_INDEX_SIZES_HPP_

#include <mcap/reader.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <unordered_set>

// Src-local building block of `bagwiz du`: per-channel payload byte totals
// read out of an mcap's chunk and message indexes instead of out of its
// messages.
namespace bagwiz::io::detail
{

// Per-channel sum of message payload bytes. Meaningful only when `error` is
// empty; a non-empty `error` means the totals could not be derived from the
// index and the caller must fall back to a full message scan.
struct ChannelPayloadSizes
{
  std::unordered_map<std::uint16_t, std::uint64_t> per_channel;
  std::string error;
};

// Sum each channel's payload bytes without materializing a single payload.
//
// A message's payload size is its record's length prefix minus the fixed
// 22-byte message header, and the bag's message index already lists where
// every message record starts inside its chunk. So the whole computation is:
// read the message indexes (well under 0.1 % of a bag), then read 11 bytes per
// message — the record's opcode, length, and channel id, the last two of which
// are cross-checked against the index so a mis-derived offset is caught rather
// than silently mis-counted. An uncompressed chunk is addressed in place, so
// those 11 bytes are all that is read of it; a compressed one still has to be
// read and decompressed to reach its record headers.
//
// `reader` must have its summary loaded; it is used only for its chunk
// indexes, and all file I/O runs on this function's own handles.
// `channels` empty = every channel. `num_threads` <= 1 scans serially.
//
// Reports an error (rather than a partial answer) for a bag whose chunks
// carry no message index, and for any framing that contradicts the index.
ChannelPayloadSizes compute_channel_sizes_from_index(
  const std::filesystem::path & path, const mcap::McapReader & reader,
  const std::unordered_set<std::uint16_t> & channels, int num_threads);

}  // namespace bagwiz::io::detail

#endif  // IO__MCAP_INDEX_SIZES_HPP_
