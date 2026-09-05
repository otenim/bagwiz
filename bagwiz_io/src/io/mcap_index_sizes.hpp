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

// Src-local building block of `bagwiz du`: per-channel on-disk byte totals
// read out of an mcap's chunk and message indexes, without touching a single
// record.
namespace bagwiz::io::detail
{

// Per-channel on-disk bytes. Meaningful only when `error` is empty; a
// non-empty `error` means the totals could not be derived from the index and
// the caller must fall back to a full message scan.
struct ChannelDiskSizes
{
  std::unordered_map<std::uint16_t, std::uint64_t> per_channel;
  std::string error;
};

// Charge each channel the bytes its messages occupy in the file, from the
// indexes alone.
//
// A chunk's message index lists where every message record starts inside the
// chunk's uncompressed records blob, so each record's extent is the gap to the
// next record (the blob's end, for the last one) — no record is read. Per
// chunk, a channel's record bytes are then scaled by the chunk's compression
// ratio (compressed over uncompressed size, from the chunk index) to the
// nearest byte: an uncompressed chunk is charged exactly, a compressed one in
// proportion to what each channel contributed to it, since a compressed blob
// has no per-record byte count of its own. Each channel's own MessageIndex
// records are charged to it in full. The chunk record's own header, the
// Schema and Channel records, and the summary section are charged to no
// channel, so the totals fall a little short of the file size.
//
// One deliberate approximation: a writer (libmcap among them) may emit a
// channel's Schema and Channel records into the chunk that first carries one
// of its messages, between other records. Being invisible to the message
// index, those bytes are absorbed into the gap after the preceding message and
// charged to that message's channel. The error is bounded by the size of the
// bag's declaration records — kilobytes against gigabytes.
//
// Reads only the message index regions (well under 0.1 % of a bag), and none
// at all for a chunk that holds none of `channels`, so a narrow selection is
// cheap and a compressed bag costs the same as an uncompressed one.
//
// `reader` must have its summary loaded; it is used only for its chunk
// indexes, and all file I/O runs on this function's own handles.
// `channels` empty = every channel. `num_threads` <= 1 scans serially.
//
// Reports an error (rather than a partial answer) for a bag whose chunks
// carry no message index, and for a message index that contradicts its chunk.
ChannelDiskSizes compute_channel_sizes_from_index(
  const std::filesystem::path & path, const mcap::McapReader & reader,
  const std::unordered_set<std::uint16_t> & channels, int num_threads);

}  // namespace bagwiz::io::detail

#endif  // IO__MCAP_INDEX_SIZES_HPP_
