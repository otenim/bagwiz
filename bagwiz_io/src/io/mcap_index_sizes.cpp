// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "mcap_index_sizes.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "prorate_bytes.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <exception>
#include <fstream>
#include <span>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{

constexpr std::uint8_t kMessageIndexOpCode = 0x07;

// Every mcap record is a 1-byte opcode plus a uint64 content length.
constexpr std::uint64_t kRecordPrefixBytes = 9;

// A Message record's fixed header, ahead of the payload: channel id (2),
// sequence (4), log time (8), publish time (8). The smallest message record
// is an empty payload behind that header and the record prefix.
constexpr std::uint64_t kMessageHeaderBytes = 22;
constexpr std::uint64_t kMinMessageRecordBytes = kRecordPrefixBytes + kMessageHeaderBytes;

// A MessageIndex record's content ahead of its entry array: channel id (2)
// and the array's byte length (4). Each entry is (log time, offset).
constexpr std::uint64_t kMessageIndexHeaderBytes = 2 + 4;
constexpr std::uint64_t kMessageIndexEntryBytes = 16;

std::uint16_t read_u16(const std::byte * p)
{
  std::uint16_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

std::uint32_t read_u32(const std::byte * p)
{
  std::uint32_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

std::uint64_t read_u64(const std::byte * p)
{
  std::uint64_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

// One message record's position inside its chunk's records blob, as the
// message index reports it.
struct MessageRef
{
  std::uint64_t offset = 0;
  std::uint16_t channel_id = 0;
};

using ChannelBytes = std::unordered_map<std::uint16_t, std::uint64_t>;

// Read `length` bytes at `offset` into `out`. Returns false on a short read.
bool read_at(
  std::ifstream & file, std::uint64_t offset, std::uint64_t length, std::vector<std::byte> & out)
{
  out.resize(length);
  file.clear();
  file.seekg(static_cast<std::streamoff>(offset));
  file.read(reinterpret_cast<char *>(out.data()), static_cast<std::streamsize>(length));
  return static_cast<bool>(file) && static_cast<std::uint64_t>(file.gcount()) == length;
}

// Parse the MessageIndex records that belong to one chunk: every message's
// position into `refs`, and each record's own on-disk bytes into
// `index_bytes` under its channel. `block` must be the chunk's whole message
// index region, `block_offset` its file offset.
//
// Every record is checked against the ChunkIndex's own messageIndexOffsets
// map, so a block that is not the expected sequence of MessageIndex records is
// rejected instead of being parsed into plausible-looking garbage.
std::string parse_message_index(
  std::span<const std::byte> block, std::uint64_t block_offset,
  const std::unordered_map<mcap::ChannelId, mcap::ByteOffset> & expected_offsets,
  std::vector<MessageRef> & refs, ChannelBytes & index_bytes)
{
  std::size_t pos = 0;
  while (pos < block.size()) {
    if (block.size() - pos < kRecordPrefixBytes) {
      return "truncated message index record";
    }
    const std::uint64_t record_offset = block_offset + pos;
    const auto opcode = static_cast<std::uint8_t>(block[pos]);
    const std::uint64_t content_length = read_u64(block.data() + pos + 1);
    pos += kRecordPrefixBytes;
    if (opcode != kMessageIndexOpCode || content_length > block.size() - pos) {
      return "unexpected record in the message index region";
    }
    const std::byte * body = block.data() + pos;
    pos += content_length;

    if (content_length < kMessageIndexHeaderBytes) {
      return "truncated message index body";
    }
    const std::uint16_t channel_id = read_u16(body);
    const auto expected = expected_offsets.find(channel_id);
    if (expected == expected_offsets.end() || expected->second != record_offset) {
      return "message index record does not match the chunk index";
    }
    const std::uint32_t array_bytes = read_u32(body + 2);
    if (
      array_bytes % kMessageIndexEntryBytes != 0 ||
      array_bytes > content_length - kMessageIndexHeaderBytes) {
      return "malformed message index entry array";
    }
    index_bytes[channel_id] += kRecordPrefixBytes + content_length;
    const std::uint32_t entries = array_bytes / kMessageIndexEntryBytes;
    refs.reserve(refs.size() + entries);
    const std::byte * entry = body + kMessageIndexHeaderBytes;
    for (std::uint32_t i = 0; i < entries; ++i, entry += kMessageIndexEntryBytes) {
      // Each entry is (log time, offset); only the offset is needed here.
      refs.push_back({read_u64(entry + 8), channel_id});
    }
  }
  // Every record matched a distinct entry of the map (two records at one
  // offset cannot both parse), so fewer records than entries means a channel
  // the chunk index promises is missing from the region — its messages would
  // go uncounted, and worse, the gap before them would be charged elsewhere.
  if (index_bytes.size() != expected_offsets.size()) {
    return "message index region does not cover every channel in the chunk index";
  }
  return {};
}

// Whether any of `channels` (empty = all) has messages in the chunk, judged
// from the chunk index alone.
bool chunk_holds_selected(
  const mcap::ChunkIndex & index, const std::unordered_set<std::uint16_t> & channels)
{
  if (channels.empty()) {
    return true;
  }
  return std::any_of(
    index.messageIndexOffsets.begin(), index.messageIndexOffsets.end(),
    [&channels](const auto & entry) { return channels.count(entry.first) != 0; });
}

// Each channel's record bytes inside one chunk's records blob: the gap from
// each of its messages to the next record, the blob's end closing the last
// one. `refs` is sorted by offset on return.
std::string measure_record_gaps(
  std::vector<MessageRef> & refs, std::uint64_t records_size, ChannelBytes & record_bytes)
{
  std::sort(refs.begin(), refs.end(), [](const MessageRef & a, const MessageRef & b) {
    return a.offset < b.offset;
  });
  for (std::size_t i = 0; i < refs.size(); ++i) {
    const std::uint64_t end = i + 1 < refs.size() ? refs[i + 1].offset : records_size;
    // A message record cannot be shorter than its prefix and header, so two
    // offsets closer than that — or one past the blob's end — are not the
    // positions of message records.
    if (end < refs[i].offset || end - refs[i].offset < kMinMessageRecordBytes) {
      return "message index offsets do not fit message records";
    }
    record_bytes[refs[i].channel_id] += end - refs[i].offset;
  }
  return {};
}

// Charge one chunk's selected channels into `sizes`. `refs`, `scratch` and
// the two byte maps are the caller's reusable buffers.
std::string accumulate_chunk(
  std::ifstream & file, const mcap::ChunkIndex & index,
  const std::unordered_set<std::uint16_t> & channels, std::vector<MessageRef> & refs,
  std::vector<std::byte> & scratch, ChannelBytes & record_bytes, ChannelBytes & index_bytes,
  ChannelBytes & sizes)
{
  if (index.messageIndexOffsets.empty() || index.messageIndexLength == 0) {
    return "chunk carries no message index";
  }
  if (!chunk_holds_selected(index, channels)) {
    // No selected channel appears in this chunk: not even its message index
    // region is read, which is what makes a narrow `-t` selection cheap on a
    // topic that lives in few chunks. This trusts the summary's per-chunk
    // channel map the way every indexed mcap reader does — the filtered
    // message stream (mcap_indexed_stream.cpp) skips chunks on the same map
    // — so a summary that omits a channel mis-serves `-t` here exactly as it
    // already mis-serves `walk -t` and `topic keep`; an unfiltered run still
    // reads every region and refuses one that contradicts its chunk index.
    return {};
  }
  std::uint64_t block_offset = index.messageIndexOffsets.begin()->second;
  for (const auto & entry : index.messageIndexOffsets) {
    block_offset = std::min(block_offset, static_cast<std::uint64_t>(entry.second));
  }
  if (!read_at(file, block_offset, index.messageIndexLength, scratch)) {
    return "short read of message index records";
  }

  // Every channel's messages are needed, selected or not: a selected
  // message's extent ends where the next message starts, whichever channel
  // that one belongs to.
  refs.clear();
  record_bytes.clear();
  index_bytes.clear();
  auto error = parse_message_index(
    std::span<const std::byte>(scratch), block_offset, index.messageIndexOffsets, refs,
    index_bytes);
  if (!error.empty()) {
    return error;
  }
  error = measure_record_gaps(refs, index.uncompressedSize, record_bytes);
  if (!error.empty()) {
    return error;
  }

  for (const auto & [channel_id, bytes] : record_bytes) {
    if (channels.empty() || channels.count(channel_id) != 0) {
      sizes[channel_id] += prorate_bytes(bytes, index.compressedSize, index.uncompressedSize);
    }
  }
  for (const auto & [channel_id, bytes] : index_bytes) {
    if (channels.empty() || channels.count(channel_id) != 0) {
      sizes[channel_id] += bytes;
    }
  }
  return {};
}

}  // namespace

ChannelDiskSizes compute_channel_sizes_from_index(
  const std::filesystem::path & path, const mcap::McapReader & reader,
  const std::unordered_set<std::uint16_t> & channels, int num_threads)
{
  ChannelDiskSizes result;
  const auto & chunk_indexes = reader.chunkIndexes();
  if (chunk_indexes.empty()) {
    result.error = "no chunk index";
    return result;
  }

  const std::size_t workers =
    std::min<std::size_t>(std::max(num_threads, 1), std::max<std::size_t>(chunk_indexes.size(), 1));
  std::vector<ChannelBytes> partial_sizes(workers);
  std::vector<std::string> partial_errors(workers);
  std::atomic<std::size_t> next_chunk{0};
  std::atomic<bool> failed{false};

  const auto worker = [&](std::size_t slot) {
    // jthread bodies must not throw: a corrupt length field can overflow a
    // resize, and that has to surface as this slot's error, not a terminate.
    try {
      std::ifstream file(path, std::ios::binary);
      if (!file) {
        partial_errors[slot] = "failed to open " + path.string();
        failed.store(true);
        return;
      }
      std::vector<MessageRef> refs;
      std::vector<std::byte> scratch;
      ChannelBytes record_bytes;
      ChannelBytes index_bytes;
      for (;;) {
        const std::size_t i = next_chunk.fetch_add(1);
        if (i >= chunk_indexes.size() || failed.load()) {
          return;
        }
        auto error = accumulate_chunk(
          file, chunk_indexes[i], channels, refs, scratch, record_bytes, index_bytes,
          partial_sizes[slot]);
        if (!error.empty()) {
          partial_errors[slot] = std::move(error);
          failed.store(true);
          return;
        }
      }
    } catch (const std::exception & e) {
      partial_errors[slot] = std::string("index size scan failed: ") + e.what();
      failed.store(true);
    }
  };

  {
    std::vector<std::jthread> pool;
    pool.reserve(workers - 1);
    for (std::size_t w = 1; w < workers; ++w) {
      pool.emplace_back([&worker, w]() { worker(w); });
    }
    worker(0);
  }

  for (const auto & error : partial_errors) {
    if (!error.empty()) {
      result.error = error;
      return result;
    }
  }
  for (const auto & partial : partial_sizes) {
    // cppcheck-suppress unassignedVariable
    for (const auto & [channel_id, bytes] : partial) {
      result.per_channel[channel_id] += bytes;
    }
  }
  return result;
}

}  // namespace bagwiz::io::detail
