// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "mcap_index_sizes.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "mcap_chunk_codec.hpp"  // NOLINT(build/include_subdir) src-local shared header

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

constexpr std::uint8_t kMessageOpCode = 0x05;
constexpr std::uint8_t kMessageIndexOpCode = 0x07;

// Every mcap record is a 1-byte opcode plus a uint64 content length.
constexpr std::uint64_t kRecordPrefixBytes = 9;

// A Message record's fixed header, ahead of the payload: channel id (2),
// sequence (4), log time (8), publish time (8). The payload is the rest of
// the record, so payload size == record content length - 22.
constexpr std::uint64_t kMessageHeaderBytes = 22;

// What we read at each message offset: the record prefix plus the channel id
// that follows it. The channel id is redundant with the message index, which
// is exactly why it is worth reading — it makes a wrong offset detectable.
constexpr std::uint64_t kMessageProbeBytes = kRecordPrefixBytes + 2;

// A Chunk record's body ahead of its records blob: message start/end time
// (8+8), uncompressed size (8), uncompressed CRC (4), the compression string's
// own length prefix (4), and the records blob's length prefix (8).
constexpr std::uint64_t kChunkBodyPrefixBytes = 8 + 8 + 8 + 4 + 4 + 8;

// Two message headers no further apart than this are fetched in one read
// rather than two. It makes the per-message probing adapt to the bag's shape
// on its own: a chunk of a few multi-MiB point clouds costs one small read per
// message, while a chunk packed with tiny messages collapses into a single
// read of the chunk — never more bytes than reading the chunk outright.
constexpr std::uint64_t kCoalesceWindowBytes = 64 * 1024;

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

// One message record's position inside its chunk's decompressed records blob,
// as the message index reports it.
struct MessageRef
{
  std::uint64_t offset = 0;
  std::uint16_t channel_id = 0;
};

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

// Parse the MessageIndex records that belong to one chunk, keeping the
// messages whose channel passes `channels` (empty = keep all). `block` must be
// the chunk's whole message index region, `block_offset` its file offset.
//
// Every record is checked against the ChunkIndex's own messageIndexOffsets
// map, so a block that is not the expected sequence of MessageIndex records is
// rejected instead of being parsed into plausible-looking garbage.
std::string parse_message_index(
  std::span<const std::byte> block, std::uint64_t block_offset,
  const std::unordered_map<mcap::ChannelId, mcap::ByteOffset> & expected_offsets,
  const std::unordered_set<std::uint16_t> & channels, std::vector<MessageRef> & out)
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

    if (content_length < 2 + 4) {
      return "truncated message index body";
    }
    const std::uint16_t channel_id = read_u16(body);
    const auto expected = expected_offsets.find(channel_id);
    if (expected == expected_offsets.end() || expected->second != record_offset) {
      return "message index record does not match the chunk index";
    }
    const std::uint32_t array_bytes = read_u32(body + 2);
    if (array_bytes % 16 != 0 || array_bytes > content_length - 6) {
      return "malformed message index entry array";
    }
    if (!channels.empty() && channels.count(channel_id) == 0) {
      continue;
    }
    const std::uint32_t entries = array_bytes / 16;
    out.reserve(out.size() + entries);
    for (std::uint32_t i = 0; i < entries; ++i) {
      // Each entry is (log time, offset); only the offset is needed here.
      out.push_back({read_u64(body + 6 + (std::size_t{i} * 16) + 8), channel_id});
    }
  }
  return {};
}

// Whether a message record probe at `offset` lies inside a records blob of
// `records_size` bytes. Written as a subtraction on the size so an offset
// straight out of the file cannot overflow the comparison.
bool probe_fits(std::uint64_t offset, std::uint64_t records_size)
{
  return records_size >= kMessageProbeBytes && offset <= records_size - kMessageProbeBytes;
}

// Add one message's payload size, given the 11 bytes at its record offset.
// `probe` is the record's opcode + content length + channel id. The caller
// must have established probe_fits(ref.offset, records_size).
std::string accumulate_message(
  const std::byte * probe, const MessageRef & ref, std::uint64_t records_size,
  std::unordered_map<std::uint16_t, std::uint64_t> & sizes)
{
  if (static_cast<std::uint8_t>(*probe) != kMessageOpCode) {
    return "message index points at a non-message record";
  }
  const std::uint64_t content_length = read_u64(probe + 1);
  if (
    content_length < kMessageHeaderBytes ||
    content_length > records_size - ref.offset - kRecordPrefixBytes) {
    return "message record length runs past the end of its chunk";
  }
  if (read_u16(probe + kRecordPrefixBytes) != ref.channel_id) {
    return "message record channel id disagrees with the message index";
  }
  sizes[ref.channel_id] += content_length - kMessageHeaderBytes;
  return {};
}

// Sum the payload sizes of an uncompressed chunk's selected messages by
// reading only their record headers, straight out of the file: an
// uncompressed chunk's records blob is stored verbatim, so an offset within
// it is just a file offset.
std::string accumulate_uncompressed_chunk(
  std::ifstream & file, const mcap::ChunkIndex & index, std::vector<MessageRef> & refs,
  std::vector<std::byte> & scratch, std::unordered_map<std::uint16_t, std::uint64_t> & sizes)
{
  const std::uint64_t blob_offset =
    index.chunkStartOffset + kRecordPrefixBytes + kChunkBodyPrefixBytes + index.compression.size();
  const std::uint64_t records_size = index.uncompressedSize;

  // Bound every offset before any arithmetic on it: the values come straight
  // out of the file, and the grouping below adds to them.
  for (const auto & ref : refs) {
    if (!probe_fits(ref.offset, records_size)) {
      return "message index points past the end of its chunk";
    }
  }
  std::sort(refs.begin(), refs.end(), [](const MessageRef & a, const MessageRef & b) {
    return a.offset < b.offset;
  });
  for (std::size_t first = 0; first < refs.size();) {
    // Extend the group while the next header is within one window of the
    // current one's end.
    std::size_t last = first;
    while (last + 1 < refs.size() &&
           refs[last + 1].offset <= refs[last].offset + kMessageProbeBytes + kCoalesceWindowBytes) {
      ++last;
    }
    const std::uint64_t span_start = refs[first].offset;
    const std::uint64_t span_end = refs[last].offset + kMessageProbeBytes;
    if (span_end > records_size) {
      return "message index points past the end of its chunk";
    }
    if (!read_at(file, blob_offset + span_start, span_end - span_start, scratch)) {
      return "short read of message record headers";
    }
    for (std::size_t i = first; i <= last; ++i) {
      auto error = accumulate_message(
        scratch.data() + (refs[i].offset - span_start), refs[i], records_size, sizes);
      if (!error.empty()) {
        return error;
      }
    }
    first = last + 1;
  }
  return {};
}

// Same for a compressed chunk. Its record headers only exist after
// decompression, so the chunk is read and decompressed in full — the payload
// bytes are still never copied out, but this chunk costs what a normal read
// of it costs.
std::string accumulate_compressed_chunk(
  std::ifstream & file, const mcap::ChunkIndex & index, const std::vector<MessageRef> & refs,
  std::vector<std::byte> & scratch, std::unordered_map<std::uint16_t, std::uint64_t> & sizes)
{
  if (!read_at(file, index.chunkStartOffset, index.chunkLength, scratch)) {
    return "short read of chunk record";
  }
  auto decoded = decompress_chunk_record(std::span<const std::byte>(scratch));
  if (!decoded.error.empty()) {
    return decoded.error;
  }
  const std::uint64_t records_size = decoded.records.size();
  for (const auto & ref : refs) {
    if (!probe_fits(ref.offset, records_size)) {
      return "message index points past the end of its chunk";
    }
    auto error = accumulate_message(decoded.records.data() + ref.offset, ref, records_size, sizes);
    if (!error.empty()) {
      return error;
    }
  }
  return {};
}

// Sum one chunk's selected messages into `sizes`. `refs` and `scratch` are the
// caller's reusable buffers.
std::string accumulate_chunk(
  std::ifstream & file, const mcap::ChunkIndex & index,
  const std::unordered_set<std::uint16_t> & channels, std::vector<MessageRef> & refs,
  std::vector<std::byte> & scratch, std::unordered_map<std::uint16_t, std::uint64_t> & sizes)
{
  if (index.messageIndexOffsets.empty() || index.messageIndexLength == 0) {
    return "chunk carries no message index";
  }
  std::uint64_t block_offset = index.messageIndexOffsets.begin()->second;
  for (const auto & [channel_id, offset] : index.messageIndexOffsets) {
    block_offset = std::min(block_offset, static_cast<std::uint64_t>(offset));
  }
  if (!read_at(file, block_offset, index.messageIndexLength, scratch)) {
    return "short read of message index records";
  }

  refs.clear();
  auto error = parse_message_index(
    std::span<const std::byte>(scratch), block_offset, index.messageIndexOffsets, channels, refs);
  if (!error.empty()) {
    return error;
  }
  if (refs.empty()) {
    // No selected channel appears in this chunk: it is never read at all,
    // which is what makes a narrow `-t` selection cheap.
    return {};
  }

  const bool uncompressed = index.compression.empty() || index.compression == "none";
  return uncompressed ? accumulate_uncompressed_chunk(file, index, refs, scratch, sizes)
                      : accumulate_compressed_chunk(file, index, refs, scratch, sizes);
}

}  // namespace

ChannelPayloadSizes compute_channel_sizes_from_index(
  const std::filesystem::path & path, const mcap::McapReader & reader,
  const std::unordered_set<std::uint16_t> & channels, int num_threads)
{
  ChannelPayloadSizes result;
  const auto & chunk_indexes = reader.chunkIndexes();
  if (chunk_indexes.empty()) {
    result.error = "no chunk index";
    return result;
  }

  const std::size_t workers =
    std::min<std::size_t>(std::max(num_threads, 1), std::max<std::size_t>(chunk_indexes.size(), 1));
  std::vector<std::unordered_map<std::uint16_t, std::uint64_t>> partial_sizes(workers);
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
      for (;;) {
        const std::size_t i = next_chunk.fetch_add(1);
        if (i >= chunk_indexes.size() || failed.load()) {
          return;
        }
        auto error =
          accumulate_chunk(file, chunk_indexes[i], channels, refs, scratch, partial_sizes[slot]);
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
