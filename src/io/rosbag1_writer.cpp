// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/rosbag1_writer.hpp"

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::io
{

namespace
{

constexpr std::string_view kMagic = "#ROSBAG V2.0\n";

constexpr uint8_t kOpBagHeader = 0x03;
constexpr uint8_t kOpChunk = 0x05;
constexpr uint8_t kOpConnection = 0x07;
constexpr uint8_t kOpMessageData = 0x02;
constexpr uint8_t kOpIndexData = 0x04;
constexpr uint8_t kOpChunkInfo = 0x06;

// Total size of the BAG_HEADER record (4 bytes header_len + header
// body + 4 bytes data_len + zero-padded data). 4096 is the standard
// rosbag value and gives plenty of room for the small fixed-size
// header body.
constexpr uint32_t kBagHeaderRecordSize = 4096;

// Per-chunk size cap. When a chunk's accumulated data section reaches
// this many bytes, the chunk is flushed and a fresh one is started.
constexpr std::size_t kChunkSizeCap = 4 * 1024 * 1024;

// ---- byte writer helpers -------------------------------------------------

void append_u32_le(std::vector<std::byte> & out, uint32_t v)
{
  for (int i = 0; i < 4; ++i) {
    out.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFu));
  }
}

void append_bytes(std::vector<std::byte> & out, std::span<const std::byte> bytes)
{
  out.insert(out.end(), bytes.begin(), bytes.end());
}

void append_str(std::vector<std::byte> & out, std::string_view s)
{
  const auto * p = reinterpret_cast<const std::byte *>(s.data());
  out.insert(out.end(), p, p + s.size());
}

// Append a single header field as <field_len:u32><name=value>. `value`
// is opaque bytes — used as-is for both string-valued fields ("topic",
// "compression") and binary-valued fields ("conn", "time", "size",
// "index_pos", ...).
void append_field(
  std::vector<std::byte> & out, std::string_view name, std::span<const std::byte> value)
{
  const uint32_t flen =
    static_cast<uint32_t>(name.size()) + 1u + static_cast<uint32_t>(value.size());
  append_u32_le(out, flen);
  append_str(out, name);
  out.push_back(static_cast<std::byte>('='));
  append_bytes(out, value);
}

// cppcheck-suppress passedByValue
void append_field_str(std::vector<std::byte> & out, std::string_view name, std::string_view value)
{
  append_field(
    out, name,
    std::span<const std::byte>(reinterpret_cast<const std::byte *>(value.data()), value.size()));
}

// cppcheck-suppress passedByValue
void append_field_u8(std::vector<std::byte> & out, std::string_view name, uint8_t v)
{
  const auto b = static_cast<std::byte>(v);
  append_field(out, name, std::span<const std::byte>(&b, 1));
}

// cppcheck-suppress passedByValue
void append_field_u32(std::vector<std::byte> & out, std::string_view name, uint32_t v)
{
  std::byte tmp[4];
  for (int i = 0; i < 4; ++i) {
    tmp[i] = static_cast<std::byte>((v >> (8 * i)) & 0xFFu);
  }
  append_field(out, name, std::span<const std::byte>(tmp, 4));
}

// cppcheck-suppress passedByValue
void append_field_u64(std::vector<std::byte> & out, std::string_view name, uint64_t v)
{
  std::byte tmp[8];
  for (int i = 0; i < 8; ++i) {
    tmp[i] = static_cast<std::byte>((v >> (8 * i)) & 0xFFull);
  }
  append_field(out, name, std::span<const std::byte>(tmp, 8));
}

// "time" header field: 8 bytes = secs:u32 + nsecs:u32, little-endian.
// cppcheck-suppress passedByValue
void append_field_time_ns(std::vector<std::byte> & out, std::string_view name, int64_t time_ns)
{
  if (time_ns < 0) {
    time_ns = 0;
  }
  const uint32_t secs = static_cast<uint32_t>(time_ns / 1'000'000'000LL);
  const uint32_t nsecs = static_cast<uint32_t>(time_ns % 1'000'000'000LL);
  std::byte tmp[8];
  for (int i = 0; i < 4; ++i) {
    tmp[i] = static_cast<std::byte>((secs >> (8 * i)) & 0xFFu);
    tmp[4 + i] = static_cast<std::byte>((nsecs >> (8 * i)) & 0xFFu);
  }
  append_field(out, name, std::span<const std::byte>(tmp, 8));
}

// Wrap a header (already-built field block) and a data section into a
// full record: <header_len><header><data_len><data>.
void append_record(
  std::vector<std::byte> & out, std::span<const std::byte> header, std::span<const std::byte> data)
{
  append_u32_le(out, static_cast<uint32_t>(header.size()));
  append_bytes(out, header);
  append_u32_le(out, static_cast<uint32_t>(data.size()));
  append_bytes(out, data);
}

}  // namespace

struct Rosbag1Writer::Impl
{
  std::ofstream f;
  std::filesystem::path path;
  bool closed = false;

  // Position of the BAG_HEADER record (just after the magic). Fixed at
  // open() time; rewritten on close().
  uint64_t bag_header_pos = 0;

  // Connection metadata, keyed by conn_id (== index in the vector).
  struct ConnMeta
  {
    std::string topic;
    std::string ros1_type;
    std::string md5sum;
    std::string message_definition;
  };
  std::vector<ConnMeta> conns;

  // Current chunk being built. Flushed when its data section exceeds
  // kChunkSizeCap or on close().
  std::vector<std::byte> chunk_data;  // raw bytes of CHUNK record's data section
  std::vector<bool>
    conn_emitted_in_chunk;  // parallel to conns: has CONNECTION been written this chunk?

  struct IndexEntry
  {
    int64_t time_ns;
    uint32_t offset;  // byte offset from start of chunk's data section
  };
  // index_entries[conn_id] -> entries within the current chunk
  std::vector<std::vector<IndexEntry>> index_entries;

  // CHUNK_INFO accumulator; one per flushed chunk.
  struct ChunkInfo
  {
    uint64_t chunk_pos = 0;  // file offset of the CHUNK record itself
    int64_t start_ns = 0;
    int64_t end_ns = 0;
    bool has_messages = false;
    // counts[conn_id] -> message count for this chunk
    std::vector<uint32_t> counts;
  };
  std::vector<ChunkInfo> chunk_infos;
  // The "current" chunk_info, updated as messages are written and
  // moved into chunk_infos on flush.
  ChunkInfo current_info;

  void grow_per_conn(uint32_t conn_id)
  {
    if (conn_emitted_in_chunk.size() <= conn_id) {
      conn_emitted_in_chunk.resize(conn_id + 1, false);
    }
    if (index_entries.size() <= conn_id) {
      index_entries.resize(conn_id + 1);
    }
    if (current_info.counts.size() <= conn_id) {
      current_info.counts.resize(conn_id + 1, 0);
    }
  }

  // Build a CONNECTION record and append it to the current chunk
  // buffer. Writes the conn_id-keyed sub-header inside the data field.
  void emit_connection_into_chunk(uint32_t conn_id)
  {
    if (conn_id >= conns.size()) {
      throw std::runtime_error("rosbag1_writer: emit_connection unknown conn_id");
    }
    const auto & c = conns[conn_id];

    std::vector<std::byte> header;
    append_field_u8(header, "op", kOpConnection);
    append_field_u32(header, "conn", conn_id);
    append_field_str(header, "topic", c.topic);

    std::vector<std::byte> data;
    append_field_str(data, "topic", c.topic);
    append_field_str(data, "type", c.ros1_type);
    append_field_str(data, "md5sum", c.md5sum);
    append_field_str(data, "message_definition", c.message_definition);

    append_record(chunk_data, header, data);
  }

  void open(const std::filesystem::path & p)
  {
    path = p;
    f.open(p, std::ios::binary | std::ios::out | std::ios::trunc);
    if (!f) {
      throw std::runtime_error("rosbag1_writer: cannot open output: " + p.string());
    }

    // Magic.
    f.write(kMagic.data(), static_cast<std::streamsize>(kMagic.size()));

    bag_header_pos = static_cast<uint64_t>(f.tellp());
    write_bag_header(0, 0, 0);
  }

  // Write a BAG_HEADER record padded to kBagHeaderRecordSize bytes
  // total. The output is positioned immediately after the record.
  void write_bag_header(uint64_t index_pos, uint32_t conn_count, uint32_t chunk_count)
  {
    std::vector<std::byte> header;
    append_field_u8(header, "op", kOpBagHeader);
    append_field_u64(header, "index_pos", index_pos);
    append_field_u32(header, "conn_count", conn_count);
    append_field_u32(header, "chunk_count", chunk_count);

    // Compute padding so total record == kBagHeaderRecordSize.
    const std::size_t prefix = 4 + header.size() + 4;  // header_len + header + data_len
    if (prefix > kBagHeaderRecordSize) {
      throw std::runtime_error("rosbag1_writer: bag header too large for fixed slot");
    }
    const std::size_t pad_len = kBagHeaderRecordSize - prefix;

    std::vector<std::byte> pad(pad_len, std::byte{' '});  // rosbag uses ASCII space padding
    append_record_to_file(header, pad);
  }

  void append_record_to_file(std::span<const std::byte> header, std::span<const std::byte> data)
  {
    std::vector<std::byte> rec;
    append_record(rec, header, data);
    f.write(reinterpret_cast<const char *>(rec.data()), static_cast<std::streamsize>(rec.size()));
    if (!f) {
      throw std::runtime_error("rosbag1_writer: write failed");
    }
  }

  // Flush the current chunk (if non-empty) to the file: the CHUNK
  // record followed by one INDEX_DATA record per connection that has
  // entries. Updates chunk_infos.
  void flush_chunk()
  {
    if (chunk_data.empty()) {
      // Nothing to flush; reset transient state and return.
      reset_chunk_state();
      return;
    }

    current_info.chunk_pos = static_cast<uint64_t>(f.tellp());

    // CHUNK record.
    std::vector<std::byte> chunk_header;
    append_field_u8(chunk_header, "op", kOpChunk);
    append_field_str(chunk_header, "compression", "none");
    append_field_u32(chunk_header, "size", static_cast<uint32_t>(chunk_data.size()));
    append_record_to_file(chunk_header, chunk_data);

    // One INDEX_DATA record per connection with entries.
    for (std::size_t cid = 0; cid < index_entries.size(); ++cid) {
      const auto & entries = index_entries[cid];
      if (entries.empty()) {
        continue;
      }
      std::vector<std::byte> idx_header;
      append_field_u8(idx_header, "op", kOpIndexData);
      append_field_u32(idx_header, "ver", 1u);
      append_field_u32(idx_header, "conn", static_cast<uint32_t>(cid));
      append_field_u32(idx_header, "count", static_cast<uint32_t>(entries.size()));

      std::vector<std::byte> idx_data;
      idx_data.reserve(entries.size() * 12);
      for (const auto & e : entries) {
        const int64_t t = e.time_ns < 0 ? 0 : e.time_ns;
        const uint32_t secs = static_cast<uint32_t>(t / 1'000'000'000LL);
        const uint32_t nsecs = static_cast<uint32_t>(t % 1'000'000'000LL);
        append_u32_le(idx_data, secs);
        append_u32_le(idx_data, nsecs);
        append_u32_le(idx_data, e.offset);
      }
      append_record_to_file(idx_header, idx_data);
    }

    chunk_infos.push_back(std::move(current_info));
    reset_chunk_state();
  }

  void reset_chunk_state()
  {
    chunk_data.clear();
    std::fill(conn_emitted_in_chunk.begin(), conn_emitted_in_chunk.end(), false);
    for (auto & v : index_entries) {
      v.clear();
    }
    current_info = ChunkInfo{};
  }

  // Emit the index section: re-emit CONNECTION records for every
  // declared connection at top level, then one CHUNK_INFO record per
  // flushed chunk.
  uint64_t emit_index_section()
  {
    const uint64_t index_pos = static_cast<uint64_t>(f.tellp());

    // Connections (top-level, outside any chunk).
    for (uint32_t cid = 0; cid < conns.size(); ++cid) {
      const auto & c = conns[cid];

      std::vector<std::byte> header;
      append_field_u8(header, "op", kOpConnection);
      append_field_u32(header, "conn", cid);
      append_field_str(header, "topic", c.topic);

      std::vector<std::byte> data;
      append_field_str(data, "topic", c.topic);
      append_field_str(data, "type", c.ros1_type);
      append_field_str(data, "md5sum", c.md5sum);
      append_field_str(data, "message_definition", c.message_definition);

      append_record_to_file(header, data);
    }

    // CHUNK_INFO records.
    for (const auto & ci : chunk_infos) {
      std::vector<std::byte> header;
      append_field_u8(header, "op", kOpChunkInfo);
      append_field_u32(header, "ver", 1u);
      append_field_u64(header, "chunk_pos", ci.chunk_pos);
      append_field_time_ns(header, "start_time", ci.start_ns);
      append_field_time_ns(header, "end_time", ci.end_ns);
      // Count of (conn, msg_count) entries in data.
      uint32_t connections_with_msgs = 0;
      for (auto n : ci.counts) {
        if (n > 0) {
          ++connections_with_msgs;
        }
      }
      append_field_u32(header, "count", connections_with_msgs);

      std::vector<std::byte> data;
      data.reserve(connections_with_msgs * 8);
      for (uint32_t cid = 0; cid < ci.counts.size(); ++cid) {
        if (ci.counts[cid] == 0) {
          continue;
        }
        append_u32_le(data, cid);
        append_u32_le(data, ci.counts[cid]);
      }
      append_record_to_file(header, data);
    }

    return index_pos;
  }

  void do_close()
  {
    if (closed) {
      return;
    }
    flush_chunk();
    const uint64_t index_pos = emit_index_section();

    // Rewrite BAG_HEADER in place with final counts.
    f.flush();
    f.seekp(static_cast<std::streamoff>(bag_header_pos));
    write_bag_header(
      index_pos, static_cast<uint32_t>(conns.size()), static_cast<uint32_t>(chunk_infos.size()));
    f.flush();

    f.close();
    closed = true;
  }

  ~Impl()
  {
    // Best-effort: if the user forgot to call close(), don't try to
    // patch the bag header on the destructor path — the bag would be
    // structurally invalid anyway, and silently corrupting it makes
    // the bug harder to find than leaving the file unfinished.
    if (f.is_open()) {
      f.close();
    }
  }
};

Rosbag1Writer::Rosbag1Writer(const std::filesystem::path & path) : impl_(std::make_unique<Impl>())
{
  impl_->open(path);
}

Rosbag1Writer::~Rosbag1Writer() = default;
Rosbag1Writer::Rosbag1Writer(Rosbag1Writer &&) noexcept = default;
Rosbag1Writer & Rosbag1Writer::operator=(Rosbag1Writer &&) noexcept = default;

uint32_t Rosbag1Writer::declare_connection(
  const std::string & topic, const std::string & ros1_type, const std::string & md5sum,
  const std::string & message_definition)
{
  // Idempotency: if an existing connection matches exactly, return it.
  for (uint32_t cid = 0; cid < impl_->conns.size(); ++cid) {
    const auto & c = impl_->conns[cid];
    if (
      c.topic == topic && c.ros1_type == ros1_type && c.md5sum == md5sum &&
      c.message_definition == message_definition) {
      return cid;
    }
  }

  Impl::ConnMeta c;
  c.topic = topic;
  c.ros1_type = ros1_type;
  c.md5sum = md5sum;
  c.message_definition = message_definition;
  const uint32_t cid = static_cast<uint32_t>(impl_->conns.size());
  impl_->conns.push_back(std::move(c));
  impl_->grow_per_conn(cid);
  return cid;
}

void Rosbag1Writer::write(
  uint32_t conn_id, int64_t timestamp_ns, std::span<const std::byte> payload)
{
  if (conn_id >= impl_->conns.size()) {
    throw std::runtime_error("rosbag1_writer: write() with unknown conn_id");
  }
  if (impl_->closed) {
    throw std::runtime_error("rosbag1_writer: write() after close()");
  }
  impl_->grow_per_conn(conn_id);

  // Lazily emit a CONNECTION record at the top of the chunk if this
  // connection has not been seen in the current chunk yet.
  if (!impl_->conn_emitted_in_chunk[conn_id]) {
    impl_->emit_connection_into_chunk(conn_id);
    impl_->conn_emitted_in_chunk[conn_id] = true;
  }

  // Record the offset of this MESSAGE_DATA record from the start of
  // the chunk's data section, then append it.
  const uint32_t offset = static_cast<uint32_t>(impl_->chunk_data.size());

  std::vector<std::byte> header;
  append_field_u8(header, "op", kOpMessageData);
  append_field_u32(header, "conn", conn_id);
  append_field_time_ns(header, "time", timestamp_ns);
  append_record(impl_->chunk_data, header, payload);

  impl_->index_entries[conn_id].push_back({timestamp_ns, offset});

  // Update CHUNK_INFO accumulators.
  if (!impl_->current_info.has_messages) {
    impl_->current_info.start_ns = timestamp_ns;
    impl_->current_info.end_ns = timestamp_ns;
    impl_->current_info.has_messages = true;
  } else {
    if (timestamp_ns < impl_->current_info.start_ns) {
      impl_->current_info.start_ns = timestamp_ns;
    }
    if (timestamp_ns > impl_->current_info.end_ns) {
      impl_->current_info.end_ns = timestamp_ns;
    }
  }
  ++impl_->current_info.counts[conn_id];

  if (impl_->chunk_data.size() >= kChunkSizeCap) {
    impl_->flush_chunk();
  }
}

void Rosbag1Writer::close()
{
  impl_->do_close();
}

}  // namespace bagwiz::io
