// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/rosbag1_reader.hpp"

#include "bagwiz/core/logging.hpp"

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::io
{

namespace
{

constexpr const char * kLogger = "bagwiz.io.rosbag1";
constexpr std::string_view kMagic = "#ROSBAG V2.0\n";

// Op codes we recognise. Values come from the ROS 1 bag v2.0 spec.
constexpr uint8_t kOpBagHeader = 0x03;
constexpr uint8_t kOpChunk = 0x05;
constexpr uint8_t kOpConnection = 0x07;
constexpr uint8_t kOpMessageData = 0x02;
constexpr uint8_t kOpIndexData = 0x04;
constexpr uint8_t kOpChunkInfo = 0x06;

// Unaligned little-endian decoders. ROS 1 raw values are always LE on the
// wire and we are guaranteed-host LE in practice; keep manual decoders so
// we are independent of any endianness assumptions.
uint32_t read_u32_le(const std::byte * p)
{
  return static_cast<uint32_t>(static_cast<unsigned char>(p[0])) |
         (static_cast<uint32_t>(static_cast<unsigned char>(p[1])) << 8) |
         (static_cast<uint32_t>(static_cast<unsigned char>(p[2])) << 16) |
         (static_cast<uint32_t>(static_cast<unsigned char>(p[3])) << 24);
}

// Parse a record header (a sequence of `<u32 len><name=value>` blobs)
// into a name -> raw-value map. Field values are arbitrary bytes.
std::unordered_map<std::string, std::span<const std::byte>> parse_header_fields(
  std::span<const std::byte> header)
{
  std::unordered_map<std::string, std::span<const std::byte>> out;
  std::size_t pos = 0;
  while (pos < header.size()) {
    if (header.size() - pos < 4) {
      throw std::runtime_error("ros1 bag: truncated header field length");
    }
    const uint32_t flen = read_u32_le(header.data() + pos);
    pos += 4;
    if (header.size() - pos < flen) {
      throw std::runtime_error("ros1 bag: truncated header field body");
    }
    const auto field = header.subspan(pos, flen);
    pos += flen;
    // Split on first '='.
    std::size_t eq = 0;
    while (eq < field.size() && static_cast<char>(field[eq]) != '=') {
      ++eq;
    }
    if (eq == field.size()) {
      throw std::runtime_error("ros1 bag: header field missing '='");
    }
    std::string name(reinterpret_cast<const char *>(field.data()), eq);
    out.emplace(std::move(name), field.subspan(eq + 1));
  }
  return out;
}

// `op` is one byte stored as the value of a field named "op". Look it up
// and return the byte; throw if absent.
uint8_t require_op(const std::unordered_map<std::string, std::span<const std::byte>> & fields)
{
  auto it = fields.find("op");
  if (it == fields.end() || it->second.size() != 1) {
    throw std::runtime_error("ros1 bag: header missing or malformed 'op' field");
  }
  return static_cast<uint8_t>(it->second[0]);
}

std::string field_to_string(std::span<const std::byte> v)
{
  return std::string(reinterpret_cast<const char *>(v.data()), v.size());
}

}  // namespace

struct Rosbag1Reader::Impl
{
  // The whole file is read into memory. ROS 1 bags can be large but the
  // primary callers (offline conversion) have plenty of RAM and this
  // keeps the streaming logic trivial. If memory pressure becomes a
  // problem we can revisit with a windowed mmap.
  std::vector<std::byte> file_bytes;

  // Position of the next record header to read at top level.
  std::size_t cursor = 0;

  // While iterating inside an uncompressed CHUNK, points into file_bytes
  // at the current sub-record header and the chunk's exclusive end
  // offset. cursor stays at the first byte after the chunk so we can
  // resume top-level iteration when chunk_end is reached.
  std::size_t chunk_pos = 0;
  std::size_t chunk_end = 0;
  bool inside_chunk = false;

  std::vector<Ros1Connection> connections;
  std::unordered_map<uint32_t, std::size_t> conn_by_id;  // conn_id -> index

  void load(const std::filesystem::path & path)
  {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
      throw std::runtime_error("cannot open ros1 bag: " + path.string());
    }
    const auto size = f.tellg();
    if (size < static_cast<std::streamoff>(kMagic.size())) {
      throw std::runtime_error("ros1 bag too small: " + path.string());
    }
    file_bytes.resize(static_cast<std::size_t>(size));
    f.seekg(0);
    f.read(reinterpret_cast<char *>(file_bytes.data()), file_bytes.size());

    // Verify magic.
    if (std::memcmp(file_bytes.data(), kMagic.data(), kMagic.size()) != 0) {
      throw std::runtime_error("not a ros1 bag (magic mismatch): " + path.string());
    }
    cursor = kMagic.size();
  }

  // Read one record header + data block at `pos`, returning the parsed
  // header map, the data span, and updating pos to the byte after the
  // record. Returns false if pos reaches end (EOF) without enough bytes
  // to start a new record.
  bool read_record(
    std::size_t & pos, std::size_t end,
    std::unordered_map<std::string, std::span<const std::byte>> & header_out,
    std::span<const std::byte> & data_out)
  {
    if (pos >= end) {
      return false;
    }
    if (end - pos < 4) {
      throw std::runtime_error("ros1 bag: truncated record header length");
    }
    const uint32_t hlen = read_u32_le(file_bytes.data() + pos);
    pos += 4;
    if (end - pos < hlen) {
      throw std::runtime_error("ros1 bag: truncated record header body");
    }
    const auto header = std::span<const std::byte>(file_bytes.data() + pos, hlen);
    pos += hlen;

    if (end - pos < 4) {
      throw std::runtime_error("ros1 bag: truncated record data length");
    }
    const uint32_t dlen = read_u32_le(file_bytes.data() + pos);
    pos += 4;
    if (end - pos < dlen) {
      throw std::runtime_error("ros1 bag: truncated record data body");
    }
    data_out = std::span<const std::byte>(file_bytes.data() + pos, dlen);
    pos += dlen;

    header_out = parse_header_fields(header);
    return true;
  }

  void register_connection(
    const std::unordered_map<std::string, std::span<const std::byte>> & header,
    std::span<const std::byte> data)
  {
    auto conn_it = header.find("conn");
    if (conn_it == header.end() || conn_it->second.size() != 4) {
      throw std::runtime_error("ros1 bag: connection record missing conn field");
    }
    const uint32_t conn_id = read_u32_le(conn_it->second.data());
    if (conn_by_id.contains(conn_id)) {
      // Duplicate (e.g. connection block at end-of-bag); ignore.
      return;
    }

    Ros1Connection c;
    c.conn_id = conn_id;

    auto topic_it = header.find("topic");
    if (topic_it != header.end()) {
      c.topic = field_to_string(topic_it->second);
    }

    // The connection's `data` is itself a header block with type / md5 /
    // message_definition / [callerid] / [latching] / topic.
    const auto sub = parse_header_fields(data);
    auto type_it = sub.find("type");
    if (type_it == sub.end()) {
      throw std::runtime_error("ros1 bag: connection record missing 'type'");
    }
    c.type = field_to_string(type_it->second);

    if (auto it = sub.find("md5sum"); it != sub.end()) {
      c.md5sum = field_to_string(it->second);
    }
    if (auto it = sub.find("message_definition"); it != sub.end()) {
      c.message_definition = field_to_string(it->second);
    }
    if (auto it = sub.find("callerid"); it != sub.end()) {
      c.callerid = field_to_string(it->second);
    }
    if (auto it = sub.find("latching"); it != sub.end()) {
      // "latching" is "0" or "1" as ASCII bytes.
      const auto raw = field_to_string(it->second);
      try {
        c.latching = std::stoi(raw);
      } catch (const std::exception &) {
        c.latching = std::nullopt;
      }
    }
    // Topic in inner header takes precedence if present.
    if (auto it = sub.find("topic"); it != sub.end()) {
      c.topic = field_to_string(it->second);
    }

    conn_by_id[conn_id] = connections.size();
    connections.push_back(std::move(c));
  }

  // Try to advance state to a record we can return as a message. The
  // function loops over chunks and skip-able records; it sets
  // header_out, data_out, op_out for the next message-or-EOF.
  // Returns true if a MESSAGE_DATA was found, false at EOF.
  bool advance_to_message(
    std::unordered_map<std::string, std::span<const std::byte>> & header_out,
    std::span<const std::byte> & data_out)
  {
    while (true) {
      if (inside_chunk) {
        if (chunk_pos >= chunk_end) {
          inside_chunk = false;
          continue;
        }
        std::unordered_map<std::string, std::span<const std::byte>> hdr;
        std::span<const std::byte> body;
        if (!read_record(chunk_pos, chunk_end, hdr, body)) {
          inside_chunk = false;
          continue;
        }
        const uint8_t op = require_op(hdr);
        if (op == kOpConnection) {
          register_connection(hdr, body);
          continue;
        }
        if (op == kOpMessageData) {
          header_out = std::move(hdr);
          data_out = body;
          return true;
        }
        // Anything else inside a chunk is unexpected; skip silently.
        continue;
      }

      // Top-level iteration.
      if (cursor >= file_bytes.size()) {
        return false;
      }
      std::unordered_map<std::string, std::span<const std::byte>> hdr;
      std::span<const std::byte> body;
      if (!read_record(cursor, file_bytes.size(), hdr, body)) {
        return false;
      }
      const uint8_t op = require_op(hdr);
      switch (op) {
        case kOpBagHeader:
          // Just metadata; ignore. (We could read index_pos to bound
          // iteration but treating any later records as "everything we
          // need is in chunks" works fine.)
          continue;
        case kOpChunk: {
          // Compression is mandatory in the chunk header; reject anything
          // we can't read raw.
          auto comp_it = hdr.find("compression");
          if (comp_it == hdr.end()) {
            throw std::runtime_error("ros1 bag: chunk missing 'compression' field");
          }
          const auto compression = field_to_string(comp_it->second);
          if (compression != "none") {
            throw std::runtime_error(
              "ros1 bag: chunk compression '" + compression +
              "' not supported (uncompressed chunks only)");
          }
          // body holds the inner records linearly; iterate in place.
          chunk_pos = static_cast<std::size_t>(body.data() - file_bytes.data());
          chunk_end = chunk_pos + body.size();
          inside_chunk = true;
          continue;
        }
        case kOpConnection:
          // Some bags duplicate connection records at the end (the
          // index region); register them so we still know every topic.
          register_connection(hdr, body);
          continue;
        case kOpIndexData:
        case kOpChunkInfo:
          // Index records past the data section; nothing useful for us.
          continue;
        case kOpMessageData:
          // Top-level MESSAGE_DATA records are unusual but spec-legal.
          header_out = std::move(hdr);
          data_out = body;
          return true;
        default:
          BAGWIZ_LOG_WARN(kLogger, "ros1 bag: unknown op 0x%02x, skipping", op);
          continue;
      }
    }
  }
};

Rosbag1Reader::Rosbag1Reader(const std::filesystem::path & path) : impl_(std::make_unique<Impl>())
{
  impl_->load(path);
}

Rosbag1Reader::~Rosbag1Reader() = default;
Rosbag1Reader::Rosbag1Reader(Rosbag1Reader &&) noexcept = default;
Rosbag1Reader & Rosbag1Reader::operator=(Rosbag1Reader &&) noexcept = default;

const std::vector<Ros1Connection> & Rosbag1Reader::connections() const
{
  return impl_->connections;
}

bool Rosbag1Reader::next(Ros1Message & out)
{
  std::unordered_map<std::string, std::span<const std::byte>> hdr;
  std::span<const std::byte> body;
  if (!impl_->advance_to_message(hdr, body)) {
    return false;
  }

  auto conn_it = hdr.find("conn");
  if (conn_it == hdr.end() || conn_it->second.size() != 4) {
    throw std::runtime_error("ros1 bag: message_data missing conn field");
  }
  auto time_it = hdr.find("time");
  if (time_it == hdr.end() || time_it->second.size() != 8) {
    throw std::runtime_error("ros1 bag: message_data missing 8-byte time field");
  }
  const uint32_t conn_id = read_u32_le(conn_it->second.data());
  const uint32_t secs = read_u32_le(time_it->second.data());
  const uint32_t nsecs = read_u32_le(time_it->second.data() + 4);

  out.conn_id = conn_id;
  out.timestamp_ns = static_cast<int64_t>(secs) * 1'000'000'000LL + static_cast<int64_t>(nsecs);
  out.payload = body;
  return true;
}

}  // namespace bagwiz::io
