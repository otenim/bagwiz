// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__ROSBAG1_READER_HPP_
#define BAGWIZ__IO__ROSBAG1_READER_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace bagwiz::io
{

// One connection record from a ROS 1 v2.0 .bag. ROS 1 type names use the
// legacy two-segment form, e.g. "std_msgs/Header" (no `/msg/` infix).
struct Ros1Connection
{
  uint32_t conn_id = 0;
  std::string topic;
  std::string type;
  std::string md5sum;
  std::string message_definition;
  std::optional<std::string> callerid;
  std::optional<int> latching;
};

struct Ros1Message
{
  uint32_t conn_id = 0;
  int64_t timestamp_ns = 0;
  std::span<const std::byte> payload{};  // valid until the next next() call
};

// Streaming reader for ROS 1 bag v2.0 files. Phase 1 supports only
// uncompressed chunks; bz2 / lz4 chunks raise a runtime_error on first
// encounter.
//
// Records outside chunks (BAG_HEADER, INDEX_DATA, CHUNK_INFO, and the
// duplicated CONNECTION block at index_pos) are skipped: connections
// appearing inside chunks are sufficient to enumerate every topic a
// well-formed bag carries.
class Rosbag1Reader
{
public:
  explicit Rosbag1Reader(const std::filesystem::path & path);
  ~Rosbag1Reader();

  Rosbag1Reader(const Rosbag1Reader &) = delete;
  Rosbag1Reader & operator=(const Rosbag1Reader &) = delete;
  Rosbag1Reader(Rosbag1Reader &&) noexcept;
  Rosbag1Reader & operator=(Rosbag1Reader &&) noexcept;

  // All connections discovered so far. Populated incrementally as the
  // reader walks through chunks; calling next() until it returns false
  // guarantees the list is complete.
  const std::vector<Ros1Connection> & connections() const;

  // Pull the next MESSAGE_DATA record. Returns false at EOF. The
  // payload span is invalidated by the next call to next() or by
  // destruction.
  bool next(Ros1Message & out);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__ROSBAG1_READER_HPP_
