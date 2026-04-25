// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__ROSBAG1_WRITER_HPP_
#define BAGWIZ__IO__ROSBAG1_WRITER_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>

namespace bagwiz::io
{

// Streaming writer for ROS 1 bag v2.0 files. Emits non-compressed
// chunks only ("compression=none"); the surrounding tooling
// (`bagwiz convert 2to1`) gates this against the read side which
// likewise only handles non-compressed chunks.
//
// Usage:
//   Rosbag1Writer w(path);
//   uint32_t cid = w.declare_connection("/topic", "std_msgs/Header",
//                                       md5, message_definition);
//   w.write(cid, time_ns, payload);
//   ...
//   w.close();
//
// The writer is *not* a subclass of io::BagWriter: that interface
// assumes ROS 2 CDR semantics (declare_topic by name + serialization
// format) and the ROS 1 connection model carries extra metadata
// (md5sum, message_definition) that does not fit into TopicInfo.
class Rosbag1Writer
{
public:
  explicit Rosbag1Writer(const std::filesystem::path & path);
  ~Rosbag1Writer();

  Rosbag1Writer(const Rosbag1Writer &) = delete;
  Rosbag1Writer & operator=(const Rosbag1Writer &) = delete;
  Rosbag1Writer(Rosbag1Writer &&) noexcept;
  Rosbag1Writer & operator=(Rosbag1Writer &&) noexcept;

  // Register a topic and return its connection id. Idempotent only on
  // exact (topic, type, md5, message_definition) match; otherwise a
  // new conn_id is allocated.
  uint32_t declare_connection(
    const std::string & topic, const std::string & ros1_type, const std::string & md5sum,
    const std::string & message_definition);

  // Append a message to the current chunk. Throws if conn_id was not
  // returned by declare_connection() on this writer.
  void write(uint32_t conn_id, int64_t timestamp_ns, std::span<const std::byte> payload);

  // Finalize the bag: flush any pending chunk, emit the index section,
  // and rewrite the bag header in place. Must be called exactly once
  // before destruction; the destructor will close the file but not
  // patch the header if close() was never called.
  void close();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__ROSBAG1_WRITER_HPP_
