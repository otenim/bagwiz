// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace bagwiz::io
{

FrozenMessage own_payload(
  std::vector<std::byte> && bytes, const TopicInfo * topic, std::int64_t timestamp_ns)
{
  auto buf = std::make_shared<std::vector<std::byte>>(std::move(bytes));
  FrozenMessage out;
  out.topic = topic;
  out.timestamp_ns = timestamp_ns;
  out.payload = std::span<const std::byte>(buf->data(), buf->size());
  out.owner = std::move(buf);
  return out;
}

// Default freeze(): copy the payload into a fresh owned buffer. Readers whose
// backing store is shareable (the parallel indexed MCAP path) override this
// to alias that store instead; for every other backing (libmcap's recycled
// chunk buffer, SQLite's row buffer, a decompressor's reusable scratch) a
// copy is the only correct way to make the payload outlive the next next()
// call.
FrozenMessage BagReader::freeze(const RawMessage & msg) const
{
  return own_payload({msg.payload.begin(), msg.payload.end()}, msg.topic, msg.timestamp_ns);
}

}  // namespace bagwiz::io
