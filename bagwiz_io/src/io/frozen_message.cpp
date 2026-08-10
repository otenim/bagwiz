// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace bagwiz::io
{

// Default freeze(): copy the payload into a fresh owned buffer. Readers whose
// backing store is shareable (the parallel indexed MCAP path) override this
// to alias that store instead; for every other backing (libmcap's recycled
// chunk buffer, SQLite's row buffer, a decompressor's reusable scratch) a
// copy is the only correct way to make the payload outlive the next next()
// call.
FrozenMessage BagReader::freeze(const RawMessage & msg) const
{
  auto buf = std::make_shared<std::vector<std::byte>>(msg.payload.begin(), msg.payload.end());
  FrozenMessage out;
  out.topic = msg.topic;
  out.timestamp_ns = msg.timestamp_ns;
  out.owner = buf;
  out.payload = std::span<const std::byte>(buf->data(), buf->size());
  return out;
}

}  // namespace bagwiz::io
