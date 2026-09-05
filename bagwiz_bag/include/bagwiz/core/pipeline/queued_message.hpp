// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__PIPELINE__QUEUED_MESSAGE_HPP_
#define BAGWIZ__CORE__PIPELINE__QUEUED_MESSAGE_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <string_view>

namespace bagwiz::core::pipeline
{

// A routed message handed from PipelinedBackend's read thread to its write
// thread. io::RawMessage's payload span and topic pointer are invalidated by
// the next BagReader::next() call, so a backend that overlaps reading and
// writing on separate threads cannot hand a RawMessage across the thread
// boundary. The read thread instead freezes the message (BagReader::freeze)
// — zero-copy where the reader's backing store is shareable, one payload copy
// where it is not — and the write thread consumes it after the reader has
// moved on. PipelinedBackend is the only user.
struct QueuedMessage
{
  // The resolved OUTPUT topic name (after routing/rename) as a view. Per the
  // Emit contract this views either reader-owned TopicInfo storage (alive for
  // the whole run) or router-owned selector storage (caller-owned, likewise),
  // so it stays valid on the write thread without a per-message copy.
  std::string_view out_topic;
  io::FrozenMessage frozen;
};

}  // namespace bagwiz::core::pipeline

#endif  // BAGWIZ__CORE__PIPELINE__QUEUED_MESSAGE_HPP_
