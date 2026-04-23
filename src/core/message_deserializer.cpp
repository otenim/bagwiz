// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/message_deserializer.hpp"

#include "bagwiz/core/introspection_loader.hpp"

#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <stdexcept>
#include <string>

namespace bagwiz::core
{

namespace
{

// Preview of the first bytes of the payload used in error messages so
// CDR header issues can be diagnosed without re-running with a debugger.
std::string hex_preview(std::span<const std::byte> payload, std::size_t max_bytes = 16)
{
  const std::size_t n = std::min(payload.size(), max_bytes);
  std::string out;
  out.reserve(n * 3 + 4);
  for (std::size_t i = 0; i < n; ++i) {
    char buf[4];
    std::snprintf(buf, sizeof(buf), "%02x ", static_cast<unsigned>(payload[i]));
    out += buf;
  }
  if (payload.size() > n) {
    out += "...";
  }
  return out;
}

}  // namespace

DeserializedMessage::DeserializedMessage(
  const IntrospectionLoad & introspection, std::span<const std::byte> cdr_payload)
{
  if (!introspection.ok()) {
    throw std::runtime_error("introspection not loaded");
  }
  members_ = introspection.members;

  // Allocate an aligned buffer. alignof(std::max_align_t) covers
  // std::string/vector alignment on all ROS 2 supported platforms.
  void * p = nullptr;
  const std::size_t size = members_->size_of_ == 0 ? 1 : members_->size_of_;
  if (::posix_memalign(&p, alignof(std::max_align_t), size) != 0 || p == nullptr) {
    throw std::bad_alloc();
  }
  buffer_ = p;

  members_->init_function(buffer_, rosidl_runtime_cpp::MessageInitialization::ALL);
  initialized_ = true;

  // Copy the payload into an rcutils-managed buffer so ownership semantics
  // are clean, regardless of what the RMW implementation chooses to do
  // with the bytes.
  rcutils_allocator_t alloc = rcutils_get_default_allocator();
  rmw_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
  const rmw_ret_t init_ret = rmw_serialized_message_init(&serialized, cdr_payload.size(), &alloc);
  if (init_ret != RMW_RET_OK) {
    throw std::runtime_error("rmw_serialized_message_init failed");
  }
  std::memcpy(serialized.buffer, cdr_payload.data(), cdr_payload.size());
  serialized.buffer_length = cdr_payload.size();

  const rmw_ret_t rc = rmw_deserialize(&serialized, introspection.typesupport, buffer_);
  std::string err;
  if (rc != RMW_RET_OK) {
    const rcutils_error_state_t * s = rcutils_get_error_state();
    err = "rmw_deserialize failed (size=" + std::to_string(cdr_payload.size()) +
          ", first bytes: " + hex_preview(cdr_payload) + "): ";
    err += s != nullptr ? s->message : "(no error message)";
    rcutils_reset_error();
  }
  rmw_serialized_message_fini(&serialized);
  if (!err.empty()) {
    throw std::runtime_error(err);
  }
}

DeserializedMessage::~DeserializedMessage()
{
  if (initialized_ && members_ != nullptr && buffer_ != nullptr) {
    members_->fini_function(buffer_);
  }
  std::free(buffer_);
}

}  // namespace bagwiz::core
