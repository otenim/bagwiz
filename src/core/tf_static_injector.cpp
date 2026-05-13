// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_static_injector.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/introspection_loader.hpp"
#include "bagwiz/core/tf_value_extract.hpp"

#include <tf2_msgs/msg/tf_message.hpp>

#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>

#include <cstddef>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::core
{

namespace
{

constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr std::string_view kTfStaticSuffix = "tf_static";

bool ends_with_tf_static(std::string_view topic_name)
{
  if (topic_name.size() < kTfStaticSuffix.size()) {
    return false;
  }
  return topic_name.compare(
           topic_name.size() - kTfStaticSuffix.size(), kTfStaticSuffix.size(), kTfStaticSuffix) ==
         0;
}

// RAII wrapper around rmw_serialized_message_t. Mirrors the
// SerializedMessageRmw helper in ros2_yaml_to_cdr.cpp; kept local here
// to avoid coupling the two modules through a private header.
class SerializedMessageRmw
{
public:
  explicit SerializedMessageRmw(std::size_t capacity)
  {
    rcutils_allocator_t alloc = rcutils_get_default_allocator();
    if (rmw_serialized_message_init(&msg_, capacity, &alloc) != RMW_RET_OK) {
      throw std::runtime_error("rmw_serialized_message_init failed");
    }
  }
  ~SerializedMessageRmw() { rmw_serialized_message_fini(&msg_); }

  SerializedMessageRmw(const SerializedMessageRmw &) = delete;
  SerializedMessageRmw & operator=(const SerializedMessageRmw &) = delete;
  SerializedMessageRmw(SerializedMessageRmw &&) = delete;
  SerializedMessageRmw & operator=(SerializedMessageRmw &&) = delete;

  rmw_serialized_message_t & get() noexcept { return msg_; }

private:
  rmw_serialized_message_t msg_ = rmw_get_zero_initialized_serialized_message();
};

}  // namespace

CollectedTfStatic collect_tf_static_from_bag(const std::filesystem::path & from_bag_path)
{
  CollectedTfStatic out;

  auto reader = io::open_read(from_bag_path);
  reader->populate_schemas();

  // Walk the source bag's topic list once to find every tf2_msgs/msg/TFMessage
  // topic whose name ends in "tf_static". Filter the reader to those names
  // up-front so SQLite3/MCAP backends can skip uninteresting chunks.
  std::vector<std::string> static_topic_names;
  for (const auto & t : reader->topics()) {
    if (t.type == kTfMessageType && ends_with_tf_static(t.name)) {
      static_topic_names.push_back(t.name);
      out.source_topic_info[t.name] = t;
    }
  }
  if (static_topic_names.empty()) {
    return out;
  }

  io::ReadFilter filter;
  filter.topics = static_topic_names;
  reader->set_filter(filter);

  // One decoder per topic — schema_text and encoding may differ across
  // topics (and across shards within an MCAP), so the factory's per-topic
  // policy is the safe place to land that decision.
  std::unordered_map<std::string, std::unique_ptr<decoder::Decoder>> decoder_by_topic;
  for (const auto & topic_info : reader->topics()) {
    if (topic_info.type != kTfMessageType || !ends_with_tf_static(topic_info.name)) {
      continue;
    }
    auto open = decoder::open_decoder(topic_info);
    if (!open.ok()) {
      throw std::runtime_error(
        "could not open decoder for static TF topic '" + topic_info.name + "': " + open.error);
    }
    decoder_by_topic.emplace(topic_info.name, std::move(open.decoder));
  }

  // Last-writer-wins dedupe keyed by (parent, child). std::map keeps a
  // deterministic emission order so tests can assert on the resulting
  // vector contents without sort jitter.
  using EdgeKey = std::pair<std::string, std::string>;
  std::unordered_map<std::string, std::map<EdgeKey, geometry_msgs::msg::TransformStamped>>
    latest_by_topic;

  io::RawMessage raw;
  while (reader->next(raw)) {
    auto it = decoder_by_topic.find(raw.topic->name);
    if (it == decoder_by_topic.end()) {
      continue;
    }
    const auto decoded = it->second->decode(raw.payload);
    if (!decoded.ok()) {
      throw std::runtime_error(
        "failed to decode TFMessage on '" + raw.topic->name + "': " + decoded.error);
    }
    const auto transforms = extract_tf_message(*decoded.value);
    auto & topic_map = latest_by_topic[raw.topic->name];
    for (const auto & t : transforms) {
      if (t.header.frame_id.empty() || t.child_frame_id.empty()) {
        continue;
      }
      topic_map[{t.header.frame_id, t.child_frame_id}] = t;
    }
  }

  for (auto & topic_and_edges : latest_by_topic) {
    auto & vec = out.by_topic[topic_and_edges.first];
    vec.reserve(topic_and_edges.second.size());
    for (auto & edge_entry : topic_and_edges.second) {
      vec.push_back(std::move(edge_entry.second));
    }
  }
  for (const auto & name : static_topic_names) {
    out.by_topic.try_emplace(name);
  }
  return out;
}

std::vector<std::byte> serialize_tf_message(
  std::span<const geometry_msgs::msg::TransformStamped> transforms)
{
  auto intro = load_introspection(kTfMessageType);
  if (!intro.ok()) {
    throw std::runtime_error(
      std::string("could not load introspection typesupport for ") + kTfMessageType + ": " +
      intro.error);
  }

  // The introspection typesupport and rosidl_generator_cpp emit
  // structurally identical layouts for the same IDL, so we can build a
  // standard C++ TFMessage and hand its address to rmw_serialize.
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.assign(transforms.begin(), transforms.end());

  SerializedMessageRmw serialized(0);
  const rmw_ret_t rc = rmw_serialize(&msg, intro.typesupport, &serialized.get());
  if (rc != RMW_RET_OK) {
    const rcutils_error_state_t * s = rcutils_get_error_state();
    std::string err = "rmw_serialize failed: ";
    err += (s != nullptr) ? s->message : "(no error message)";
    rcutils_reset_error();
    throw std::runtime_error(err);
  }

  const auto * sm = &serialized.get();
  std::vector<std::byte> out;
  out.resize(sm->buffer_length);
  if (sm->buffer_length > 0 && sm->buffer != nullptr) {
    std::memcpy(out.data(), sm->buffer, sm->buffer_length);
  }
  return out;
}

}  // namespace bagwiz::core
