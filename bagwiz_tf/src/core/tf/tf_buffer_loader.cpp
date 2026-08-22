// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_buffer_loader.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::core
{

std::optional<std::string> load_tf_buffer(
  const std::filesystem::path & input, tf2::BufferCore & buffer)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    return "failed to open '" + input.string() + "': " + e.what();
  }

  std::vector<const io::TopicInfo *> tf_topics;
  for (const auto & t : reader->topics()) {
    if (t.type == kTfMessageTypeName) {
      tf_topics.push_back(&t);
    }
  }
  if (tf_topics.empty()) {
    return "no tf2_msgs/msg/TFMessage topics found; cannot resolve point-cloud transform";
  }

  io::ReadFilter filter;
  for (const auto * t : tf_topics) {
    filter.topics.push_back(t->name);
  }
  reader->set_filter(filter);

  std::unordered_map<std::string, std::unique_ptr<decoder::Decoder>> decoders;
  for (const auto * t : tf_topics) {
    auto open = decoder::open_decoder(*t);
    if (!open.ok()) {
      return "could not open decoder for TF topic '" + t->name + "': " + open.error;
    }
    decoders.emplace(t->name, std::move(open.decoder));
  }

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      auto it = decoders.find(raw.topic->name);
      if (it == decoders.end()) {
        continue;
      }
      const auto decoded = it->second->decode(raw.payload);
      if (!decoded.ok()) {
        return "failed to decode TF message on '" + raw.topic->name + "': " + decoded.error;
      }
      const auto transforms = extract_tf_message(*decoded.value);
      const bool is_static = is_static_tf_topic(*raw.topic);
      for (const auto & t : transforms) {
        buffer.setTransform(t, "bagwiz", is_static);
      }
    }
  } catch (const std::exception & e) {
    return "error reading TF topics: " + std::string(e.what());
  }
  return std::nullopt;
}

StaticTfTransforms load_static_tf_transforms(const std::filesystem::path & input, StaticTfRead read)
{
  StaticTfTransforms out;
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    out.error = std::string("failed to reopen bag for static TF: ") + e.what();
    return out;
  }

  std::vector<const io::TopicInfo *> static_topics;
  for (const auto & t : reader->topics()) {
    if (is_static_tf_topic(t)) {
      static_topics.push_back(&t);
    }
  }
  if (static_topics.empty()) {
    // Caller-neutral: this helper is shared across commands with different
    // flags (pcd concat's --frame, pcd undistort's --ref/--of, ...), so it
    // must not bake any one of them into the message. Callers that want
    // flag-specific context should prepend their own.
    out.error = "bag has no static TF topic (…/tf_static)";
    return out;
  }

  io::ReadFilter filter;
  for (const auto * t : static_topics) {
    filter.topics.push_back(t->name);
  }
  reader->set_filter(filter);

  std::unordered_map<std::string, std::unique_ptr<decoder::Decoder>> decoders;
  for (const auto * t : static_topics) {
    auto open = decoder::open_decoder(*t);
    if (!open.ok()) {
      out.error = "could not open decoder for '" + t->name + "': " + open.error;
      return out;
    }
    decoders.emplace(t->name, std::move(open.decoder));
  }

  // kFirstMessagePerTopic: the topics still waiting for their first message.
  // A topic leaves the set once it has one, and an empty set ends the read —
  // the storage-level topic filter cannot express "first only", and the rest
  // of a long recording is republications of the same latched set.
  std::unordered_set<std::string> pending;
  if (read == StaticTfRead::kFirstMessagePerTopic) {
    for (const auto * t : static_topics) {
      pending.insert(t->name);
    }
  }

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      const auto it = decoders.find(raw.topic->name);
      if (it == decoders.end()) {
        continue;
      }
      if (read == StaticTfRead::kFirstMessagePerTopic && !pending.contains(raw.topic->name)) {
        continue;
      }
      const auto decoded = it->second->decode(raw.payload);
      if (!decoded.ok()) {
        out.error = "failed to decode static TF on '" + raw.topic->name + "': " + decoded.error;
        out.transforms.clear();
        return out;
      }
      const auto transforms = extract_tf_message(*decoded.value);
      out.transforms.insert(out.transforms.end(), transforms.begin(), transforms.end());
      if (read == StaticTfRead::kFirstMessagePerTopic) {
        pending.erase(raw.topic->name);
        if (pending.empty()) {
          break;
        }
      }
    }
  } catch (const std::exception & e) {
    out.error = std::string("error reading static TF: ") + e.what();
    out.transforms.clear();
    return out;
  }
  return out;
}

void set_static_transforms(
  tf2::BufferCore & buffer, std::span<const geometry_msgs::msg::TransformStamped> transforms)
{
  for (const auto & t : transforms) {
    buffer.setTransform(t, "bagwiz", /*is_static=*/true);
  }
}

std::optional<std::string> load_static_tf_buffer(
  const std::filesystem::path & input, tf2::BufferCore & buffer, StaticTfRead read)
{
  const auto loaded = load_static_tf_transforms(input, read);
  if (!loaded.ok()) {
    return loaded.error;
  }
  set_static_transforms(buffer, loaded.transforms);
  return std::nullopt;
}

void replay_tf_topics(
  io::BagReader & reader, const std::vector<TfTopic> & tf_topics, const TfReplayOutputs & outputs)
{
  io::ReadFilter filter;
  std::unordered_map<std::string, bool> is_static_by_topic;
  for (const auto & t : tf_topics) {
    filter.topics.push_back(t.name);
    is_static_by_topic[t.name] = t.is_static;
  }
  reader.set_filter(filter);

  // One decoder per TF topic so per-topic schema_text differences are handled
  // by the decoder factory rather than by the caller.
  std::unordered_map<std::string, std::unique_ptr<decoder::Decoder>> decoder_by_topic;
  for (const auto & topic_info : reader.topics()) {
    if (topic_info.type != kTfMessageTypeName) {
      continue;
    }
    if (is_static_by_topic.find(topic_info.name) == is_static_by_topic.end()) {
      continue;
    }
    auto open = decoder::open_decoder(topic_info);
    if (!open.ok()) {
      throw std::runtime_error(
        "Could not open decoder for TF topic '" + topic_info.name + "': " + open.error);
    }
    decoder_by_topic.emplace(topic_info.name, std::move(open.decoder));
  }

  io::RawMessage raw;
  while (reader.next(raw)) {
    auto it = decoder_by_topic.find(raw.topic->name);
    if (it == decoder_by_topic.end()) {
      continue;
    }
    const auto decoded = it->second->decode(raw.payload);
    if (!decoded.ok()) {
      throw std::runtime_error(
        "Failed to decode TF message on '" + raw.topic->name + "': " + decoded.error);
    }
    const auto transforms = extract_tf_message(*decoded.value);
    const bool is_static = is_static_by_topic.at(raw.topic->name);
    for (const auto & t : transforms) {
      // Transforms with an empty parent/child frame id say nothing about the
      // tree's shape, so they are excluded from conflict detection and edge
      // collection; the buffer, stamps, and input edges still record them.
      if (
        outputs.conflict_checker != nullptr && !t.header.frame_id.empty() &&
        !t.child_frame_id.empty()) {
        if (
          const auto conflict = outputs.conflict_checker->add(
            t.header.frame_id, t.child_frame_id, raw.topic->name, is_static)) {
          throw std::runtime_error("TF merge conflict: " + *conflict);
        }
      }
      if (outputs.buffer != nullptr) {
        outputs.buffer->setTransform(t, "bagwiz", is_static);
      }
      if (
        outputs.edges_by_topic != nullptr && !t.header.frame_id.empty() &&
        !t.child_frame_id.empty()) {
        (*outputs.edges_by_topic)[raw.topic->name].insert(
          std::make_pair(t.header.frame_id, t.child_frame_id));
      }
      if (outputs.stamps != nullptr) {
        outputs.stamps->emplace_back(
          std::chrono::seconds(t.header.stamp.sec) +
          std::chrono::nanoseconds(t.header.stamp.nanosec));
      }
      if (outputs.input_edges != nullptr && raw.topic->name == outputs.input_topic) {
        const std::int64_t ns = static_cast<std::int64_t>(t.header.stamp.sec) * 1'000'000'000LL +
                                static_cast<std::int64_t>(t.header.stamp.nanosec);
        outputs.input_edges->push_back({t.header.frame_id, t.child_frame_id, ns});
      }
    }
  }
}

}  // namespace bagwiz::core
