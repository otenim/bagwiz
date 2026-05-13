// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF_STATIC_INJECTOR_HPP_
#define BAGWIZ__CORE__TF_STATIC_INJECTOR_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cstddef>
#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <vector>

// Helpers backing `bagwiz tf inject-static`. The command reads every
// *tf_static topic in a source bag, consolidates the cumulative latest
// view of static TF edges into a single TFMessage per topic, and
// injects them into a copy of the destination bag at the destination's
// start timestamp.
namespace bagwiz::core
{

// Outcome of scanning the source bag.
//
// Each entry in `by_topic` is the deduplicated set of TransformStamped
// values for one *tf_static topic — keyed by (header.frame_id,
// child_frame_id), last-writer-wins (the last message in bag order is
// authoritative because that is how tf2's static buffer would resolve
// repeated edges on replay).
//
// `source_topic_info` mirrors the source bag's TopicInfo for the same
// topics so the caller can declare a new topic in the destination when
// the destination did not previously carry it.
struct CollectedTfStatic
{
  std::map<std::string, std::vector<geometry_msgs::msg::TransformStamped>> by_topic;
  std::map<std::string, io::TopicInfo> source_topic_info;
};

// Scan `from_bag_path` end-to-end and collect every TransformStamped
// across all topics whose name ends in "tf_static" and whose type is
// `tf2_msgs/msg/TFMessage`. Throws std::runtime_error on IO / decode
// failure. Returns an empty CollectedTfStatic when no static TF topic
// is present — the caller decides whether that is an error.
CollectedTfStatic collect_tf_static_from_bag(const std::filesystem::path & from_bag_path);

// Encode a single tf2_msgs/msg/TFMessage carrying `transforms` into
// the on-wire CDR payload. Uses the introspection typesupport (dlopen'd
// at runtime from libtf2_msgs__rosidl_typesupport_introspection_cpp.so)
// + rmw_serialize. Throws std::runtime_error when the typesupport
// cannot be loaded or rmw_serialize fails.
std::vector<std::byte> serialize_tf_message(
  std::span<const geometry_msgs::msg::TransformStamped> transforms);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF_STATIC_INJECTOR_HPP_
