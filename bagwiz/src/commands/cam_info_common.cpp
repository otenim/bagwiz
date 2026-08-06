// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "cam_info_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/str_utils.hpp"
#include "bagwiz/core/cdr_walker/cdr_writer.hpp"

#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>

#include <cstddef>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

std::string take_rmw_error()
{
  // rcutils_get_error_state() returns the (always allocated) thread-local
  // state, so "no error" reads as an empty message rather than a null pointer.
  const rcutils_error_state_t * s = rcutils_get_error_state();
  std::string err = (s != nullptr && s->message[0] != '\0') ? s->message : "(no error message)";
  rcutils_reset_error();
  return err;
}

sensor_msgs::msg::CameraInfo deserialize_camera_info(
  std::span<const std::byte> payload, const rosidl_message_type_support_t * typesupport,
  const std::string & topic)
{
  // Wrap the reader's payload as an rmw_serialized_message_t without taking
  // ownership of it (no _fini on this view; the bytes belong to the reader).
  rmw_serialized_message_t in_view = rmw_get_zero_initialized_serialized_message();
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast) rmw API is non-const but only reads.
  in_view.buffer =
    const_cast<std::uint8_t *>(reinterpret_cast<const std::uint8_t *>(payload.data()));
  in_view.buffer_length = payload.size();
  in_view.buffer_capacity = payload.size();
  in_view.allocator = rcutils_get_default_allocator();

  sensor_msgs::msg::CameraInfo msg;
  if (rmw_deserialize(&in_view, typesupport, &msg) != RMW_RET_OK) {
    throw std::runtime_error(
      "failed to deserialize a message on '" + topic + "' as " + kCameraInfoType + ": " +
      take_rmw_error());
  }
  return msg;
}

std::vector<std::byte> serialize_camera_info(const sensor_msgs::msg::CameraInfo & msg)
{
  core::cdr_walker::CdrWriter writer;
  writer.write_i32(msg.header.stamp.sec);
  writer.write_u32(msg.header.stamp.nanosec);
  writer.write_string(msg.header.frame_id);
  writer.write_u32(msg.height);
  writer.write_u32(msg.width);
  writer.write_string(msg.distortion_model);
  writer.write_sequence_length(static_cast<std::uint32_t>(msg.d.size()));
  for (const double v : msg.d) {
    writer.write_f64(v);
  }
  for (const double v : msg.k) {
    writer.write_f64(v);
  }
  for (const double v : msg.r) {
    writer.write_f64(v);
  }
  for (const double v : msg.p) {
    writer.write_f64(v);
  }
  writer.write_u32(msg.binning_x);
  writer.write_u32(msg.binning_y);
  writer.write_u32(msg.roi.x_offset);
  writer.write_u32(msg.roi.y_offset);
  writer.write_u32(msg.roi.height);
  writer.write_u32(msg.roi.width);
  writer.write_bool(msg.roi.do_rectify);
  return writer.take();
}

std::vector<std::string> camera_info_topic_names(std::span<const io::TopicInfo> topics)
{
  std::vector<std::string> names;
  for (const auto & t : topics) {
    if (t.type == kCameraInfoType) {
      names.push_back(t.name);
    }
  }
  return names;
}

CameraInfoTargets validate_camera_info_targets(
  std::span<const io::TopicInfo> topics, const std::vector<std::string> & requested,
  const std::filesystem::path & input_path, const char * logger)
{
  std::unordered_map<std::string, const io::TopicInfo *> topics_by_name;
  for (const auto & t : topics) {
    topics_by_name.emplace(t.name, &t);
  }

  // Validate each requested topic, deduplicating while preserving command-line
  // order. Collect all failures before bailing so one run reports every bad
  // topic, and stop before the bag is touched if any failed.
  CameraInfoTargets result;
  std::unordered_set<std::string> seen;
  for (const auto & topic : requested) {
    if (!seen.insert(topic).second) {
      continue;  // duplicate on the command line; validated on its first occurrence
    }
    const auto it = topics_by_name.find(topic);
    if (it == topics_by_name.end()) {
      BAGWIZ_LOG_ERROR(
        logger, "Topic '%s' is not present in %s.", topic.c_str(), input_path.c_str());
      result.all_valid = false;
      continue;
    }
    if (it->second->type != kCameraInfoType) {
      BAGWIZ_LOG_ERROR(
        logger, "Topic '%s' has type '%s', expected '%s'.", topic.c_str(), it->second->type.c_str(),
        kCameraInfoType);
      result.all_valid = false;
      continue;
    }
    result.topics.push_back(topic);
  }
  if (!result.all_valid) {
    const std::string available = core::join_csv(camera_info_topic_names(topics));
    BAGWIZ_LOG_ERROR(logger, "Available %s topic(s): %s", kCameraInfoType, available.c_str());
  }
  return result;
}

CamInfoReplaceTargets parse_cam_info_replace_targets(
  const std::vector<std::string> & entries,
  const std::optional<std::filesystem::path> & default_yaml, const char * logger)
{
  CamInfoReplaceTargets result;
  // topic -> index into result.targets, for duplicate detection.
  std::unordered_map<std::string, std::size_t> seen;
  bool used_default = false;
  for (const auto & entry : entries) {
    const auto sep = entry.find('=');
    std::string topic;
    std::filesystem::path yaml;
    if (sep == std::string::npos) {
      if (!default_yaml.has_value()) {
        BAGWIZ_LOG_ERROR(
          logger, "-t entry '%s' names no calibration: give --yaml or use <topic>=<yaml>.",
          entry.c_str());
        result.all_valid = false;
        continue;
      }
      topic = entry;
      yaml = *default_yaml;
      used_default = true;
    } else if (sep == 0 || sep + 1 == entry.size()) {
      BAGWIZ_LOG_ERROR(
        logger, "-t entry '%s' is not of the form <topic> or <topic>=<yaml>.", entry.c_str());
      result.all_valid = false;
      continue;
    } else {
      // Topic names cannot contain '=', so the first '=' ends the topic half;
      // any further '=' belongs to the YAML path.
      topic = entry.substr(0, sep);
      yaml = entry.substr(sep + 1);
    }
    const auto it = seen.find(topic);
    if (it != seen.end()) {
      if (result.targets[it->second].yaml_path != yaml) {
        BAGWIZ_LOG_ERROR(
          logger, "Topic '%s' is listed with two different YAMLs ('%s' and '%s').", topic.c_str(),
          result.targets[it->second].yaml_path.c_str(), yaml.c_str());
        result.all_valid = false;
      }
      continue;  // an exact duplicate is harmless
    }
    seen.emplace(topic, result.targets.size());
    result.targets.push_back({std::move(topic), std::move(yaml)});
  }
  if (default_yaml.has_value() && !used_default) {
    BAGWIZ_LOG_ERROR(
      logger,
      "--yaml was given but every -t entry carries its own '=<yaml>'; drop --yaml or leave one "
      "entry bare.");
    result.all_valid = false;
  }
  return result;
}

CameraInfoProcessor::CameraInfoProcessor(
  const std::vector<std::string> & topics, const rosidl_message_type_support_t * typesupport)
: targets_(topics.begin(), topics.end()), typesupport_(typesupport)
{
  for (const auto & topic : topics) {
    rewritten_.emplace(topic, 0);
  }
}

std::uint64_t CameraInfoProcessor::rewritten_count(const std::string & topic) const
{
  const auto it = rewritten_.find(topic);
  return it == rewritten_.end() ? 0 : it->second;
}

core::pipeline::Emit CameraInfoProcessor::route(const std::string & in_topic) const
{
  return core::pipeline::Emit{true, in_topic};
}

core::pipeline::TransformAction CameraInfoProcessor::transform(
  const io::RawMessage & msg, std::vector<std::byte> & out) const
{
  const std::string & in_topic = msg.topic->name;
  if (targets_.find(in_topic) == targets_.end()) {
    return core::pipeline::TransformAction::kPassthrough;
  }

  // The deserialize -> mutate -> serialize round-trip preserves every field
  // mutate() does not touch (header, binning, roi, and the calibration fields
  // the command leaves alone).
  sensor_msgs::msg::CameraInfo typed = deserialize_camera_info(msg.payload, typesupport_, in_topic);
  mutate(in_topic, typed);
  out = serialize_camera_info(typed);

  ++rewritten_.at(in_topic);
  return core::pipeline::TransformAction::kWrite;
}

}  // namespace bagwiz::commands
