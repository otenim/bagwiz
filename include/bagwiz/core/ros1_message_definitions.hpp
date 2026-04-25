// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__ROS1_MESSAGE_DEFINITIONS_HPP_
#define BAGWIZ__CORE__ROS1_MESSAGE_DEFINITIONS_HPP_

#include <string>
#include <string_view>

namespace bagwiz::core
{

// Connection-level metadata required by the ROS 1 bag v2.0 connection
// record. md5sum is the canonical MD5 of the ROS 1 .msg schema (used
// by ROS 1 publishers/subscribers to verify type compatibility);
// message_definition is the full .msg text concatenated with any
// dependent message types. Both come from authoritative upstream
// sources (ros/std_msgs, ros/common_msgs, ros/geometry2).
struct Ros1TypeMeta
{
  std::string md5sum;
  std::string message_definition;
};

// Lookup md5sum + message_definition for a ROS 1 type name (e.g.
// "std_msgs/Header"). Returns nullptr for any type not in the
// whitelist; callers should fall back to skipping the topic.
const Ros1TypeMeta * find_ros1_meta(std::string_view ros1_type);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__ROS1_MESSAGE_DEFINITIONS_HPP_
