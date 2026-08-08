// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_TOPICS_HPP_
#define BAGWIZ__CORE__TF__TF_TOPICS_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core
{

// The ROS type name of TF messages carried in bags, shared by every place that
// filters topics down to TF.
inline constexpr std::string_view kTfMessageTypeName = "tf2_msgs/msg/TFMessage";

// Name half of the static-TF rule: true when the topic name's final path
// segment is exactly "tf_static" — the name is "tf_static" or ends with
// "/tf_static" (e.g. "/tf_static", "/sensing/tf_static"; "/xtf_static" does
// not qualify). This checks the name alone — neither the topic's type nor its
// recorded QoS (offered_qos_profiles) is consulted — so use it for validating
// user-supplied topic names; when a bag topic is at hand, use the
// io::TopicInfo overload below, which is the full definition.
bool is_static_tf_topic(std::string_view topic_name);

// The canonical definition of a static TF topic, applied by every bagwiz
// static-TF reader, writer, and completion path: the topic carries
// tf2_msgs/msg/TFMessage AND its name passes the rule above. Static topics
// hold one-shot, time-independent transforms; every other TFMessage topic is
// treated as dynamic.
bool is_static_tf_topic(const io::TopicInfo & topic);

// A tf2_msgs/msg/TFMessage topic in the bag plus the static flag used to
// populate a tf2 buffer with the correct static/dynamic storage.
struct TfTopic
{
  std::string name;
  bool is_static = false;
};

// Every tf2_msgs/msg/TFMessage topic the bag carries, in the reader's topic
// order, each tagged with its static flag (see is_static_tf_topic).
std::vector<TfTopic> collect_tf_topics(const io::BagReader & reader);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_TOPICS_HPP_
