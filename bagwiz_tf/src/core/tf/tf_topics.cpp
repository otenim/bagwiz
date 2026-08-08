// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_topics.hpp"

#include <vector>

namespace bagwiz::core
{

namespace
{

// Leaf-segment form: anchoring on the '/' keeps sibling names like
// "/xtf_static" out; the bare "tf_static" (no namespace) is matched separately.
constexpr std::string_view kTfStaticLeaf = "tf_static";
constexpr std::string_view kTfStaticSuffix = "/tf_static";

}  // namespace

bool is_static_tf_topic(std::string_view topic_name)
{
  if (topic_name == kTfStaticLeaf) {
    return true;
  }
  return topic_name.size() >= kTfStaticSuffix.size() &&
         topic_name.compare(
           topic_name.size() - kTfStaticSuffix.size(), kTfStaticSuffix.size(), kTfStaticSuffix) ==
           0;
}

bool is_static_tf_topic(const io::TopicInfo & topic)
{
  return topic.type == kTfMessageTypeName && is_static_tf_topic(std::string_view{topic.name});
}

std::vector<TfTopic> collect_tf_topics(const io::BagReader & reader)
{
  std::vector<TfTopic> topics;
  for (const auto & t : reader.topics()) {
    if (t.type == kTfMessageTypeName) {
      topics.push_back({t.name, is_static_tf_topic(std::string_view{t.name})});
    }
  }
  return topics;
}

}  // namespace bagwiz::core
