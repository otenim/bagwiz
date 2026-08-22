// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_buffer_loader.hpp"

#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::core::load_static_tf_buffer;
using bagwiz::core::load_static_tf_transforms;
using bagwiz::core::set_static_transforms;
using bagwiz::core::StaticTfRead;

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions o;
  o.format = bagwiz::io::Format::Mcap;
  o.layout = bagwiz::io::Layout::SingleFile;
  o.mcap_compression = "none";
  return o;
}

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child, double tx)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.child_frame_id = child;
  ts.transform.translation.x = tx;
  ts.transform.translation.y = 0.0;
  ts.transform.translation.z = 0.0;
  ts.transform.rotation.x = 0.0;
  ts.transform.rotation.y = 0.0;
  ts.transform.rotation.z = 0.0;
  ts.transform.rotation.w = 1.0;
  return ts;
}

// Writes a single-topic bag with one /tf_static TFMessage: base_link -> lidar,
// translation (1,0,0).
void write_tf_static_bag(const std::filesystem::path & path)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  std::vector<geometry_msgs::msg::TransformStamped> edges;
  edges.push_back(make_edge("base_link", "lidar", 1.0));
  const auto payload = bagwiz::core::serialize_tf_message(edges);
  w->write("/tf_static", 0, std::span<const std::byte>(payload.data(), payload.size()));
  w->close();
}

TEST(LoadStaticTfBuffer, ResolvesStaticEdge)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tfstatic_test.mcap";
  std::filesystem::remove(bag);
  write_tf_static_bag(bag);

  tf2::BufferCore buffer{std::chrono::hours(24 * 365)};
  ASSERT_FALSE(load_static_tf_buffer(bag, buffer).has_value());
  const auto ts = buffer.lookupTransform("base_link", "lidar", tf2::TimePointZero);
  EXPECT_NEAR(ts.transform.translation.x, 1.0, 1e-9);

  std::filesystem::remove(bag);
}

TEST(LoadStaticTfBuffer, ErrorsWhenNoStaticTfTopic)
{
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tfstatic_missing_test.mcap";
  std::filesystem::remove(bag);
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    w->close();
  }

  tf2::BufferCore buffer{std::chrono::hours(24 * 365)};
  const auto error = load_static_tf_buffer(bag, buffer);
  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("tf_static"), std::string::npos);

  std::filesystem::remove(bag);
}

}  // namespace

namespace
{

// One static TF message: its topic, its bag stamp and the edges it carries.
struct StaticMessage
{
  std::string topic;
  std::int64_t stamp_ns;
  std::vector<geometry_msgs::msg::TransformStamped> edges;
};

// Writes `messages` (declared topics first, in order of first appearance).
void write_static_messages_bag(
  const std::filesystem::path & path, const std::vector<StaticMessage> & messages)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  std::vector<std::string> declared;
  for (const auto & m : messages) {
    if (std::find(declared.begin(), declared.end(), m.topic) == declared.end()) {
      w->declare_topic(bagwiz::core::make_tf_message_topic_info(m.topic));
      declared.push_back(m.topic);
    }
  }
  for (const auto & m : messages) {
    const auto payload = bagwiz::core::serialize_tf_message(m.edges);
    w->write(m.topic, m.stamp_ns, std::span<const std::byte>(payload.data(), payload.size()));
  }
  w->close();
}

double lookup_tx(const tf2::BufferCore & buffer, const std::string & child)
{
  return buffer.lookupTransform("base_link", child, tf2::TimePointZero).transform.translation.x;
}

}  // namespace

TEST(LoadStaticTfBuffer, FirstMessageModeKeepsTheFirstValue)
{
  // /tf_static re-published with a different value: the whole-topic read
  // lets the later message win (today's drain), the first-message read stops
  // at the first one.
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tfstatic_first_test.mcap";
  std::filesystem::remove(bag);
  write_static_messages_bag(
    bag, {{"/tf_static", 0, {make_edge("base_link", "lidar", 1.0)}},
          {"/tf_static", 1'000'000'000LL, {make_edge("base_link", "lidar", 2.0)}}});

  tf2::BufferCore whole{std::chrono::hours(24 * 365)};
  ASSERT_FALSE(load_static_tf_buffer(bag, whole, StaticTfRead::kWholeTopic).has_value());
  EXPECT_NEAR(lookup_tx(whole, "lidar"), 2.0, 1e-9);

  tf2::BufferCore first{std::chrono::hours(24 * 365)};
  ASSERT_FALSE(load_static_tf_buffer(bag, first, StaticTfRead::kFirstMessagePerTopic).has_value());
  EXPECT_NEAR(lookup_tx(first, "lidar"), 1.0, 1e-9);

  const auto whole_list = load_static_tf_transforms(bag, StaticTfRead::kWholeTopic);
  ASSERT_TRUE(whole_list.ok()) << whole_list.error;
  EXPECT_EQ(whole_list.transforms.size(), 2U);
  const auto first_list = load_static_tf_transforms(bag, StaticTfRead::kFirstMessagePerTopic);
  ASSERT_TRUE(first_list.ok()) << first_list.error;
  ASSERT_EQ(first_list.transforms.size(), 1U);
  EXPECT_NEAR(first_list.transforms[0].transform.translation.x, 1.0, 1e-9);

  std::filesystem::remove(bag);
}

TEST(LoadStaticTfBuffer, FirstMessageModeWaitsForEveryStaticTopic)
{
  // Two static topics; the second one's first message comes after the first
  // topic's republication. First-message mode must read on until the second
  // topic has produced its message, while still taking the first topic's
  // first value.
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tfstatic_two_topics_test.mcap";
  std::filesystem::remove(bag);
  write_static_messages_bag(
    bag, {{"/tf_static", 0, {make_edge("base_link", "lidar", 1.0)}},
          {"/tf_static", 2'000'000'000LL, {make_edge("base_link", "lidar", 9.0)}},
          {"/robot/tf_static", 5'000'000'000LL, {make_edge("base_link", "cam", 3.0)}}});

  tf2::BufferCore first{std::chrono::hours(24 * 365)};
  ASSERT_FALSE(load_static_tf_buffer(bag, first, StaticTfRead::kFirstMessagePerTopic).has_value());
  EXPECT_NEAR(lookup_tx(first, "lidar"), 1.0, 1e-9);
  EXPECT_NEAR(lookup_tx(first, "cam"), 3.0, 1e-9);

  tf2::BufferCore whole{std::chrono::hours(24 * 365)};
  ASSERT_FALSE(load_static_tf_buffer(bag, whole).has_value());
  EXPECT_NEAR(lookup_tx(whole, "lidar"), 9.0, 1e-9);
  EXPECT_NEAR(lookup_tx(whole, "cam"), 3.0, 1e-9);

  std::filesystem::remove(bag);
}

TEST(LoadStaticTfBuffer, SetStaticTransformsFeedsABufferLikeTheLoader)
{
  // The transform list fed by hand gives the same lookups as the loader's
  // own buffer — what lets one read serve several buffers.
  const std::filesystem::path bag =
    std::filesystem::temp_directory_path() / "bagwiz_tfstatic_set_test.mcap";
  std::filesystem::remove(bag);
  write_static_messages_bag(
    bag, {{"/tf_static",
           0,
           {make_edge("base_link", "lidar", 1.0), make_edge("base_link", "cam", 3.0)}}});
  const auto loaded = load_static_tf_transforms(bag);
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  tf2::BufferCore fed{std::chrono::hours(24 * 365)};
  set_static_transforms(fed, loaded.transforms);
  tf2::BufferCore direct{std::chrono::hours(24 * 365)};
  ASSERT_FALSE(load_static_tf_buffer(bag, direct).has_value());
  EXPECT_EQ(lookup_tx(fed, "lidar"), lookup_tx(direct, "lidar"));
  EXPECT_EQ(lookup_tx(fed, "cam"), lookup_tx(direct, "cam"));
  std::filesystem::remove(bag);
}
