// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_drop.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::commands::run_tf_static_drop;

constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  return options;
}

bagwiz::io::TopicInfo tf_topic_info(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = kTfMessageType;
  t.serialization_format = "cdr";
  t.schema_encoding = "ros2msg";
  t.schema_text = bagwiz::core::kTfMessageWireSchema;
  return t;
}

// An identity-rotation edge parent -> child carrying `tx` as its translation.x,
// so tests can tell edges apart by a single number.
geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child, double tx)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.child_frame_id = child;
  ts.transform.translation.x = tx;
  ts.transform.rotation.w = 1.0;
  return ts;
}

void write_tf_message(
  bagwiz::io::BagWriter & writer, const std::string & topic, std::int64_t stamp_ns,
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  const auto cdr = bagwiz::core::serialize_tf_message(
    std::span<const geometry_msgs::msg::TransformStamped>(transforms.data(), transforms.size()));
  writer.write(topic, stamp_ns, std::span<const std::byte>(cdr.data(), cdr.size()));
}

// A bag holding one /clock topic (whose earliest message fixes the bag's start
// time at `start_ns`) plus, per entry of `static_topics`, one static TF message
// carrying the given edges. The static message is written first, as a real
// latched publication would be.
void write_bag(
  const std::filesystem::path & path, std::int64_t start_ns,
  const std::vector<std::pair<std::string, std::vector<geometry_msgs::msg::TransformStamped>>> &
    static_topics = {})
{
  bagwiz::io::TopicInfo clock;
  clock.name = "/clock";
  clock.type = "std_msgs/msg/String";
  clock.serialization_format = "cdr";

  constexpr std::array<std::byte, 4> kPayload{
    std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  const auto bytes = std::span<const std::byte>(kPayload.data(), kPayload.size());

  auto writer = bagwiz::io::open_write(path, mcap_options());
  writer->declare_topic(clock);
  for (const auto & [topic, edges] : static_topics) {
    writer->declare_topic(tf_topic_info(topic));
  }
  for (const auto & [topic, edges] : static_topics) {
    write_tf_message(*writer, topic, start_ns, edges);
  }
  writer->write("/clock", start_ns, bytes);
  writer->write("/clock", start_ns + 1'000'000'000LL, bytes);
  writer->close();
}

struct ReadTfResult
{
  bool present = false;
  std::int64_t stamp_ns = 0;
  int message_count = 0;
  // Storage position of the topic's first message among all messages, in the
  // order the reader hands them back.
  int first_index = -1;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
};

ReadTfResult read_tf_topic(const std::filesystem::path & path, const std::string & topic)
{
  ReadTfResult result;
  auto reader = bagwiz::io::open_read(path);
  reader->populate_schemas();

  const bagwiz::io::TopicInfo * info = nullptr;
  for (const auto & t : reader->topics()) {
    if (t.name == topic) {
      info = &t;
      break;
    }
  }
  if (info == nullptr) {
    return result;
  }
  result.present = true;

  auto open = bagwiz::core::decoder::open_decoder(*info);
  EXPECT_TRUE(open.ok()) << open.error;

  bagwiz::io::RawMessage raw;
  int index = 0;
  while (reader->next(raw)) {
    if (raw.topic->name != topic) {
      ++index;
      continue;
    }
    if (result.first_index < 0) {
      result.first_index = index;
    }
    ++index;
    ++result.message_count;
    result.stamp_ns = raw.timestamp_ns;
    const auto decoded = open.decoder->decode(raw.payload);
    EXPECT_TRUE(decoded.ok()) << decoded.error;
    result.transforms = bagwiz::core::extract_tf_message(*decoded.value);
  }
  return result;
}

bool topic_present(const std::filesystem::path & path, const std::string & topic)
{
  auto reader = bagwiz::io::open_read(path);
  for (const auto & t : reader->topics()) {
    if (t.name == topic) {
      return true;
    }
  }
  return false;
}

class TfStaticDropTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_tf_static_drop_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  // base_link -> drs_base_link (x=1) and drs_base_link -> lidar_front (x=2).
  static std::vector<geometry_msgs::msg::TransformStamped> sample_edges()
  {
    return {
      make_edge("base_link", "drs_base_link", 1.0), make_edge("drs_base_link", "lidar_front", 2.0)};
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(TfStaticDropTest, DropsALeaf)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});

  ASSERT_EQ(run_tf_static_drop(bag, {"lidar_front"}, out, /*overwrite=*/false), 0);

  const auto edited = read_tf_topic(out, "/tf_static");
  ASSERT_EQ(edited.transforms.size(), 1U);
  EXPECT_EQ(edited.transforms[0].child_frame_id, "drs_base_link");
}

TEST_F(TfStaticDropTest, DropsASubtree)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  // base_link -> drs_base_link -> {lidar_front, lidar_rear}: dropping
  // drs_base_link removes all three edges.
  const std::vector<geometry_msgs::msg::TransformStamped> edges{
    make_edge("base_link", "drs_base_link", 1.0), make_edge("drs_base_link", "lidar_front", 2.0),
    make_edge("drs_base_link", "lidar_rear", 3.0)};
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", edges}});

  ASSERT_EQ(run_tf_static_drop(bag, {"drs_base_link"}, out, /*overwrite=*/false), 0);

  // The topic survives as a bare declaration with no messages, and the rest of
  // the bag is intact.
  const auto edited = read_tf_topic(out, "/tf_static");
  EXPECT_TRUE(edited.present);
  EXPECT_EQ(edited.message_count, 0);
  EXPECT_TRUE(topic_present(out, "/clock"));
}

TEST_F(TfStaticDropTest, DropsSeveralFramesInOneRun)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  // base_link -> drs_base_link -> {lidar_front, lidar_rear, imu_link}: drop two
  // leaves in one run, leaving imu_link and the parent edge.
  const std::vector<geometry_msgs::msg::TransformStamped> edges{
    make_edge("base_link", "drs_base_link", 1.0), make_edge("drs_base_link", "lidar_front", 2.0),
    make_edge("drs_base_link", "lidar_rear", 3.0), make_edge("drs_base_link", "imu_link", 4.0)};
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", edges}});

  ASSERT_EQ(run_tf_static_drop(bag, {"lidar_front", "lidar_rear"}, out, /*overwrite=*/false), 0);

  const auto edited = read_tf_topic(out, "/tf_static");
  ASSERT_EQ(edited.transforms.size(), 2U);
  EXPECT_EQ(edited.transforms[0].child_frame_id, "drs_base_link");
  EXPECT_EQ(edited.transforms[1].child_frame_id, "imu_link");
}

TEST_F(TfStaticDropTest, DropsAFrameAndOneOfItsDescendantsTogether)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  // Listing both a subtree root and one of its descendants is well defined: the
  // whole subtree goes regardless of order.
  const std::vector<geometry_msgs::msg::TransformStamped> edges{
    make_edge("base_link", "drs_base_link", 1.0), make_edge("drs_base_link", "lidar_front", 2.0)};
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", edges}});

  ASSERT_EQ(run_tf_static_drop(bag, {"drs_base_link", "lidar_front"}, out, /*overwrite=*/false), 0);

  const auto edited = read_tf_topic(out, "/tf_static");
  EXPECT_TRUE(edited.present);
  EXPECT_EQ(edited.message_count, 0);
}

TEST_F(TfStaticDropTest, DropOfAMissingFrameFails)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});

  EXPECT_EQ(run_tf_static_drop(bag, {"oxts_link"}, out, /*overwrite=*/false), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(TfStaticDropTest, DropOfARootFails)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});

  // base_link parents edges but is nobody's child: --frame cannot name it.
  EXPECT_EQ(run_tf_static_drop(bag, {"base_link"}, out, /*overwrite=*/false), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(TfStaticDropTest, DropLandsInTheTopicThatCarriesTheEdge)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  // Two static topics splitting the tree; the dropped edge lives in the second.
  write_bag(
    bag, 1'000'000'000LL,
    {{"/tf_static", {make_edge("base_link", "drs_base_link", 1.0)}},
     {"/sensing/tf_static", {make_edge("drs_base_link", "lidar_front", 2.0)}}});

  ASSERT_EQ(run_tf_static_drop(bag, {"lidar_front"}, out, /*overwrite=*/false), 0);

  // /tf_static is untouched (not rewritten); /sensing/tf_static lost its edge.
  const auto base = read_tf_topic(out, "/tf_static");
  ASSERT_EQ(base.transforms.size(), 1U);
  EXPECT_EQ(base.transforms[0].child_frame_id, "drs_base_link");
  const auto sensing = read_tf_topic(out, "/sensing/tf_static");
  EXPECT_TRUE(sensing.present);
  EXPECT_EQ(sensing.message_count, 0);
}

TEST_F(TfStaticDropTest, NoFrameIsAUsageError)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});

  EXPECT_EQ(run_tf_static_drop(bag, {}, out, /*overwrite=*/false), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

// The rewritten topic's message carries the bag's lowest timestamp, so it must
// also hold the lowest storage position — the same row-order constraint the
// join path documents (Foxglove reads a .db3 in row order).
TEST_F(TfStaticDropTest, RewrittenStaticTfIsEmittedInTimestampOrder)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  constexpr std::int64_t kStart = 5'000'000'000LL;
  const std::vector<geometry_msgs::msg::TransformStamped> edges{
    make_edge("base_link", "drs_base_link", 1.0), make_edge("drs_base_link", "lidar_front", 2.0),
    make_edge("drs_base_link", "lidar_rear", 3.0)};
  write_bag(bag, kStart, {{"/tf_static", edges}});

  ASSERT_EQ(run_tf_static_drop(bag, {"lidar_front"}, out, /*overwrite=*/false), 0);

  const auto edited = read_tf_topic(out, "/tf_static");
  ASSERT_TRUE(edited.present);
  EXPECT_EQ(edited.message_count, 1);
  EXPECT_EQ(edited.stamp_ns, kStart);
  EXPECT_EQ(edited.first_index, 0) << "static TF must be the first message in storage order";
  for (const auto & t : edited.transforms) {
    EXPECT_EQ(t.header.stamp.sec, 5);
    EXPECT_EQ(t.header.stamp.nanosec, 0U);
  }
}

TEST_F(TfStaticDropTest, RewritesTheBagInPlaceWithoutOutput)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});

  ASSERT_EQ(run_tf_static_drop(bag, {"lidar_front"}, std::nullopt, /*overwrite=*/false), 0);

  const auto edited = read_tf_topic(bag, "/tf_static");
  ASSERT_EQ(edited.transforms.size(), 1U);
  EXPECT_EQ(edited.transforms[0].child_frame_id, "drs_base_link");
  EXPECT_TRUE(topic_present(bag, "/clock"));
}

}  // namespace
