// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_edit.hpp"

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
#include <fstream>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::commands::run_tf_static_edit;

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

class TfStaticEditTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_tf_static_edit_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  std::filesystem::path write_yaml(const std::string & contents) const
  {
    const auto path = tmp_dir_ / "edit.yaml";
    std::ofstream(path) << contents;
    return path;
  }

  // base_link -> drs_base_link (x=1) and drs_base_link -> lidar_front (x=2).
  static std::vector<geometry_msgs::msg::TransformStamped> sample_edges()
  {
    return {
      make_edge("base_link", "drs_base_link", 1.0), make_edge("drs_base_link", "lidar_front", 2.0)};
  }

  // One YAML edge: `parent` -> `child` with translation.x = tx.
  static std::string one_edge_yaml(
    const std::string & parent, const std::string & child, const std::string & tx)
  {
    return parent + ":\n  " + child + ":\n    x: " + tx +
           "\n    y: 0.0\n    z: 0.0\n    roll: 0.0\n    pitch: 0.0\n    yaw: 0.0\n";
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(TfStaticEditTest, AddsAnEdgeToABagWithoutStaticTf)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  constexpr std::int64_t kStart = 5'000'000'000LL;
  write_bag(bag, kStart);
  const auto yaml = write_yaml(one_edge_yaml("base_link", "drs_base_link", "1.0"));

  ASSERT_EQ(run_tf_static_edit(bag, yaml, {}, "/tf_static", out, /*overwrite=*/false), 0);

  const auto edited = read_tf_topic(out, "/tf_static");
  ASSERT_TRUE(edited.present);
  EXPECT_EQ(edited.message_count, 1);
  EXPECT_EQ(edited.stamp_ns, kStart);
  ASSERT_EQ(edited.transforms.size(), 1U);
  EXPECT_EQ(edited.transforms[0].header.frame_id, "base_link");
  EXPECT_EQ(edited.transforms[0].child_frame_id, "drs_base_link");
  // The input is untouched in -o mode.
  EXPECT_FALSE(topic_present(bag, "/tf_static"));
}

TEST_F(TfStaticEditTest, AddsANewChildAlongsideExistingEdges)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});
  const auto yaml = write_yaml(one_edge_yaml("drs_base_link", "oxts_link", "3.0"));

  ASSERT_EQ(run_tf_static_edit(bag, yaml, {}, "/tf_static", out, /*overwrite=*/false), 0);

  const auto edited = read_tf_topic(out, "/tf_static");
  ASSERT_EQ(edited.transforms.size(), 3U);
  // The pre-existing edges keep their values, the new child is appended.
  EXPECT_EQ(edited.transforms[0].child_frame_id, "drs_base_link");
  EXPECT_DOUBLE_EQ(edited.transforms[0].transform.translation.x, 1.0);
  EXPECT_EQ(edited.transforms[1].child_frame_id, "lidar_front");
  EXPECT_DOUBLE_EQ(edited.transforms[1].transform.translation.x, 2.0);
  EXPECT_EQ(edited.transforms[2].header.frame_id, "drs_base_link");
  EXPECT_EQ(edited.transforms[2].child_frame_id, "oxts_link");
  EXPECT_DOUBLE_EQ(edited.transforms[2].transform.translation.x, 3.0);
  EXPECT_EQ(edited.message_count, 1);
  EXPECT_TRUE(topic_present(out, "/clock"));
}

TEST_F(TfStaticEditTest, UpdatesAnExistingEdgeInPlace)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});
  // Same parent, same child, new translation: an update, not an addition.
  const auto yaml = write_yaml(one_edge_yaml("drs_base_link", "lidar_front", "9.0"));

  ASSERT_EQ(run_tf_static_edit(bag, yaml, {}, "/tf_static", out, /*overwrite=*/false), 0);

  const auto edited = read_tf_topic(out, "/tf_static");
  ASSERT_EQ(edited.transforms.size(), 2U);
  EXPECT_EQ(edited.transforms[0].child_frame_id, "drs_base_link");
  EXPECT_DOUBLE_EQ(edited.transforms[0].transform.translation.x, 1.0);
  EXPECT_EQ(edited.transforms[1].child_frame_id, "lidar_front");
  EXPECT_DOUBLE_EQ(edited.transforms[1].transform.translation.x, 9.0);
}

TEST_F(TfStaticEditTest, ReparentsAnExistingChild)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});
  // lidar_front moves from drs_base_link to base_link.
  const auto yaml = write_yaml(one_edge_yaml("base_link", "lidar_front", "2.0"));

  ASSERT_EQ(run_tf_static_edit(bag, yaml, {}, "/tf_static", out, /*overwrite=*/false), 0);

  const auto edited = read_tf_topic(out, "/tf_static");
  ASSERT_EQ(edited.transforms.size(), 2U);
  EXPECT_EQ(edited.transforms[1].child_frame_id, "lidar_front");
  EXPECT_EQ(edited.transforms[1].header.frame_id, "base_link");
}

TEST_F(TfStaticEditTest, PrunesALeaf)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});

  ASSERT_EQ(
    run_tf_static_edit(bag, std::nullopt, {"lidar_front"}, "/tf_static", out, /*overwrite=*/false),
    0);

  const auto edited = read_tf_topic(out, "/tf_static");
  ASSERT_EQ(edited.transforms.size(), 1U);
  EXPECT_EQ(edited.transforms[0].child_frame_id, "drs_base_link");
}

TEST_F(TfStaticEditTest, PrunesASubtree)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  // base_link -> drs_base_link -> {lidar_front, lidar_rear}: pruning
  // drs_base_link drops all three edges.
  const std::vector<geometry_msgs::msg::TransformStamped> edges{
    make_edge("base_link", "drs_base_link", 1.0), make_edge("drs_base_link", "lidar_front", 2.0),
    make_edge("drs_base_link", "lidar_rear", 3.0)};
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", edges}});

  ASSERT_EQ(
    run_tf_static_edit(
      bag, std::nullopt, {"drs_base_link"}, "/tf_static", out, /*overwrite=*/false),
    0);

  // The topic survives as a bare declaration with no messages, and the rest of
  // the bag is intact.
  const auto edited = read_tf_topic(out, "/tf_static");
  EXPECT_TRUE(edited.present);
  EXPECT_EQ(edited.message_count, 0);
  EXPECT_TRUE(topic_present(out, "/clock"));
}

TEST_F(TfStaticEditTest, PruneOfAMissingFrameFails)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});

  EXPECT_EQ(
    run_tf_static_edit(bag, std::nullopt, {"oxts_link"}, "/tf_static", out, /*overwrite=*/false),
    1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(TfStaticEditTest, PruneOfARootFails)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});

  // base_link parents edges but is nobody's child: --prune cannot name it.
  EXPECT_EQ(
    run_tf_static_edit(bag, std::nullopt, {"base_link"}, "/tf_static", out, /*overwrite=*/false),
    1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(TfStaticEditTest, PruneThenReaddReplacesTheSubtree)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});
  // Prune drs_base_link (dropping lidar_front with it) and re-add it with a
  // different child in one run.
  const auto yaml = write_yaml(one_edge_yaml("base_link", "drs_base_link", "7.0"));

  ASSERT_EQ(
    run_tf_static_edit(bag, yaml, {"drs_base_link"}, "/tf_static", out, /*overwrite=*/false), 0);

  const auto edited = read_tf_topic(out, "/tf_static");
  ASSERT_EQ(edited.transforms.size(), 1U);
  EXPECT_EQ(edited.transforms[0].child_frame_id, "drs_base_link");
  EXPECT_DOUBLE_EQ(edited.transforms[0].transform.translation.x, 7.0);
}

TEST_F(TfStaticEditTest, EditsLandInTheTopicThatCarriesTheEdge)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  // Two static topics splitting the tree; the update targets an edge living in
  // /sensing/tf_static, and the new child goes to the default -t.
  write_bag(
    bag, 1'000'000'000LL,
    {{"/tf_static", {make_edge("base_link", "drs_base_link", 1.0)}},
     {"/sensing/tf_static", {make_edge("drs_base_link", "lidar_front", 2.0)}}});
  const auto yaml = write_yaml(
    one_edge_yaml("drs_base_link", "lidar_front", "9.0") + "\n" +
    one_edge_yaml("drs_base_link", "oxts_link", "3.0"));

  ASSERT_EQ(run_tf_static_edit(bag, yaml, {}, "/tf_static", out, /*overwrite=*/false), 0);

  // /tf_static: its own edge untouched, plus the appended new child.
  const auto base = read_tf_topic(out, "/tf_static");
  ASSERT_EQ(base.transforms.size(), 2U);
  EXPECT_EQ(base.transforms[0].child_frame_id, "drs_base_link");
  EXPECT_DOUBLE_EQ(base.transforms[0].transform.translation.x, 1.0);
  EXPECT_EQ(base.transforms[1].child_frame_id, "oxts_link");
  // /sensing/tf_static: the update landed in the edge's home topic.
  const auto sensing = read_tf_topic(out, "/sensing/tf_static");
  ASSERT_EQ(sensing.transforms.size(), 1U);
  EXPECT_EQ(sensing.transforms[0].child_frame_id, "lidar_front");
  EXPECT_DOUBLE_EQ(sensing.transforms[0].transform.translation.x, 9.0);
  // Both rewritten topics carry exactly one message.
  EXPECT_EQ(base.message_count, 1);
  EXPECT_EQ(sensing.message_count, 1);
}

TEST_F(TfStaticEditTest, AnEditClosingACycleFails)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});
  // Re-parenting drs_base_link under its own descendant lidar_front closes a
  // cycle against the edges the bag already carries.
  const auto yaml = write_yaml(one_edge_yaml("lidar_front", "drs_base_link", "1.0"));

  EXPECT_EQ(run_tf_static_edit(bag, yaml, {}, "/tf_static", out, /*overwrite=*/false), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

// The rewritten topic's message carries the bag's lowest timestamp, so it must
// also hold the lowest storage position — the same row-order constraint the
// join path documents (Foxglove reads a .db3 in row order).
TEST_F(TfStaticEditTest, RewrittenStaticTfIsEmittedInTimestampOrder)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  constexpr std::int64_t kStart = 5'000'000'000LL;
  write_bag(bag, kStart, {{"/tf_static", sample_edges()}});
  const auto yaml = write_yaml(one_edge_yaml("drs_base_link", "oxts_link", "3.0"));

  ASSERT_EQ(run_tf_static_edit(bag, yaml, {}, "/tf_static", out, /*overwrite=*/false), 0);

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

TEST_F(TfStaticEditTest, RewritesTheBagInPlaceWithoutOutput)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});
  const auto yaml = write_yaml(one_edge_yaml("drs_base_link", "oxts_link", "3.0"));

  ASSERT_EQ(run_tf_static_edit(bag, yaml, {}, "/tf_static", std::nullopt, /*overwrite=*/false), 0);

  const auto edited = read_tf_topic(bag, "/tf_static");
  ASSERT_EQ(edited.transforms.size(), 3U);
  EXPECT_TRUE(topic_present(bag, "/clock"));
}

TEST_F(TfStaticEditTest, HonoursACustomTopicForNewChildren)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});
  const auto yaml = write_yaml(one_edge_yaml("drs_base_link", "oxts_link", "3.0"));

  ASSERT_EQ(run_tf_static_edit(bag, yaml, {}, "/oxts/tf_static", out, /*overwrite=*/false), 0);

  // The new child is homed under -t, declared fresh; the existing topic is
  // untouched.
  const auto oxts = read_tf_topic(out, "/oxts/tf_static");
  ASSERT_TRUE(oxts.present);
  ASSERT_EQ(oxts.transforms.size(), 1U);
  EXPECT_EQ(oxts.transforms[0].child_frame_id, "oxts_link");
  const auto base = read_tf_topic(out, "/tf_static");
  ASSERT_EQ(base.transforms.size(), 2U);
}

TEST_F(TfStaticEditTest, NeitherYamlNorPruneIsAUsageError)
{
  const auto bag = tmp_dir_ / "bag.mcap";
  const auto out = tmp_dir_ / "out.mcap";
  write_bag(bag, 1'000'000'000LL, {{"/tf_static", sample_edges()}});

  EXPECT_EQ(run_tf_static_edit(bag, std::nullopt, {}, "/tf_static", out, /*overwrite=*/false), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

}  // namespace
