// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/pcd_compress.hpp"

#include "bagwiz/commands/pcd_decompress.hpp"
#include "bagwiz/core/base/tolerance.hpp"
#include "bagwiz/core/pointcloud/compressed_pointcloud2.hpp"
#include "bagwiz/core/pointcloud/draco_codec.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/io/bag_describe.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::commands::PcdCompressArgs;
using bagwiz::commands::PcdDecompressArgs;
using bagwiz::commands::run_pcd_compress;
using bagwiz::commands::run_pcd_decompress;
namespace pc = bagwiz::core::pointcloud;

constexpr std::int64_t kT0Ns = 1'000'000'000;
constexpr std::int64_t kT1Ns = 1'100'000'000;
constexpr std::int64_t kT2Ns = 1'200'000'000;

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions o;
  o.format = bagwiz::io::Format::Mcap;
  o.layout = bagwiz::io::Layout::SingleFile;
  o.mcap_compression = "none";
  return o;
}

bagwiz::io::TopicInfo topic_info(const std::string & name, const std::string & type)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = type;
  t.serialization_format = "cdr";
  return t;
}

bagwiz::io::TopicInfo pcd_topic_info(const std::string & name)
{
  return topic_info(name, "sensor_msgs/msg/PointCloud2");
}

// One dense cloud: x, y, z, intensity (all float32), tightly packed,
// num_points points with coordinates kept within +-10 m so the 14-bit default
// quantization error (half a step, ~0.6 mm over a 20 m span) stays inside
// tolerance::kPointMeters. The pattern differs per `seed` so two clouds in one
// bag are distinguishable byte-wise.
pc::PointCloud2 make_cloud(
  std::int64_t stamp_ns, const std::string & frame_id, std::uint32_t num_points, float seed)
{
  pc::PointCloud2 cloud;
  cloud.timestamp_ns = stamp_ns;
  cloud.frame_id = frame_id;
  cloud.height = 1;
  cloud.width = num_points;
  cloud.fields = {
    {"x", 0, pc::PointFieldType::kFloat32, 1},
    {"y", 4, pc::PointFieldType::kFloat32, 1},
    {"z", 8, pc::PointFieldType::kFloat32, 1},
    {"intensity", 12, pc::PointFieldType::kFloat32, 1},
  };
  cloud.point_step = 16;
  cloud.row_step = 16 * num_points;
  cloud.is_dense = true;
  cloud.data.resize(cloud.row_step);
  for (std::uint32_t i = 0; i < num_points; ++i) {
    const float x = -10.0f + 0.01f * static_cast<float>(i % 2000) + seed;
    const float y = 5.0f - 0.007f * static_cast<float>(i % 1000);
    const float z = 0.5f + 0.003f * static_cast<float>(i % 500);
    const float intensity = static_cast<float>(i % 256);
    std::byte * p = cloud.data.data() + static_cast<std::size_t>(i) * cloud.point_step;
    std::memcpy(p + 0, &x, 4);
    std::memcpy(p + 4, &y, 4);
    std::memcpy(p + 8, &z, 4);
    std::memcpy(p + 12, &intensity, 4);
  }
  return cloud;
}

// A CompressedPointCloud2 message wrapping `cloud`'s Draco encoding (lossless,
// so decompress round-trips bit-exactly). Used to build decompress fixtures
// without going through run_pcd_compress.
std::vector<std::byte> make_compressed_payload(const pc::PointCloud2 & cloud)
{
  pc::DracoEncodeConfig config;
  config.lossless = true;
  const auto encoded = pc::encode_draco(cloud, config);
  EXPECT_TRUE(encoded.ok()) << encoded.error;

  pc::CompressedPointCloud2 message;
  message.timestamp_ns = cloud.timestamp_ns;
  message.frame_id = cloud.frame_id;
  message.height = cloud.height;
  message.width = cloud.width;
  message.fields = cloud.fields;
  message.is_bigendian = cloud.is_bigendian;
  message.point_step = cloud.point_step;
  message.row_step = cloud.row_step;
  message.is_dense = cloud.is_dense;
  message.format = pc::kDracoFormatName;
  message.compressed_data = *encoded.data;
  return pc::serialize_compressed_pointcloud2(message);
}

// The standard fixture bag:
//   /points_a, /points_b  sensor_msgs/msg/PointCloud2, two clouds each
//   /meta                 tf2_msgs/msg/TFMessage, one empty message,
//                         interleaved between the clouds so copy-through
//                         order preservation is non-trivial
void write_compress_input(
  const std::filesystem::path & path, const bagwiz::io::CreateOptions & options = mcap_options())
{
  auto w = bagwiz::io::open_write(path, options);
  w->declare_topic(pcd_topic_info("/points_a"));
  w->declare_topic(pcd_topic_info("/points_b"));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/meta"));

  const auto a0 = pc::serialize_pointcloud2(make_cloud(kT0Ns, "lidar_a", 64, 0.0f));
  const auto b0 = pc::serialize_pointcloud2(make_cloud(kT0Ns, "lidar_b", 32, 1.0f));
  const auto a1 = pc::serialize_pointcloud2(make_cloud(kT1Ns, "lidar_a", 64, 2.0f));
  const auto b1 = pc::serialize_pointcloud2(make_cloud(kT1Ns, "lidar_b", 32, 3.0f));
  const std::vector<geometry_msgs::msg::TransformStamped> no_edges;
  const auto meta = bagwiz::core::serialize_tf_message(no_edges);

  w->write("/points_a", kT0Ns, a0);
  w->write("/meta", kT0Ns, std::span<const std::byte>(meta.data(), meta.size()));
  w->write("/points_b", kT0Ns, b0);
  w->write("/points_a", kT1Ns, a1);
  w->write("/points_b", kT1Ns, b1);
  w->close();
}

// A bag with a CompressedPointCloud2 ("draco") topic /points/draco (two
// clouds) plus a pass-through /meta topic — the shape `pcd compress` writes.
void write_decompress_input(const std::filesystem::path & path)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(topic_info("/points/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/meta"));

  const auto c0 = make_compressed_payload(make_cloud(kT0Ns, "lidar", 64, 0.0f));
  const auto c1 = make_compressed_payload(make_cloud(kT1Ns, "lidar", 64, 1.0f));
  const std::vector<geometry_msgs::msg::TransformStamped> no_edges;
  const auto meta = bagwiz::core::serialize_tf_message(no_edges);

  w->write("/points/draco", kT0Ns, c0);
  w->write("/meta", kT0Ns, std::span<const std::byte>(meta.data(), meta.size()));
  w->write("/points/draco", kT1Ns, c1);
  w->close();
}

// All payloads of one topic, in bag order.
std::vector<std::vector<std::byte>> read_payloads(
  const std::filesystem::path & path, const std::string & topic)
{
  std::vector<std::vector<std::byte>> out;
  auto reader = bagwiz::io::open_read(path);
  bagwiz::io::ReadFilter filter;
  filter.topics = {topic};
  reader->set_filter(filter);
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    out.emplace_back(raw.payload.begin(), raw.payload.end());
  }
  return out;
}

// Every message's (topic, timestamp) in bag order, for order-preservation
// assertions across a rewrite.
std::vector<std::pair<std::string, std::int64_t>> read_sequence(const std::filesystem::path & path)
{
  std::vector<std::pair<std::string, std::int64_t>> out;
  auto reader = bagwiz::io::open_read(path);
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    out.emplace_back(raw.topic->name, raw.timestamp_ns);
  }
  return out;
}

// The bag's topic names and types, in declaration order.
std::vector<std::pair<std::string, std::string>> read_topic_types(
  const std::filesystem::path & path)
{
  std::vector<std::pair<std::string, std::string>> out;
  auto reader = bagwiz::io::open_read(path);
  for (const auto & t : reader->topics()) {
    out.emplace_back(t.name, t.type);
  }
  return out;
}

bool has_topic(
  const std::filesystem::path & path, const std::string & name, const std::string & type)
{
  for (const auto & [n, t] : read_topic_types(path)) {
    if (n == name && t == type) {
      return true;
    }
  }
  return false;
}

PcdCompressArgs compress_args(const std::filesystem::path & in, const std::filesystem::path & out)
{
  PcdCompressArgs a;
  a.input_path = in;
  a.output_path = out;
  a.force = true;  // the tests' out paths are created fresh per run anyway
  return a;
}

PcdDecompressArgs decompress_args(
  const std::filesystem::path & in, const std::filesystem::path & out)
{
  PcdDecompressArgs a;
  a.input_path = in;
  a.output_path = out;
  a.force = true;
  return a;
}

class PcdCompressTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_ = std::filesystem::temp_directory_path() /
           ("bagwiz_pcd_compress_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_);
    std::filesystem::create_directories(tmp_);
    in_ = tmp_ / "in.mcap";
    out_ = tmp_ / "out.mcap";
  }
  void TearDown() override { std::filesystem::remove_all(tmp_); }

  std::filesystem::path tmp_;
  std::filesystem::path in_;
  std::filesystem::path out_;
};

}  // namespace

// The headline workflow: compress every PointCloud2 topic (no -t), then
// decompress the result back. Lossless mode round-trips the point bytes
// bit-exactly — exact agreement is asserted because each cloud is encoded
// independently from its own immutable input bytes (a per-element
// computation), so the round trip is a pure function of the input.
TEST_F(PcdCompressTest, LosslessRoundTripIsBitExact)
{
  write_compress_input(in_);
  const auto meta_before = read_payloads(in_, "/meta");

  auto cargs = compress_args(in_, out_);
  cargs.lossless = true;
  ASSERT_EQ(run_pcd_compress(cargs), 0);

  // The selected topics are replaced by <topic>/draco CompressedPointCloud2
  // topics; /meta passes through byte-identically.
  const auto types = read_topic_types(out_);
  EXPECT_TRUE(
    has_topic(out_, "/points_a/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));
  EXPECT_TRUE(
    has_topic(out_, "/points_b/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));
  EXPECT_TRUE(has_topic(out_, "/meta", "tf2_msgs/msg/TFMessage"));
  for (const auto & [name, type] : types) {
    EXPECT_NE(name, "/points_a");
    EXPECT_NE(name, "/points_b");
  }
  EXPECT_EQ(read_payloads(out_, "/meta"), meta_before);

  // Message order is preserved (modulo the topic rename).
  const std::vector<std::pair<std::string, std::int64_t>> expected{
    {"/points_a/draco", kT0Ns},
    {"/meta", kT0Ns},
    {"/points_b/draco", kT0Ns},
    {"/points_a/draco", kT1Ns},
    {"/points_b/draco", kT1Ns}};
  EXPECT_EQ(read_sequence(out_), expected);

  // Decompressing restores the original topics with bit-exact payloads.
  const auto round = tmp_ / "round.mcap";
  ASSERT_EQ(run_pcd_decompress(decompress_args(out_, round)), 0);
  EXPECT_EQ(read_payloads(round, "/points_a"), read_payloads(in_, "/points_a"));
  EXPECT_EQ(read_payloads(round, "/points_b"), read_payloads(in_, "/points_b"));
  EXPECT_EQ(read_payloads(round, "/meta"), meta_before);
}

// Default (lossy) round trip: fields, point count, header and the non-position
// attributes survive; xyz stays within tolerance::kPointMeters.
TEST_F(PcdCompressTest, LossyRoundTripStaysWithinTolerance)
{
  write_compress_input(in_);
  ASSERT_EQ(run_pcd_compress(compress_args(in_, out_)), 0);
  const auto round = tmp_ / "round.mcap";
  ASSERT_EQ(run_pcd_decompress(decompress_args(out_, round)), 0);

  const auto before = read_payloads(in_, "/points_a");
  const auto after = read_payloads(round, "/points_a");
  ASSERT_EQ(before.size(), after.size());
  for (std::size_t m = 0; m < before.size(); ++m) {
    const auto c0 = pc::parse_pointcloud2(before[m]);
    const auto c1 = pc::parse_pointcloud2(after[m]);
    ASSERT_TRUE(c0.ok());
    ASSERT_TRUE(c1.ok());
    EXPECT_EQ(c1.cloud->timestamp_ns, c0.cloud->timestamp_ns);
    EXPECT_EQ(c1.cloud->frame_id, c0.cloud->frame_id);
    EXPECT_EQ(c1.cloud->width, c0.cloud->width);
    EXPECT_EQ(c1.cloud->point_step, c0.cloud->point_step);
    ASSERT_EQ(c1.cloud->fields.size(), c0.cloud->fields.size());
    for (std::size_t f = 0; f < c0.cloud->fields.size(); ++f) {
      EXPECT_EQ(c1.cloud->fields[f].name, c0.cloud->fields[f].name);
      EXPECT_EQ(c1.cloud->fields[f].offset, c0.cloud->fields[f].offset);
    }
    for (std::uint32_t i = 0; i < c0.cloud->width; ++i) {
      for (const std::uint32_t offset : {0u, 4u, 8u}) {
        float v0 = 0.0f;
        float v1 = 0.0f;
        std::memcpy(&v0, c0.cloud->data.data() + i * c0.cloud->point_step + offset, 4);
        std::memcpy(&v1, c1.cloud->data.data() + i * c1.cloud->point_step + offset, 4);
        EXPECT_NEAR(v1, v0, bagwiz::core::base::tolerance::kPointMeters);
      }
      // intensity (a generic float attribute) is quantized too, but with the
      // 0..255 range here the 14-bit grid is far below one intensity step.
      float i0 = 0.0f;
      float i1 = 0.0f;
      std::memcpy(&i0, c0.cloud->data.data() + i * c0.cloud->point_step + 12, 4);
      std::memcpy(&i1, c1.cloud->data.data() + i * c1.cloud->point_step + 12, 4);
      EXPECT_NEAR(i1, i0, 0.1f);
    }
  }
}

// -t restricts the compression to the named topics; the other PointCloud2
// topic passes through byte-identically.
TEST_F(PcdCompressTest, TopicsFilterLeavesUnselectedTopicsUntouched)
{
  write_compress_input(in_);
  const auto b_before = read_payloads(in_, "/points_b");

  auto a = compress_args(in_, out_);
  a.topics = {"/points_a"};
  ASSERT_EQ(run_pcd_compress(a), 0);

  EXPECT_TRUE(
    has_topic(out_, "/points_a/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));
  EXPECT_TRUE(has_topic(out_, "/points_b", "sensor_msgs/msg/PointCloud2"));
  EXPECT_EQ(read_payloads(out_, "/points_b"), b_before);
}

// --as renames the single selected topic's output; with more than one
// selected topic it is an error.
TEST_F(PcdCompressTest, AsRenamesOutputAndRequiresExactlyOneTopic)
{
  write_compress_input(in_);

  auto a = compress_args(in_, out_);
  a.topics = {"/points_a"};
  a.output_topic = "/compressed";
  ASSERT_EQ(run_pcd_compress(a), 0);
  EXPECT_TRUE(has_topic(out_, "/compressed", "point_cloud_interfaces/msg/CompressedPointCloud2"));

  auto bad = compress_args(in_, tmp_ / "bad.mcap");
  bad.output_topic = "/compressed";
  EXPECT_EQ(run_pcd_compress(bad), 1);  // two topics selected, one --as
}

// Decompress strips the /draco suffix by default; --as overrides it, and a
// selected topic without the suffix (and no --as) is an error naming it.
TEST_F(PcdCompressTest, DecompressOutputNaming)
{
  write_decompress_input(in_);

  ASSERT_EQ(run_pcd_decompress(decompress_args(in_, out_)), 0);
  EXPECT_TRUE(has_topic(out_, "/points", "sensor_msgs/msg/PointCloud2"));
  for (const auto & [name, type] : read_topic_types(out_)) {
    EXPECT_NE(name, "/points/draco");
  }

  auto renamed = tmp_ / "renamed.mcap";
  auto a = decompress_args(in_, renamed);
  a.output_topic = "/raw_points";
  ASSERT_EQ(run_pcd_decompress(a), 0);
  EXPECT_TRUE(has_topic(renamed, "/raw_points", "sensor_msgs/msg/PointCloud2"));

  // A CompressedPointCloud2 topic without the /draco suffix cannot be named
  // by the default rule.
  const auto no_suffix = tmp_ / "no_suffix.mcap";
  {
    auto w = bagwiz::io::open_write(no_suffix, mcap_options());
    w->declare_topic(topic_info("/points", "point_cloud_interfaces/msg/CompressedPointCloud2"));
    const auto c = make_compressed_payload(make_cloud(kT0Ns, "lidar", 8, 0.0f));
    w->write("/points", kT0Ns, c);
    w->close();
  }
  EXPECT_EQ(run_pcd_decompress(decompress_args(no_suffix, tmp_ / "ns_out.mcap")), 1);
}

// An output topic name that already exists in the input bag requires -f;
// with it, the pre-existing topic is replaced.
TEST_F(PcdCompressTest, OutputTopicCollisionRequiresForce)
{
  // The bag already carries a /points/draco topic (an unrelated PointCloud2
  // topic that happens to carry the suffix), so compressing /points collides.
  {
    auto w = bagwiz::io::open_write(in_, mcap_options());
    w->declare_topic(pcd_topic_info("/points"));
    w->declare_topic(pcd_topic_info("/points/draco"));
    const auto p = pc::serialize_pointcloud2(make_cloud(kT0Ns, "lidar", 16, 0.0f));
    const auto stale = pc::serialize_pointcloud2(make_cloud(kT2Ns, "lidar", 4, 9.0f));
    w->write("/points", kT0Ns, p);
    w->write("/points/draco", kT2Ns, stale);
    w->close();
  }

  auto no_force = compress_args(in_, out_);
  no_force.topics = {"/points"};
  no_force.force = false;
  EXPECT_EQ(run_pcd_compress(no_force), 1);
  EXPECT_FALSE(std::filesystem::exists(out_));

  auto forced = compress_args(in_, out_);
  forced.topics = {"/points"};
  ASSERT_EQ(run_pcd_compress(forced), 0);
  EXPECT_TRUE(has_topic(out_, "/points/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));
  // The pre-existing /points/draco messages are dropped, not copied through.
  EXPECT_EQ(read_payloads(out_, "/points/draco").size(), 1u);
}

// A message that cannot be encoded — here a cloud whose bytes do not parse as
// PointCloud2 — is copied through under the ORIGINAL topic (declared on first
// occurrence) with a warning, instead of failing the run.
TEST_F(PcdCompressTest, MalformedMessagePassesThroughUnderOriginalTopic)
{
  {
    auto w = bagwiz::io::open_write(in_, mcap_options());
    w->declare_topic(pcd_topic_info("/points"));
    const auto good = pc::serialize_pointcloud2(make_cloud(kT0Ns, "lidar", 16, 0.0f));
    const std::vector<std::byte> garbage{std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
    w->write("/points", kT0Ns, good);
    w->write("/points", kT1Ns, garbage);
    w->close();
  }
  const auto before = read_payloads(in_, "/points");
  ASSERT_EQ(before.size(), 2u);

  ASSERT_EQ(run_pcd_compress(compress_args(in_, out_)), 0);

  // The good cloud was compressed; the garbage message survives byte-identical
  // under /points.
  EXPECT_TRUE(has_topic(out_, "/points", "sensor_msgs/msg/PointCloud2"));
  EXPECT_TRUE(has_topic(out_, "/points/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));
  ASSERT_EQ(read_payloads(out_, "/points").size(), 1u);
  EXPECT_EQ(read_payloads(out_, "/points").front(), before[1]);
  EXPECT_EQ(read_payloads(out_, "/points/draco").size(), 1u);
}

// An organized cloud with row padding (row_step != width * point_step)
// parses fine but cannot be encoded (the codec reads points contiguously);
// it passes through under the original topic rather than being corrupted.
TEST_F(PcdCompressTest, PaddedOrganizedCloudPassesThrough)
{
  {
    auto w = bagwiz::io::open_write(in_, mcap_options());
    w->declare_topic(pcd_topic_info("/points"));
    auto cloud = make_cloud(kT0Ns, "lidar", 4, 0.0f);
    cloud.height = 2;
    cloud.width = 2;
    cloud.row_step = 2 * cloud.point_step + 8;  // 8 padding bytes per row
    cloud.data.resize(cloud.row_step * 2, std::byte{0});
    const auto p = pc::serialize_pointcloud2(cloud);
    w->write("/points", kT0Ns, p);
    w->close();
  }
  const auto before = read_payloads(in_, "/points");
  ASSERT_EQ(before.size(), 1u);

  ASSERT_EQ(run_pcd_compress(compress_args(in_, out_)), 0);

  EXPECT_TRUE(has_topic(out_, "/points", "sensor_msgs/msg/PointCloud2"));
  EXPECT_EQ(read_payloads(out_, "/points"), before);
  EXPECT_EQ(read_payloads(out_, "/points/draco").size(), 0u);
}

// --as naming the selected input topic itself is rejected: the transform
// replaces that topic, so a same-named output would mix two types under one
// name if any message passed through.
TEST_F(PcdCompressTest, AsSameAsInputTopicIsRejected)
{
  {
    auto w = bagwiz::io::open_write(in_, mcap_options());
    w->declare_topic(pcd_topic_info("/points"));
    const auto p = pc::serialize_pointcloud2(make_cloud(kT0Ns, "lidar", 16, 0.0f));
    w->write("/points", kT0Ns, p);
    w->close();
  }
  auto a = compress_args(in_, out_);
  a.topics = {"/points"};
  a.output_topic = "/points";
  a.force = true;  // even force does not allow it
  EXPECT_EQ(run_pcd_compress(a), 1);
}

// A cloud flagged is_dense=false but holding only finite values IS compressed
// in lossy mode (real drivers/pipelines ship such clouds); only clouds with
// actual NaN/Inf values pass through, and --lossless compresses even those.
TEST_F(PcdCompressTest, NonFiniteCloudPassesThroughUnlessLossless)
{
  auto write_cloud = [this](bool with_nan) {
    auto w = bagwiz::io::open_write(in_, mcap_options());
    w->declare_topic(pcd_topic_info("/points"));
    auto cloud = make_cloud(kT0Ns, "lidar", 16, 0.0f);
    cloud.is_dense = false;
    if (with_nan) {
      const float nan = std::numeric_limits<float>::quiet_NaN();
      std::memcpy(cloud.data.data() + 8, &nan, 4);  // z of point 0
    }
    const auto p = pc::serialize_pointcloud2(cloud);
    w->write("/points", kT0Ns, p);
    w->close();
  };

  // Finite despite the flag: compressed normally.
  write_cloud(false);
  ASSERT_EQ(run_pcd_compress(compress_args(in_, out_)), 0);
  EXPECT_EQ(read_payloads(out_, "/points").size(), 0u);
  EXPECT_EQ(read_payloads(out_, "/points/draco").size(), 1u);

  // Actual NaN: passes through in lossy mode, compresses with --lossless.
  write_cloud(true);
  const auto nan_out = tmp_ / "nan.mcap";
  const auto before = read_payloads(in_, "/points");
  ASSERT_EQ(run_pcd_compress(compress_args(in_, nan_out)), 0);
  EXPECT_TRUE(has_topic(nan_out, "/points", "sensor_msgs/msg/PointCloud2"));
  EXPECT_EQ(read_payloads(nan_out, "/points"), before);
  EXPECT_EQ(read_payloads(nan_out, "/points/draco").size(), 0u);

  const auto lossless_out = tmp_ / "lossless.mcap";
  auto a = compress_args(in_, lossless_out);
  a.lossless = true;
  ASSERT_EQ(run_pcd_compress(a), 0);
  EXPECT_EQ(read_payloads(lossless_out, "/points").size(), 0u);
  EXPECT_EQ(read_payloads(lossless_out, "/points/draco").size(), 1u);
}

// On decompress, a CompressedPointCloud2 message whose format is not "draco"
// passes through under the original topic (byte-identical) with a warning;
// only draco payloads are decoded.
TEST_F(PcdCompressTest, NonDracoFormatPassesThroughOnDecompress)
{
  {
    auto w = bagwiz::io::open_write(in_, mcap_options());
    w->declare_topic(
      topic_info("/points/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));
    const auto draco = make_compressed_payload(make_cloud(kT0Ns, "lidar", 16, 0.0f));
    auto zlib = pc::serialize_compressed_pointcloud2([&] {
      pc::CompressedPointCloud2 m;
      m.timestamp_ns = kT1Ns;
      m.frame_id = "lidar";
      m.format = "zlib";
      m.compressed_data = {std::byte{0x78}, std::byte{0x9C}};
      return m;
    }());
    w->write("/points/draco", kT0Ns, draco);
    w->write("/points/draco", kT1Ns, zlib);
    w->close();
  }
  const auto before = read_payloads(in_, "/points/draco");
  ASSERT_EQ(before.size(), 2u);

  ASSERT_EQ(run_pcd_decompress(decompress_args(in_, out_)), 0);

  // The draco message was decoded to /points; the zlib one stayed on the
  // original topic, byte-identical.
  EXPECT_TRUE(has_topic(out_, "/points/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));
  EXPECT_TRUE(has_topic(out_, "/points", "sensor_msgs/msg/PointCloud2"));
  ASSERT_EQ(read_payloads(out_, "/points/draco").size(), 1u);
  EXPECT_EQ(read_payloads(out_, "/points/draco").front(), before[1]);
  ASSERT_EQ(read_payloads(out_, "/points").size(), 1u);
}

// A decoded point count that contradicts height*width is a hard error: the
// run fails rather than writing a corrupt cloud.
TEST_F(PcdCompressTest, PointCountMismatchIsFatal)
{
  {
    auto w = bagwiz::io::open_write(in_, mcap_options());
    w->declare_topic(
      topic_info("/points/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));
    // The draco stream encodes 16 points but the metadata claims 32.
    pc::CompressedPointCloud2 m;
    const auto cloud = make_cloud(kT0Ns, "lidar", 16, 0.0f);
    pc::DracoEncodeConfig config;
    config.lossless = true;
    const auto encoded = pc::encode_draco(cloud, config);
    ASSERT_TRUE(encoded.ok());
    m.timestamp_ns = kT0Ns;
    m.frame_id = "lidar";
    m.height = 1;
    m.width = 32;  // inconsistent with the encoded 16 points
    m.fields = cloud.fields;
    m.point_step = cloud.point_step;
    m.row_step = cloud.point_step * 32;
    m.is_dense = true;
    m.format = pc::kDracoFormatName;
    m.compressed_data = *encoded.data;
    const auto payload = pc::serialize_compressed_pointcloud2(m);
    w->write("/points/draco", kT0Ns, payload);
    w->close();
  }
  EXPECT_EQ(run_pcd_decompress(decompress_args(in_, out_)), 1);
}

// Thread-count independence: the compressed payloads are identical between
// -j1 and -j4. Exact agreement is asserted because each cloud is encoded
// independently from its own immutable input bytes — a per-element
// computation with no cross-message accumulation — so the work split cannot
// influence the result.
TEST_F(PcdCompressTest, CompressedPayloadsAreThreadCountIndependent)
{
  write_compress_input(in_);
  const auto sync_out = tmp_ / "sync.mcap";
  const auto par_out = tmp_ / "par.mcap";

  auto sync_args = compress_args(in_, sync_out);
  sync_args.threads = 1;
  ASSERT_EQ(run_pcd_compress(sync_args), 0);
  auto par_args = compress_args(in_, par_out);
  par_args.threads = 4;
  ASSERT_EQ(run_pcd_compress(par_args), 0);

  EXPECT_EQ(read_payloads(sync_out, "/points_a/draco"), read_payloads(par_out, "/points_a/draco"));
  EXPECT_EQ(read_payloads(sync_out, "/points_b/draco"), read_payloads(par_out, "/points_b/draco"));
  EXPECT_EQ(read_sequence(sync_out), read_sequence(par_out));

  // Decompression likewise.
  const auto dec_sync = tmp_ / "dec_sync.mcap";
  const auto dec_par = tmp_ / "dec_par.mcap";
  auto d1 = decompress_args(sync_out, dec_sync);
  d1.threads = 1;
  ASSERT_EQ(run_pcd_decompress(d1), 0);
  auto d4 = decompress_args(sync_out, dec_par);
  d4.threads = 4;
  ASSERT_EQ(run_pcd_decompress(d4), 0);
  EXPECT_EQ(read_payloads(dec_sync, "/points_a"), read_payloads(dec_par, "/points_a"));
  EXPECT_EQ(read_sequence(dec_sync), read_sequence(dec_par));
}

// An existing -o path stops the run unless -w/--overwrite is passed (the
// shared -o clobber rule); -f/--force does not cover it.
TEST_F(PcdCompressTest, ExistingOutputPathRequiresOverwrite)
{
  write_compress_input(in_);
  ASSERT_EQ(run_pcd_compress(compress_args(in_, out_)), 0);
  EXPECT_EQ(run_pcd_compress(compress_args(in_, out_)), 1);

  auto a = compress_args(in_, out_);
  a.overwrite = true;
  ASSERT_EQ(run_pcd_compress(a), 0);
  EXPECT_TRUE(
    has_topic(out_, "/points_a/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));
}

// In-place mode (no -o) atomically rewrites the input bag.
// The output bag is shaped like the input: a directory -o output takes the
// input's storage backend and carries its chunk compression over — which is
// unrelated to the Draco payload compression these commands apply. lz4
// rather than zstd, so a run that left the writer's zstd default in place
// would fail here.
TEST_F(PcdCompressTest, OutputInheritsStorageAndCompression)
{
  auto lz4 = mcap_options();
  lz4.mcap_compression = "lz4";
  write_compress_input(in_, lz4);
  const auto compressed = tmp_ / "compressed_dir";

  auto a = compress_args(in_, compressed);
  a.lossless = true;
  ASSERT_EQ(run_pcd_compress(a), 0);
  ASSERT_TRUE(std::filesystem::is_directory(compressed));
  auto d = bagwiz::io::describe_bag(compressed);
  EXPECT_EQ(d.format, bagwiz::io::Format::Mcap);
  EXPECT_EQ(d.compression.codecs, std::vector<std::string>{"lz4"});
  EXPECT_TRUE(
    has_topic(compressed, "/points_a/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));

  const auto decompressed = tmp_ / "decompressed_dir";
  ASSERT_EQ(run_pcd_decompress(decompress_args(compressed, decompressed)), 0);
  d = bagwiz::io::describe_bag(decompressed);
  EXPECT_EQ(d.format, bagwiz::io::Format::Mcap);
  EXPECT_EQ(d.compression.codecs, std::vector<std::string>{"lz4"});
  EXPECT_TRUE(has_topic(decompressed, "/points_a", "sensor_msgs/msg/PointCloud2"));
}

TEST_F(PcdCompressTest, InPlaceRewrite)
{
  write_compress_input(in_);
  const auto meta_before = read_payloads(in_, "/meta");

  PcdCompressArgs a;
  a.input_path = in_;
  a.lossless = true;
  // a.output_path left unset -> in-place.
  ASSERT_EQ(run_pcd_compress(a), 0);

  EXPECT_TRUE(
    has_topic(in_, "/points_a/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));
  EXPECT_EQ(read_payloads(in_, "/meta"), meta_before);
  for (const auto & [name, type] : read_topic_types(in_)) {
    EXPECT_NE(name, "/points_a");
  }
}

// A zero-point cloud round-trips: Draco cannot encode an empty cloud, so the
// empty case travels as an empty compressed_data buffer.
TEST_F(PcdCompressTest, EmptyCloudRoundTrips)
{
  {
    auto w = bagwiz::io::open_write(in_, mcap_options());
    w->declare_topic(pcd_topic_info("/points"));
    const auto p = pc::serialize_pointcloud2(make_cloud(kT0Ns, "lidar", 0, 0.0f));
    w->write("/points", kT0Ns, p);
    w->close();
  }
  auto cargs = compress_args(in_, out_);
  cargs.lossless = true;
  ASSERT_EQ(run_pcd_compress(cargs), 0);
  const auto round = tmp_ / "round.mcap";
  ASSERT_EQ(run_pcd_decompress(decompress_args(out_, round)), 0);
  EXPECT_EQ(read_payloads(round, "/points"), read_payloads(in_, "/points"));
}

// --lossless rejects the quantization-bit flags (there is nothing to
// quantize), and the bit counts are range-checked by the runner as well (the
// CLI also enforces this, but run_pcd_compress is a direct API entry).
TEST_F(PcdCompressTest, QuantizationFlagValidation)
{
  write_compress_input(in_);

  auto both = compress_args(in_, out_);
  both.lossless = true;
  both.position_bits = 12;
  EXPECT_EQ(run_pcd_compress(both), 1);

  auto out_of_range = compress_args(in_, out_);
  out_of_range.position_bits = 0;
  EXPECT_EQ(run_pcd_compress(out_of_range), 1);
  out_of_range.position_bits = 32;
  EXPECT_EQ(run_pcd_compress(out_of_range), 1);

  auto valid = compress_args(in_, out_);
  valid.position_bits = 10;
  valid.color_bits = 8;
  ASSERT_EQ(run_pcd_compress(valid), 0);
  EXPECT_TRUE(
    has_topic(out_, "/points_a/draco", "point_cloud_interfaces/msg/CompressedPointCloud2"));
}

// Selecting a topic of the wrong type is an error, whether on compress
// (a non-PointCloud2 topic) or on decompress (a non-CompressedPointCloud2 one).
TEST_F(PcdCompressTest, WrongTypeSelectionIsFatal)
{
  write_compress_input(in_);
  auto a = compress_args(in_, out_);
  a.topics = {"/meta"};
  EXPECT_EQ(run_pcd_compress(a), 1);

  const auto compressed_in = tmp_ / "compressed_in.mcap";
  write_decompress_input(compressed_in);
  auto d = decompress_args(compressed_in, tmp_ / "d_out.mcap");
  d.topics = {"/meta"};
  EXPECT_EQ(run_pcd_decompress(d), 1);
}
