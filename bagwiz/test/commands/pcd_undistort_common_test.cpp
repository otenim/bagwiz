// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "pcd_undistort_common.hpp"  // NOLINT(build/include_subdir) testing a src-local unit

#include "bagwiz/core/introspection/introspection_loader.hpp"
#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>
#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rmw/types.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

using bagwiz::commands::ExtrinsicMap;
using bagwiz::commands::PcdTopicState;
using bagwiz::commands::PoseComposeKind;
namespace pc = bagwiz::core::pointcloud;

constexpr const char * kLogger = "bagwiz.test.pcd_undistort_common";
constexpr std::chrono::hours kTfBufferCacheTime{24 * 365};
constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kT0Ns = 1000 * kMs;
constexpr std::int64_t kT1Ns = 1100 * kMs;

constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr const char * kOdometryType = "nav_msgs/msg/Odometry";
constexpr const char * kPoseStampedType = "geometry_msgs/msg/PoseStamped";
constexpr const char * kTwistType = "geometry_msgs/msg/Twist";
constexpr const char * kTwistStampedType = "geometry_msgs/msg/TwistStamped";
constexpr const char * kTwistWithCovarianceStampedType =
  "geometry_msgs/msg/TwistWithCovarianceStamped";

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

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child, std::int64_t stamp_ns, double tx)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  ts.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  ts.child_frame_id = child;
  ts.transform.translation.x = tx;
  ts.transform.rotation.w = 1.0;
  return ts;
}

// Typed-message CDR round-trip through the introspection typesupport (the
// cam_info_*_test idiom).
template <typename T>
std::vector<std::byte> serialize_typed(const T & msg, const char * type_name)
{
  auto intro = bagwiz::core::load_introspection(type_name);
  EXPECT_TRUE(intro.ok()) << intro.error;

  rmw_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t alloc = rcutils_get_default_allocator();
  EXPECT_EQ(rmw_serialized_message_init(&serialized, 0, &alloc), RMW_RET_OK);
  EXPECT_EQ(rmw_serialize(&msg, intro.typesupport, &serialized), RMW_RET_OK);
  std::vector<std::byte> out(serialized.buffer_length);
  if (serialized.buffer_length > 0) {
    std::memcpy(out.data(), serialized.buffer, serialized.buffer_length);
  }
  rmw_serialized_message_fini(&serialized);
  return out;
}

// A one-point cloud (all-zero values) with an optional "t" field.
std::vector<std::byte> serialize_cloud(
  std::int64_t stamp_ns, const std::string & frame_id, bool with_time)
{
  pc::PointCloud2 c;
  c.timestamp_ns = stamp_ns;
  c.frame_id = frame_id;
  c.height = 1;
  c.width = 1;
  c.fields = {
    {"x", 0, pc::PointFieldType::kFloat32, 1},
    {"y", 4, pc::PointFieldType::kFloat32, 1},
    {"z", 8, pc::PointFieldType::kFloat32, 1},
  };
  c.point_step = 12;
  if (with_time) {
    c.fields.push_back({"t", 12, pc::PointFieldType::kFloat32, 1});
    c.point_step = 16;
  }
  c.row_step = c.point_step;
  c.is_dense = true;
  c.data.assign(c.point_step, std::byte{0});
  return pc::serialize_pointcloud2(c);
}

void write_payload(
  bagwiz::io::BagWriter & w, const std::string & topic, std::int64_t stamp_ns,
  const std::vector<std::byte> & payload)
{
  w.write(topic, stamp_ns, std::span<const std::byte>(payload.data(), payload.size()));
}

// serialize_tf_message takes a span; wrap the single-/zero-edge cases.
std::vector<std::byte> tf_payload(const geometry_msgs::msg::TransformStamped & edge)
{
  const std::vector<geometry_msgs::msg::TransformStamped> edges{edge};
  return bagwiz::core::serialize_tf_message(edges);
}

std::vector<std::byte> tf_payload_empty()
{
  const std::vector<geometry_msgs::msg::TransformStamped> no_edges;
  return bagwiz::core::serialize_tf_message(no_edges);
}

// /points: frame "lidar_top", per-point time; /raw: frame "base_link", no
// time field; /empty: declared but never written.
void write_peek_bag(const std::filesystem::path & path)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(topic_info("/points", kPointCloud2Type));
  w->declare_topic(topic_info("/raw", kPointCloud2Type));
  w->declare_topic(topic_info("/empty", kPointCloud2Type));
  write_payload(*w, "/points", kT0Ns, serialize_cloud(kT0Ns, "lidar_top", true));
  write_payload(*w, "/raw", kT0Ns, serialize_cloud(kT0Ns, "base_link", false));
  w->close();
}

// /pose_tf: TFMessage with map->base_link at kT0Ns (tx=0) and kT1Ns (tx=1);
// /tf_static present but carrying no edges. Omitting /tf_static exercises the
// static-TF load failure.
void write_tf_message_bag(const std::filesystem::path & path, bool with_static_topic)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/pose_tf"));
  if (with_static_topic) {
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  }
  write_payload(*w, "/pose_tf", kT0Ns, tf_payload(make_edge("map", "base_link", kT0Ns, 0.0)));
  write_payload(*w, "/pose_tf", kT1Ns, tf_payload(make_edge("map", "base_link", kT1Ns, 1.0)));
  if (with_static_topic) {
    write_payload(*w, "/tf_static", 0, tf_payload_empty());
  }
  w->close();
}

// /pose: PoseStamped in frame "odom" at x=5 (kT0Ns); /tf_static carries the
// static map<-odom bridge (tx=1), so composition must yield map x=6.
void write_pose_stamped_bag(const std::filesystem::path & path)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(topic_info("/pose", kPoseStampedType));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.header.stamp.sec = static_cast<std::int32_t>(kT0Ns / 1'000'000'000LL);
  pose.header.stamp.nanosec = static_cast<std::uint32_t>(kT0Ns % 1'000'000'000LL);
  pose.pose.position.x = 5.0;
  pose.pose.orientation.w = 1.0;
  write_payload(*w, "/pose", kT0Ns, serialize_typed(pose, kPoseStampedType));

  write_payload(*w, "/tf_static", 0, tf_payload(make_edge("map", "odom", 0, 1.0)));
  w->close();
}

// /odom: Odometry frame "map" child "base_link" at x=3 (kT0Ns); /tf_static
// present but empty (no bridge needed: header frame is --ref, child is --of).
void write_odometry_bag(const std::filesystem::path & path)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(topic_info("/odom", kOdometryType));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map";
  odom.header.stamp.sec = static_cast<std::int32_t>(kT0Ns / 1'000'000'000LL);
  odom.header.stamp.nanosec = static_cast<std::uint32_t>(kT0Ns % 1'000'000'000LL);
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = 3.0;
  odom.pose.pose.orientation.w = 1.0;
  write_payload(*w, "/odom", kT0Ns, serialize_typed(odom, kOdometryType));

  write_payload(*w, "/tf_static", 0, tf_payload_empty());
  w->close();
}

// make_edge variant with an arbitrary orientation (rotation given as a
// quaternion) and no translation.
geometry_msgs::msg::TransformStamped make_rotation_edge(
  const std::string & parent, const std::string & child, double qz, double qw)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.child_frame_id = child;
  ts.transform.rotation.z = qz;
  ts.transform.rotation.w = qw;
  return ts;
}

geometry_msgs::msg::Twist make_twist(double vx, double wz)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = vx;
  twist.angular.z = wz;
  return twist;
}

void stamp_header(std_msgs::msg::Header & header, const std::string & frame, std::int64_t stamp_ns)
{
  header.frame_id = frame;
  header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
}

// /twist: TwistStamped in `frame` carrying `twist` at every `stamp_ns` in
// `stamps_ns`. /tf_static carries `static_edges` (possibly empty).
void write_twist_stamped_bag(
  const std::filesystem::path & path, const std::string & frame,
  const geometry_msgs::msg::Twist & twist, const std::vector<std::int64_t> & stamps_ns,
  const std::vector<geometry_msgs::msg::TransformStamped> & static_edges)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(topic_info("/twist", kTwistStampedType));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  for (const auto stamp_ns : stamps_ns) {
    geometry_msgs::msg::TwistStamped ts;
    stamp_header(ts.header, frame, stamp_ns);
    ts.twist = twist;
    write_payload(*w, "/twist", stamp_ns, serialize_typed(ts, kTwistStampedType));
  }
  write_payload(*w, "/tf_static", 0, bagwiz::core::serialize_tf_message(static_edges));
  w->close();
}

// /twist: TwistWithCovarianceStamped in `frame` (covariance left zero) at every
// stamp; /tf_static present but empty.
void write_twist_with_covariance_bag(
  const std::filesystem::path & path, const std::string & frame,
  const geometry_msgs::msg::Twist & twist, const std::vector<std::int64_t> & stamps_ns)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(topic_info("/twist", kTwistWithCovarianceStampedType));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  for (const auto stamp_ns : stamps_ns) {
    geometry_msgs::msg::TwistWithCovarianceStamped ts;
    stamp_header(ts.header, frame, stamp_ns);
    ts.twist.twist = twist;
    write_payload(*w, "/twist", stamp_ns, serialize_typed(ts, kTwistWithCovarianceStampedType));
  }
  write_payload(*w, "/tf_static", 0, tf_payload_empty());
  w->close();
}

// /twist: bare Twist (no header) at every stamp; the bag's log time is the
// only stamp the builder can use. /tf_static present but empty.
void write_bare_twist_bag(
  const std::filesystem::path & path, const geometry_msgs::msg::Twist & twist,
  const std::vector<std::int64_t> & stamps_ns)
{
  auto w = bagwiz::io::open_write(path, mcap_options());
  w->declare_topic(topic_info("/twist", kTwistType));
  w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
  for (const auto stamp_ns : stamps_ns) {
    write_payload(*w, "/twist", stamp_ns, serialize_typed(twist, kTwistType));
  }
  write_payload(*w, "/tf_static", 0, tf_payload_empty());
  w->close();
}

class PcdUndistortCommonTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_ = std::filesystem::temp_directory_path() /
           ("bagwiz_pcd_undistort_common_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_);
    std::filesystem::create_directories(tmp_);
    bag_ = tmp_ / "in.mcap";
  }
  void TearDown() override { std::filesystem::remove_all(tmp_); }

  std::filesystem::path tmp_;
  std::filesystem::path bag_;
};

}  // namespace

TEST_F(PcdUndistortCommonTest, SupportedPoseTopicTypes)
{
  EXPECT_TRUE(bagwiz::commands::is_supported_pose_topic_type(kTfMessageType));
  EXPECT_TRUE(bagwiz::commands::is_supported_pose_topic_type(kOdometryType));
  EXPECT_TRUE(bagwiz::commands::is_supported_pose_topic_type(kPoseStampedType));
  EXPECT_TRUE(
    bagwiz::commands::is_supported_pose_topic_type("geometry_msgs/msg/PoseWithCovarianceStamped"));
  EXPECT_FALSE(bagwiz::commands::is_supported_pose_topic_type(kPointCloud2Type));
  EXPECT_FALSE(bagwiz::commands::is_supported_pose_topic_type(""));
}

TEST_F(PcdUndistortCommonTest, PoseComposeKindMapping)
{
  EXPECT_EQ(bagwiz::commands::pose_compose_kind(kOdometryType), PoseComposeKind::kOdometry);
  EXPECT_EQ(bagwiz::commands::pose_compose_kind(kPoseStampedType), PoseComposeKind::kPoseStamped);
  EXPECT_EQ(
    bagwiz::commands::pose_compose_kind("geometry_msgs/msg/PoseWithCovarianceStamped"),
    PoseComposeKind::kPoseWithCovarianceStamped);
  // Callers pre-validate the type, so anything else lands on the covariance kind.
  EXPECT_EQ(
    bagwiz::commands::pose_compose_kind(kTfMessageType),
    PoseComposeKind::kPoseWithCovarianceStamped);
}

TEST_F(PcdUndistortCommonTest, ResolveNumThreadsDefaultsAndClamp)
{
  EXPECT_EQ(bagwiz::commands::resolve_num_threads(0, 8), 8);   // auto (0) -> hardware
  EXPECT_EQ(bagwiz::commands::resolve_num_threads(-1, 8), 8);  // non-positive -> hardware
  EXPECT_EQ(bagwiz::commands::resolve_num_threads(0, 0), 1);   // unknown hardware -> 1
  EXPECT_EQ(bagwiz::commands::resolve_num_threads(2, 8), 2);   // explicit, under the cap
  EXPECT_EQ(bagwiz::commands::resolve_num_threads(16, 8), 8);  // clamped to hardware
  EXPECT_EQ(bagwiz::commands::resolve_num_threads(4, 0), 4);   // unknown hardware: no clamp
}

TEST_F(PcdUndistortCommonTest, CloudHasUsablePointTime)
{
  using bagwiz::core::pointcloud::PointFieldType::kFloat32;
  using bagwiz::core::pointcloud::PointFieldType::kFloat64;
  using bagwiz::core::pointcloud::PointFieldType::kUint32;
  using bagwiz::core::pointcloud::PointFieldType::kUint8;

  const std::vector<pc::PointField> xyz = {
    {"x", 0, kFloat32, 1}, {"y", 4, kFloat32, 1}, {"z", 8, kFloat32, 1}};

  // No per-point time field at all.
  EXPECT_FALSE(bagwiz::commands::cloud_has_usable_point_time(xyz, 12));

  // "t" fits exactly at the end of the point.
  auto fields = xyz;
  fields.push_back({"t", 12, kFloat32, 1});
  EXPECT_TRUE(bagwiz::commands::cloud_has_usable_point_time(fields, 16));
  // ... but is out of bounds when point_step cannot hold it.
  EXPECT_FALSE(bagwiz::commands::cloud_has_usable_point_time(fields, 15));
  EXPECT_FALSE(bagwiz::commands::cloud_has_usable_point_time(fields, 12));

  // The other accepted names and datatypes.
  for (const char * name : {"time", "time_stamp", "timestamp"}) {
    auto named = xyz;
    named.push_back({name, 12, kFloat32, 1});
    EXPECT_TRUE(bagwiz::commands::cloud_has_usable_point_time(named, 16)) << name;
  }
  auto u32 = xyz;
  u32.push_back({"t", 12, kUint32, 1});
  EXPECT_TRUE(bagwiz::commands::cloud_has_usable_point_time(u32, 16));
  auto f64 = xyz;
  f64.push_back({"t", 12, kFloat64, 1});
  EXPECT_TRUE(bagwiz::commands::cloud_has_usable_point_time(f64, 20));
  EXPECT_FALSE(bagwiz::commands::cloud_has_usable_point_time(f64, 19));

  // count != 1 and unsupported datatypes are not usable time fields.
  auto counted = xyz;
  counted.push_back({"t", 12, kFloat32, 2});
  EXPECT_FALSE(bagwiz::commands::cloud_has_usable_point_time(counted, 20));
  auto u8 = xyz;
  u8.push_back({"t", 12, kUint8, 1});
  EXPECT_FALSE(bagwiz::commands::cloud_has_usable_point_time(u8, 16));
}

TEST_F(PcdUndistortCommonTest, ValidateTopicsSuccess)
{
  {
    auto w = bagwiz::io::open_write(bag_, mcap_options());
    w->declare_topic(topic_info("/pose", kTfMessageType));
    w->declare_topic(topic_info("/a", kPointCloud2Type));
    w->declare_topic(topic_info("/b", kPointCloud2Type));
    w->close();
  }
  auto reader = bagwiz::io::open_read(bag_);
  const bagwiz::io::TopicInfo * pose_ti = bagwiz::commands::validate_undistort_topics(
    *reader, "/pose", /*motion_is_twist=*/false, {"/a", "/b"}, bag_, kLogger);
  ASSERT_NE(pose_ti, nullptr);
  EXPECT_EQ(pose_ti->name, "/pose");
  EXPECT_EQ(pose_ti->type, kTfMessageType);
}

TEST_F(PcdUndistortCommonTest, ValidateTopicsRejects)
{
  {
    auto w = bagwiz::io::open_write(bag_, mcap_options());
    w->declare_topic(topic_info("/pose", kTfMessageType));
    w->declare_topic(topic_info("/a", kPointCloud2Type));
    w->close();
  }
  auto reader = bagwiz::io::open_read(bag_);
  // Pose topic missing.
  EXPECT_EQ(
    bagwiz::commands::validate_undistort_topics(
      *reader, "/nope", /*motion_is_twist=*/false, {"/a"}, bag_, kLogger),
    nullptr);
  // Pose topic of an unsupported type.
  EXPECT_EQ(
    bagwiz::commands::validate_undistort_topics(
      *reader, "/a", /*motion_is_twist=*/false, {"/a"}, bag_, kLogger),
    nullptr);
  // --pcd topic missing.
  EXPECT_EQ(
    bagwiz::commands::validate_undistort_topics(
      *reader, "/pose", /*motion_is_twist=*/false, {"/nope"}, bag_, kLogger),
    nullptr);
  // --pcd topic of the wrong type.
  EXPECT_EQ(
    bagwiz::commands::validate_undistort_topics(
      *reader, "/pose", /*motion_is_twist=*/false, {"/pose"}, bag_, kLogger),
    nullptr);
}

TEST_F(PcdUndistortCommonTest, ValidateTopicsTwistGate)
{
  {
    auto w = bagwiz::io::open_write(bag_, mcap_options());
    w->declare_topic(topic_info("/twist", kTwistStampedType));
    w->declare_topic(topic_info("/pose", kPoseStampedType));
    w->declare_topic(topic_info("/a", kPointCloud2Type));
    w->close();
  }
  auto reader = bagwiz::io::open_read(bag_);
  // Twist gate accepts a twist type and rejects a pose type...
  const bagwiz::io::TopicInfo * twist_ti = bagwiz::commands::validate_undistort_topics(
    *reader, "/twist", /*motion_is_twist=*/true, {"/a"}, bag_, kLogger);
  ASSERT_NE(twist_ti, nullptr);
  EXPECT_EQ(twist_ti->type, kTwistStampedType);
  EXPECT_EQ(
    bagwiz::commands::validate_undistort_topics(
      *reader, "/pose", /*motion_is_twist=*/true, {"/a"}, bag_, kLogger),
    nullptr);
  // ... and the pose gate rejects a twist type.
  EXPECT_EQ(
    bagwiz::commands::validate_undistort_topics(
      *reader, "/twist", /*motion_is_twist=*/false, {"/a"}, bag_, kLogger),
    nullptr);
}

TEST_F(PcdUndistortCommonTest, PeekCollectsFirstCloudStates)
{
  write_peek_bag(bag_);
  const auto states =
    bagwiz::commands::peek_pcd_topic_states(bag_, {"/points", "/raw", "/empty"}, kLogger);
  ASSERT_TRUE(states.has_value());
  // /empty never produced a message, so it is simply absent.
  ASSERT_EQ(states->size(), 2u);
  EXPECT_EQ(states->at("/points").frame_id, "lidar_top");
  EXPECT_TRUE(states->at("/points").has_time);
  EXPECT_EQ(states->at("/raw").frame_id, "base_link");
  EXPECT_FALSE(states->at("/raw").has_time);
  EXPECT_EQ(states->count("/empty"), 0u);
  // The peeked span folds header.stamp (kT0Ns) together with the per-point
  // relative time (0s in the fixture), so it covers exactly [kT0Ns, kT0Ns];
  // a topic without a usable time field carries no span.
  const auto & span = states->at("/points").time_span;
  ASSERT_TRUE(span.has_value());
  EXPECT_EQ(span->min_ns, kT0Ns);
  EXPECT_EQ(span->max_ns, kT0Ns);
  EXPECT_FALSE(states->at("/raw").time_span.has_value());
}

TEST_F(PcdUndistortCommonTest, PeekRejectsUndecodableFirstMessage)
{
  {
    auto w = bagwiz::io::open_write(bag_, mcap_options());
    w->declare_topic(topic_info("/bad", kPointCloud2Type));
    write_payload(*w, "/bad", kT0Ns, {std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0}});
    w->close();
  }
  EXPECT_EQ(bagwiz::commands::peek_pcd_topic_states(bag_, {"/bad"}, kLogger), std::nullopt);
}

TEST_F(PcdUndistortCommonTest, PeekRejectsMissingBag)
{
  EXPECT_EQ(
    bagwiz::commands::peek_pcd_topic_states(tmp_ / "nope.mcap", {"/points"}, kLogger),
    std::nullopt);
}

TEST_F(PcdUndistortCommonTest, ValidatePcdTopicStates)
{
  std::unordered_map<std::string, PcdTopicState> states = {
    {"/a", {"lidar", true}}, {"/b", {"base", true}}};
  EXPECT_TRUE(bagwiz::commands::validate_pcd_topic_states({"/a", "/b"}, states, kLogger));
  // /c was never peeked (no decodable message).
  EXPECT_FALSE(bagwiz::commands::validate_pcd_topic_states({"/a", "/b", "/c"}, states, kLogger));
  // A usable per-point time field is required.
  states["/b"].has_time = false;
  EXPECT_FALSE(bagwiz::commands::validate_pcd_topic_states({"/a", "/b"}, states, kLogger));
}

TEST_F(PcdUndistortCommonTest, ResolveExtrinsicsResolvesAndSkipsIdentity)
{
  tf2::BufferCore buffer{kTfBufferCacheTime};
  buffer.setTransform(make_edge("base_link", "lidar", 0, 1.0), "test", true);

  const std::unordered_map<std::string, PcdTopicState> states = {
    {"/points", {"lidar", true}}, {"/self", {"base_link", true}}};
  const auto extrinsics = bagwiz::commands::resolve_pcd_extrinsics(
    buffer, "base_link", {"/points", "/self"}, states, kLogger);
  ASSERT_TRUE(extrinsics.has_value());
  ASSERT_EQ(extrinsics->size(), 2u);
  // Cloud frame already is --of: no transform needed.
  EXPECT_EQ(extrinsics->at("/self"), std::nullopt);
  ASSERT_TRUE(extrinsics->at("/points").has_value());
  EXPECT_NEAR(extrinsics->at("/points")->translation.x, 1.0, 1e-9);
}

TEST_F(PcdUndistortCommonTest, ResolveExtrinsicsFailsOnMissingFrame)
{
  tf2::BufferCore buffer{kTfBufferCacheTime};
  buffer.setTransform(make_edge("base_link", "lidar", 0, 1.0), "test", true);

  const std::unordered_map<std::string, PcdTopicState> states = {{"/points", {"ghost", true}}};
  EXPECT_EQ(
    bagwiz::commands::resolve_pcd_extrinsics(buffer, "base_link", {"/points"}, states, kLogger),
    std::nullopt);
}

TEST_F(PcdUndistortCommonTest, ResolveExtrinsicsFailsOnBrokenChain)
{
  tf2::BufferCore buffer{kTfBufferCacheTime};
  buffer.setTransform(make_edge("base_link", "lidar", 0, 1.0), "test", true);
  buffer.setTransform(make_edge("map", "odom", 0, 0.0), "test", true);

  // Both frames exist in the tree, but no chain connects them.
  const std::unordered_map<std::string, PcdTopicState> states = {{"/points", {"odom", true}}};
  EXPECT_EQ(
    bagwiz::commands::resolve_pcd_extrinsics(buffer, "base_link", {"/points"}, states, kLogger),
    std::nullopt);
}

TEST_F(PcdUndistortCommonTest, TrajectoryFromTfMessageTopic)
{
  write_tf_message_bag(bag_, /*with_static_topic=*/true);
  tf2::BufferCore buffer{kTfBufferCacheTime};
  const auto pose_ti = topic_info("/pose_tf", kTfMessageType);
  const auto built = bagwiz::commands::build_sorted_of_ref_trajectory(
    bag_, pose_ti, "map", "base_link", /*motion_is_twist=*/false, buffer, kLogger);
  ASSERT_TRUE(built.ok()) << built.error;
  ASSERT_EQ(built.trajectory.size(), 2u);
  EXPECT_LT(built.trajectory[0].timestamp_ns, built.trajectory[1].timestamp_ns);
  EXPECT_NEAR(built.trajectory[0].tx, 0.0, 1e-9);
  EXPECT_NEAR(built.trajectory[1].tx, 1.0, 1e-9);
}

TEST_F(PcdUndistortCommonTest, TrajectoryFromPoseStampedUsesStaticBridge)
{
  write_pose_stamped_bag(bag_);
  tf2::BufferCore buffer{kTfBufferCacheTime};
  const auto pose_ti = topic_info("/pose", kPoseStampedType);
  const auto built = bagwiz::commands::build_sorted_of_ref_trajectory(
    bag_, pose_ti, "map", "base_link", /*motion_is_twist=*/false, buffer, kLogger);
  ASSERT_TRUE(built.ok()) << built.error;
  ASSERT_EQ(built.trajectory.size(), 1u);
  // pose x=5 in "odom", bridged through the static map<-odom (tx=1) edge.
  EXPECT_NEAR(built.trajectory[0].tx, 6.0, 1e-6);
}

TEST_F(PcdUndistortCommonTest, TrajectoryFromOdometryTopic)
{
  write_odometry_bag(bag_);
  tf2::BufferCore buffer{kTfBufferCacheTime};
  const auto pose_ti = topic_info("/odom", kOdometryType);
  const auto built = bagwiz::commands::build_sorted_of_ref_trajectory(
    bag_, pose_ti, "map", "base_link", /*motion_is_twist=*/false, buffer, kLogger);
  ASSERT_TRUE(built.ok()) << built.error;
  ASSERT_EQ(built.trajectory.size(), 1u);
  EXPECT_NEAR(built.trajectory[0].tx, 3.0, 1e-6);
}

TEST_F(PcdUndistortCommonTest, TrajectoryFailsWithoutTfPath)
{
  write_tf_message_bag(bag_, /*with_static_topic=*/true);
  tf2::BufferCore buffer{kTfBufferCacheTime};
  const auto pose_ti = topic_info("/pose_tf", kTfMessageType);
  const auto built = bagwiz::commands::build_sorted_of_ref_trajectory(
    bag_, pose_ti, "map", "ghost", /*motion_is_twist=*/false, buffer, kLogger);
  EXPECT_FALSE(built.ok());
  EXPECT_FALSE(built.error.empty());
}

TEST_F(PcdUndistortCommonTest, TrajectoryFailsWithoutStaticTfTopic)
{
  write_tf_message_bag(bag_, /*with_static_topic=*/false);
  tf2::BufferCore buffer{kTfBufferCacheTime};
  const auto pose_ti = topic_info("/pose_tf", kTfMessageType);
  const auto built = bagwiz::commands::build_sorted_of_ref_trajectory(
    bag_, pose_ti, "map", "base_link", /*motion_is_twist=*/false, buffer, kLogger);
  EXPECT_FALSE(built.ok());
  EXPECT_FALSE(built.error.empty());
}

TEST_F(PcdUndistortCommonTest, SupportedTwistTopicTypes)
{
  EXPECT_TRUE(bagwiz::commands::is_supported_twist_topic_type(kTwistType));
  EXPECT_TRUE(bagwiz::commands::is_supported_twist_topic_type(kTwistStampedType));
  EXPECT_TRUE(bagwiz::commands::is_supported_twist_topic_type(kTwistWithCovarianceStampedType));
  EXPECT_FALSE(bagwiz::commands::is_supported_twist_topic_type(kPoseStampedType));
  EXPECT_FALSE(bagwiz::commands::is_supported_twist_topic_type(kOdometryType));
  EXPECT_FALSE(bagwiz::commands::is_supported_twist_topic_type(""));
}

TEST_F(PcdUndistortCommonTest, TwistTrajectoryConstantVelocity)
{
  // v = (1, 0, 0) m/s in --of, three samples 100 ms apart: the trajectory
  // starts at identity and ends 0.2 m ahead with identity rotation.
  write_twist_stamped_bag(
    bag_, "base_link", make_twist(1.0, 0.0), {kT0Ns, kT0Ns + 100 * kMs, kT0Ns + 200 * kMs}, {});
  tf2::BufferCore buffer{kTfBufferCacheTime};
  const auto twist_ti = topic_info("/twist", kTwistStampedType);
  const auto built = bagwiz::commands::build_sorted_of_ref_trajectory(
    bag_, twist_ti, "map", "base_link", /*motion_is_twist=*/true, buffer, kLogger);
  ASSERT_TRUE(built.ok()) << built.error;
  ASSERT_EQ(built.trajectory.size(), 3u);
  EXPECT_EQ(built.trajectory[0].timestamp_ns, kT0Ns);
  EXPECT_NEAR(built.trajectory[0].tx, 0.0, 1e-12);
  EXPECT_NEAR(built.trajectory[0].qw, 1.0, 1e-12);
  EXPECT_NEAR(built.trajectory[1].tx, 0.1, 1e-12);
  EXPECT_NEAR(built.trajectory[2].tx, 0.2, 1e-12);
  EXPECT_NEAR(built.trajectory[2].ty, 0.0, 1e-12);
  EXPECT_NEAR(built.trajectory[2].qw, 1.0, 1e-12);
}

TEST_F(PcdUndistortCommonTest, TwistTrajectoryConstantArcIsExact)
{
  // Constant body-frame twist v = (1, 0, 0), w = (0, 0, 1) over 1 s. Each
  // interval is integrated with the exact constant-twist map, and those exact
  // relative transforms compose to the exact overall arc regardless of the
  // sample partition, so the end pose is (sin 1, 1 - cos 1) with a 1 rad
  // rotation about z. (Exact agreement here is the algorithmically guaranteed
  // kind: a fixed-order, per-element integration over immutable input.)
  std::vector<std::int64_t> stamps;
  for (int k = 0; k <= 10; ++k) {
    stamps.push_back(kT0Ns + k * 100 * kMs);
  }
  write_twist_stamped_bag(bag_, "base_link", make_twist(1.0, 1.0), stamps, {});
  tf2::BufferCore buffer{kTfBufferCacheTime};
  const auto twist_ti = topic_info("/twist", kTwistStampedType);
  const auto built = bagwiz::commands::build_sorted_of_ref_trajectory(
    bag_, twist_ti, "map", "base_link", /*motion_is_twist=*/true, buffer, kLogger);
  ASSERT_TRUE(built.ok()) << built.error;
  ASSERT_EQ(built.trajectory.size(), 11u);
  const auto & end = built.trajectory.back();
  EXPECT_NEAR(end.tx, std::sin(1.0), 1e-9);
  EXPECT_NEAR(end.ty, 1.0 - std::cos(1.0), 1e-9);
  EXPECT_NEAR(end.tz, 0.0, 1e-12);
  EXPECT_NEAR(end.qz, std::sin(0.5), 1e-9);
  EXPECT_NEAR(end.qw, std::cos(0.5), 1e-9);
}

TEST_F(PcdUndistortCommonTest, TwistWithCovarianceTrajectory)
{
  // Same trajectory as the plain TwistStamped case; the covariance is ignored.
  write_twist_with_covariance_bag(
    bag_, "base_link", make_twist(2.0, 0.0), {kT0Ns, kT0Ns + 500 * kMs});
  tf2::BufferCore buffer{kTfBufferCacheTime};
  const auto twist_ti = topic_info("/twist", kTwistWithCovarianceStampedType);
  const auto built = bagwiz::commands::build_sorted_of_ref_trajectory(
    bag_, twist_ti, "map", "base_link", /*motion_is_twist=*/true, buffer, kLogger);
  ASSERT_TRUE(built.ok()) << built.error;
  ASSERT_EQ(built.trajectory.size(), 2u);
  EXPECT_NEAR(built.trajectory.back().tx, 1.0, 1e-12);
}

TEST_F(PcdUndistortCommonTest, BareTwistUsesLogTime)
{
  // No header: sample stamps come from the bag's log time, and the twist is
  // assumed to be in --of already.
  write_bare_twist_bag(bag_, make_twist(1.0, 0.0), {kT0Ns, kT0Ns + 100 * kMs, kT0Ns + 200 * kMs});
  tf2::BufferCore buffer{kTfBufferCacheTime};
  const auto twist_ti = topic_info("/twist", kTwistType);
  const auto built = bagwiz::commands::build_sorted_of_ref_trajectory(
    bag_, twist_ti, "map", "base_link", /*motion_is_twist=*/true, buffer, kLogger);
  ASSERT_TRUE(built.ok()) << built.error;
  ASSERT_EQ(built.trajectory.size(), 3u);
  EXPECT_EQ(built.trajectory[0].timestamp_ns, kT0Ns);
  EXPECT_NEAR(built.trajectory.back().tx, 0.2, 1e-12);
}

TEST_F(PcdUndistortCommonTest, TwistFrameRotatedViaStaticTf)
{
  // Twist expressed in "sensor", 90 deg about z from base_link: v = (1, 0, 0)
  // in sensor must integrate along +y in --of.
  write_twist_stamped_bag(
    bag_, "sensor", make_twist(1.0, 0.0), {kT0Ns, kT0Ns + 200 * kMs},
    {make_rotation_edge("base_link", "sensor", std::sqrt(0.5), std::sqrt(0.5))});
  tf2::BufferCore buffer{kTfBufferCacheTime};
  const auto twist_ti = topic_info("/twist", kTwistStampedType);
  const auto built = bagwiz::commands::build_sorted_of_ref_trajectory(
    bag_, twist_ti, "map", "base_link", /*motion_is_twist=*/true, buffer, kLogger);
  ASSERT_TRUE(built.ok()) << built.error;
  ASSERT_EQ(built.trajectory.size(), 2u);
  EXPECT_NEAR(built.trajectory.back().tx, 0.0, 1e-9);
  EXPECT_NEAR(built.trajectory.back().ty, 0.2, 1e-9);
}

TEST_F(PcdUndistortCommonTest, TwistUnresolvableFrameFails)
{
  write_twist_stamped_bag(bag_, "ghost", make_twist(1.0, 0.0), {kT0Ns, kT1Ns}, {});
  tf2::BufferCore buffer{kTfBufferCacheTime};
  const auto twist_ti = topic_info("/twist", kTwistStampedType);
  const auto built = bagwiz::commands::build_sorted_of_ref_trajectory(
    bag_, twist_ti, "map", "base_link", /*motion_is_twist=*/true, buffer, kLogger);
  EXPECT_FALSE(built.ok());
  EXPECT_FALSE(built.error.empty());
}

TEST_F(PcdUndistortCommonTest, TwistNoSamplesFails)
{
  {
    auto w = bagwiz::io::open_write(bag_, mcap_options());
    w->declare_topic(topic_info("/twist", kTwistStampedType));
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
    write_payload(*w, "/tf_static", 0, tf_payload_empty());
    w->close();
  }
  tf2::BufferCore buffer{kTfBufferCacheTime};
  const auto twist_ti = topic_info("/twist", kTwistStampedType);
  const auto built = bagwiz::commands::build_sorted_of_ref_trajectory(
    bag_, twist_ti, "map", "base_link", /*motion_is_twist=*/true, buffer, kLogger);
  EXPECT_FALSE(built.ok());
  EXPECT_FALSE(built.error.empty());
}
