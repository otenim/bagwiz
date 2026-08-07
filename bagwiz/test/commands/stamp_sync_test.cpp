// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/stamp_sync.hpp"

#include "bagwiz/io/bag_io.hpp"
#include "trim_stamp.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{

constexpr std::int64_t kT0 = 1'000'000'000LL;
constexpr std::int64_t kSecond = 1'000'000'000LL;

bagwiz::io::CreateOptions mcap_dir_opts()
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";
  return opts;
}

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

// Topic whose embedded schema declares a leading std_msgs/Header, so the
// classification works hermetically (no $AMENT_PREFIX_PATH lookup in tests).
bagwiz::io::TopicInfo make_stamped_topic(std::string name)
{
  auto t = make_topic(std::move(name), "sensor_msgs/msg/Imu");
  t.schema_text = "std_msgs/Header header\nfloat64 x\n";
  t.schema_encoding = "ros2msg";
  return t;
}

// Headerless twin: the embedded schema pins the classification so the test
// does not depend on resolving std_msgs from the environment.
bagwiz::io::TopicInfo make_headerless_topic(std::string name)
{
  auto t = make_topic(std::move(name), "std_msgs/msg/String");
  t.schema_text = "string data\n";
  t.schema_encoding = "ros2msg";
  return t;
}

// CDR-encapsulated payload whose leading std_msgs/Header stamp is `stamp_ns`
// (little-endian: 4-byte encapsulation, int32 sec, uint32 nanosec), followed
// by `trailing` extra bytes that a stamp rewrite must leave untouched.
std::vector<std::byte> stamped_payload(std::int64_t stamp_ns, std::size_t trailing = 4)
{
  const auto sec = static_cast<std::uint32_t>(stamp_ns / 1'000'000'000LL);
  const auto nsec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  std::vector<std::byte> buf(12 + trailing, std::byte{0});
  buf[1] = std::byte{0x01};  // little-endian CDR representation id
  for (std::size_t i = 0; i < 4; ++i) {
    buf[4 + i] = static_cast<std::byte>((sec >> (8 * i)) & 0xFFU);
    buf[8 + i] = static_cast<std::byte>((nsec >> (8 * i)) & 0xFFU);
  }
  for (std::size_t i = 0; i < trailing; ++i) {
    buf[12 + i] = static_cast<std::byte>(0xA0U + i);
  }
  return buf;
}

// Big-endian twin of stamped_payload().
std::vector<std::byte> stamped_payload_be(std::int64_t stamp_ns)
{
  const auto sec = static_cast<std::uint32_t>(stamp_ns / 1'000'000'000LL);
  const auto nsec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  std::vector<std::byte> buf(12, std::byte{0});  // representation id 0x0000 = big endian
  for (std::size_t i = 0; i < 4; ++i) {
    buf[4 + i] = static_cast<std::byte>((sec >> (8 * (3 - i))) & 0xFFU);
    buf[8 + i] = static_cast<std::byte>((nsec >> (8 * (3 - i))) & 0xFFU);
  }
  return buf;
}

struct CollectedMessage
{
  std::int64_t timestamp_ns = 0;
  std::vector<std::byte> payload;
};

// Per-topic messages (receive time + owned payload copy) of the bag at `path`.
std::map<std::string, std::vector<CollectedMessage>> collect(const std::filesystem::path & path)
{
  auto reader = bagwiz::io::open_read(path);
  std::map<std::string, std::vector<CollectedMessage>> messages;
  for (const auto & t : reader->topics()) {
    messages[t.name];  // ensure declared topics appear, even at zero messages
  }
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    if (raw.topic != nullptr) {
      messages[raw.topic->name].push_back(
        {raw.timestamp_ns, {raw.payload.begin(), raw.payload.end()}});
    }
  }
  return messages;
}

// Fixture bag: /imu (leading-Header type) with three messages whose header
// stamps deliberately differ from their receive times, and /chatter
// (headerless) with one message. Receive times: /imu at kT0 + {0, 1s, 2s},
// /chatter at kT0 + 0.5s.
std::filesystem::path build_input(const std::filesystem::path & dir)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_stamped_topic("/imu"));
  writer->declare_topic(make_headerless_topic("/chatter"));
  for (int i = 0; i < 3; ++i) {
    const auto payload = stamped_payload(kT0 / 2 + i * kSecond);  // != receive time
    writer->write("/imu", kT0 + i * kSecond, payload);
  }
  const std::vector<std::byte> chatter{std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}};
  writer->write("/chatter", kT0 + kSecond / 2, chatter);
  writer->close();
  return path;
}

class StampSyncTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_stamp_sync_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
       "_" +
       std::to_string(
         reinterpret_cast<std::uintptr_t>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
           this)));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

}  // namespace

TEST(StampSyncHelperTest, WriteLeadingHeaderStampRoundTripLittleEndian)
{
  using bagwiz::commands::read_leading_header_stamp_ns;
  using bagwiz::commands::write_leading_header_stamp_ns;

  auto payload = stamped_payload(kT0 / 2);
  const auto original = payload;
  const std::int64_t target = 1'699'999'999'123'456'789LL;  // sec fits int32

  ASSERT_TRUE(write_leading_header_stamp_ns(payload, target));
  EXPECT_EQ(read_leading_header_stamp_ns(payload), target);
  // Only the stamp bytes change: encapsulation and trailing bytes survive.
  EXPECT_EQ(payload[1], original[1]);
  for (std::size_t i = 12; i < payload.size(); ++i) {
    EXPECT_EQ(payload[i], original[i]);
  }
}

TEST(StampSyncHelperTest, WriteLeadingHeaderStampRoundTripBigEndian)
{
  using bagwiz::commands::read_leading_header_stamp_ns;
  using bagwiz::commands::write_leading_header_stamp_ns;

  auto payload = stamped_payload_be(kT0 / 2);
  const std::int64_t target = 5 * kSecond + 123;

  ASSERT_TRUE(write_leading_header_stamp_ns(payload, target));
  EXPECT_EQ(read_leading_header_stamp_ns(payload), target);
  EXPECT_EQ(payload[1], std::byte{0x00});  // still declared big endian
}

TEST(StampSyncHelperTest, WriteLeadingHeaderStampRejectsShortPayload)
{
  using bagwiz::commands::write_leading_header_stamp_ns;

  std::vector<std::byte> payload(8, std::byte{0});
  payload[1] = std::byte{0x01};
  const auto original = payload;
  EXPECT_FALSE(write_leading_header_stamp_ns(payload, kT0));
  EXPECT_EQ(payload, original);
}

TEST(StampSyncHelperTest, WriteLeadingHeaderStampRejectsUnrepresentableTime)
{
  using bagwiz::commands::write_leading_header_stamp_ns;

  auto payload = stamped_payload(kT0);
  const auto original = payload;
  EXPECT_FALSE(write_leading_header_stamp_ns(payload, -1));
  // One past the largest builtin_interfaces/Time (int32 sec + 999'999'999 ns).
  const std::int64_t over =
    std::int64_t{std::numeric_limits<std::int32_t>::max()} * kSecond + 999'999'999LL + 1;
  EXPECT_FALSE(write_leading_header_stamp_ns(payload, over));
  EXPECT_EQ(payload, original);
}

TEST_F(StampSyncTest, SyncsHeaderedTopicsToOutput)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::StampSyncArgs args;
  args.input_path = in_path;
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_stamp_sync(args), 0);

  const auto out = collect(out_path);
  ASSERT_EQ(out.size(), 2U);  // both topics stay declared

  // Every /imu header.stamp now equals its receive time; receive times and the
  // bytes past the stamp are untouched.
  const auto in = collect(in_path);
  ASSERT_EQ(out.at("/imu").size(), 3U);
  for (std::size_t i = 0; i < 3; ++i) {
    const auto & msg = out.at("/imu")[i];
    EXPECT_EQ(msg.timestamp_ns, kT0 + static_cast<std::int64_t>(i) * kSecond);
    EXPECT_EQ(bagwiz::commands::read_leading_header_stamp_ns(msg.payload), msg.timestamp_ns);
    ASSERT_EQ(msg.payload.size(), in.at("/imu")[i].payload.size());
    for (std::size_t b = 12; b < msg.payload.size(); ++b) {
      EXPECT_EQ(msg.payload[b], in.at("/imu")[i].payload[b]);
    }
  }

  // The headerless topic is byte-identical.
  ASSERT_EQ(out.at("/chatter").size(), 1U);
  EXPECT_EQ(out.at("/chatter")[0].payload, in.at("/chatter")[0].payload);

  // The input bag is untouched in -o mode: header stamps still differ from the
  // receive times.
  EXPECT_EQ(bagwiz::commands::read_leading_header_stamp_ns(in.at("/imu")[0].payload), kT0 / 2);
}

TEST_F(StampSyncTest, InPlaceRewrite)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::StampSyncArgs args;
  args.input_path = in_path;

  ASSERT_EQ(bagwiz::commands::run_stamp_sync(args), 0);

  const auto in = collect(in_path);
  ASSERT_EQ(in.at("/imu").size(), 3U);
  for (const auto & msg : in.at("/imu")) {
    EXPECT_EQ(bagwiz::commands::read_leading_header_stamp_ns(msg.payload), msg.timestamp_ns);
  }
  ASSERT_EQ(in.at("/chatter").size(), 1U);
}

TEST_F(StampSyncTest, ExistingOutputRequiresOverwrite)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";
  {
    std::ofstream stream(out_path);
    stream << "occupied";
  }

  bagwiz::commands::StampSyncArgs args;
  args.input_path = in_path;
  args.output_path = out_path;

  EXPECT_EQ(bagwiz::commands::run_stamp_sync(args), 1);

  args.overwrite = true;
  ASSERT_EQ(bagwiz::commands::run_stamp_sync(args), 0);
  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/imu").size(), 3U);
}

TEST_F(StampSyncTest, NoHeaderedTopicFails)
{
  const auto path = tmp_dir_ / "headerless";
  {
    auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
    writer->declare_topic(make_headerless_topic("/chatter"));
    const std::vector<std::byte> payload{std::byte{0x01}};
    writer->write("/chatter", kT0, payload);
    writer->close();
  }

  bagwiz::commands::StampSyncArgs args;
  args.input_path = path;

  EXPECT_EQ(bagwiz::commands::run_stamp_sync(args), 1);

  // The input is left untouched.
  const auto in = collect(path);
  ASSERT_EQ(in.at("/chatter").size(), 1U);
  EXPECT_EQ(in.at("/chatter")[0].payload, (std::vector<std::byte>{std::byte{0x01}}));
}

TEST_F(StampSyncTest, ShortHeaderedPayloadCopiedVerbatim)
{
  const auto path = tmp_dir_ / "short";
  const std::vector<std::byte> truncated{
    std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}, std::byte{0x42}};
  {
    auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
    writer->declare_topic(make_stamped_topic("/imu"));
    writer->write("/imu", kT0, truncated);
    writer->close();
  }

  bagwiz::commands::StampSyncArgs args;
  args.input_path = path;
  args.output_path = tmp_dir_ / "out";

  ASSERT_EQ(bagwiz::commands::run_stamp_sync(args), 0);

  const auto out = collect(*args.output_path);
  ASSERT_EQ(out.at("/imu").size(), 1U);
  EXPECT_EQ(out.at("/imu")[0].payload, truncated);
}
