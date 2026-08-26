// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/trim.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"
#include "trim_stamp.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{

constexpr std::array<std::uint8_t, 4> kPayload{0x01, 0x02, 0x03, 0x04};

// Bag start time and per-topic message stamps of the fixture built by
// build_input(): /fast at t0 + {0, 1s, 2s, 3s, 4s}, /slow at
// t0 + {0.5s, 2.5s}. Bag extent: [t0, t0 + 4s].
constexpr std::int64_t kT0 = 1'000'000'000LL;
constexpr std::int64_t kSecond = 1'000'000'000LL;

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

std::span<const std::byte> payload_view()
{
  static_assert(sizeof(std::uint8_t) == sizeof(std::byte));
  return std::span<const std::byte>(
    reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      kPayload.data()),
    kPayload.size());
}

bagwiz::io::CreateOptions mcap_dir_opts()
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";
  return opts;
}

bagwiz::io::CreateOptions sqlite3_file_opts()
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Sqlite3;
  opts.layout = bagwiz::io::Layout::SingleFile;
  return opts;
}

void write_fixture_messages(bagwiz::io::BagWriter & writer)
{
  writer.declare_topic(make_topic("/fast", "std_msgs/msg/String"));
  writer.declare_topic(make_topic("/slow", "std_msgs/msg/String"));
  for (int i = 0; i <= 4; ++i) {
    writer.write("/fast", kT0 + i * kSecond, payload_view());
  }
  writer.write("/slow", kT0 + kSecond / 2, payload_view());
  writer.write("/slow", kT0 + 2 * kSecond + kSecond / 2, payload_view());
}

std::filesystem::path build_input(const std::filesystem::path & dir)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  write_fixture_messages(*writer);
  writer->close();
  return path;
}

// Topic whose embedded schema declares a leading std_msgs/Header, so the
// --stamp header classification works hermetically (no $AMENT_PREFIX_PATH
// lookup in tests).
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
// (little-endian: 4-byte encapsulation, int32 sec, uint32 nanosec).
std::vector<std::byte> stamped_payload(std::int64_t stamp_ns)
{
  const auto sec = static_cast<std::uint32_t>(stamp_ns / 1'000'000'000LL);
  const auto nsec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  std::vector<std::byte> buf(12, std::byte{0});
  buf[1] = std::byte{0x01};  // little-endian CDR representation id
  for (std::size_t i = 0; i < 4; ++i) {
    buf[4 + i] = static_cast<std::byte>((sec >> (8 * i)) & 0xFFU);
    buf[8 + i] = static_cast<std::byte>((nsec >> (8 * i)) & 0xFFU);
  }
  return buf;
}

// Per-topic message timestamps of the bag at `path`. Declared topics appear
// even with zero surviving messages.
std::map<std::string, std::vector<std::int64_t>> collect(const std::filesystem::path & path)
{
  auto reader = bagwiz::io::open_read(path);
  std::map<std::string, std::vector<std::int64_t>> stamps;
  for (const auto & t : reader->topics()) {
    stamps[t.name];  // ensure declared topics appear, even at zero messages
  }
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    if (raw.topic != nullptr) {
      stamps[raw.topic->name].push_back(raw.timestamp_ns);
    }
  }
  return stamps;
}

class TrimTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_trim_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
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

TEST_F(TrimTest, StartOnlyToOutput)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "2s";
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  ASSERT_EQ(out.size(), 2U);  // both topics stay declared
  EXPECT_EQ(
    out.at("/fast"),
    (std::vector<std::int64_t>{kT0 + 2 * kSecond, kT0 + 3 * kSecond, kT0 + 4 * kSecond}));
  EXPECT_EQ(out.at("/slow"), (std::vector<std::int64_t>{kT0 + 2 * kSecond + kSecond / 2}));

  // The input bag is untouched in -o mode.
  const auto in = collect(in_path);
  EXPECT_EQ(in.at("/fast").size(), 5U);
  EXPECT_EQ(in.at("/slow").size(), 2U);
}

TEST_F(TrimTest, EndOnlyIsExclusive)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.end = "2s";
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  // The /fast message stamped exactly at t0+2s is excluded ([start, end)).
  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast"), (std::vector<std::int64_t>{kT0, kT0 + kSecond}));
  EXPECT_EQ(out.at("/slow"), (std::vector<std::int64_t>{kT0 + kSecond / 2}));
}

TEST_F(TrimTest, StartAndEnd)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "1s";
  args.end = "3s";
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast"), (std::vector<std::int64_t>{kT0 + kSecond, kT0 + 2 * kSecond}));
  EXPECT_EQ(out.at("/slow"), (std::vector<std::int64_t>{kT0 + 2 * kSecond + kSecond / 2}));
}

TEST_F(TrimTest, DurationEqualsEnd)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "1s";
  args.duration = "2s";  // same window as --start 1s --end 3s
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast"), (std::vector<std::int64_t>{kT0 + kSecond, kT0 + 2 * kSecond}));
  EXPECT_EQ(out.at("/slow"), (std::vector<std::int64_t>{kT0 + 2 * kSecond + kSecond / 2}));
}

TEST_F(TrimTest, BothTrimsBothEnds)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.both = "0.5s";  // window [t0+0.5s, t0+3.5s)
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  // The /slow message stamped exactly at the resolved start (t0+0.5s) is
  // included; the /fast messages at t0 and t0+4s fall outside.
  const auto out = collect(out_path);
  EXPECT_EQ(
    out.at("/fast"),
    (std::vector<std::int64_t>{kT0 + kSecond, kT0 + 2 * kSecond, kT0 + 3 * kSecond}));
  EXPECT_EQ(
    out.at("/slow"),
    (std::vector<std::int64_t>{kT0 + kSecond / 2, kT0 + 2 * kSecond + kSecond / 2}));
}

TEST_F(TrimTest, BothEqualsExplicitStartEnd)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.both = "1s";  // == --start 1s --end 3s on the 4s fixture
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast"), (std::vector<std::int64_t>{kT0 + kSecond, kT0 + 2 * kSecond}));
  EXPECT_EQ(out.at("/slow"), (std::vector<std::int64_t>{kT0 + 2 * kSecond + kSecond / 2}));
}

TEST_F(TrimTest, BothWithOtherWindowFlagFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.both = "1s";
  args.start = "1s";  // the CLI forbids this; the API must too
  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);

  args.start.reset();
  args.duration = "2s";
  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, BothHalfDurationFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.both = "2s";  // 2 * 2s >= the fixture's 4s duration: nothing remains

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, BothZeroFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.both = "0s";  // trims nothing: rejected like a windowless run

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, AlignToSingleTopic)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.align = {"/slow"};  // window [t0+0.5s, t0+2.5s], both bounds inclusive
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  // Unlike --end, the align topic's LAST message (t0+2.5s) is inside the
  // window; /fast keeps only what falls between /slow's first and last.
  const auto out = collect(out_path);
  EXPECT_EQ(
    out.at("/slow"),
    (std::vector<std::int64_t>{kT0 + kSecond / 2, kT0 + 2 * kSecond + kSecond / 2}));
  EXPECT_EQ(out.at("/fast"), (std::vector<std::int64_t>{kT0 + kSecond, kT0 + 2 * kSecond}));
}

TEST_F(TrimTest, AlignToMultipleTopicsUsesCommonSpan)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  // Latest first message: /slow (t0+0.5s); earliest last message: /slow
  // (t0+2.5s) — the common span of the two topics.
  args.align = {"/fast", "/slow"};
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(
    out.at("/slow"),
    (std::vector<std::int64_t>{kT0 + kSecond / 2, kT0 + 2 * kSecond + kSecond / 2}));
  EXPECT_EQ(out.at("/fast"), (std::vector<std::int64_t>{kT0 + kSecond, kT0 + 2 * kSecond}));
}

// run_trim() called directly bypasses the CLI's expansion pass entirely, so
// this does not exercise --align's TopicSlotSpec::require_present (that only
// runs from commands/topic_expand.cpp, reached via expand_topic_selectors()
// in main.cpp — see bagwiz_topic_option_test's
// RequirePresentRejectsAnAbsentLiteral for that layer). resolve_align_window()
// itself has no independent presence check either, but it fails anyway for a
// different reason: a topic with no messages in the filtered scan — absent or
// merely empty, both look the same to the span map — trips the same
// "topic '%s' has no messages" check as AlignTopicWithNoMessagesFails below.
TEST_F(TrimTest, AlignUnmatchedSelectorFails)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.align = {"/does/not/exist"};
  args.output_path = out_path;

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out_path));
}

TEST_F(TrimTest, AlignWithOffsetFlagFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.align = {"/slow"};
  args.start = "1s";  // the CLI forbids this; the API must too
  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);

  args.start.reset();
  args.both = "1s";
  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, AlignTopicWithNoMessagesFails)
{
  const auto in_path = tmp_dir_ / "sparse";
  {
    auto writer = bagwiz::io::open_write(in_path, mcap_dir_opts());
    writer->declare_topic(make_topic("/fast", "std_msgs/msg/String"));
    writer->declare_topic(make_topic("/empty", "std_msgs/msg/String"));
    writer->write("/fast", kT0, payload_view());
    writer->close();
  }

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.align = {"/empty"};  // declared, but carries no message to anchor on

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, AlignDisjointTopicsFail)
{
  const auto in_path = tmp_dir_ / "disjoint";
  {
    auto writer = bagwiz::io::open_write(in_path, mcap_dir_opts());
    writer->declare_topic(make_topic("/a", "std_msgs/msg/String"));
    writer->declare_topic(make_topic("/b", "std_msgs/msg/String"));
    writer->write("/a", kT0 + kSecond, payload_view());
    writer->write("/a", kT0 + 2 * kSecond, payload_view());
    writer->write("/b", kT0 + 3 * kSecond, payload_view());
    writer->write("/b", kT0 + 4 * kSecond, payload_view());
    writer->close();
  }

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.align = {"/a", "/b"};  // no instant where both topics have data

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

// --keep exempts a topic from the window: every one of its messages is copied
// whatever the window resolved to, while every other topic is still trimmed.
// Unlike `topic keep`, it is not a filter — the non-exempt topics stay in the
// output, cut as usual.
TEST_F(TrimTest, KeepTopicSurvivesOutsideWindow)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "2s";
  args.keep = {"/slow"};
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  // /fast is trimmed exactly as StartOnlyToOutput leaves it; /slow is exempt,
  // so its t0+0.5s message survives even though the window starts at t0+2s.
  EXPECT_EQ(
    out.at("/fast"),
    (std::vector<std::int64_t>{kT0 + 2 * kSecond, kT0 + 3 * kSecond, kT0 + 4 * kSecond}));
  EXPECT_EQ(
    out.at("/slow"),
    (std::vector<std::int64_t>{kT0 + kSecond / 2, kT0 + 2 * kSecond + kSecond / 2}));
}

// Under --stamp recv the window normally rides in the reader's ReadFilter, so
// the storage layer never yields out-of-range rows. --keep has to suppress
// that pushdown, or the exempt topic's out-of-window messages would be gone
// before any predicate could spare them.
TEST_F(TrimTest, KeepUnderRecvStampSuppressesTheIndexPushdown)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "2s";
  args.stamp = "recv";
  args.keep = {"/slow"};
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(
    out.at("/fast"),
    (std::vector<std::int64_t>{kT0 + 2 * kSecond, kT0 + 3 * kSecond, kT0 + 4 * kSecond}));
  EXPECT_EQ(
    out.at("/slow"),
    (std::vector<std::int64_t>{kT0 + kSecond / 2, kT0 + 2 * kSecond + kSecond / 2}));
}

// The SQLite backend pushes the window down as a WHERE clause rather than a
// chunk index, so it needs the same suppression as the MCAP path above.
TEST_F(TrimTest, KeepUnderRecvStampSuppressesTheSqlite3Pushdown)
{
  const auto in_path = tmp_dir_ / "input.db3";
  {
    auto writer = bagwiz::io::open_write(in_path, sqlite3_file_opts());
    write_fixture_messages(*writer);
    writer->close();
  }
  const auto out_path = tmp_dir_ / "out.db3";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "2s";
  args.stamp = "recv";
  args.keep = {"/slow"};
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(
    out.at("/fast"),
    (std::vector<std::int64_t>{kT0 + 2 * kSecond, kT0 + 3 * kSecond, kT0 + 4 * kSecond}));
  EXPECT_EQ(
    out.at("/slow"),
    (std::vector<std::int64_t>{kT0 + kSecond / 2, kT0 + 2 * kSecond + kSecond / 2}));
}

// A chunk lying entirely outside the window can still carry messages of an
// exempt topic, so --keep must decline the chunk pass-through and take the
// decoded pipeline — which is what the forced-pipeline reference proves here,
// along with the compression the declined fast path no longer preserves.
TEST_F(TrimTest, KeepDeclinesThePassthroughAndMatchesThePipeline)
{
  const auto in_path = tmp_dir_ / "input_zstd";
  {
    auto opts = mcap_dir_opts();
    opts.mcap_compression = "zstd";
    auto writer = bagwiz::io::open_write(in_path, opts);
    const std::vector<std::byte> big(2048, std::byte{0x42});
    const std::span<const std::byte> big_view(big.data(), big.size());
    writer->declare_topic(make_topic("/fast", "std_msgs/msg/String"));
    writer->declare_topic(make_topic("/slow", "std_msgs/msg/String"));
    for (int i = 0; i <= 4; ++i) {
      writer->write("/fast", kT0 + i * kSecond, big_view);
    }
    writer->write("/slow", kT0 + kSecond / 2, big_view);
    writer->write("/slow", kT0 + 2 * kSecond + kSecond / 2, big_view);
    writer->close();
  }

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "2s";
  args.stamp = "recv";
  args.keep = {"/slow"};

  ::setenv("BAGWIZ_PASSTHROUGH", "off", 1);
  args.output_path = tmp_dir_ / "ref";
  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);
  ::unsetenv("BAGWIZ_PASSTHROUGH");
  args.output_path = tmp_dir_ / "out";
  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  EXPECT_EQ(collect(tmp_dir_ / "ref"), collect(tmp_dir_ / "out"));
  EXPECT_EQ(
    collect(tmp_dir_ / "out").at("/slow"),
    (std::vector<std::int64_t>{kT0 + kSecond / 2, kT0 + 2 * kSecond + kSecond / 2}));

  // With the fast path declined, --keep leaves the same uncompressed output
  // the decoded pipeline always writes — unlike the pass-through, which keeps
  // the input's zstd chunks (PassthroughMatchesPipelineAndPreservesCompression).
  // Either way the metadata's compression pair stays empty: it names rosbag2's
  // own compression layer, not the shard's chunk codec.
  const auto shard_bytes = [this](const std::string & name) {
    return std::filesystem::file_size(tmp_dir_ / name / (name + "_0.mcap"));
  };
  EXPECT_EQ(shard_bytes("out"), shard_bytes("ref"));
  const auto md = bagwiz::io::load_metadata_yaml(tmp_dir_ / "out" / "metadata.yaml");
  EXPECT_TRUE(md.compression_format.empty()) << md.compression_format;
  EXPECT_TRUE(md.compression_mode.empty()) << md.compression_mode;
}

// --keep is not a window input: exempting topics without saying what to cut is
// still the windowless trim the command rejects.
TEST_F(TrimTest, KeepWithoutAWindowFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.keep = {"/slow"};
  args.output_path = tmp_dir_ / "out";

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

// --keep changes which messages survive, never where the window sits: a `msg`
// count still ranks every message in the bag, exempt topics included.
TEST_F(TrimTest, KeepDoesNotShiftAMsgCountWindow)
{
  const auto in_path = build_input(tmp_dir_);

  // Clock order: /fast t0, /slow t0+0.5s, /fast t0+1s, ... — so 2msg resolves
  // the start to t0+1s only while /slow's message is counted. Dropping exempt
  // topics from the ranking would land on t0+2s and cut /fast one further.
  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "2msg";
  args.output_path = tmp_dir_ / "control";
  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  args.keep = {"/slow"};
  args.output_path = tmp_dir_ / "out";
  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto control = collect(tmp_dir_ / "control");
  const auto out = collect(tmp_dir_ / "out");
  EXPECT_EQ(
    control.at("/fast"),
    (std::vector<std::int64_t>{
      kT0 + kSecond, kT0 + 2 * kSecond, kT0 + 3 * kSecond, kT0 + 4 * kSecond}));
  EXPECT_EQ(out.at("/fast"), control.at("/fast"));
  EXPECT_EQ(
    out.at("/slow"),
    (std::vector<std::int64_t>{kT0 + kSecond / 2, kT0 + 2 * kSecond + kSecond / 2}));
}

// --align resolves the window, --keep exempts a topic from it: the two compose
// rather than conflict.
TEST_F(TrimTest, KeepComposesWithAlign)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.align = {"/slow"};  // window [t0+0.5s, t0+2.5s], both bounds inclusive
  args.keep = {"/fast"};
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  // AlignToSingleTopic leaves /fast at {t0+1s, t0+2s}; exempting it keeps all five.
  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast").size(), 5U);
  EXPECT_EQ(
    out.at("/slow"),
    (std::vector<std::int64_t>{kT0 + kSecond / 2, kT0 + 2 * kSecond + kSecond / 2}));
}

TEST_F(TrimTest, EndPastBagEndWarnsAndSucceeds)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "1s";
  args.end = "100s";
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast").size(), 4U);  // everything >= t0+1s
  EXPECT_EQ(out.at("/slow").size(), 1U);
}

TEST_F(TrimTest, StartPastBagEndFails)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "100s";
  args.output_path = out_path;

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out_path));

  // The input is left fully intact.
  const auto in = collect(in_path);
  EXPECT_EQ(in.at("/fast").size(), 5U);
}

TEST_F(TrimTest, StartEqualsEndFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "2s";
  args.end = "2s";

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, ZeroDurationFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "2s";
  args.duration = "0s";

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, NegativeOffsetFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "-1s";

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, BareNumberOffsetFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "5";  // missing unit: rejected, never read as 5 ms

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, UnparseableOffsetFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "abc";

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, EndAndDurationBothFail)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.end = "3s";
  args.duration = "2s";  // the CLI forbids this; the API must too

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, NoBoundsFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;  // no --start/--end/--duration

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, InPlaceRewritesInput)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "2s";
  // No output_path -> in-place.

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto result = collect(in_path);
  EXPECT_EQ(result.at("/fast").size(), 3U);
  EXPECT_EQ(result.at("/slow").size(), 1U);
}

TEST_F(TrimTest, ExistingOutputWithoutOverwriteFails)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";
  std::filesystem::create_directories(out_path);  // pre-existing collision

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "2s";
  args.output_path = out_path;
  args.overwrite = false;

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, OverwriteReplacesExistingOutput)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";
  std::filesystem::create_directories(out_path);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "2s";
  args.output_path = out_path;
  args.overwrite = true;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast").size(), 3U);
}

TEST_F(TrimTest, EmptyWindowSucceedsWithDeclaredTopics)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "3.2s";
  args.end = "3.8s";  // a data gap: no /fast or /slow message in the window
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  ASSERT_EQ(out.size(), 2U);  // topics stay declared
  EXPECT_TRUE(out.at("/fast").empty());
  EXPECT_TRUE(out.at("/slow").empty());
}

TEST_F(TrimTest, EmptyBagFails)
{
  const auto in_path = tmp_dir_ / "empty";
  {
    auto writer = bagwiz::io::open_write(in_path, mcap_dir_opts());
    writer->declare_topic(make_topic("/fast", "std_msgs/msg/String"));
    writer->close();
  }

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "1s";

  // No messages -> no time extent -> relative offsets have no anchor.
  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

// The fixture's 7 messages in clock order: t0 + {0, 0.5, 1, 2, 2.5, 3, 4} s
// (/fast at whole seconds, /slow at 0.5 and 2.5).

TEST_F(TrimTest, StartMsgSkipsFirstMessages)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "2msg";  // skip the 2 earliest messages (t0, t0+0.5s)
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(
    out.at("/fast"), (std::vector<std::int64_t>{
                       kT0 + kSecond, kT0 + 2 * kSecond, kT0 + 3 * kSecond, kT0 + 4 * kSecond}));
  EXPECT_EQ(out.at("/slow"), (std::vector<std::int64_t>{kT0 + 2 * kSecond + kSecond / 2}));
}

TEST_F(TrimTest, EndMsgKeepsFirstMessages)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.end = "3msg";  // keep only the 3 earliest messages
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast"), (std::vector<std::int64_t>{kT0, kT0 + kSecond}));
  EXPECT_EQ(out.at("/slow"), (std::vector<std::int64_t>{kT0 + kSecond / 2}));
}

TEST_F(TrimTest, BothMsgTrimsBothEnds)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.both = "2msg";  // drop the 2 earliest and the 2 latest messages
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast"), (std::vector<std::int64_t>{kT0 + kSecond, kT0 + 2 * kSecond}));
  EXPECT_EQ(out.at("/slow"), (std::vector<std::int64_t>{kT0 + 2 * kSecond + kSecond / 2}));
}

TEST_F(TrimTest, MsgStartMixesWithTimeEnd)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "1msg";  // window starts at the 2nd message (t0+0.5s)
  args.end = "3s";
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast"), (std::vector<std::int64_t>{kT0 + kSecond, kT0 + 2 * kSecond}));
  EXPECT_EQ(
    out.at("/slow"),
    (std::vector<std::int64_t>{kT0 + kSecond / 2, kT0 + 2 * kSecond + kSecond / 2}));
}

TEST_F(TrimTest, MsgStartMixesWithDuration)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "1msg";   // t0+0.5s
  args.duration = "2s";  // window [t0+0.5s, t0+2.5s): the 2.5s msg is out
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast"), (std::vector<std::int64_t>{kT0 + kSecond, kT0 + 2 * kSecond}));
  EXPECT_EQ(out.at("/slow"), (std::vector<std::int64_t>{kT0 + kSecond / 2}));
}

TEST_F(TrimTest, StartMsgPastEndFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "7msg";  // == the fixture's total message count

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, EndMsgPastEndWarnsAndSucceeds)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.end = "100msg";
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast").size(), 5U);
  EXPECT_EQ(out.at("/slow").size(), 2U);
}

TEST_F(TrimTest, MsgCountRejectsFractionalAndNegative)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "1.5msg";
  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);

  args.start = "-2msg";
  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, DurationRejectsMsgUnit)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "1s";
  args.duration = "5msg";  // counts are only valid for --start/--end/--both

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, BothMsgZeroFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.both = "0msg";

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST_F(TrimTest, MsgCountsRankOnHeaderClock)
{
  const auto in_path = tmp_dir_ / "skewed_rank";
  {
    auto writer = bagwiz::io::open_write(in_path, mcap_dir_opts());
    writer->declare_topic(make_stamped_topic("/cam"));
    // Stamps run BACKWARDS relative to receive order: recv t0+{0,1,2}s carry
    // stamps t0+{12,11,10}s. Clock-sorted order is stamps 10, 11, 12.
    for (int i = 0; i <= 2; ++i) {
      const auto payload = stamped_payload(kT0 + (12 - i) * kSecond);
      writer->write("/cam", kT0 + i * kSecond, std::span<const std::byte>(payload));
    }
    writer->close();
  }
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "1msg";  // skip the message with the EARLIEST STAMP (recv t0+2s)
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  // The survivors are the stamp-11s and stamp-12s messages — received at
  // t0+1s and t0+0s. Receive-time ranking would have dropped recv t0 instead.
  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/cam"), (std::vector<std::int64_t>{kT0, kT0 + kSecond}));
}

TEST_F(TrimTest, HeaderStampIsDefaultReference)
{
  const auto in_path = tmp_dir_ / "skewed";
  {
    auto writer = bagwiz::io::open_write(in_path, mcap_dir_opts());
    writer->declare_topic(make_stamped_topic("/cam"));
    writer->declare_topic(make_headerless_topic("/status"));
    // /cam: received at t0+{0,1,2}s but stamped 10 s later — a sensor
    // pipeline latency far larger than life, to make the clocks distinct.
    writer->write("/status", kT0 + kSecond / 5, payload_view());
    for (int i = 0; i <= 2; ++i) {
      const auto payload = stamped_payload(kT0 + (10 + i) * kSecond);
      writer->write("/cam", kT0 + i * kSecond, std::span<const std::byte>(payload));
    }
    writer->write("/status", kT0 + 10 * kSecond + kSecond / 2, payload_view());
    writer->close();
  }
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.align = {"/cam"};  // default --stamp header: window from /cam's STAMPS
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  // Window on the header clock: [t0+10s, t0+12s], both bounds inclusive.
  // Every /cam message is inside it (stamps 10..12 s) even though their
  // receive times (0..2 s) all lie far before the window — the receive-time
  // interpretation would have dropped them all. The headerless /status
  // messages fall back to their receive time: 0.2 s is outside, 10.5 s inside.
  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/cam").size(), 3U);
  EXPECT_EQ(out.at("/status"), (std::vector<std::int64_t>{kT0 + 10 * kSecond + kSecond / 2}));
}

TEST_F(TrimTest, HeaderStampZeroFallsBackToRecv)
{
  const auto in_path = tmp_dir_ / "zerostamp";
  {
    auto writer = bagwiz::io::open_write(in_path, mcap_dir_opts());
    writer->declare_topic(make_stamped_topic("/cam"));
    const auto p10 = stamped_payload(kT0 + 10 * kSecond);
    const auto p0 = stamped_payload(0);  // unset stamp: clock = receive time
    const auto p12 = stamped_payload(kT0 + 12 * kSecond);
    writer->write("/cam", kT0, std::span<const std::byte>(p10));
    writer->write("/cam", kT0 + 11 * kSecond, std::span<const std::byte>(p0));
    writer->write("/cam", kT0 + 2 * kSecond, std::span<const std::byte>(p12));
    writer->close();
  }
  const auto out_path = tmp_dir_ / "out";

  // Clock extent [t0+10s, t0+12s] (the zero-stamp message contributes its
  // receive time t0+11s). Window [t0+10s, t0+11.5s) keeps the stamp-10s
  // message and the zero-stamp one; the stamp-12s message is out.
  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.end = "1.5s";
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/cam"), (std::vector<std::int64_t>{kT0, kT0 + 11 * kSecond}));
}

TEST_F(TrimTest, RecvModeUsesRecordTime)
{
  const auto in_path = tmp_dir_ / "skewed_recv";
  {
    auto writer = bagwiz::io::open_write(in_path, mcap_dir_opts());
    writer->declare_topic(make_stamped_topic("/cam"));
    for (int i = 0; i <= 2; ++i) {
      const auto payload = stamped_payload(kT0 + (10 + i) * kSecond);
      writer->write("/cam", kT0 + i * kSecond, std::span<const std::byte>(payload));
    }
    writer->close();
  }
  const auto out_path = tmp_dir_ / "out";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.stamp = "recv";  // ignore the (skewed) stamps: window on receive times
  args.start = "1s";
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/cam"), (std::vector<std::int64_t>{kT0 + kSecond, kT0 + 2 * kSecond}));
}

TEST_F(TrimTest, InvalidStampValueFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "1s";
  args.stamp = "bogus";  // the CLI's IsMember forbids this; the API must too

  EXPECT_EQ(bagwiz::commands::run_trim(args), 1);
}

TEST(TrimStampTest, SchemaLeadsWithHeader)
{
  using bagwiz::commands::schema_leads_with_header;
  EXPECT_TRUE(schema_leads_with_header("std_msgs/Header header\nuint32 x\n"));
  EXPECT_TRUE(schema_leads_with_header("std_msgs/msg/Header header\n"));
  EXPECT_TRUE(schema_leads_with_header("Header header\n"));
  EXPECT_TRUE(schema_leads_with_header("# comment\n\nstd_msgs/Header header\n"));
  EXPECT_TRUE(schema_leads_with_header("uint8 KIND=1\nstd_msgs/Header header\n"));
  EXPECT_FALSE(schema_leads_with_header(""));
  EXPECT_FALSE(schema_leads_with_header("string data\n"));
  EXPECT_FALSE(schema_leads_with_header("uint32 x\nstd_msgs/Header header\n"));
  EXPECT_FALSE(schema_leads_with_header("std_msgs/Header not_header\n"));
  EXPECT_FALSE(schema_leads_with_header("# only comments\n"));
}

TEST(TrimStampTest, ReadLeadingHeaderStampNs)
{
  using bagwiz::commands::read_leading_header_stamp_ns;
  const auto le = stamped_payload(1'700'000'000'123'456'789LL);
  EXPECT_EQ(
    read_leading_header_stamp_ns(std::span<const std::byte>(le)), 1'700'000'000'123'456'789LL);

  // Big-endian encapsulation: sec = 2, nanosec = 3.
  const std::array<std::uint8_t, 12> be{0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x02, 0x00, 0x00, 0x00, 0x03};
  EXPECT_EQ(read_leading_header_stamp_ns(std::as_bytes(std::span(be))), 2'000'000'003LL);

  const std::array<std::uint8_t, 8> short_buf{};
  EXPECT_EQ(read_leading_header_stamp_ns(std::as_bytes(std::span(short_buf))), std::nullopt);
}

TEST_F(TrimTest, Sqlite3EndExclusive)
{
  const auto in_path = tmp_dir_ / "input.db3";
  {
    auto writer = bagwiz::io::open_write(in_path, sqlite3_file_opts());
    write_fixture_messages(*writer);
    writer->close();
  }
  const auto out_path = tmp_dir_ / "out.db3";

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "1s";
  args.end = "3s";
  // Pin recv mode: this test proves the SQLite backend's pushed-down end_ns
  // is exclusive (header mode would filter via the predicate instead).
  args.stamp = "recv";
  args.output_path = out_path;

  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  // Same window as StartAndEnd: proves the SQLite backend's end_ns is
  // exclusive end-to-end (the /fast message at exactly t0+3s is excluded).
  const auto out = collect(out_path);
  EXPECT_EQ(out.at("/fast"), (std::vector<std::int64_t>{kT0 + kSecond, kT0 + 2 * kSecond}));
  EXPECT_EQ(out.at("/slow"), (std::vector<std::int64_t>{kT0 + 2 * kSecond + kSecond / 2}));
}

// The default path (chunk pass-through) and the decoded pipeline
// (BAGWIZ_PASSTHROUGH=off) must produce the same bag content — and only the
// pass-through preserves the input's chunk compression.
TEST_F(TrimTest, PassthroughMatchesPipelineAndPreservesCompression)
{
  const auto in_path = tmp_dir_ / "input_zstd";
  {
    auto opts = mcap_dir_opts();
    opts.mcap_compression = "zstd";
    auto writer = bagwiz::io::open_write(in_path, opts);
    // Same timestamps as write_fixture_messages, but with large compressible
    // payloads: libmcap silently stores chunks whose payload does not shrink
    // as uncompressed, which would defeat the compression-preservation
    // assertion below.
    const std::vector<std::byte> big(2048, std::byte{0x42});
    const std::span<const std::byte> big_view(big.data(), big.size());
    writer->declare_topic(make_topic("/fast", "std_msgs/msg/String"));
    writer->declare_topic(make_topic("/slow", "std_msgs/msg/String"));
    for (int i = 0; i <= 4; ++i) {
      writer->write("/fast", kT0 + i * kSecond, big_view);
    }
    writer->write("/slow", kT0 + kSecond / 2, big_view);
    writer->write("/slow", kT0 + 2 * kSecond + kSecond / 2, big_view);
    writer->close();
  }

  bagwiz::commands::TrimArgs args;
  args.input_path = in_path;
  args.start = "1s";
  args.end = "3s";
  args.stamp = "recv";

  ::setenv("BAGWIZ_PASSTHROUGH", "off", 1);
  args.output_path = tmp_dir_ / "ref";
  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);
  ::unsetenv("BAGWIZ_PASSTHROUGH");
  args.output_path = tmp_dir_ / "out";
  ASSERT_EQ(bagwiz::commands::run_trim(args), 0);

  EXPECT_EQ(collect(tmp_dir_ / "ref"), collect(tmp_dir_ / "out"));

  // Neither output declares compression in metadata.yaml — those fields name
  // rosbag2's own compression layer, and an mcap shard that fills them stops
  // being readable. The pass-through nonetheless kept the input's zstd chunks,
  // which shows as a smaller shard than the decoded pipeline's uncompressed
  // rewrite of the same messages.
  for (const char * name : {"ref", "out"}) {
    const auto md = bagwiz::io::load_metadata_yaml(tmp_dir_ / name / "metadata.yaml");
    EXPECT_TRUE(md.compression_format.empty()) << name << ": " << md.compression_format;
    EXPECT_TRUE(md.compression_mode.empty()) << name << ": " << md.compression_mode;
  }
  const auto shard_bytes = [this](const std::string & name) {
    return std::filesystem::file_size(tmp_dir_ / name / (name + "_0.mcap"));
  };
  EXPECT_LT(shard_bytes("out"), shard_bytes("ref"));
}

// Exercises the real TrimCommand::configure() — reached through the
// process-wide command registry that its BAGWIZ_REGISTER_COMMAND registrar
// populates — rather than a hand-mirrored copy of its wiring. Deleting
// `.require_present = true` from trim.cpp's --align or --keep declaration
// fails this test directly; it does not rest on a manual CLI run staying
// correct.
TEST(TrimCliWiring, TopicFlagsRequirePresence)
{
  bagwiz::commands::Command * trim_cmd = nullptr;
  for (const auto & cmd : bagwiz::commands::Registry::instance().all()) {
    if (cmd->name() == "trim") {
      trim_cmd = cmd.get();
      break;
    }
  }
  ASSERT_NE(trim_cmd, nullptr);

  CLI::App app{"trim"};
  trim_cmd->configure(app);

  const auto slots = bagwiz::commands::topic_slots_of(app);
  ASSERT_EQ(slots.size(), 2U);  // --align and --keep
  std::map<std::string, bool> require_present;
  for (const auto & slot : slots) {
    ASSERT_NE(slot.option, nullptr);
    require_present[slot.option->get_name()] = slot.spec.require_present;
  }
  EXPECT_TRUE(require_present.at("--align"));
  EXPECT_TRUE(require_present.at("--keep"));
}

}  // namespace
