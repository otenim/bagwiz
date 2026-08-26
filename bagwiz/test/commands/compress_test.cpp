// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/compress.hpp"

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{

// The payload is large and repetitive on purpose: MCAP chunk compression is
// storage-internal and deliberately absent from metadata.yaml, so the tests
// below tell a compressed shard from a plain one by its size. That only reads
// as a signal when the messages are worth compressing. Which codec landed on
// the chunks is asserted one layer down, on CreateOptions, by
// bagwiz_bag's rewrite_test (McapChunkCompressions*).
constexpr std::size_t kPayloadBytes = 4096;
constexpr std::int64_t kT0 = 1'000'000'000LL;
constexpr std::int64_t kSecond = 1'000'000'000LL;

const std::vector<std::byte> & payload_bytes()
{
  static const std::vector<std::byte> bytes = [] {
    std::vector<std::byte> v(kPayloadBytes);
    for (std::size_t i = 0; i < v.size(); ++i) {
      v[i] = static_cast<std::byte>(i % 8);
    }
    return v;
  }();
  return bytes;
}

// Size of the single shard inside a directory-layout MCAP bag.
std::uintmax_t mcap_shard_bytes(const std::filesystem::path & dir)
{
  return std::filesystem::file_size(dir / (dir.filename().string() + "_0.mcap"));
}

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
  return payload_bytes();
}

// Uncompressed MCAP directory bag: /fast with 5 messages, /slow with 2.
std::filesystem::path build_input(const std::filesystem::path & dir)
{
  const auto path = dir / "input";
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::Directory;
  options.mcap_compression = "none";
  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(make_topic("/fast", "std_msgs/msg/String"));
  writer->declare_topic(make_topic("/slow", "std_msgs/msg/String"));
  for (int i = 0; i <= 4; ++i) {
    writer->write("/fast", kT0 + i * kSecond, payload_view());
  }
  writer->write("/slow", kT0 + kSecond / 2, payload_view());
  writer->write("/slow", kT0 + 2 * kSecond + kSecond / 2, payload_view());
  writer->close();
  return path;
}

// Per-topic message counts of the bag at `path`, asserting every payload
// round-trips byte-for-byte.
std::map<std::string, int> collect_counts(const std::filesystem::path & path)
{
  auto reader = bagwiz::io::open_read(path);
  std::map<std::string, int> counts;
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    EXPECT_EQ(raw.payload.size(), kPayloadBytes);
    ++counts[raw.topic->name];
  }
  return counts;
}

void expect_fixture_intact(const std::filesystem::path & path)
{
  const auto counts = collect_counts(path);
  ASSERT_EQ(counts.size(), 2U);
  EXPECT_EQ(counts.at("/fast"), 5);
  EXPECT_EQ(counts.at("/slow"), 2);
}

bagwiz::commands::CompressArgs make_args(
  const std::filesystem::path & input, const std::filesystem::path & output)
{
  bagwiz::commands::CompressArgs args;
  args.input_path = input;
  args.output_path = output;
  return args;
}

class CompressTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_compress_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
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

TEST_F(CompressTest, McapAutoCompressesChunks)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  auto args = make_args(in_path, out_path);
  ASSERT_EQ(bagwiz::commands::run_compress(args), 0);

  // auto on an MCAP target resolves to chunk compression. That compression is
  // storage-internal, so metadata.yaml's compression pair — which names
  // rosbag2's own compression layer — stays empty: filling it makes rosbag2
  // expand the shard as a whole-file zstd envelope and fail to open the bag.
  const auto md = bagwiz::io::load_metadata_yaml(out_path / "metadata.yaml");
  EXPECT_EQ(md.storage_identifier, "mcap");
  EXPECT_TRUE(md.compression_format.empty()) << md.compression_format;
  EXPECT_TRUE(md.compression_mode.empty()) << md.compression_mode;

  // The chunks really were compressed: the fixture holds the same messages
  // written plain, and is much larger.
  EXPECT_LT(mcap_shard_bytes(out_path), mcap_shard_bytes(in_path));

  expect_fixture_intact(out_path);
  expect_fixture_intact(in_path);  // the input is never rewritten
}

TEST_F(CompressTest, McapLz4ChunkCompression)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  auto args = make_args(in_path, out_path);
  args.codec = "lz4";
  ASSERT_EQ(bagwiz::commands::run_compress(args), 0);

  // Same contract as the zstd case: the codec reaches the chunks, not
  // metadata.yaml.
  const auto md = bagwiz::io::load_metadata_yaml(out_path / "metadata.yaml");
  EXPECT_TRUE(md.compression_format.empty()) << md.compression_format;
  EXPECT_TRUE(md.compression_mode.empty()) << md.compression_mode;
  EXPECT_LT(mcap_shard_bytes(out_path), mcap_shard_bytes(in_path));
  expect_fixture_intact(out_path);
}

TEST_F(CompressTest, McapDecompressWithModeNone)
{
  const auto in_path = build_input(tmp_dir_);
  const auto compressed = tmp_dir_ / "compressed";
  const auto plain = tmp_dir_ / "plain";

  auto compress_args = make_args(in_path, compressed);
  ASSERT_EQ(bagwiz::commands::run_compress(compress_args), 0);

  auto decompress_args = make_args(compressed, plain);
  decompress_args.mode = "none";
  ASSERT_EQ(bagwiz::commands::run_compress(decompress_args), 0);

  const auto md = bagwiz::io::load_metadata_yaml(plain / "metadata.yaml");
  EXPECT_TRUE(md.compression_format.empty() || md.compression_format == "none");
  EXPECT_TRUE(md.compression_mode.empty());
  // --mode none really unpacked the chunks rather than copying them across.
  EXPECT_GT(mcap_shard_bytes(plain), mcap_shard_bytes(compressed));
  expect_fixture_intact(plain);
}

TEST_F(CompressTest, McapRejectsMessageMode)
{
  const auto in_path = build_input(tmp_dir_);
  auto args = make_args(in_path, tmp_dir_ / "out");
  args.mode = "message";
  EXPECT_EQ(bagwiz::commands::run_compress(args), 1);
}

TEST_F(CompressTest, Sqlite3AutoCompressesPerMessage)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  auto args = make_args(in_path, out_path);
  args.storage = "sqlite3";
  ASSERT_EQ(bagwiz::commands::run_compress(args), 0);

  // auto on a sqlite3 target resolves to MESSAGE-mode per-message zstd.
  const auto md = bagwiz::io::load_metadata_yaml(out_path / "metadata.yaml");
  EXPECT_EQ(md.storage_identifier, "sqlite3");
  EXPECT_EQ(md.compression_format, "zstd");
  EXPECT_EQ(md.compression_mode, "message");
  expect_fixture_intact(out_path);
}

TEST_F(CompressTest, Sqlite3FileModeWritesZstdEnvelope)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  auto args = make_args(in_path, out_path);
  args.storage = "sqlite3";
  args.mode = "file";
  ASSERT_EQ(bagwiz::commands::run_compress(args), 0);

  const auto md = bagwiz::io::load_metadata_yaml(out_path / "metadata.yaml");
  EXPECT_EQ(md.compression_format, "zstd");
  EXPECT_EQ(md.compression_mode, "file");
  ASSERT_EQ(md.relative_file_paths.size(), 1U);
  EXPECT_EQ(md.relative_file_paths[0], out_path.filename().string() + "_0.db3.zstd");
  EXPECT_TRUE(std::filesystem::exists(out_path / md.relative_file_paths[0]));
  expect_fixture_intact(out_path);
}

TEST_F(CompressTest, Sqlite3MessageModeDecompressesWithModeNone)
{
  const auto in_path = build_input(tmp_dir_);
  const auto compressed = tmp_dir_ / "compressed";
  const auto plain = tmp_dir_ / "plain";

  auto compress_args = make_args(in_path, compressed);
  compress_args.storage = "sqlite3";
  ASSERT_EQ(bagwiz::commands::run_compress(compress_args), 0);

  auto decompress_args = make_args(compressed, plain);
  decompress_args.storage = "sqlite3";
  decompress_args.mode = "none";
  ASSERT_EQ(bagwiz::commands::run_compress(decompress_args), 0);

  const auto md = bagwiz::io::load_metadata_yaml(plain / "metadata.yaml");
  EXPECT_TRUE(md.compression_mode.empty());
  expect_fixture_intact(plain);
}

TEST_F(CompressTest, Sqlite3RejectsLz4)
{
  const auto in_path = build_input(tmp_dir_);
  auto args = make_args(in_path, tmp_dir_ / "out");
  args.storage = "sqlite3";
  args.codec = "lz4";
  EXPECT_EQ(bagwiz::commands::run_compress(args), 1);
}

TEST_F(CompressTest, Sqlite3SingleFileOutputRejectsCompression)
{
  const auto in_path = build_input(tmp_dir_);
  // A .db3 output names a single file; sqlite3 compression needs the
  // directory layout's metadata.yaml to declare itself.
  auto args = make_args(in_path, tmp_dir_ / "out.db3");
  args.mode = "message";
  EXPECT_EQ(bagwiz::commands::run_compress(args), 1);
}

TEST_F(CompressTest, CodecWithoutCompressionRejected)
{
  const auto in_path = build_input(tmp_dir_);
  auto args = make_args(in_path, tmp_dir_ / "out");
  args.mode = "none";
  args.codec = "lz4";
  EXPECT_EQ(bagwiz::commands::run_compress(args), 1);
}

TEST_F(CompressTest, ExistingOutputRequiresOverwrite)
{
  const auto in_path = build_input(tmp_dir_);
  const auto out_path = tmp_dir_ / "out";

  auto args = make_args(in_path, out_path);
  ASSERT_EQ(bagwiz::commands::run_compress(args), 0);
  EXPECT_EQ(bagwiz::commands::run_compress(args), 1) << "existing output must stop the run";

  args.overwrite = true;
  EXPECT_EQ(bagwiz::commands::run_compress(args), 0);
  expect_fixture_intact(out_path);
}

TEST_F(CompressTest, ExplicitStorageOverridesExtensionInference)
{
  const auto in_path = build_input(tmp_dir_);
  // An extension-less output with --storage mcap stays MCAP even though the
  // input is also MCAP — this pins the --storage > extension > input order
  // at the command level.
  const auto out_path = tmp_dir_ / "out";
  auto args = make_args(in_path, out_path);
  args.storage = "mcap";
  args.mode = "none";
  ASSERT_EQ(bagwiz::commands::run_compress(args), 0);
  const auto md = bagwiz::io::load_metadata_yaml(out_path / "metadata.yaml");
  EXPECT_EQ(md.storage_identifier, "mcap");
}

}  // namespace
