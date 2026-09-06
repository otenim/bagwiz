// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/rewrite.hpp"

#include "bagwiz/io/bag_describe.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

#include <mcap/reader.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace
{

constexpr std::array<std::uint8_t, 4> kPayload{0x11, 0x22, 0x33, 0x44};
constexpr const char * kLogger = "bagwiz.test.rewrite";

std::span<const std::byte> payload_span()
{
  return {
    reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      kPayload.data()),
    kPayload.size()};
}

bagwiz::io::TopicInfo make_topic(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = "std_msgs/msg/Int32";
  t.serialization_format = "cdr";
  return t;
}

// Materialise a small bag at `path` holding a single "/input" message.
void seed_bag(
  const std::filesystem::path & path, bagwiz::io::Format format, bagwiz::io::Layout layout)
{
  bagwiz::io::CreateOptions opts;
  opts.format = format;
  opts.layout = layout;
  opts.mcap_compression = "none";
  auto writer = bagwiz::io::open_write(path, opts);
  writer->declare_topic(make_topic("/input"));
  writer->write("/input", 1'000'000'000LL, payload_span());
  writer->close();
}

// Materialise a bag at `path` holding one compressible "/input" message
// through whatever `opts` asks for, so a codec the writer was asked for
// actually lands on the chunk (libmcap stores a chunk uncompressed when
// compression would not pay for itself).
void seed_compressed_bag(const std::filesystem::path & path, const bagwiz::io::CreateOptions & opts)
{
  const std::vector<std::byte> big(64 * 1024, std::byte{0xCD});
  auto writer = bagwiz::io::open_write(path, opts);
  writer->declare_topic(make_topic("/input"));
  writer->write("/input", 1'000'000'000LL, {big.data(), big.size()});
  writer->close();
}

bagwiz::io::CreateOptions mcap_opts(bagwiz::io::Layout layout, const std::string & codec)
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = layout;
  opts.mcap_compression = codec;
  return opts;
}

bagwiz::io::CreateOptions sqlite3_dir_opts(const std::string & mode, const std::string & format)
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Sqlite3;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.sqlite3_compression_mode = mode;
  opts.sqlite3_compression_format = format;
  return opts;
}

// The stand-in for a command's pass: write a fresh one-topic bag through the
// injected factory and report success. The dispatch under test owns which
// path the factory targets (-o output or in-place tmp).
int write_replacement_pass(const bagwiz::io::WriterFactory & open_writer)
{
  auto writer = open_writer();
  writer->declare_topic(make_topic("/rewritten"));
  writer->write("/rewritten", 2'000'000'000LL, payload_span());
  writer->close();
  return 0;
}

std::string read_file_bytes(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// A pass writing a payload big and repetitive enough that zstd actually
// shrinks it. libmcap stores a chunk uncompressed when compression would not
// pay for itself, so the few-byte payload the other tests use would report
// "no compression" even with zstd correctly requested.
int write_compressible_replacement_pass(const bagwiz::io::WriterFactory & open_writer)
{
  const std::vector<std::byte> big(64 * 1024, std::byte{0xAB});
  auto writer = open_writer();
  writer->declare_topic(make_topic("/rewritten"));
  writer->write("/rewritten", 2'000'000'000LL, {big.data(), big.size()});
  writer->close();
  return 0;
}

// The compression recorded on each chunk of an MCAP file, deduplicated. An
// empty string is mcap::Compression::None — that is what the writer emits when
// CreateOptions::mcap_compression is "none", so it is the value a rewrite
// with the default mcap_compression = "none" override must produce.
std::set<std::string> mcap_chunk_compressions(const std::filesystem::path & path)
{
  mcap::McapReader reader;
  const auto open_status = reader.open(path.string());
  EXPECT_TRUE(open_status.ok()) << open_status.message;
  const auto summary_status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  EXPECT_TRUE(summary_status.ok()) << summary_status.message;

  std::set<std::string> compressions;
  for (const auto & index : reader.chunkIndexes()) {
    compressions.insert(index.compression);
  }
  reader.close();
  return compressions;
}

bool tmp_leftover_in(const std::filesystem::path & dir)
{
  for (const auto & entry : std::filesystem::directory_iterator(dir)) {
    if (entry.path().filename().string().find(".bagwiz-inplace-tmp-") != std::string::npos) {
      return true;
    }
  }
  return false;
}

class RewriteTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_rewrite_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);

    options_.logger = kLogger;
    options_.format_unknown_error = "test: could not detect storage format of input bag '%s'.";
    options_.pass_failed_error = "test: pass failed; aborting in-place swap";
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
  bagwiz::core::BagRewriteOptions options_;
};

}  // namespace

TEST_F(RewriteTest, OutputModeWritesNewBagAndLeavesInputUntouched)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  const auto input_before = read_file_bytes(input);

  int pass_calls = 0;
  const int status = bagwiz::core::run_bag_rewrite(
    input, output, /*overwrite=*/false, options_, [&](const bagwiz::io::WriterFactory & factory) {
      ++pass_calls;
      return write_replacement_pass(factory);
    });

  EXPECT_EQ(status, 0);
  EXPECT_EQ(pass_calls, 1);
  EXPECT_EQ(read_file_bytes(input), input_before);
  const auto reader = bagwiz::io::open_read(output);
  ASSERT_EQ(reader->topics().size(), 1u);
  EXPECT_EQ(reader->topics()[0].name, "/rewritten");
}

TEST_F(RewriteTest, OutputModeFailsWhenOutputExistsWithoutOverwrite)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  {
    std::ofstream out(output);
    out << "PRE-EXISTING";
  }

  int pass_calls = 0;
  const int status = bagwiz::core::run_bag_rewrite(
    input, output, /*overwrite=*/false, options_, [&](const bagwiz::io::WriterFactory & factory) {
      ++pass_calls;
      return write_replacement_pass(factory);
    });

  EXPECT_EQ(status, 1);
  EXPECT_EQ(pass_calls, 0);  // prepare_output_path fails before the pass runs
  EXPECT_EQ(read_file_bytes(output), "PRE-EXISTING");
}

TEST_F(RewriteTest, OutputModeOverwriteReplacesExisting)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  {
    std::ofstream out(output);
    out << "PRE-EXISTING";
  }

  const int status = bagwiz::core::run_bag_rewrite(
    input, output, /*overwrite=*/true, options_, write_replacement_pass);

  EXPECT_EQ(status, 0);
  const auto reader = bagwiz::io::open_read(output);
  ASSERT_EQ(reader->topics().size(), 1u);
  EXPECT_EQ(reader->topics()[0].name, "/rewritten");
}

TEST_F(RewriteTest, OutputModeDirectoryOutputInheritsInputFormat)
{
  const auto input = tmp_dir_ / "input_db3";
  const auto output = tmp_dir_ / "output_dir";
  seed_bag(input, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::Directory);

  const int status = bagwiz::core::run_bag_rewrite(
    input, output, /*overwrite=*/false, options_, write_replacement_pass);

  EXPECT_EQ(status, 0);
  ASSERT_TRUE(std::filesystem::is_directory(output));
  EXPECT_EQ(bagwiz::io::detect_format(output), bagwiz::io::Format::Sqlite3);
}

TEST_F(RewriteTest, OutputModeDirectoryOutputInheritsInputFormatWithBareOptions)
{
  // Inheritance is not a knob a command can leave off: options carrying
  // nothing but the mandatory messages still give a directory output the
  // input's backend rather than the factory's Mcap default.
  const auto input = tmp_dir_ / "input_db3";
  const auto output = tmp_dir_ / "output_dir";
  seed_bag(input, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::Directory);
  bagwiz::core::BagRewriteOptions bare;
  bare.logger = kLogger;
  bare.format_unknown_error = options_.format_unknown_error;
  bare.pass_failed_error = options_.pass_failed_error;

  const int status =
    bagwiz::core::run_bag_rewrite(input, output, /*overwrite=*/false, bare, write_replacement_pass);

  EXPECT_EQ(status, 0);
  ASSERT_TRUE(std::filesystem::is_directory(output));
  EXPECT_EQ(bagwiz::io::detect_format(output), bagwiz::io::Format::Sqlite3);
}

TEST_F(RewriteTest, OutputModeSingleFileExtensionWinsOverInherit)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.db3";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);

  const int status = bagwiz::core::run_bag_rewrite(
    input, output, /*overwrite=*/false, options_, write_replacement_pass);

  EXPECT_EQ(status, 0);
  ASSERT_FALSE(std::filesystem::is_directory(output));
  EXPECT_EQ(bagwiz::io::detect_format(output), bagwiz::io::Format::Sqlite3);
}

TEST_F(RewriteTest, InPlaceRewritesInputAndPreservesMcapSingleFile)
{
  const auto input = tmp_dir_ / "input.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);

  const int status = bagwiz::core::run_bag_rewrite(
    input, std::nullopt, /*overwrite=*/false, options_, write_replacement_pass);

  EXPECT_EQ(status, 0);
  ASSERT_FALSE(std::filesystem::is_directory(input));
  EXPECT_EQ(bagwiz::io::detect_format(input), bagwiz::io::Format::Mcap);
  const auto reader = bagwiz::io::open_read(input);
  ASSERT_EQ(reader->topics().size(), 1u);
  EXPECT_EQ(reader->topics()[0].name, "/rewritten");
  EXPECT_FALSE(tmp_leftover_in(tmp_dir_));
}

TEST_F(RewriteTest, InPlacePreservesSqlite3SingleFile)
{
  const auto input = tmp_dir_ / "input.db3";
  seed_bag(input, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::SingleFile);

  const int status = bagwiz::core::run_bag_rewrite(
    input, std::nullopt, /*overwrite=*/false, options_, write_replacement_pass);

  EXPECT_EQ(status, 0);
  ASSERT_FALSE(std::filesystem::is_directory(input));
  EXPECT_EQ(bagwiz::io::detect_format(input), bagwiz::io::Format::Sqlite3);
}

TEST_F(RewriteTest, InPlacePassFailureReturnsStatusAndLeavesInputUntouched)
{
  const auto input = tmp_dir_ / "input.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  const auto input_before = read_file_bytes(input);

  // The pass fully writes the replacement bag but still reports failure; the
  // swap must not happen and the pass's exit code must propagate.
  const int status = bagwiz::core::run_bag_rewrite(
    input, std::nullopt, /*overwrite=*/false, options_,
    [](const bagwiz::io::WriterFactory & factory) {
      write_replacement_pass(factory);
      return 3;
    });

  EXPECT_EQ(status, 3);
  EXPECT_EQ(read_file_bytes(input), input_before);
  EXPECT_FALSE(tmp_leftover_in(tmp_dir_));
}

TEST_F(RewriteTest, InPlacePassThrowReturnsOneAndLeavesInputUntouched)
{
  const auto input = tmp_dir_ / "input.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  const auto input_before = read_file_bytes(input);

  const int status = bagwiz::core::run_bag_rewrite(
    input, std::nullopt, /*overwrite=*/false, options_,
    [](const bagwiz::io::WriterFactory &) -> int { throw std::runtime_error("pass exploded"); });

  EXPECT_EQ(status, 1);
  EXPECT_EQ(read_file_bytes(input), input_before);
  EXPECT_FALSE(tmp_leftover_in(tmp_dir_));
}

TEST_F(RewriteTest, InPlaceFormatAutoGuardRejectsNonBag)
{
  const auto input = tmp_dir_ / "notes.txt";
  {
    std::ofstream out(input);
    out << "this is not a bag";
  }

  int pass_calls = 0;
  const int status = bagwiz::core::run_bag_rewrite(
    input, std::nullopt, /*overwrite=*/false, options_,
    [&](const bagwiz::io::WriterFactory & factory) {
      ++pass_calls;
      return write_replacement_pass(factory);
    });

  EXPECT_EQ(status, 1);
  EXPECT_EQ(pass_calls, 0);
  EXPECT_EQ(read_file_bytes(input), "this is not a bag");
  EXPECT_FALSE(tmp_leftover_in(tmp_dir_));
}

// ---------------------------------------------------------------------------
// Compression wiring.
//
// Every rewrite command drives the writer through run_bag_rewrite, and by
// default the output carries the input's compression over, translated to
// the output storage, so a rewrite never silently strips or adds
// compression. A command pins a knob only to change it (`compress`,
// `pcd undistort --compression`). The effect is invisible in a bag's
// topics/messages, so these tests read the produced MCAP's chunk
// compression and the sqlite3 metadata directly.
// ---------------------------------------------------------------------------

TEST(BagRewriteOptionsDefaults, CompressionDefaultsToInheritingTheInput)
{
  // Empty knobs mean "carry the input's compression over"; a new command
  // that never touches them gets the shared policy for free.
  const bagwiz::core::BagRewriteOptions defaults;
  EXPECT_TRUE(defaults.mcap_compression.empty());
  EXPECT_TRUE(defaults.mcap_compression_level.empty());
  EXPECT_TRUE(defaults.sqlite3_compression_mode.empty());
  EXPECT_TRUE(defaults.sqlite3_compression_format.empty());
  EXPECT_TRUE(defaults.sqlite3_compression_level.empty());
  EXPECT_EQ(defaults.output_format, bagwiz::io::Format::Auto);
}

TEST_F(RewriteTest, OutputModeExplicitFormatOutranksTheOutputExtension)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.db3";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  // `compress --storage mcap` shape: the command already picked the backend,
  // so the .db3 extension may only decide the layout.
  options_.output_format = bagwiz::io::Format::Mcap;

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_replacement_pass),
    0);

  EXPECT_FALSE(std::filesystem::is_directory(output));
  EXPECT_EQ(bagwiz::io::detect_format(output), bagwiz::io::Format::Mcap);
}

TEST_F(RewriteTest, OutputModeSqlite3CompressionOverridesReachTheWriter)
{
  const auto input = tmp_dir_ / "input_db3";
  const auto output = tmp_dir_ / "output_dir";
  seed_bag(input, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::Directory);
  options_.output_format = bagwiz::io::Format::Sqlite3;
  options_.sqlite3_compression_mode = "message";
  options_.sqlite3_compression_format = "zstd";

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_replacement_pass),
    0);

  const auto md = bagwiz::io::load_metadata_yaml(output / "metadata.yaml");
  EXPECT_EQ(md.compression_mode, "message");
  EXPECT_EQ(md.compression_format, "zstd");
}

TEST_F(RewriteTest, InPlaceSqlite3CompressionOverridesReachTheWriter)
{
  const auto input = tmp_dir_ / "input_db3";
  seed_bag(input, bagwiz::io::Format::Sqlite3, bagwiz::io::Layout::Directory);
  options_.sqlite3_compression_mode = "message";
  options_.sqlite3_compression_format = "zstd";

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, std::nullopt, /*overwrite=*/false, options_, write_replacement_pass),
    0);

  // create_options_preserving_storage pins format/layout and leaves the
  // compression knobs to the override, exactly as the -o path does.
  const auto md = bagwiz::io::load_metadata_yaml(input / "metadata.yaml");
  EXPECT_EQ(md.compression_mode, "message");
  EXPECT_EQ(md.compression_format, "zstd");
  EXPECT_FALSE(tmp_leftover_in(tmp_dir_));
}

TEST_F(RewriteTest, OutputModePlainInputWritesUncompressedChunks)
{
  // CreateOptions' own default is zstd; inheriting from a plain input has
  // to override it, or every rewrite of an uncompressed bag would compress.
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.mcap";
  seed_bag(input, bagwiz::io::Format::Mcap, bagwiz::io::Layout::SingleFile);
  ASSERT_TRUE(options_.mcap_compression.empty());

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  EXPECT_EQ(mcap_chunk_compressions(output), (std::set<std::string>{""}));
}

TEST_F(RewriteTest, OutputModeInheritsZstdChunks)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.mcap";
  seed_compressed_bag(input, mcap_opts(bagwiz::io::Layout::SingleFile, "zstd"));

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  EXPECT_EQ(mcap_chunk_compressions(output), (std::set<std::string>{"zstd"}));
}

TEST_F(RewriteTest, OutputModeInheritsLz4ChunksIntoADirectory)
{
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output_dir";
  seed_compressed_bag(input, mcap_opts(bagwiz::io::Layout::SingleFile, "lz4"));

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  ASSERT_TRUE(std::filesystem::is_directory(output));
  EXPECT_EQ(mcap_chunk_compressions(output / "output_dir_0.mcap"), (std::set<std::string>{"lz4"}));
}

TEST_F(RewriteTest, OutputModeCodecOverrideOutranksTheInput)
{
  const auto input = tmp_dir_ / "input.mcap";
  seed_compressed_bag(input, mcap_opts(bagwiz::io::Layout::SingleFile, "zstd"));

  // `compress --mode none` shape: the command pins "none" on a zstd input.
  const auto plain = tmp_dir_ / "plain.mcap";
  options_.mcap_compression = "none";
  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, plain, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);
  EXPECT_EQ(mcap_chunk_compressions(plain), (std::set<std::string>{""}));

  // A user-chosen codec (pcd undistort --compression lz4) flows through the
  // same override; the level accompanies it.
  const auto lz4 = tmp_dir_ / "lz4.mcap";
  options_.mcap_compression = "lz4";
  options_.mcap_compression_level = "fastest";
  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, lz4, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);
  EXPECT_EQ(mcap_chunk_compressions(lz4), (std::set<std::string>{"lz4"}));
}

TEST_F(RewriteTest, OutputModeLevelAloneKeepsTheInheritedCodec)
{
  // pcd undistort --compression-level without --compression: the effort is
  // the user's, the codec is still the input's. lz4 rather than zstd, so a
  // dispatch that skipped inheritance (and left the writer's zstd default)
  // would fail here.
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.mcap";
  seed_compressed_bag(input, mcap_opts(bagwiz::io::Layout::SingleFile, "lz4"));
  options_.mcap_compression_level = "fastest";

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  EXPECT_EQ(mcap_chunk_compressions(output), (std::set<std::string>{"lz4"}));
}

TEST_F(RewriteTest, OutputModeSqlite3MessageModeInputKeepsMessageMode)
{
  const auto input = tmp_dir_ / "input_dir";
  const auto output = tmp_dir_ / "output_dir";
  seed_compressed_bag(input, sqlite3_dir_opts("message", "zstd"));

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  EXPECT_EQ(bagwiz::io::detect_format(output), bagwiz::io::Format::Sqlite3);
  const auto md = bagwiz::io::load_metadata_yaml(output / "metadata.yaml");
  EXPECT_EQ(md.compression_mode, "message");
  EXPECT_EQ(md.compression_format, "zstd");
}

TEST_F(RewriteTest, OutputModeSqlite3FileModeInputKeepsFileMode)
{
  const auto input = tmp_dir_ / "input_dir";
  const auto output = tmp_dir_ / "output_dir";
  seed_compressed_bag(input, sqlite3_dir_opts("file", "zstd"));

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  const auto md = bagwiz::io::load_metadata_yaml(output / "metadata.yaml");
  EXPECT_EQ(md.compression_mode, "file");
  EXPECT_EQ(md.compression_format, "zstd");
  ASSERT_EQ(md.relative_file_paths.size(), 1u);
  EXPECT_EQ(md.relative_file_paths.front().extension(), ".zstd");
  EXPECT_TRUE(std::filesystem::exists(output / md.relative_file_paths.front()));
}

TEST_F(RewriteTest, OutputModeZstdMcapInputToSqlite3DirectoryBecomesMessageMode)
{
  // `compress --storage sqlite3`-style pinned backend, compression left to
  // inheritance: chunk compression has no sqlite3 counterpart, so the
  // output takes rosbag2 MESSAGE mode.
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output_dir";
  seed_compressed_bag(input, mcap_opts(bagwiz::io::Layout::SingleFile, "zstd"));
  options_.output_format = bagwiz::io::Format::Sqlite3;

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  const auto md = bagwiz::io::load_metadata_yaml(output / "metadata.yaml");
  EXPECT_EQ(md.storage_identifier, "sqlite3");
  EXPECT_EQ(md.compression_mode, "message");
  EXPECT_EQ(md.compression_format, "zstd");
}

TEST_F(RewriteTest, OutputModeCompressedInputToSingleFileSqlite3IsWrittenPlain)
{
  // A bare .db3 cannot carry compression (rosbag2 reads the mode from
  // metadata.yaml alone), so the rewrite must still succeed — and the
  // single-file writer must not be handed a mode it refuses.
  const auto input = tmp_dir_ / "input_dir";
  const auto output = tmp_dir_ / "output.db3";
  seed_compressed_bag(input, sqlite3_dir_opts("message", "zstd"));

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  ASSERT_FALSE(std::filesystem::is_directory(output));
  EXPECT_EQ(bagwiz::io::describe_bag(output).compression.mode, "none");
}

TEST_F(RewriteTest, OutputModeSqlite3MessageModeInputToMcapBecomesZstdChunks)
{
  const auto input = tmp_dir_ / "input_dir";
  const auto output = tmp_dir_ / "output.mcap";
  seed_compressed_bag(input, sqlite3_dir_opts("message", "zstd"));

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  EXPECT_EQ(mcap_chunk_compressions(output), (std::set<std::string>{"zstd"}));
}

TEST_F(RewriteTest, InPlaceInheritsLz4Chunks)
{
  // lz4 rather than zstd, so a dispatch that skipped inheritance (and left
  // the writer's zstd default) would fail here.
  const auto input = tmp_dir_ / "input.mcap";
  seed_compressed_bag(input, mcap_opts(bagwiz::io::Layout::SingleFile, "lz4"));

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, std::nullopt, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  // create_options_preserving_storage pins format/layout and the dispatch
  // carries the compression over, exactly like the -o path.
  EXPECT_EQ(mcap_chunk_compressions(input), (std::set<std::string>{"lz4"}));
  EXPECT_FALSE(tmp_leftover_in(tmp_dir_));
}

TEST_F(RewriteTest, OutputModeUnreadableCompressionIsWrittenPlain)
{
  // An MCAP that was never finalized has no summary section, so its codec
  // cannot be read without a scan. The rewrite still runs (open_read falls
  // back to scanning) but must not guess: plain output, never compression
  // the input may not have had.
  const auto input = tmp_dir_ / "input.mcap";
  const auto output = tmp_dir_ / "output.mcap";
  seed_compressed_bag(input, mcap_opts(bagwiz::io::Layout::SingleFile, "none"));
  std::filesystem::resize_file(input, std::filesystem::file_size(input) - 64);

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, output, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  EXPECT_EQ(mcap_chunk_compressions(output), (std::set<std::string>{""}));
}

TEST_F(RewriteTest, InPlaceCodecOverrideOutranksTheInput)
{
  const auto input = tmp_dir_ / "input.mcap";
  seed_compressed_bag(input, mcap_opts(bagwiz::io::Layout::SingleFile, "zstd"));
  options_.mcap_compression = "none";

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, std::nullopt, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  EXPECT_EQ(mcap_chunk_compressions(input), (std::set<std::string>{""}));
}

TEST_F(RewriteTest, InPlacePreservesFileModeSqlite3Directory)
{
  // A FILE-mode directory bag rewrites in place like any other: the
  // directory writer reproduces the .db3.zstd envelope.
  const auto input = tmp_dir_ / "input_dir";
  seed_compressed_bag(input, sqlite3_dir_opts("file", "zstd"));

  ASSERT_EQ(
    bagwiz::core::run_bag_rewrite(
      input, std::nullopt, /*overwrite=*/false, options_, write_compressible_replacement_pass),
    0);

  const auto md = bagwiz::io::load_metadata_yaml(input / "metadata.yaml");
  EXPECT_EQ(md.compression_mode, "file");
  EXPECT_EQ(md.compression_format, "zstd");
  ASSERT_EQ(md.relative_file_paths.size(), 1u);
  EXPECT_EQ(md.relative_file_paths.front().extension(), ".zstd");
  EXPECT_TRUE(std::filesystem::exists(input / md.relative_file_paths.front()));
  const auto reader = bagwiz::io::open_read(input);
  ASSERT_EQ(reader->topics().size(), 1u);
  EXPECT_EQ(reader->topics()[0].name, "/rewritten");
  EXPECT_FALSE(tmp_leftover_in(tmp_dir_));
}

TEST_F(RewriteTest, InPlaceRejectsABareEnvelope)
{
  // A standalone .db3.zstd has no directory writer to reproduce its
  // envelope, so the in-place guard still refuses it — before the pass runs.
  const auto dir = tmp_dir_ / "env_dir";
  seed_compressed_bag(dir, sqlite3_dir_opts("file", "zstd"));
  const auto input = tmp_dir_ / "bare.db3.zstd";
  std::filesystem::copy_file(dir / "env_dir_0.db3.zstd", input);
  const auto input_before = read_file_bytes(input);

  int pass_calls = 0;
  const int status = bagwiz::core::run_bag_rewrite(
    input, std::nullopt, /*overwrite=*/false, options_,
    [&](const bagwiz::io::WriterFactory & factory) {
      ++pass_calls;
      return write_replacement_pass(factory);
    });

  EXPECT_EQ(status, 1);
  EXPECT_EQ(pass_calls, 0);
  EXPECT_EQ(read_file_bytes(input), input_before);
  EXPECT_FALSE(tmp_leftover_in(tmp_dir_));
}
