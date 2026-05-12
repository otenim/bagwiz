// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/atomic_replace.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

namespace
{

namespace fs = std::filesystem;

fs::path make_unique_dir(const std::string & tag)
{
  static std::atomic<unsigned int> counter{0};
  const auto n = counter.fetch_add(1U);
  auto base =
    fs::temp_directory_path() / ("bagwiz_atomic_replace_" + tag + "_" + std::to_string(n));
  fs::remove_all(base);
  fs::create_directories(base);
  return base;
}

void write_file(const fs::path & p, const std::string & contents)
{
  std::ofstream out(p);
  out << contents;
}

std::string read_file(const fs::path & p)
{
  std::ifstream in(p);
  std::string s((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return s;
}

}  // namespace

TEST(AtomicReplaceTest, ReplacesExistingFileWithNewContents)
{
  const auto root = make_unique_dir("replace_file");
  const auto target = root / "bag.mcap";
  const auto staged = root / "new.mcap";

  write_file(target, "old");
  write_file(staged, "new");

  bagwiz::io::atomic_replace(staged, target);

  EXPECT_TRUE(fs::exists(target));
  EXPECT_FALSE(fs::exists(staged));
  EXPECT_EQ(read_file(target), "new");

  fs::remove_all(root);
}

TEST(AtomicReplaceTest, ReplacesExistingDirectoryWithStagedDirectory)
{
  const auto root = make_unique_dir("replace_dir");
  const auto target = root / "bag_dir";
  const auto staged = root / "bag_dir_staged";

  fs::create_directories(target);
  write_file(target / "metadata.yaml", "old-meta");
  write_file(target / "bag_0.mcap", "old-data");

  fs::create_directories(staged);
  write_file(staged / "metadata.yaml", "new-meta");
  write_file(staged / "bag_0.mcap", "new-data");

  bagwiz::io::atomic_replace(staged, target);

  ASSERT_TRUE(fs::exists(target));
  EXPECT_FALSE(fs::exists(staged));
  EXPECT_EQ(read_file(target / "metadata.yaml"), "new-meta");
  EXPECT_EQ(read_file(target / "bag_0.mcap"), "new-data");

  fs::remove_all(root);
}

TEST(AtomicReplaceTest, MovesStagedWhenTargetMissing)
{
  const auto root = make_unique_dir("missing_target");
  const auto target = root / "bag.mcap";
  const auto staged = root / "new.mcap";

  write_file(staged, "fresh");

  bagwiz::io::atomic_replace(staged, target);

  EXPECT_TRUE(fs::exists(target));
  EXPECT_FALSE(fs::exists(staged));
  EXPECT_EQ(read_file(target), "fresh");

  fs::remove_all(root);
}

TEST(AtomicReplaceTest, ThrowsWhenStagedMissing)
{
  const auto root = make_unique_dir("missing_staged");
  const auto target = root / "bag.mcap";
  const auto staged = root / "does_not_exist.mcap";

  write_file(target, "old");

  EXPECT_THROW(bagwiz::io::atomic_replace(staged, target), std::system_error);
  // Target must not be touched.
  ASSERT_TRUE(fs::exists(target));
  EXPECT_EQ(read_file(target), "old");

  fs::remove_all(root);
}
