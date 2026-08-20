// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/output_path.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

namespace
{

class OutputPathTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_output_path_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
                std::to_string(
                  reinterpret_cast<std::uintptr_t>(
                    this)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

void touch(const std::filesystem::path & p)
{
  std::ofstream out(p);
  out << "x";
}

}  // namespace

TEST_F(OutputPathTest, NonExistentPathIsOk)
{
  const auto target = tmp_dir_ / "no_such_file.tum";
  const auto r = bagwiz::core::prepare_output_path(target, /*overwrite=*/false);
  EXPECT_TRUE(r.ok);
  EXPECT_TRUE(r.error.empty());
  EXPECT_FALSE(std::filesystem::exists(target));
}

TEST_F(OutputPathTest, ExistingFileWithoutOverwriteIsError)
{
  const auto target = tmp_dir_ / "existing.tum";
  touch(target);

  const auto r = bagwiz::core::prepare_output_path(target, /*overwrite=*/false);
  EXPECT_FALSE(r.ok);
  // The error must point the user at -w/--overwrite so the resolution path is
  // discoverable from the message alone.
  EXPECT_NE(r.error.find("-w/--overwrite"), std::string::npos);
  EXPECT_NE(r.error.find(target.string()), std::string::npos);
  // The pre-existing file must be left intact when we refuse.
  EXPECT_TRUE(std::filesystem::exists(target));
}

TEST_F(OutputPathTest, ExistingFileWithOverwriteIsRemoved)
{
  const auto target = tmp_dir_ / "existing.tum";
  touch(target);

  const auto r = bagwiz::core::prepare_output_path(target, /*overwrite=*/true);
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_FALSE(std::filesystem::exists(target));
}

TEST_F(OutputPathTest, ExistingDirectoryWithOverwriteIsRemovedRecursively)
{
  // Mirrors a rosbag2 directory layout: the output "file" is actually a
  // directory containing shards and metadata.yaml.
  const auto target = tmp_dir_ / "existing_bag_dir";
  std::filesystem::create_directories(target / "nested");
  touch(target / "metadata.yaml");
  touch(target / "nested" / "shard_0.mcap");

  const auto r = bagwiz::core::prepare_output_path(target, /*overwrite=*/true);
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_FALSE(std::filesystem::exists(target));
}

TEST_F(OutputPathTest, ExistingDirectoryWithoutOverwriteIsError)
{
  const auto target = tmp_dir_ / "existing_bag_dir";
  std::filesystem::create_directories(target);

  const auto r = bagwiz::core::prepare_output_path(target, /*overwrite=*/false);
  EXPECT_FALSE(r.ok);
  EXPECT_TRUE(std::filesystem::exists(target));
}

// check_output_path_free() is the non-destructive half of the policy: it must
// return the same verdicts as prepare_output_path() while leaving the
// filesystem exactly as it found it, so a command can call it before work that
// can still fail.

TEST_F(OutputPathTest, CheckOnlyNonExistentPathIsOk)
{
  const auto target = tmp_dir_ / "no_such_file.tum";
  const auto r = bagwiz::core::check_output_path_free(target, /*overwrite=*/false);
  EXPECT_TRUE(r.ok);
  EXPECT_TRUE(r.error.empty());
  EXPECT_FALSE(std::filesystem::exists(target));
}

TEST_F(OutputPathTest, CheckOnlyExistingFileWithoutOverwriteIsError)
{
  const auto target = tmp_dir_ / "existing.tum";
  touch(target);

  const auto r = bagwiz::core::check_output_path_free(target, /*overwrite=*/false);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("-w/--overwrite"), std::string::npos);
  EXPECT_NE(r.error.find(target.string()), std::string::npos);
  EXPECT_TRUE(std::filesystem::exists(target));
}

TEST_F(OutputPathTest, CheckOnlyExistingFileWithOverwriteIsOkAndKeepsTheFile)
{
  const auto target = tmp_dir_ / "existing.tum";
  touch(target);

  const auto r = bagwiz::core::check_output_path_free(target, /*overwrite=*/true);
  EXPECT_TRUE(r.ok) << r.error;
  // The whole point of the check-only half: the caller may still bail, so the
  // file must survive until prepare_output_path() claims it at the write site.
  EXPECT_TRUE(std::filesystem::exists(target));
}

TEST_F(OutputPathTest, CheckOnlyExistingDirectoryWithOverwriteIsOkAndKeepsTheTree)
{
  const auto target = tmp_dir_ / "existing_bag_dir";
  std::filesystem::create_directories(target / "nested");
  touch(target / "metadata.yaml");

  const auto r = bagwiz::core::check_output_path_free(target, /*overwrite=*/true);
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(std::filesystem::exists(target / "metadata.yaml"));
}

TEST_F(OutputPathTest, CheckOnlyExistingDirectoryWithoutOverwriteIsError)
{
  const auto target = tmp_dir_ / "existing_bag_dir";
  std::filesystem::create_directories(target);

  const auto r = bagwiz::core::check_output_path_free(target, /*overwrite=*/false);
  EXPECT_FALSE(r.ok);
  EXPECT_TRUE(std::filesystem::exists(target));
}

TEST_F(OutputPathTest, CheckOnlyAndPrepareAgreeOnTheCollisionMessage)
{
  // The early refusal and the late one are the same message, so a user cannot
  // tell from the text which phase rejected the run.
  const auto target = tmp_dir_ / "existing.tum";
  touch(target);

  const auto checked = bagwiz::core::check_output_path_free(target, /*overwrite=*/false);
  const auto prepared = bagwiz::core::prepare_output_path(target, /*overwrite=*/false);
  EXPECT_FALSE(checked.ok);
  EXPECT_FALSE(prepared.ok);
  EXPECT_EQ(checked.error, prepared.error);
}

TEST_F(OutputPathTest, CheckOnlyExistingSymlinkWithoutOverwriteIsError)
{
  // symlink_status, not status: a dangling symlink still occupies the path, so
  // the early check must refuse it exactly as prepare_output_path() does.
  const auto target = tmp_dir_ / "dangling.tum";
  std::error_code ec;
  std::filesystem::create_symlink(tmp_dir_ / "missing_target", target, ec);
  if (ec) {
    GTEST_SKIP() << "cannot create symlinks here: " << ec.message();
  }

  const auto r = bagwiz::core::check_output_path_free(target, /*overwrite=*/false);
  EXPECT_FALSE(r.ok);
  EXPECT_TRUE(std::filesystem::is_symlink(std::filesystem::symlink_status(target)));
}
