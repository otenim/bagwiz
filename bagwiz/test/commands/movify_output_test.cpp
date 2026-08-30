// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_output.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/image/image_decoder.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "movify_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

// Unit tests for the movify output side: tmp-path handling, the RAII
// partial-file guard, finalize (rename/clobber), the output-path pre-flight
// check, and the encoder close-out.

namespace
{

using bagwiz::commands::finalize_video_output;
using bagwiz::commands::finish_video_encode;
using bagwiz::commands::partial_tmp_path_for;
using bagwiz::commands::PartialFileGuard;
using bagwiz::commands::validate_video_output_path;
using bagwiz::commands::VideoFrameEncoder;
using bagwiz::test::movify_read_file;
using bagwiz::test::movify_write_file;
using bagwiz::test::MovifyTmpDirTest;

// ---- partial_tmp_path_for ---------------------------------------------------

TEST(PartialTmpPath, KeepsExtensionAfterMarker)
{
  EXPECT_EQ(
    partial_tmp_path_for("/dir/out.avi"), std::filesystem::path("/dir") / "out.bagwiz-partial.avi");
}

TEST(PartialTmpPath, MultiDotNameKeepsOnlyFinalExtension)
{
  EXPECT_EQ(partial_tmp_path_for("a.b.mp4"), std::filesystem::path("a.b.bagwiz-partial.mp4"));
}

TEST(PartialTmpPath, NoExtensionAppendsMarker)
{
  EXPECT_EQ(partial_tmp_path_for("/dir/out"), std::filesystem::path("/dir") / "out.bagwiz-partial");
}

// ---- PartialFileGuard -------------------------------------------------------

TEST_F(MovifyTmpDirTest, PartialFileGuardCtorRemovesStaleFile)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  movify_write_file(tmp, "stale");
  {
    PartialFileGuard guard(tmp);
  }
  EXPECT_FALSE(std::filesystem::exists(tmp));
}

TEST_F(MovifyTmpDirTest, PartialFileGuardDtorRemovesLeftover)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  {
    PartialFileGuard guard(tmp);
    movify_write_file(tmp, "partial");  // simulate an aborted encode
  }
  EXPECT_FALSE(std::filesystem::exists(tmp));
}

TEST_F(MovifyTmpDirTest, PartialFileGuardDtorLeavesRenamedAwayOutput)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  const auto out = tmp_dir_ / "out.avi";
  {
    PartialFileGuard guard(tmp);
    movify_write_file(tmp, "video");
    std::filesystem::rename(tmp, out);
  }
  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_EQ(movify_read_file(out), "video");
}

// ---- finalize_video_output --------------------------------------------------

TEST_F(MovifyTmpDirTest, FinalizeRenamesTmpIntoPlace)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  const auto out = tmp_dir_ / "out.avi";
  movify_write_file(tmp, "video");
  EXPECT_EQ(finalize_video_output(tmp, out, false), "");
  EXPECT_FALSE(std::filesystem::exists(tmp));
  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_EQ(movify_read_file(out), "video");
}

TEST_F(MovifyTmpDirTest, FinalizeRejectsExistingOutputWithoutOverwrite)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  const auto out = tmp_dir_ / "out.avi";
  movify_write_file(tmp, "video");
  movify_write_file(out, "old");
  const auto err = finalize_video_output(tmp, out, false);
  EXPECT_EQ(
    err, "output path '" + out.string() + "' already exists; pass -w/--overwrite to replace it");
  // The tmp is left for the caller's PartialFileGuard to remove.
  EXPECT_TRUE(std::filesystem::exists(tmp));
  EXPECT_EQ(movify_read_file(out), "old");
}

TEST_F(MovifyTmpDirTest, FinalizeOverwriteReplacesExistingOutput)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  const auto out = tmp_dir_ / "out.avi";
  movify_write_file(tmp, "video");
  movify_write_file(out, "old");
  EXPECT_EQ(finalize_video_output(tmp, out, true), "");
  EXPECT_FALSE(std::filesystem::exists(tmp));
  EXPECT_EQ(movify_read_file(out), "video");
}

// ---- validate_video_output_path ---------------------------------------------

TEST_F(MovifyTmpDirTest, ValidateOutputPathRejectsCollisionWithoutOverwrite)
{
  const auto out = tmp_dir_ / "out.avi";
  movify_write_file(out, "old");
  // The early refusal and finalize's late one share one message, produced by
  // core::check_output_path_free / core::prepare_output_path.
  EXPECT_EQ(
    validate_video_output_path(out, false),
    "output path '" + out.string() + "' already exists; pass -w/--overwrite to replace it");
}

TEST_F(MovifyTmpDirTest, ValidateOutputPathAcceptsCollisionWithOverwrite)
{
  const auto out = tmp_dir_ / "out.avi";
  movify_write_file(out, "old");
  EXPECT_EQ(validate_video_output_path(out, true), "");
}

TEST_F(MovifyTmpDirTest, ValidateOutputPathCreatesMissingParentDirectories)
{
  const auto out = tmp_dir_ / "a" / "b" / "out.avi";
  EXPECT_EQ(validate_video_output_path(out, false), "");
  EXPECT_TRUE(std::filesystem::is_directory(tmp_dir_ / "a" / "b"));
}

// ---- finish_video_encode ----------------------------------------------------

TEST_F(MovifyTmpDirTest, FinishEncodeRequiresAStartedEncoder)
{
  const auto output = tmp_dir_ / "out.avi";
  VideoFrameEncoder encoder(partial_tmp_path_for(output), bagwiz::core::video::FrameRate{10, 1});
  EXPECT_FALSE(encoder.started());
  EXPECT_EQ(
    finish_video_encode(encoder, "/cam/image", partial_tmp_path_for(output), output, false),
    "topic '/cam/image' yielded no frames in the encode pass.");
  EXPECT_FALSE(std::filesystem::exists(output));
  EXPECT_FALSE(std::filesystem::exists(partial_tmp_path_for(output)));
}

}  // namespace
// frame in the other range is refused like a size change, since the planes
// are copied as they are.
TEST_F(MovifyTmpDirTest, EncodeYuv420RefusesAFrameInTheOtherRange)
{
  constexpr std::uint32_t kW = 32;
  constexpr std::uint32_t kH = 16;
  std::vector<std::uint8_t> y(static_cast<std::size_t>(kW) * kH, 128);
  std::vector<std::uint8_t> u(static_cast<std::size_t>(kW / 2) * (kH / 2), 128);
  std::vector<std::uint8_t> v(u.size(), 128);
  bagwiz::core::image::DecodedYuvView view;
  view.width = kW;
  view.height = kH;
  view.y = y.data();
  view.y_stride = static_cast<int>(kW);
  view.u = u.data();
  view.u_stride = static_cast<int>(kW / 2);
  view.v = v.data();
  view.v_stride = static_cast<int>(kW / 2);
  view.chroma = bagwiz::core::image::YuvChroma::k420;
  view.full_range = true;

  VideoFrameEncoder encoder(tmp_dir_ / "out.avi", bagwiz::core::video::FrameRate{10, 1});
  EXPECT_TRUE(encoder.encode_yuv420(view));  // opens full-range
  EXPECT_TRUE(encoder.encode_yuv420(view));
  view.full_range = false;
  EXPECT_FALSE(encoder.encode_yuv420(view));  // the other range: refused
  EXPECT_EQ(encoder.written(), 2u);
  EXPECT_TRUE(encoder.finish().empty());
}
