// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_OUTPUT_HPP_
#define COMMANDS__MOVIFY_OUTPUT_HPP_

#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/core/video/video_encoder.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>

// The output side of `movify`: the fail-fast output-path check, the partial
// tmp file the video is encoded into (and the RAII guard that removes it on
// any failure), the frame encoder wrapper, the move into place, and the
// end-of-run summary. CLI-internal: this header lives with the command
// sources and is not installed.
namespace bagwiz::commands
{

// Fail-fast output checks run before the expensive encode: an existing
// `output_path` without --overwrite stops the run, and the output's parent
// directory is created when missing. Returns "" on success; on failure logs
// and returns the message.
[[nodiscard]] std::string validate_video_output_path(
  const std::filesystem::path & output_path, bool overwrite);

// The sibling temp path the video is encoded into before being moved into
// place, e.g. out.avi -> out.bagwiz-partial.avi. The real extension is kept on
// the temp file: both the encoder's codec choice and the libav muxer are
// selected from the extension, so a bare ".bagwiz-partial" suffix would be
// rejected.
[[nodiscard]] std::filesystem::path partial_tmp_path_for(const std::filesystem::path & output);

// RAII owner of the partial tmp output's lifecycle: construction clears any
// stale tmp left by a previous aborted run; destruction removes the tmp when
// it still exists (a no-op once finalize_video_output renamed it away), so no
// error path can leave a partial file behind. Declare it BEFORE the encoder so
// the encoder is destroyed — closing the file — before the tmp is removed.
class PartialFileGuard
{
public:
  explicit PartialFileGuard(std::filesystem::path tmp_path);
  ~PartialFileGuard();
  PartialFileGuard(const PartialFileGuard &) = delete;
  PartialFileGuard & operator=(const PartialFileGuard &) = delete;
  PartialFileGuard(PartialFileGuard &&) = delete;
  PartialFileGuard & operator=(PartialFileGuard &&) = delete;

  [[nodiscard]] const std::filesystem::path & path() const noexcept { return tmp_path_; }

private:
  std::filesystem::path tmp_path_;
};

// Move the finished tmp video into place: apply the --overwrite clobber policy
// via core::prepare_output_path, then rename, falling back to copy + remove
// across filesystems. Returns "" on success; on failure logs and returns the
// message (the caller's PartialFileGuard removes the tmp).
[[nodiscard]] std::string finalize_video_output(
  const std::filesystem::path & tmp_path, const std::filesystem::path & output_path,
  bool overwrite);

// Encode half of the frame pipeline: owns the video encoder (opened lazily on
// the first composed frame, which fixes the run's geometry). All failures are
// logged and reported as false / a non-empty string.
class VideoFrameEncoder
{
public:
  VideoFrameEncoder(
    const std::filesystem::path & tmp_path, core::video::FrameRate fps,
    core::video::VideoEncoderOptions options = {});

  // Encode one composed packed-BGR24 frame (row stride width*3). Returns
  // false after logging on any failure, including a mid-run size change.
  [[nodiscard]] bool encode(
    std::span<const std::byte> bgr, std::uint32_t frame_w, std::uint32_t frame_h);

  // Flush and close the stream. Returns "" on success; on failure logs and
  // returns the message. Either way the encoder is closed afterwards (the tmp
  // file can be renamed or removed).
  [[nodiscard]] std::string finish();

  // True once the first frame opened the encoder. A finished run still reports
  // its geometry and frame count for the summary line.
  [[nodiscard]] bool started() const { return encoder_ != nullptr; }
  [[nodiscard]] std::uint64_t written() const { return written_; }
  [[nodiscard]] std::uint32_t width() const { return enc_w_; }
  [[nodiscard]] std::uint32_t height() const { return enc_h_; }

private:
  std::filesystem::path tmp_path_;
  core::video::FrameRate fps_;
  core::video::VideoEncoderOptions options_;

  std::unique_ptr<core::video::VideoEncoder> encoder_;
  std::uint32_t enc_w_ = 0;
  std::uint32_t enc_h_ = 0;
  std::uint64_t written_ = 0;
};

// Close out the encode: require at least one rendered frame (pass 1 saw
// messages, so a frameless pass 2 means the bag changed between passes), flush
// + close the encoder, and move the tmp output into place. Returns "" on
// success; logs and returns the message on failure.
[[nodiscard]] std::string finish_video_encode(
  VideoFrameEncoder & encoder, const std::string & topic, const std::filesystem::path & tmp_path,
  const std::filesystem::path & output_path, bool overwrite);

// The end-of-run INFO line plus the H.264 playback guidance (with the VLC
// install hint when no vlc executable is on the host).
void log_video_summary(
  const std::filesystem::path & output_path, std::uint64_t written, std::uint32_t width,
  std::uint32_t height, core::video::FrameRate fps);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_OUTPUT_HPP_
