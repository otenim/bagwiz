// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_output.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"

#include <cctype>
#include <cinttypes>
#include <cstdlib>
#include <string>
#include <system_error>
#include <utility>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.movify";

// Returns true for extensions that the encoder maps to H.264. Used only for
// user-facing playback guidance.
bool is_h264_extension(const std::filesystem::path & output)
{
  std::string ext = output.extension().string();
  for (auto & c : ext) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return ext == ".mp4" || ext == ".mkv" || ext == ".mov";
}

// Best-effort check for a vlc executable on the host. Used only to decide
// whether to append an install hint to the H.264 playback guidance.
bool is_vlc_available()
{
#ifdef _WIN32
  return std::system("where vlc >nul 2>nul") == 0;
#else
  return std::system("command -v vlc >/dev/null 2>&1") == 0;
#endif
}

// Platform-specific one-line hint for installing VLC.
const char * vlc_install_hint()
{
#ifdef __linux__
  return "Install VLC with your package manager (e.g. 'sudo apt install vlc').";
#elif __APPLE__
  return "Install VLC with: brew install vlc";
#elif _WIN32
  return "Install VLC from https://www.videolan.org/vlc/";
#else
  return "Install VLC from https://www.videolan.org/vlc/";
#endif
}

}  // namespace

std::string validate_video_output_path(const std::filesystem::path & output_path, bool overwrite)
{
  // Fail fast on an output collision before the expensive encode, through the
  // same check every other subcommand's -o path runs. The check is
  // non-destructive; finalize_video_output() does the removal just before the
  // rename, so an existing file is only replaced once the new video is fully
  // written.
  if (const auto r = core::check_output_path_free(output_path, overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    return r.error;
  }

  // Create the output's parent directory if needed.
  if (const auto parent = output_path.parent_path(); !parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      BAGWIZ_LOG_ERROR(
        kLogger, "could not create output directory '%s': %s", parent.string().c_str(),
        ec.message().c_str());
      return "could not create output directory '" + parent.string() + "': " + ec.message();
    }
  }
  return "";
}

std::filesystem::path partial_tmp_path_for(const std::filesystem::path & output)
{
  return output.parent_path() /
         (output.stem().string() + ".bagwiz-partial" + output.extension().string());
}

PartialFileGuard::PartialFileGuard(std::filesystem::path tmp_path) : tmp_path_(std::move(tmp_path))
{
  // Clear any stale temp from a previous aborted run.
  std::error_code ec;
  std::filesystem::remove(tmp_path_, ec);
}

PartialFileGuard::~PartialFileGuard()
{
  std::error_code ec;
  std::filesystem::remove(tmp_path_, ec);
}

std::string finalize_video_output(
  const std::filesystem::path & tmp_path, const std::filesystem::path & output_path, bool overwrite)
{
  // Now that the new video is complete, replace any existing output and move
  // the temp into place.
  if (const auto r = core::prepare_output_path(output_path, overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    return r.error;
  }
  std::error_code ec;
  std::filesystem::rename(tmp_path, output_path, ec);
  if (ec) {
    // Fall back to copy + remove across filesystems.
    std::error_code copy_ec;
    std::filesystem::copy_file(
      tmp_path, output_path, std::filesystem::copy_options::overwrite_existing, copy_ec);
    std::error_code remove_ec;
    std::filesystem::remove(tmp_path, remove_ec);
    if (copy_ec) {
      BAGWIZ_LOG_ERROR(kLogger, "could not move output into place: %s", copy_ec.message().c_str());
      return "could not move output into place: " + copy_ec.message();
    }
  }
  return "";
}

VideoFrameEncoder::VideoFrameEncoder(
  const std::filesystem::path & tmp_path, core::video::FrameRate fps,
  core::video::VideoEncoderOptions options)
: tmp_path_(tmp_path), fps_(fps), options_(std::move(options))
{
}

bool VideoFrameEncoder::encode(
  std::span<const std::byte> bgr, std::uint32_t frame_w, std::uint32_t frame_h)
{
  if (encoder_ == nullptr) {
    // The first frame fixes the geometry and pixel encoding for the run.
    enc_w_ = frame_w;
    enc_h_ = frame_h;
    auto opened =
      core::video::open_video_encoder(tmp_path_, enc_w_, enc_h_, fps_.num, fps_.den, options_);
    if (!opened.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", opened.error.c_str());
      return false;
    }
    if (!opened.fallback_note.empty()) {
      BAGWIZ_LOG_WARN(
        kLogger, "NVENC is not usable here (%s); encoding with %s.", opened.fallback_note.c_str(),
        opened.backend.c_str());
    } else {
      BAGWIZ_LOG_INFO(kLogger, "encoding with %s.", opened.backend.c_str());
    }
    encoder_ = std::move(opened.encoder);
  } else if (frame_w != enc_w_ || frame_h != enc_h_) {
    BAGWIZ_LOG_ERROR(
      kLogger, "frame %" PRIu64 " changed to %ux%u from the first frame's %ux%u; aborting.",
      written_, frame_w, frame_h, enc_w_, enc_h_);
    return false;
  }

  if (auto e = encoder_->write_frame(bgr, frame_w * 3U, core::video::SourcePixelFormat::kBgr8);
      !e.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written_, e.c_str());
    return false;
  }
  ++written_;
  return true;
}

std::string VideoFrameEncoder::finish()
{
  if (auto e = encoder_->finish(); !e.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", e.c_str());
    encoder_.reset();
    return e;
  }
  encoder_.reset();  // close the temp file before the rename/clobber
  return "";
}

std::string finish_video_encode(
  VideoFrameEncoder & encoder, const std::string & topic, const std::filesystem::path & tmp_path,
  const std::filesystem::path & output_path, bool overwrite)
{
  // Pass 1 saw messages, but if pass 2 yielded none (e.g. the bag changed
  // between passes) the encoder was never created. Nothing was rendered.
  if (!encoder.started()) {
    BAGWIZ_LOG_ERROR(kLogger, "topic '%s' yielded no frames in the encode pass.", topic.c_str());
    return "topic '" + topic + "' yielded no frames in the encode pass.";
  }
  if (const auto err = encoder.finish(); !err.empty()) {
    return err;
  }
  return finalize_video_output(tmp_path, output_path, overwrite);
}

void log_video_summary(
  const std::filesystem::path & output_path, std::uint64_t written, std::uint32_t width,
  std::uint32_t height, core::video::FrameRate fps)
{
  const double fps_value = static_cast<double>(fps.num) / static_cast<double>(fps.den);
  BAGWIZ_LOG_INFO(
    kLogger, "movify: wrote %" PRIu64 " frame(s) to %s (%ux%u bgr8 @ %.3g fps).", written,
    output_path.string().c_str(), width, height, fps_value);

  if (is_h264_extension(output_path)) {
    if (is_vlc_available()) {
      BAGWIZ_LOG_INFO(
        kLogger, "H.264 output saved. If mpv fails to play, try VLC or run mpv --hwdec=no.");
    } else {
      BAGWIZ_LOG_WARN(
        kLogger,
        "H.264 output saved. If mpv fails to play, run mpv --hwdec=no, or install VLC (%s)",
        vlc_install_hint());
    }
  }
}

}  // namespace bagwiz::commands
