// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_direct.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/image/compressed_image.hpp"
#include "bagwiz/core/image/image_decoder.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <future>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.movify";
constexpr const char * kCompressedImageType = "sensor_msgs/msg/CompressedImage";
constexpr unsigned int kMinSlots = 2;
constexpr unsigned int kMaxSlots = 4;
constexpr unsigned int kCoresPerSlot = 4;

// One decode in flight: its own decoder (a decoded view points into the
// decoder's frame, valid until that decoder's next decode — which only
// happens after this slot's frame was encoded), the message it works on, and
// its outcome: the YUV planes, or the packed-BGR fallback.
struct DecodeSlot
{
  core::image::ImageDecoder decoder;
  std::vector<std::byte> payload;
  std::int64_t record_ns = 0;
  std::future<std::string> job;
  std::optional<core::image::DecodedYuvView> yuv;
  std::optional<core::image::DecodedImage> bgr;
};

// Decode the slot's message: the YUV planes when the frame is 4:2:0, else a
// packed-BGR raster. Returns "" on success, or the error.
std::string decode_slot(DecodeSlot & slot)
{
  try {
    const auto compressed = core::image::extract_compressed_image(slot.payload);
    if (!compressed.ok()) {
      return compressed.error;
    }
    auto yuv = slot.decoder.decode_to_yuv(compressed.image->data, compressed.image->format);
    if (yuv.ok() && yuv.view->chroma == core::image::YuvChroma::k420) {
      slot.yuv = yuv.view;
      return "";
    }
    auto bgr = slot.decoder.decode(compressed.image->data, compressed.image->format);
    if (!bgr.ok()) {
      return bgr.error;
    }
    slot.bgr = std::move(*bgr.image);
    return "";
  } catch (const std::exception & e) {
    return std::string("decode failed: ") + e.what();
  }
}

bool encode_slot(VideoFrameEncoder & encoder, const DecodeSlot & slot)
{
  if (slot.yuv.has_value()) {
    return encoder.encode_yuv420(*slot.yuv);
  }
  return encoder.encode(slot.bgr->bgr, slot.bgr->width, slot.bgr->height);
}
}  // namespace

bool can_stream_camera_direct(const MovifyArgs & args, const VideoInputValidation & validation)
{
  if (
    validation.views.size() != 1 || !validation.pcd_topics.empty() ||
    validation.gnss_topic.has_value() || validation.pose_topic.has_value()) {
    return false;
  }
  const ViewInput & view = validation.views.front();
  return view.topic_type == kCompressedImageType && view.pcd_topics.empty() &&
         !view_rectifies(args.rectify, view) && args.resize_scale == 1.0f &&
         !args.width.has_value() && validation.grid.cols == 1 && validation.grid.rows == 1;
}

unsigned int direct_decode_slots(bool enable_parallel, unsigned int hardware_concurrency)
{
  if (!enable_parallel || hardware_concurrency < 2) {
    return 1;
  }
  return std::clamp(hardware_concurrency / kCoresPerSlot, kMinSlots, kMaxSlots);
}

int run_direct_encode_pass(
  io::BagReader & reader, const std::string & topic, VideoFrameEncoder & encoder,
  unsigned int slots)
{
  std::vector<DecodeSlot> ring(std::max(slots, 1U));

  // Hand message `raw` to the next free slot: the one the frame encoded
  // `ring.size()` ticks ago used, so the decode-ahead window never outruns
  // the encoder.
  std::uint64_t launched = 0;
  const auto launch = [&](const io::RawMessage & raw) {
    DecodeSlot & slot = ring[launched % ring.size()];
    slot.payload.assign(raw.payload.begin(), raw.payload.end());
    slot.record_ns = raw.timestamp_ns;
    slot.yuv.reset();
    slot.bgr.reset();
    slot.job = std::async(std::launch::async, [&slot] { return decode_slot(slot); });
    ++launched;
  };

  try {
    io::RawMessage raw;
    while (launched < ring.size() && reader.next(raw)) {
      launch(raw);
    }
    if (launched == 0) {
      BAGWIZ_LOG_ERROR(kLogger, "topic '%s' yielded no frames in the encode pass.", topic.c_str());
      return 1;
    }

    for (std::uint64_t tick = 0; tick < launched; ++tick) {
      DecodeSlot & slot = ring[tick % ring.size()];
      if (const std::string error = slot.job.get(); !error.empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "frame %" PRIu64 ": topic '%s': %s", tick, topic.c_str(), error.c_str());
        return 1;
      }
      if (!encode_slot(encoder, slot)) {
        return 1;
      }
      // The slot is free again: refill the window from the bag.
      if (reader.next(raw)) {
        launch(raw);
      }
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "error reading topic '%s': %s", topic.c_str(), e.what());
    return 1;
  }
  return 0;
}

}  // namespace bagwiz::commands
