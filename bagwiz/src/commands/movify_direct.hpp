// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_DIRECT_HPP_
#define COMMANDS__MOVIFY_DIRECT_HPP_

#include "bagwiz/commands/movify.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "movify_inputs.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "movify_output.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <string>

// The direct camera pass: one JPEG camera shown as decoded streams each
// frame's YUV planes straight to the encoder — no BGR conversion, no
// composed canvas, no conversion back — decoding several frames ahead on
// worker threads while the main thread encodes. CLI-internal: this header
// lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Whether the run is one camera panel shown as decoded: a CompressedImage
// topic on a 1x1 grid, no rectification in effect, no --resize / --width,
// no --cam-pcd overlay, and no other panel (no --pcd, no --gnss). Anything
// else composes a canvas.
[[nodiscard]] bool can_stream_camera_direct(
  const MovifyArgs & args, const VideoInputValidation & validation);

// How many frames the direct pass decodes ahead: one decoder per slot, a
// quarter of the cores between two and four when the parallel pipeline is
// enabled, else one (decode and encode then alternate).
[[nodiscard]] unsigned int direct_decode_slots(
  bool enable_parallel, unsigned int hardware_concurrency);

// Encode every message of `topic` (the only topic `reader` yields) with
// `slots` decoders in flight. A frame that does not decode to 4:2:0 YUV (a
// PNG, say) goes through the packed-BGR path instead. Returns a process
// exit code: 0 on success, 1 after logging the first frame that fails.
[[nodiscard]] int run_direct_encode_pass(
  io::BagReader & reader, const std::string & topic, VideoFrameEncoder & encoder,
  unsigned int slots);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_DIRECT_HPP_
