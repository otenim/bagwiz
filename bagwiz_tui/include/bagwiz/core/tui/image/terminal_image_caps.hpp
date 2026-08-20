// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TUI__IMAGE__TERMINAL_IMAGE_CAPS_HPP_
#define BAGWIZ__CORE__TUI__IMAGE__TERMINAL_IMAGE_CAPS_HPP_

#include "bagwiz/core/tui/layout.hpp"

#include <optional>
#include <ostream>
#include <string_view>

// Detects which terminal graphics protocol (if any) the current terminal
// supports, so `walk` can offer an image preview only where it can actually
// render. This is protocol-only: Kitty graphics and DEC Sixel. There is
// deliberately no half-block / truecolor fallback — a terminal that supports
// neither protocol reports kNone and the preview is simply unavailable.
namespace bagwiz::core::tui::image
{

enum class ImageBackend {
  kNone,   // no supported graphics protocol; preview unavailable
  kSixel,  // DEC Sixel (libsixel)
  kKitty,  // Kitty graphics protocol
};

// How a frame's pixels are handed to the terminal. Kitty accepts both raw RGB
// (`f=24`) and a PNG bitstream (`f=100`); PNG spends sender CPU to roughly halve
// the bytes crossing the pty, which is the right trade over ssh and the wrong
// one on a local terminal that can absorb raw pixels faster than we can deflate
// them. Sixel has no format negotiation at all — 256-color palette plus RLE is
// its only encoding — so it is always kRawRgb.
enum class ImageTransfer {
  kRawRgb,  // Kitty f=24: base64 of packed RGB24, four wire bytes per pixel
  kPng,     // Kitty f=100: base64 of a PNG bitstream
};

// Pixel size of one character cell. Always strictly positive.
struct CellPixels
{
  int width = 0;
  int height = 0;
};

struct TerminalImageCaps
{
  ImageBackend backend = ImageBackend::kNone;
  CellPixels cell;
  ImageTransfer transfer = ImageTransfer::kRawRgb;

  [[nodiscard]] bool can_render() const noexcept { return backend != ImageBackend::kNone; }
};

// An explicit `BAGWIZ_WALK_PREVIEW_TRANSFER` setting. kAuto means "decide from
// the session", which is also what an unset variable yields.
enum class TransferOverride { kAuto, kRawRgb, kPng };

// True when this session's terminal sits on the far end of an ssh link, judged
// by the variables sshd puts in the login environment. Pure: the caller passes
// the values so the policy is testable without mutating the process
// environment. A defined-but-empty variable (`export SSH_TTY=`) does not count —
// sshd always sets a non-empty one.
[[nodiscard]] bool session_is_remote(const char * ssh_connection, const char * ssh_tty) noexcept;

// Parse a `BAGWIZ_WALK_PREVIEW_TRANSFER` value (case-insensitive): "auto",
// "raw", or "png". Unset (nullptr) or empty is kAuto. Returns nullopt when the
// variable is set to something unrecognised, so the caller can warn rather than
// let a typo silently read as "auto".
[[nodiscard]] std::optional<TransferOverride> parse_transfer_override(const char * value) noexcept;

// The transfer format to use: the override when one is given, else PNG on a
// remote session and raw RGB locally. Sixel (and kNone) is always kRawRgb — it
// cannot decode a PNG payload, so neither the heuristic nor an explicit
// override may hand it one. Pure.
[[nodiscard]] ImageTransfer preferred_transfer(
  ImageBackend backend, bool remote, TransferOverride override_value) noexcept;

// Per-cell pixel size derived from `term`. When the terminal reports pixel
// dimensions (xpixel/ypixel > 0) the cell size is xpixel/cols by ypixel/rows;
// otherwise it falls back to an assumed ~1:2 cell. Never returns a zero
// dimension.
[[nodiscard]] CellPixels cell_pixels(Size term) noexcept;

// Classify an accumulated query-reply byte string into a backend. Pure; exposed
// so the parser can be tested without real terminal I/O.
[[nodiscard]] ImageBackend classify_query_reply(std::string_view reply) noexcept;

// Probe the terminal exactly once, BEFORE the pager takes over stdin. Writes the
// graphics-capability query escapes to `out`, then reads replies from `in_fd`
// with a bounded timeout and fully drains them so no reply bytes leak into later
// key reads. `term` supplies the cell geometry. A terminal that answers neither
// query (or is not a TTY) yields kNone. Does not throw under normal operation.
[[nodiscard]] TerminalImageCaps detect_terminal_image_caps(
  std::ostream & out, int in_fd, Size term);

}  // namespace bagwiz::core::tui::image

#endif  // BAGWIZ__CORE__TUI__IMAGE__TERMINAL_IMAGE_CAPS_HPP_
