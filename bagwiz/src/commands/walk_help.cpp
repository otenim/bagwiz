// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_help.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::commands
{

namespace
{

// Paint `text` as a rainbow by assigning each character a standard ANSI
// foreground color in sequence. The returned string contains the SGR escapes
// and a trailing reset; width-aware code treats those escapes as zero-width,
// so wrapping/layout is unaffected.
std::string rainbow_text(std::string_view text)
{
  // Red, yellow, green, cyan, blue, magenta — a classic 6-step rainbow.
  constexpr const char * kColors[] = {"\x1B[31m", "\x1B[33m", "\x1B[32m",
                                      "\x1B[36m", "\x1B[34m", "\x1B[35m"};
  std::string out;
  out.reserve(text.size() * 6);
  for (std::size_t i = 0; i < text.size(); ++i) {
    out += kColors[i % std::size(kColors)];
    out.push_back(text[i]);
  }
  out += "\x1B[0m";
  return out;
}

}  // namespace

std::string yaml_footer_legend(bool preview_available)
{
  // Space/b and j/k are unlabeled: paging and scrolling are intuitive
  // enough, and the '?' reference still lists them.
  std::string legend = "  [S] save   ";
  if (preview_available) {
    legend += rainbow_text("[i] preview");
    legend += "   ";
  }
  legend += "[?] help   [q] quit";
  return legend;
}

std::string preview_footer_legend()
{
  // Space/b navigation is unlabeled here too — the working set starts at the
  // view toggles.
  return "  [u] rectify   [p] pcd overlay   [?] help   [q] back";
}

std::vector<std::string> yaml_help_lines()
{
  return {
    "Navigate",
    "  Space / →      next message",
    "  b / ←          previous message",
    "  . / ,          forward / back 1s",
    "  > / <          forward / back 10s",
    "  g / G          first / last message (G scans to the end)",
    "View",
    "  j / k          scroll down / up (also ↓ / ↑)",
    "  Home / H       top of the message",
    "  End / T        bottom of the message",
    "  a              expand long arrays",
    "  i              open the image preview (image topics only)",
    "Other",
    "  S              save the message as YAML",
    "  q              close this help",
    "  q              quit walk (Ctrl-C / Ctrl-D work too)",
  };
}

std::vector<std::string> preview_help_lines()
{
  return {
    "Navigate",
    "  Space / →      next frame",
    "  b / ←          previous frame",
    "  . / ,          forward / back 1s",
    "  > / <          forward / back 10s",
    "  g / G          first / last frame (G scans to the end)",
    "View",
    "  u              toggle rectify (needs camera_info)",
    "  S              save the frame as PNG",
    "PCD overlay",
    "  p              toggle the projection overlay",
    "  t              select cloud topics",
    "  f / c / r      cycle property / color scheme / value range",
    "  = / -          point size",
    "  ] / [          opacity",
    "Other",
    "  q              close this help / back to the YAML view",
    "  Ctrl-C / Ctrl-D  quit walk",
  };
}

}  // namespace bagwiz::commands
