// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_help.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <string>
#include <vector>

namespace bagwiz::commands
{

std::string yaml_footer_legend(bool preview_available)
{
  std::string legend = "  [Space/b] next/prev   [j/k] scroll   [S] save   ";
  if (preview_available) {
    legend += "[i] preview   ";
  }
  legend += "[?] keys   [q] quit";
  return legend;
}

std::string preview_footer_legend(bool pcd_topic_selected, bool edit_active)
{
  if (edit_active) {
    // The nudge keys are the working set here; navigation and the overlay
    // toggles keep working but live behind [?] while the mode is on.
    return "  [x/y/z] move   [l/n/w] rotate   [m] step   [0] reset   [A] apply   [D] yaml"
           "   [e] done   [?] keys";
  }
  std::string legend = "  [Space/b] next/prev   [u] rectify   [p] pcd   ";
  if (pcd_topic_selected) {
    // The edit mode and the scene pins judge a projection against the image,
    // so their entry hints wait for an overlay topic like the keys do.
    legend += "[e] edit   [P] pin   ";
  }
  legend += "[?] keys   [q] back";
  return legend;
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
    "  ?              close this help",
    "  q              quit",
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
    "  P              pin / unpin the scene as a compare tile",
    "Edit extrinsic ([e] enters; needs the overlay)",
    "  x/X y/Y z/Z    nudge translation",
    "  l/L n/N w/W    nudge roll / pitch / yaw",
    "  m / M          step size",
    "  0              reset the edge to the bag value",
    "  E              choose the edited edge",
    "  D              export the edits as static-TF YAML",
    "  A              apply the edits to the input bag (asks first)",
    "Other",
    "  ?              close this help",
    "  q              back to the YAML view",
  };
}

}  // namespace bagwiz::commands
