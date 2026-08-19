// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_HELP_HPP_
#define COMMANDS__WALK_HELP_HPP_

#include <string>
#include <vector>

// Key footers and the '?' key reference of the `bagwiz walk` interactive
// views. The footers advertise only the working set of the current mode —
// one row on a modest terminal — and every other binding is discoverable
// through the [?] overlay whose content the *_help_lines() builders return.
// Split out of the view renderers so the advertised keys can be unit-tested
// as pure strings, the same way walk_preview_legend once was. CLI-internal:
// this header lives with the command sources and is not installed.
namespace bagwiz::commands
{

// One-row footer of the YAML pager. `preview_available` gates the
// rainbow-painted [i] hint the way the pager gates the key: image topics on
// a graphics-capable terminal only.
[[nodiscard]] std::string yaml_footer_legend(bool preview_available);

// One-row footer of the image preview. A selected PointCloud2 topic adds
// the [e]/[P] entries that need an overlay to be useful; the extrinsic edit
// mode replaces the footer with its own working set instead of appending,
// so the nudge keys are not buried among navigation hints.
[[nodiscard]] std::string preview_footer_legend(bool pcd_topic_selected, bool edit_active);

// Full key reference of the YAML pager, one logical line per entry, grouped
// by section. The caller wraps the lines to the terminal width.
[[nodiscard]] std::vector<std::string> yaml_help_lines();

// Full key reference of the image preview, same shape as yaml_help_lines().
[[nodiscard]] std::vector<std::string> preview_help_lines();

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_HELP_HPP_
