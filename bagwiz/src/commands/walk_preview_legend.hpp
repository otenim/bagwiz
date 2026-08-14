// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_PREVIEW_LEGEND_HPP_
#define COMMANDS__WALK_PREVIEW_LEGEND_HPP_

#include <string>

// Key legend of the `bagwiz walk` image preview. Split out of the preview
// renderer so the advertised keys can be unit-tested on their own: the
// renderer is a private member of ImagePreviewSession and needs a live
// cursor, pager and TTY, while the legend is a pure string. CLI-internal:
// this header lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Build the preview's key-legend line, before it is wrapped to the terminal
// width. The pcd overlay adjustment keys (f/c/r/=/-/[/]) are only meaningful
// once a PointCloud2 topic is selected, so they appear only when
// `pcd_topic_selected`; the toggle/select keys ([p]/[t]) stay visible
// unconditionally to guide the user to enable the overlay in the first place.
[[nodiscard]] std::string build_preview_legend(bool pcd_topic_selected);

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_PREVIEW_LEGEND_HPP_
