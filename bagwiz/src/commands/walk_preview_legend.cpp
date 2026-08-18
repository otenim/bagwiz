// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_preview_legend.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <string>

namespace bagwiz::commands
{

std::string build_preview_legend(
  bool pcd_topic_selected, bool edit_active, bool current_frame_pinned)
{
  std::string legend =
    "  [→ / Space] next   [← / b] prev   [,] -1s   [.] +1s   [<] -10s   [>] +10s   [g] first "
    "  [G] last   [S] save   "
    "[u] rectify   [p] project pcd   [t] select pcd topics";
  if (pcd_topic_selected) {
    legend += "   [f] property   [c] scheme   [r] range   [= / -] size   [ [ / ] ] alpha";
    // Pinning judges one projection against several scenes, so it needs an
    // overlay topic just like the adjustment keys above. The hint names the
    // direction [P] will take on the frame currently shown.
    legend += current_frame_pinned ? "   [P] unpin scene" : "   [P] pin scene";
  }
  if (edit_active) {
    legend +=
      "   [x/X y/Y z/Z] translate   [l/L n/N w/W] roll/pitch/yaw   [m/M] step   [0] reset"
      "   [E] edge   [D] export yaml   [A] apply to bag   [e] done";
  } else if (pcd_topic_selected) {
    // The edit mode needs an overlay to judge the alignment against, so its
    // entry hint rides the same condition as the adjustment keys.
    legend += "   [e] edit extrinsic";
  }
  legend += "   [q] back";
  return legend;
}

}  // namespace bagwiz::commands
