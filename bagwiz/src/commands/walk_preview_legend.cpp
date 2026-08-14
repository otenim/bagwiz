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

std::string build_preview_legend(bool pcd_topic_selected)
{
  std::string legend =
    "  [→ / Space] next   [← / b] prev   [,] -1s   [.] +1s   [<] -10s   [>] +10s   [g] first "
    "  [G] last   [S] save   "
    "[u] rectify   [p] project pcd   [t] select pcd topics";
  if (pcd_topic_selected) {
    legend += "   [f] property   [c] scheme   [r] range   [= / -] size   [ [ / ] ] alpha";
  }
  legend += "   [q] back";
  return legend;
}

}  // namespace bagwiz::commands
