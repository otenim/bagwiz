// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_PANEL_HPP_
#define COMMANDS__MOVIFY_PANEL_HPP_

#include "movify_layout.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

// The unit `movify`'s encode loops compose: one panel per grid cell. The
// loops drive every panel through the same two steps per output tick —
// select the panel's input for the tick, then render it into the panel's
// cell — without knowing what the panel shows. The first panel is the clock:
// its topic's messages define the ticks, its first selection fixes the grid's
// cell size, and each tick hands it its own message payload. CLI-internal:
// this header lives with the command sources and is not installed.
namespace bagwiz::commands
{

// One output tick: the clock message's position in the stream (the output
// frame index, used in log lines), its bag record time — the time every other
// panel matches its own topic against — and its payload, valid for the
// duration of the tick's select() calls. Only the clock panel reads the
// payload.
struct TickInfo
{
  std::uint64_t index = 0;
  std::int64_t record_ns = 0;
  std::span<const std::byte> payload;
};

// A cell size in pixels.
struct PanelSize
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
};

// The clock role's cell-size inputs for a panel without a native pixel size
// (a point-cloud or map panel is drawn, not decoded): the --width the output
// is fixed to, if any, and the grid's column count it is split across.
struct SyntheticSizing
{
  std::optional<std::uint32_t> total_width;
  std::uint32_t grid_cols = 1;
};

// The cell such a panel renders into in the clock role: 16:9 at 1280x720,
// or the output width split across the grid columns at that aspect ratio,
// both rounded down to even (the codecs' 4:2:0 formats require even
// dimensions).
inline constexpr std::uint32_t kSyntheticCellWidth = 1280;
inline constexpr std::uint32_t kSyntheticCellHeight = 720;

[[nodiscard]] inline PanelSize synthetic_clock_cell(const SyntheticSizing & sizing)
{
  if (!sizing.total_width.has_value()) {
    return PanelSize{kSyntheticCellWidth, kSyntheticCellHeight};
  }
  const std::uint32_t cell_w = (*sizing.total_width / std::max(sizing.grid_cols, 1U)) & ~1U;
  const auto cell_h =
    static_cast<std::uint32_t>(
      std::lround(cell_w * (static_cast<double>(kSyntheticCellHeight) / kSyntheticCellWidth))) &
    ~1U;
  return PanelSize{cell_w, cell_h};
}

class Panel
{
public:
  Panel() = default;
  virtual ~Panel() = default;
  Panel(const Panel &) = delete;
  Panel & operator=(const Panel &) = delete;
  Panel(Panel &&) = delete;
  Panel & operator=(Panel &&) = delete;

  // Select this panel's input for `tick`: decode the clock payload (the clock
  // panel) or pick the panel's own message nearest the tick (any other
  // panel), and bring it to its render size. `cell` is the grid's cell size —
  // zero on the clock panel's first tick, whose selection is what fixes it.
  // Returns "" on success, or the error the loop logs with the tick's frame
  // index. A panel is driven by one thread at a time, but not always the
  // same thread.
  [[nodiscard]] virtual std::string select(const TickInfo & tick, PanelSize cell) = 0;

  // Clock panel only, after select(): the render size of the selected input,
  // which the first tick adopts as the grid's cell size. nullopt for every
  // other panel, and when nothing was selected.
  [[nodiscard]] virtual std::optional<PanelSize> clock_cell_size() const = 0;

  // Draw the selected input into `cell`, which was cleared to black. A panel
  // with nothing selected leaves the cell black and succeeds. Returns "" on
  // success, or the error the loop logs.
  [[nodiscard]] virtual std::string render(const CellView & cell) = 0;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_PANEL_HPP_
