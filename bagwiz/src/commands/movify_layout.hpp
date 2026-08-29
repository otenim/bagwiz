// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_LAYOUT_HPP_
#define COMMANDS__MOVIFY_LAYOUT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// The composed output frame of `movify`: a fixed grid of equally sized cells,
// one panel per cell, and the `--grid` value that lays it out. CLI-internal:
// this header lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Grid dimensions in cells. Panels fill the grid left to right, top to bottom;
// cells past the last panel stay black.
struct GridSpec
{
  std::uint32_t cols = 0;
  std::uint32_t rows = 0;
};

// The near-square default layout for a panel count: 2 panels -> 2x1, 3-4 ->
// 2x2, 5-6 -> 3x2, and so on.
[[nodiscard]] GridSpec auto_grid_spec(std::size_t panel_count);

struct GridParseResult
{
  GridSpec grid;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// Parse the --grid value "<cols>x<rows>". Empty `text` selects
// auto_grid_spec(panel_count). Errors: malformed text, a zero dimension, fewer
// cells than panels.
[[nodiscard]] GridParseResult parse_grid_spec(const std::string & text, std::size_t panel_count);

// A writable view of one grid cell inside the composed output frame.
struct CellView
{
  std::byte * data = nullptr;  // the cell's top-left pixel
  std::uint32_t width = 0;     // cell dimensions
  std::uint32_t height = 0;
  std::size_t stride = 0;  // the composed frame's row stride in bytes
};

// The composed multi-panel output frame: a fixed grid of equally sized cells,
// row-major, in packed BGR24. The cell size is fixed by the clock panel's
// first frame; the composed size (cols*cell_w x rows*cell_h) then never
// changes, which is what the video encoder's fixed-geometry requirement needs.
class GridCanvas
{
public:
  explicit GridCanvas(GridSpec grid) : grid_(grid) {}

  // Fix the cell size and allocate the composed buffer. Called once, on the
  // first tick.
  void set_cell_size(std::uint32_t w, std::uint32_t h);

  [[nodiscard]] bool ready() const { return !pixels_.empty(); }
  [[nodiscard]] std::uint32_t cell_width() const { return cell_w_; }
  [[nodiscard]] std::uint32_t cell_height() const { return cell_h_; }
  [[nodiscard]] std::uint32_t width() const { return grid_.cols * cell_w_; }
  [[nodiscard]] std::uint32_t height() const { return grid_.rows * cell_h_; }
  [[nodiscard]] GridSpec grid() const { return grid_; }

  // Black out the whole canvas for a new output frame.
  void clear();

  // Writable view of cell `index` (row-major; must be < cols*rows). Pure
  // arithmetic over the fixed geometry, so concurrent callers may take
  // distinct cells of one canvas at the same time.
  [[nodiscard]] CellView cell(std::size_t index);

  [[nodiscard]] const std::vector<std::byte> & pixels() const { return pixels_; }

private:
  GridSpec grid_;
  std::uint32_t cell_w_ = 0;
  std::uint32_t cell_h_ = 0;
  std::vector<std::byte> pixels_;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_LAYOUT_HPP_
