// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_layout.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <charconv>
#include <string>

namespace bagwiz::commands
{

GridSpec auto_grid_spec(std::size_t panel_count)
{
  std::uint32_t cols = 1;
  while (static_cast<std::uint64_t>(cols) * cols < panel_count) {
    ++cols;
  }
  const auto rows = static_cast<std::uint32_t>((panel_count + cols - 1) / cols);
  return GridSpec{cols, rows};
}

GridParseResult parse_grid_spec(const std::string & text, std::size_t panel_count)
{
  if (text.empty()) {
    return GridParseResult{auto_grid_spec(panel_count), ""};
  }
  const auto malformed = [&text]() {
    return GridParseResult{{}, "--grid: expected <cols>x<rows> (e.g. 2x2), got '" + text + "'"};
  };
  const auto x = text.find('x');
  if (x == std::string::npos || x == 0 || x + 1 >= text.size()) {
    return malformed();
  }
  const std::string cols_text = text.substr(0, x);
  const std::string rows_text = text.substr(x + 1);
  const auto digits = [](const std::string & s) {
    return !s.empty() &&
           std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; });
  };
  if (!digits(cols_text) || !digits(rows_text)) {
    return malformed();
  }
  std::uint32_t cols = 0;
  std::uint32_t rows = 0;
  std::from_chars(cols_text.data(), cols_text.data() + cols_text.size(), cols);
  std::from_chars(rows_text.data(), rows_text.data() + rows_text.size(), rows);
  if (cols == 0 || rows == 0) {
    return GridParseResult{{}, "--grid: both dimensions must be positive (got '" + text + "')"};
  }
  const std::uint64_t cells = static_cast<std::uint64_t>(cols) * rows;
  if (cells < panel_count) {
    return GridParseResult{
      {},
      "--grid '" + text + "' provides " + std::to_string(cells) + " cell(s) for " +
        std::to_string(panel_count) + " view(s)"};
  }
  return GridParseResult{GridSpec{cols, rows}, ""};
}

void GridCanvas::set_cell_size(std::uint32_t w, std::uint32_t h)
{
  cell_w_ = w;
  cell_h_ = h;
  pixels_.assign(
    static_cast<std::size_t>(grid_.cols) * cell_w_ * 3U * grid_.rows * cell_h_, std::byte{0});
}

void GridCanvas::clear()
{
  std::fill(pixels_.begin(), pixels_.end(), std::byte{0});
}

CellView GridCanvas::cell(std::size_t index)
{
  const std::size_t stride = static_cast<std::size_t>(width()) * 3U;
  const std::size_t col = index % grid_.cols;
  const std::size_t row = index / grid_.cols;
  return CellView{
    pixels_.data() + row * cell_h_ * stride + col * cell_w_ * 3U, cell_w_, cell_h_, stride};
}

}  // namespace bagwiz::commands
