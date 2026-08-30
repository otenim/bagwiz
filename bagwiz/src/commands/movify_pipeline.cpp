// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_pipeline.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "movify_output_size.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <fmt/core.h>

#include <cinttypes>
#include <exception>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.movify";

// Below this many frames the parallel pipeline cannot hide the per-tick job
// launch overhead.
constexpr std::uint64_t kThreadingMinFrames = 4;

// Report a composed output large enough that the user probably did not intend
// it. Called once per run, at the point the first tick fixes the cell size and
// with it the output resolution for the rest of the encode — the earliest
// moment it is knowable, since the pre-flight checks never decode a frame. A
// single panel names no grid: "1x1 grid of WxH cells" would only restate the
// size that precedes it.
void warn_if_output_oversized(const GridCanvas & canvas)
{
  const GridSpec grid = canvas.grid();
  const std::string detail = (grid.cols * grid.rows > 1U)
                               ? fmt::format(
                                   "{}x{} grid of {}x{} cells", grid.cols, grid.rows,
                                   canvas.cell_width(), canvas.cell_height())
                               : std::string{};
  const auto warning = oversized_output_warning(
    canvas.width(), canvas.height(), detail,
    "Pass --width to cap the output width, or --resize to scale the cells down.");
  if (warning.has_value()) {
    BAGWIZ_LOG_WARN(kLogger, "%s", warning->c_str());
  }
}

[[nodiscard]] PanelSize cell_size_of(const GridCanvas & canvas)
{
  return PanelSize{canvas.cell_width(), canvas.cell_height()};
}

// The clock panel's first selection fixes the grid's cell size — and with it
// the composed output size for the whole run. Returns "" on success, else the
// error to log.
[[nodiscard]] std::string pin_cell_size(const Panel & clock, GridCanvas & canvas)
{
  const auto size = clock.clock_cell_size();
  if (!size.has_value()) {
    return "internal error — the clock panel selected no frame on the first tick";
  }
  canvas.set_cell_size(size->width, size->height);
  warn_if_output_oversized(canvas);
  return "";
}

int run_encode_loop_sync(
  io::BagReader & reader, std::vector<std::unique_ptr<Panel>> & panels, GridSpec grid,
  std::size_t clock, VideoFrameEncoder & encoder)
{
  GridCanvas canvas(grid);
  io::RawMessage raw;
  std::uint64_t tick = 0;
  while (reader.next(raw)) {
    const TickInfo info{tick, raw.timestamp_ns, raw.payload};
    // The clock panel selects first: on the first tick its render size is
    // what fixes the cell every other panel fits into.
    if (const auto err = panels[clock]->select(info, cell_size_of(canvas)); !err.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", tick, err.c_str());
      return 1;
    }
    if (!canvas.ready()) {
      if (const auto err = pin_cell_size(*panels[clock], canvas); !err.empty()) {
        BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", tick, err.c_str());
        return 1;
      }
    }
    for (std::size_t i = 0; i < panels.size(); ++i) {
      if (i == clock) {
        continue;
      }
      if (const auto err = panels[i]->select(info, cell_size_of(canvas)); !err.empty()) {
        BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", tick, err.c_str());
        return 1;
      }
    }
    canvas.clear();
    for (std::size_t i = 0; i < panels.size(); ++i) {
      if (const auto err = panels[i]->render(canvas.cell(i)); !err.empty()) {
        BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", tick, err.c_str());
        return 1;
      }
    }
    if (!encoder.encode(canvas.pixels(), canvas.width(), canvas.height())) {
      return 1;
    }
    ++tick;
  }
  return 0;
}

int run_encode_loop_parallel(
  io::BagReader & reader, std::vector<std::unique_ptr<Panel>> & panels, GridSpec grid,
  std::size_t clock, const std::string & clock_topic, VideoFrameEncoder & encoder)
{
  // Two canvases in alternation: one being composed by the current tick's
  // jobs, one being drained by the encoder for the previous tick. A tick's
  // jobs never touch the canvas the encoder reads.
  GridCanvas canvases[2] = {GridCanvas(grid), GridCanvas(grid)};

  // One panel's work for one tick, run on a worker thread: select the panel's
  // input (unless the caller already did, for the clock panel's first tick)
  // and render it into the panel's cell of `canvas`. Returns "" on success,
  // else the error the main thread logs with the tick's frame index.
  auto run_panel_job = [&panels](
                         std::size_t index, GridCanvas & canvas, const TickInfo & info,
                         bool select) -> std::string {
    try {
      if (select) {
        if (const auto err = panels[index]->select(info, cell_size_of(canvas)); !err.empty()) {
          return err;
        }
      }
      return panels[index]->render(canvas.cell(index));
    } catch (const std::exception & e) {
      return std::string("view render failed: ") + e.what();
    }
  };

  io::RawMessage raw;
  if (!reader.next(raw)) {
    BAGWIZ_LOG_ERROR(
      kLogger, "topic '%s' yielded no frames in the encode pass.", clock_topic.c_str());
    return 1;
  }

  std::vector<std::optional<std::future<std::string>>> pending(panels.size());

  // Tick 0: the clock panel's selection fixes the grid's cell size, so it runs
  // first and completes before any other panel's job starts. The payload is
  // copied out of the reader (its span does not outlive the next read) and
  // shared with the jobs, which outlive this block.
  {
    auto payload =
      std::make_shared<const std::vector<std::byte>>(raw.payload.begin(), raw.payload.end());
    const TickInfo info{0, raw.timestamp_ns, *payload};
    if (const auto err = panels[clock]->select(info, PanelSize{}); !err.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "frame 0: %s", err.c_str());
      return 1;
    }
    if (const auto err = pin_cell_size(*panels[clock], canvases[0]); !err.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "frame 0: %s", err.c_str());
      return 1;
    }
    // The cell size is fixed for the whole run; the second canvas renders its
    // first tick before it would otherwise learn the size.
    canvases[1].set_cell_size(canvases[0].cell_width(), canvases[0].cell_height());
    canvases[0].clear();
    pending[clock] = std::async(std::launch::async, [&, payload, info] {
      return run_panel_job(clock, canvases[0], info, false);
    });
    for (std::size_t i = 0; i < panels.size(); ++i) {
      if (i == clock) {
        continue;
      }
      pending[i] = std::async(std::launch::async, [&, i, payload, info] {
        return run_panel_job(i, canvases[0], info, true);
      });
    }
  }

  // Wait for the current tick's jobs; on the first error (lowest panel index)
  // log it and stop. Every job is always collected, so an error return never
  // leaves a job running past the locals it captured by reference.
  std::uint64_t tick = 0;
  auto wait_jobs = [&]() -> bool {
    std::string first_error;
    for (auto & job : pending) {
      if (!job.has_value()) {
        continue;
      }
      const std::string error = job->get();
      job.reset();
      if (first_error.empty()) {
        first_error = error;
      }
    }
    if (!first_error.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", tick, first_error.c_str());
      return false;
    }
    return true;
  };

  while (true) {
    if (!wait_jobs()) {
      return 1;
    }
    io::RawMessage next_raw;
    if (!reader.next(next_raw)) {
      break;
    }
    const std::uint64_t next_tick = tick + 1;
    GridCanvas & next_canvas = canvases[next_tick % 2];
    // Clear before launch: this canvas's previous tick finished two iterations
    // ago — its jobs were waited and its frame encoded.
    next_canvas.clear();
    auto payload = std::make_shared<const std::vector<std::byte>>(
      next_raw.payload.begin(), next_raw.payload.end());
    const TickInfo info{next_tick, next_raw.timestamp_ns, *payload};
    for (std::size_t i = 0; i < panels.size(); ++i) {
      pending[i] = std::async(std::launch::async, [&, i, payload, info] {
        return run_panel_job(i, next_canvas, info, true);
      });
    }
    // Encode the finished tick's canvas while the new tick's jobs run.
    const GridCanvas & done_canvas = canvases[tick % 2];
    if (!encoder.encode(done_canvas.pixels(), done_canvas.width(), done_canvas.height())) {
      // Drain the in-flight jobs before returning (see wait_jobs).
      for (auto & job : pending) {
        if (job.has_value()) {
          (void)job->get();
        }
      }
      return 1;
    }
    tick = next_tick;
  }
  // The last tick was never encoded inside the loop.
  if (!wait_jobs()) {
    return 1;
  }
  const GridCanvas & last_canvas = canvases[tick % 2];
  return encoder.encode(last_canvas.pixels(), last_canvas.width(), last_canvas.height()) ? 0 : 1;
}

}  // namespace

bool should_use_parallel_pipeline(
  std::size_t panel_count, bool has_pointcloud_topics, bool enable_parallel,
  std::uint64_t frame_count, unsigned int hardware_concurrency)
{
  // Even a single panel gains from the parallel loop: its decode runs on a
  // worker while the main thread encodes the previous tick's frame.
  (void)panel_count;
  (void)has_pointcloud_topics;
  return enable_parallel && frame_count >= kThreadingMinFrames && hardware_concurrency > 1;
}

int run_encode_pass(
  io::BagReader & reader, std::vector<std::unique_ptr<Panel>> & panels, GridSpec grid,
  bool parallel, std::size_t clock, const std::string & clock_topic, VideoFrameEncoder & encoder)
{
  try {
    if (parallel) {
      return run_encode_loop_parallel(reader, panels, grid, clock, clock_topic, encoder);
    }
    return run_encode_loop_sync(reader, panels, grid, clock, encoder);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "error reading topic '%s': %s", clock_topic.c_str(), e.what());
    return 1;
  }
}

}  // namespace bagwiz::commands
