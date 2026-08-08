// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_odometry_grouping.hpp"  // NOLINT(build/include_subdir) src-local header

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{

GroupingBuffer::GroupingBuffer(Config config) : config_(config)
{
  last_stamp_.assign(config_.camera_count, std::numeric_limits<std::int64_t>::min());
  // Fail fast on an impossible rig description: insert() indexes
  // last_stamp_ by anchor_camera_id unguarded, so an out-of-range anchor
  // would be UB on the very first call.
  if (
    config_.camera_count == 0 || config_.anchor_camera_id < 0 ||
    static_cast<std::size_t>(config_.anchor_camera_id) >= config_.camera_count) {
    throw std::invalid_argument("GroupingBuffer: anchor_camera_id must index a camera");
  }
}

void GroupingBuffer::insert(std::span<const VisualObservation> observations)
{
  // Pass 1: anchors first, so windows opened by this batch can receive this
  // batch's own non-anchor observations. An observation whose camera_id has
  // no head slot cannot gate pops, so honoring it would reintroduce
  // arrival-order-dependent drops for that stream — refuse it (counted) here
  // rather than mis-model it.
  for (const auto & o : observations) {
    if (o.camera_id < 0 || static_cast<std::size_t>(o.camera_id) >= last_stamp_.size()) {
      continue;  // counted in pass 2
    }
    if (o.camera_id == config_.anchor_camera_id) {
      groups_[o.stamp_ns].anchor_stamp_ns = o.stamp_ns;
      groups_[o.stamp_ns].observations.push_back(o);
    }
    if (o.stamp_ns > last_stamp_[o.camera_id]) {
      last_stamp_[o.camera_id] = o.stamp_ns;
    }
  }
  const std::int64_t anchor_head = last_stamp_[config_.anchor_camera_id];
  // Pass 2: non-anchor observations. Assignable only once the anchor stream
  // has reached the observation's stamp — anchors arrive in stamp order, so
  // only then is the covering window guaranteed known.
  for (const auto & o : observations) {
    if (o.camera_id < 0 || static_cast<std::size_t>(o.camera_id) >= last_stamp_.size()) {
      ++dropped_;
      continue;
    }
    if (o.camera_id == config_.anchor_camera_id) {
      continue;
    }
    if (o.stamp_ns <= anchor_head) {
      assign(o);
    } else {
      pending_.push_back(o);
    }
  }
  drain_pending(anchor_head);
}

void GroupingBuffer::note_frame(std::int32_t camera_id, std::int64_t stamp_ns)
{
  // Per-frame heartbeat: a frame that yielded no observations (decode
  // failure, textureless view, all tracks lost) still proves its camera's
  // stream has reached stamp_ns, so the head advances and the other cameras'
  // windows keep releasing. Without this, one camera with systematically
  // undecodable payloads would defer every group to finish() (unbounded
  // buffering; see the pop_ready() comment). Heartbeats never open windows —
  // a featureless ANCHOR frame leaves its would-be window uncovered exactly
  // like an anchor frame drop, and is counted the same way.
  if (camera_id < 0 || static_cast<std::size_t>(camera_id) >= last_stamp_.size()) {
    return;
  }
  if (stamp_ns > last_stamp_[camera_id]) {
    last_stamp_[camera_id] = stamp_ns;
  }
  // An advanced ANCHOR head makes pending observations assignable; drain now,
  // before any pop_ready() can close the windows they belong to. Draining
  // only on insert() would let a heartbeat-closed window pop without its
  // pending observations — the old escape hatch's data loss, resurrected.
  drain_pending(last_stamp_[config_.anchor_camera_id]);
}

void GroupingBuffer::drain_pending(std::int64_t anchor_head)
{
  // Drain pending entries the anchor head has caught up with. In-place stable
  // compaction: this runs on every insert (under the caller's feed mutex), so
  // rebuilding the vector each time would pay an allocation + full copy per
  // camera batch for entries that mostly stay pending.
  std::size_t kept = 0;
  for (const VisualObservation & o : pending_) {
    if (o.stamp_ns <= anchor_head) {
      assign(o);
    } else {
      pending_[kept++] = o;
    }
  }
  pending_.resize(kept);
}

void GroupingBuffer::assign(const VisualObservation & o)
{
  // Newest anchor window at or before the stamp; drop when it does not
  // cover it (anchor-camera frame drop) or was already popped — which, with
  // pops gated on every camera's head, only a stamp-non-decreasing contract
  // violation can produce.
  auto it = groups_.upper_bound(o.stamp_ns);
  if (it == groups_.begin()) {
    ++dropped_;
    return;
  }
  --it;
  if (o.stamp_ns >= it->first + config_.period_ns || it->first <= popped_until_) {
    ++dropped_;
    return;
  }
  it->second.observations.push_back(o);
}

std::vector<ObservationGroup> GroupingBuffer::pop_ready()
{
  std::int64_t min_head = std::numeric_limits<std::int64_t>::max();
  for (const std::int64_t s : last_stamp_) {
    min_head = std::min(min_head, s);
  }
  // Ready only when every camera has passed the window end — the sole
  // condition under which the window's content is provably complete. Any
  // release keyed to how far the FASTEST camera ran (the old max_lag escape
  // hatch) turns cross-camera thread skew into data loss (issue #16). The
  // cost of that guarantee: a camera whose head stops advancing (a truly
  // message-less topic — per-frame heartbeats cover every other case, see
  // note_frame) defers the remaining groups to finish(), buffering them for
  // the rest of the bag. Acceptable for an offline tool; do not "fix" it by
  // reintroducing a time-based release.
  bool any_ready = false;
  std::int64_t limit = std::numeric_limits<std::int64_t>::min();
  for (const auto & kv : groups_) {
    const std::int64_t anchor = kv.first;
    if (min_head >= anchor + config_.period_ns) {
      any_ready = true;
      limit = anchor;
    } else {
      break;  // groups_ is anchor-ordered; later windows end later
    }
  }
  // any_ready (not a sentinel compare) so a window legitimately anchored at
  // INT64_MIN — the "nothing ready" sentinel's own value — cannot leak out.
  if (!any_ready) {
    return {};
  }
  return take_groups_up_to(limit);
}

std::vector<ObservationGroup> GroupingBuffer::finish()
{
  // No more anchors will arrive: resolve every pending observation with full
  // knowledge, then flush all remaining groups in order.
  for (const auto & o : pending_) {
    assign(o);
  }
  pending_.clear();
  return take_groups_up_to(std::numeric_limits<std::int64_t>::max());
}

std::vector<ObservationGroup> GroupingBuffer::take_groups_up_to(std::int64_t anchor_limit)
{
  std::vector<ObservationGroup> out;
  auto it = groups_.begin();
  while (it != groups_.end() && it->first <= anchor_limit) {
    popped_until_ = it->first;
    out.push_back(std::move(it->second));
    it = groups_.erase(it);
  }
  return out;
}

std::int64_t GroupingBuffer::dropped_count() const
{
  return dropped_;
}

}  // namespace bagwiz::core::slam
