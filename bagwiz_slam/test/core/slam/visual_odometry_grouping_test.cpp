// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_odometry_grouping.hpp"  // NOLINT(build/include_subdir) src-local header

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;

slam::VisualObservation obs(std::int32_t cam, std::uint64_t track, std::int64_t stamp_ns)
{
  slam::VisualObservation o;
  o.camera_id = cam;
  o.track_id = track;
  o.stamp_ns = stamp_ns;
  return o;
}

constexpr std::int64_t kPeriod = 100'000'000;  // 100 ms

slam::GroupingBuffer::Config three_cameras()
{
  slam::GroupingBuffer::Config cfg;
  cfg.anchor_camera_id = 0;
  cfg.period_ns = kPeriod;
  cfg.camera_count = 3;
  return cfg;
}

TEST(GroupingBuffer, StaggeredObservationsJoinTheirAnchorWindow)
{
  slam::GroupingBuffer buf(three_cameras());
  // Anchor frames at 0 and 100 ms; camera 1 staggered +30 ms, camera 2 +79 ms
  // (the validation rigs' worst spread) — all must join the window their
  // stamp falls in, not the nearest cluster.
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, 0), obs(0, 2, 0)});
  buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, 30'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(2, 1, 79'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, kPeriod)});
  buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, kPeriod + 30'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(2, 1, kPeriod + 79'000'000)});

  // Every camera has advanced past window [0, 100ms) only after the second
  // round of frames above.
  const auto ready = buf.pop_ready();
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_EQ(ready[0].anchor_stamp_ns, 0);
  EXPECT_EQ(ready[0].observations.size(), 4U);  // 2 anchor obs + cam1 + cam2

  const auto rest = buf.finish();
  ASSERT_EQ(rest.size(), 1U);
  EXPECT_EQ(rest[0].anchor_stamp_ns, kPeriod);
  EXPECT_EQ(rest[0].observations.size(), 3U);
  EXPECT_EQ(buf.dropped_count(), 0);
}

TEST(GroupingBuffer, SimultaneousTriggersAreTheDegenerateCase)
{
  slam::GroupingBuffer buf(three_cameras());
  for (std::int64_t k = 0; k < 3; ++k) {
    buf.insert(
      std::vector<slam::VisualObservation>{
        obs(0, 1, k * kPeriod), obs(1, 1, k * kPeriod), obs(2, 1, k * kPeriod)});
  }
  auto groups = buf.pop_ready();
  const auto rest = buf.finish();
  groups.insert(groups.end(), rest.begin(), rest.end());
  ASSERT_EQ(groups.size(), 3U);
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_EQ(groups[k].anchor_stamp_ns, static_cast<std::int64_t>(k) * kPeriod);
    EXPECT_EQ(groups[k].observations.size(), 3U);
  }
  EXPECT_EQ(buf.dropped_count(), 0);
}

TEST(GroupingBuffer, ObservationBeforeItsAnchorArrivesIsHeldNotDropped)
{
  slam::GroupingBuffer buf(three_cameras());
  // Camera 1's frame for window [100ms, 200ms) arrives BEFORE the anchor
  // frame that opens the window (cross-camera arrival order is arbitrary).
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, 0)});
  buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, 130'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, kPeriod)});
  const auto groups = buf.finish();
  ASSERT_EQ(groups.size(), 2U);
  EXPECT_EQ(groups[1].anchor_stamp_ns, kPeriod);
  EXPECT_EQ(groups[1].observations.size(), 2U);  // anchor obs + held camera-1 obs
  EXPECT_EQ(buf.dropped_count(), 0);
}

TEST(GroupingBuffer, AnchorFrameDropDropsUncoveredObservations)
{
  slam::GroupingBuffer buf(three_cameras());
  // Anchor frames at 0 and 300 ms (frames at 100/200 ms dropped). Camera 1
  // observations in the uncovered gap must be dropped and counted, and one
  // in a covered window must survive.
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, 0)});
  buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, 150'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(1, 2, 330'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, 3 * kPeriod)});
  const auto groups = buf.finish();
  ASSERT_EQ(groups.size(), 2U);
  EXPECT_EQ(groups[0].observations.size(), 1U);
  EXPECT_EQ(groups[1].observations.size(), 2U);
  EXPECT_EQ(buf.dropped_count(), 1);
}

TEST(GroupingBuffer, CamerasSkewedBeyondFivePeriodsDropNothing)
{
  slam::GroupingBuffer buf(three_cameras());
  // The production failure mode (issue #16): per-camera worker threads let one
  // camera trail the others by many periods. Cameras 0/1 advance 10 windows
  // before camera 2 delivers a single frame; every camera-2 observation still
  // lands in its stamp's window — lag must never cost data.
  for (std::int64_t k = 0; k < 10; ++k) {
    buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, k * kPeriod)});
    buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, k * kPeriod + 30'000'000)});
    const auto popped = buf.pop_ready();
    EXPECT_TRUE(popped.empty());  // camera 2's head has not passed any window end
  }
  for (std::int64_t k = 0; k < 10; ++k) {
    buf.insert(std::vector<slam::VisualObservation>{obs(2, 1, k * kPeriod + 50'000'000)});
  }
  auto groups = buf.pop_ready();
  const auto rest = buf.finish();
  groups.insert(groups.end(), rest.begin(), rest.end());
  ASSERT_EQ(groups.size(), 10U);
  for (std::size_t k = 0; k < groups.size(); ++k) {
    EXPECT_EQ(groups[k].anchor_stamp_ns, static_cast<std::int64_t>(k) * kPeriod);
    EXPECT_EQ(groups[k].observations.size(), 3U);
  }
  EXPECT_EQ(buf.dropped_count(), 0);
}

// A group's observations sorted into a stamp-independent canonical order, so
// group content can be compared across insertion interleavings (arrival order
// within a group legitimately differs).
std::vector<std::tuple<std::int32_t, std::uint64_t, std::int64_t>> canonical(
  const slam::ObservationGroup & group)
{
  std::vector<std::tuple<std::int32_t, std::uint64_t, std::int64_t>> out;
  out.reserve(group.observations.size());
  for (const auto & o : group.observations) {
    out.emplace_back(o.camera_id, o.track_id, o.stamp_ns);
  }
  std::sort(out.begin(), out.end());
  return out;
}

TEST(GroupingBuffer, InterleavingOrderDoesNotChangeGroupsOrDrops)
{
  // One fixed observation set — 3 cameras x 8 windows, staggered +30/+79 ms —
  // fed under several interleavings that each keep the per-camera streams
  // stamp-non-decreasing (the class's only ordering contract). Grouping is
  // pure stamp arithmetic, so every interleaving must yield identical groups
  // and zero drops.
  constexpr std::int64_t kWindows = 8;
  const std::array<std::int64_t, 3> offsets{0, 30'000'000, 79'000'000};
  const auto observation = [&](std::int32_t cam, std::int64_t k) {
    return obs(cam, 1, k * kPeriod + offsets[static_cast<std::size_t>(cam)]);
  };

  // Each order is a list of (camera, window) insertions.
  using Feed = std::vector<std::pair<std::int32_t, std::int64_t>>;
  std::vector<Feed> orders;
  Feed round_robin;
  Feed cam_major_012;
  Feed cam_major_210;
  Feed anchor_last;
  for (std::int64_t k = 0; k < kWindows; ++k) {
    for (std::int32_t cam = 0; cam < 3; ++cam) {
      round_robin.emplace_back(cam, k);
    }
  }
  for (std::int32_t cam = 0; cam < 3; ++cam) {
    for (std::int64_t k = 0; k < kWindows; ++k) {
      cam_major_012.emplace_back(cam, k);
      cam_major_210.emplace_back(2 - cam, k);
    }
  }
  for (std::int64_t k = 0; k < kWindows; ++k) {
    anchor_last.emplace_back(1, k);
    anchor_last.emplace_back(2, k);
  }
  for (std::int64_t k = 0; k < kWindows; ++k) {
    anchor_last.emplace_back(0, k);
  }
  orders.push_back(std::move(round_robin));
  orders.push_back(std::move(cam_major_012));
  orders.push_back(std::move(cam_major_210));
  orders.push_back(std::move(anchor_last));

  std::vector<std::vector<std::tuple<std::int32_t, std::uint64_t, std::int64_t>>> reference;
  std::vector<std::int64_t> reference_anchors;
  for (std::size_t i = 0; i < orders.size(); ++i) {
    slam::GroupingBuffer buf(three_cameras());
    std::vector<slam::ObservationGroup> groups;
    // pop_ready() after every insert, mirroring the production feed loop.
    for (const auto & [cam, k] : orders[i]) {
      buf.insert(std::vector<slam::VisualObservation>{observation(cam, k)});
      auto popped = buf.pop_ready();
      groups.insert(
        groups.end(), std::make_move_iterator(popped.begin()),
        std::make_move_iterator(popped.end()));
    }
    auto rest = buf.finish();
    groups.insert(
      groups.end(), std::make_move_iterator(rest.begin()), std::make_move_iterator(rest.end()));

    EXPECT_EQ(buf.dropped_count(), 0) << "order " << i;
    ASSERT_EQ(groups.size(), static_cast<std::size_t>(kWindows)) << "order " << i;
    if (i == 0) {
      for (const auto & group : groups) {
        reference_anchors.push_back(group.anchor_stamp_ns);
        reference.push_back(canonical(group));
      }
      continue;
    }
    for (std::size_t g = 0; g < groups.size(); ++g) {
      EXPECT_EQ(groups[g].anchor_stamp_ns, reference_anchors[g]) << "order " << i;
      EXPECT_EQ(canonical(groups[g]), reference[g]) << "order " << i << " group " << g;
    }
  }
}

TEST(GroupingBuffer, SilentCameraDefersGroupsToFinish)
{
  slam::GroupingBuffer buf(three_cameras());
  // Camera 2 never delivers. No window may release early — a window's content
  // is only provably complete once EVERY camera's head has passed its end, so
  // the groups defer to finish(), where no more observations can arrive.
  for (std::int64_t k = 0; k < 10; ++k) {
    buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, k * kPeriod)});
    buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, k * kPeriod + 30'000'000)});
  }
  EXPECT_TRUE(buf.pop_ready().empty());
  const auto groups = buf.finish();
  ASSERT_EQ(groups.size(), 10U);
  for (const auto & group : groups) {
    EXPECT_EQ(group.observations.size(), 2U);  // anchor + camera 1
  }
  EXPECT_EQ(buf.dropped_count(), 0);
}

TEST(GroupingBuffer, NonMonotoneLateObservationForPoppedGroupIsDropped)
{
  slam::GroupingBuffer buf(three_cameras());
  // Window 0 pops legitimately: every camera's head passes its end.
  for (std::int64_t k = 0; k < 2; ++k) {
    buf.insert(
      std::vector<slam::VisualObservation>{
        obs(0, 1, k * kPeriod), obs(1, 1, k * kPeriod + 30'000'000),
        obs(2, 1, k * kPeriod + 79'000'000)});
  }
  const auto ready = buf.pop_ready();
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_EQ(ready[0].anchor_stamp_ns, 0);
  // Camera 2's stream then REGRESSES to a stamp inside the popped window — a
  // violation of the per-camera monotonicity contract. The observation cannot
  // be honored (its group is gone); it must be dropped and counted, never
  // resurrected into a later window.
  buf.insert(std::vector<slam::VisualObservation>{obs(2, 1, 50'000'000)});
  EXPECT_EQ(buf.dropped_count(), 1);
}

TEST(GroupingBuffer, NoteFrameAdvancesHeadsSoFramesWithoutObservationsDoNotStall)
{
  slam::GroupingBuffer buf(three_cameras());
  // Camera 2 produces frames but zero observations (undecodable payloads or a
  // textureless view). Its per-frame heartbeats must advance its head so the
  // other cameras' windows keep releasing incrementally instead of deferring
  // the whole run to finish().
  for (std::int64_t k = 0; k < 3; ++k) {
    buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, k * kPeriod)});
    buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, k * kPeriod + 30'000'000)});
    buf.note_frame(2, k * kPeriod + 79'000'000);
  }
  const auto ready = buf.pop_ready();
  ASSERT_EQ(ready.size(), 2U);  // windows 0 and 1: every head passed their ends
  EXPECT_EQ(ready[0].anchor_stamp_ns, 0);
  EXPECT_EQ(ready[0].observations.size(), 2U);  // anchor + camera 1, no camera-2 obs
  EXPECT_EQ(ready[1].anchor_stamp_ns, kPeriod);
  const auto rest = buf.finish();
  ASSERT_EQ(rest.size(), 1U);
  EXPECT_EQ(buf.dropped_count(), 0);
}

TEST(GroupingBuffer, NoteFrameOnAnchorDrainsPendingBeforeItsWindowCanPop)
{
  slam::GroupingBuffer buf(three_cameras());
  // Camera 1's observation for window [0, 100ms) waits pending on the anchor
  // head. The anchor's NEXT frame is featureless — only a heartbeat. That
  // heartbeat is what closes the window (all heads pass its end), so it must
  // drain the pending observation into the window first; popping without the
  // drain would lose it exactly the way the old escape hatch did.
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, 0)});
  buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, 30'000'000)});
  buf.insert(std::vector<slam::VisualObservation>{obs(2, 1, 79'000'000)});
  buf.note_frame(0, kPeriod);  // featureless anchor frame
  buf.note_frame(1, kPeriod + 30'000'000);
  buf.note_frame(2, kPeriod + 79'000'000);
  const auto ready = buf.pop_ready();
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_EQ(ready[0].anchor_stamp_ns, 0);
  EXPECT_EQ(ready[0].observations.size(), 3U);  // anchor + drained cam1 + cam2
  EXPECT_EQ(buf.dropped_count(), 0);
}

TEST(GroupingBuffer, FinishDropsAndCountsUncoveredPendingObservations)
{
  // The assign-or-drop half of finish()'s contract, for observations that are
  // still pending at end of bag. Window [0, 100ms) exists; camera 1's 150 ms
  // observation is past its end with no covering anchor — dropped and counted.
  slam::GroupingBuffer buf(three_cameras());
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, 0)});
  buf.insert(std::vector<slam::VisualObservation>{obs(1, 1, 150'000'000)});
  const auto groups = buf.finish();
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_EQ(groups[0].observations.size(), 1U);  // the anchor observation only
  EXPECT_EQ(buf.dropped_count(), 1);

  // Degenerate variant: the anchor camera never delivered at all — there is
  // no window anywhere, so the pending observation drops at finish.
  slam::GroupingBuffer empty_anchor(three_cameras());
  empty_anchor.insert(std::vector<slam::VisualObservation>{obs(1, 1, 50'000'000)});
  EXPECT_TRUE(empty_anchor.finish().empty());
  EXPECT_EQ(empty_anchor.dropped_count(), 1);
}

TEST(GroupingBuffer, ObservationWithUnregisteredCameraIdIsDroppedAndCounted)
{
  slam::GroupingBuffer buf(three_cameras());
  // camera_id == camera_count has no head slot, so it can never gate pops; a
  // window could pop before its observations arrive, which would reintroduce
  // arrival-order-dependent drops. Refuse such observations outright instead
  // of mis-modeling them.
  buf.insert(std::vector<slam::VisualObservation>{obs(0, 1, 0)});
  buf.insert(std::vector<slam::VisualObservation>{obs(3, 1, 30'000'000)});
  const auto groups = buf.finish();
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_EQ(groups[0].observations.size(), 1U);
  EXPECT_EQ(buf.dropped_count(), 1);
}

TEST(GroupingBuffer, MinSentinelStampsNeverPopIncompleteWindows)
{
  slam::GroupingBuffer buf(three_cameras());
  // INT64_MIN doubles as the "nothing poppable" sentinel inside pop_ready()
  // and as a representable stamp. A window anchored there must not leak out
  // while the other cameras' heads have not passed its end.
  buf.insert(
    std::vector<slam::VisualObservation>{obs(0, 1, std::numeric_limits<std::int64_t>::min())});
  EXPECT_TRUE(buf.pop_ready().empty());
  const auto groups = buf.finish();
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_EQ(groups[0].observations.size(), 1U);
}

TEST(GroupingBuffer, RejectsAnchorOutsideCameraCount)
{
  auto cfg = three_cameras();
  cfg.anchor_camera_id = 3;  // cameras are 0..2
  EXPECT_THROW(slam::GroupingBuffer{cfg}, std::invalid_argument);
  cfg.anchor_camera_id = 0;
  cfg.camera_count = 0;
  EXPECT_THROW(slam::GroupingBuffer{cfg}, std::invalid_argument);
}

}  // namespace
