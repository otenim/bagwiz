// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_EDIT_HPP_
#define COMMANDS__WALK_EDIT_HPP_

#include "bagwiz/core/base/terminal_input.hpp"
#include "bagwiz/core/tf/tf_static_collect.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

// Extrinsic edit mode of `bagwiz walk`'s image preview: the editable-edge
// derivation (the static TF edges on the chains between the overlaid cloud
// frames and the camera frame), the six-scalar edit state the nudge keys act
// on, the tf2 buffer application that makes the overlay re-project live, and
// the YAML payload `bagwiz tf static update` applies back to the bag.
// Everything here is TTY-free and unit-tested; walk_preview owns the
// interactive glue. CLI-internal: this header lives with the command sources
// and is not installed.
namespace bagwiz::commands
{

// The six scalars of the static-transform-publisher schema, the same
// parametrization `bagwiz tf static dump` writes: translation in meters and
// roll/pitch/yaw in radians under tf2's fixed-axis convention (see
// core::quaternion_to_rpy). The edit state is kept in this form — never
// re-extracted from a quaternion after a nudge — so repeated nudges stay
// monotonic even near the RPY gimbal ambiguity.
struct EditPose
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

// Convert between a geometry_msgs Transform and the six-scalar pose.
// pose_from_transform(pose_to_transform(p)) returns the same rotation; the
// RPY triple itself round-trips only up to getRPY's branch choice, which is
// why the edit state never converts back after the initial extraction.
[[nodiscard]] EditPose pose_from_transform(const geometry_msgs::msg::Transform & transform);
[[nodiscard]] geometry_msgs::msg::Transform pose_to_transform(const EditPose & pose);

// One editable static TF edge: the topic that carries it in the bag, its
// original value as recorded, and the current edited pose (equal to the
// original's pose until nudged).
struct EditableEdge
{
  std::string topic;
  geometry_msgs::msg::TransformStamped original;
  EditPose edited;
};

// Nudge step preset: how far one keypress moves a translation or rotation
// component. Rotation steps are round radian values; the preview renders
// them in degrees.
struct EditStep
{
  double translation_m = 0.0;
  double rotation_rad = 0.0;
};

// Step presets ordered finest to coarsest; m/M move the index up/down,
// clamped at the ends. The default (index 1: 1 cm, 0.005 rad ~ 0.29 deg) is
// coarse enough to see the overlay move at typical lidar ranges and fine
// enough to converge without switching.
inline constexpr std::size_t kEditStepPresetCount = 3;
inline constexpr std::size_t kEditStepDefaultIndex = 1;
[[nodiscard]] EditStep edit_step_preset(std::size_t index);

// State behind the preview's extrinsic edit mode. `edges` are the editable
// candidates (collect_editable_edges); nudges act on `edges[active]`.
// `editing` is the key-routing flag the preview toggles with [e].
struct ExtrinsicEditState
{
  bool editing = false;
  std::vector<EditableEdge> edges;
  std::size_t active = 0;
  std::size_t step_index = kEditStepDefaultIndex;
};

// Apply one kEdit* nudge/step/reset event to the state. Returns true when
// the event mutated the state (the caller re-applies the active edge to the
// TF buffer and repaints); false for unrelated events or when there is no
// active edge to act on. kToggleEditExtrinsic / kSelectEditEdge /
// kEditDumpYaml are interactive-flow events, not state mutations, so they
// are not handled here.
[[nodiscard]] bool apply_edit_key(ExtrinsicEditState & state, core::KeyEvent event);

// True when the edge's edited pose differs from its original. Reset assigns
// the original's pose verbatim, so an untouched or reset edge compares
// exactly equal.
[[nodiscard]] bool is_edited(const EditableEdge & edge);

// The edge's TransformStamped with the edited pose substituted; frame ids
// and the header stamp are preserved from the original, which is what both
// the TF buffer application and the YAML export feed on.
[[nodiscard]] geometry_msgs::msg::TransformStamped edited_transform(const EditableEdge & edge);

// Overwrite the edge's value in `buffer`. tf2 stores a static edge in a
// single-slot cache, so re-setting the same (parent, child) pair replaces
// the transform every projection lookup composes with — this is what makes
// the overlay track the nudges live.
void apply_edge_to_buffer(const EditableEdge & edge, tf2::BufferCore & buffer);

// Derive the editable candidates: every (parent, child) edge on a TF chain
// between one of `cloud_frames` and `camera_frame` at `time` that a static
// TF topic in `static_topics` carries. Edges keep chain order (cloud side
// first) with `cloud_frames` visited in the given order, de-duplicated when
// chains share edges. Dynamic edges on the chain are skipped: they are not
// tf_static data, so the edit mode cannot fix them. A frame with no chain
// contributes nothing.
[[nodiscard]] std::vector<EditableEdge> collect_editable_edges(
  const tf2::BufferCore & buffer, const std::vector<std::string> & cloud_frames,
  const std::string & camera_frame, tf2::TimePoint time,
  const std::vector<core::StaticTopicTransforms> & static_topics);

// Preserve edits across a candidate re-collection (the overlay topic
// selection changed, so the chains — and the TF buffer — were rebuilt).
// An edited previous edge matching a fresh candidate by (topic, parent,
// child) hands its edited pose over; an edited previous edge with no fresh
// counterpart is appended so it stays exportable and resettable. Untouched
// previous edges are dropped with the old candidate list.
void carry_over_edits(
  std::vector<EditableEdge> & fresh, const std::vector<EditableEdge> & previous);

// Render every edited edge as the static-TF-tree YAML `bagwiz tf static
// update --yaml` consumes (see core::emit_static_tf_tree_yaml).
// `source_label` lands in the header comment. Returns an empty string when
// no edge is edited — there would be nothing to apply.
[[nodiscard]] std::string edit_yaml(
  const ExtrinsicEditState & state, std::string_view source_label);

// Picker row for an editable edge: "parent -> child  [topic]", with an
// "  (edited)" suffix once the edge differs from the bag.
[[nodiscard]] std::string edge_label(const EditableEdge & edge);

// Info-row text for the active edge: the six current scalars (rotations in
// degrees), each nudged component's delta from the bag in parentheses, and
// the active step preset. "edit: (no editable edge)" when there is none.
[[nodiscard]] std::string edit_info_text(const ExtrinsicEditState & state);

// Human rendering of every edited edge (core::format_transform_human, one
// block per edge, annotated with the carrying topic), printed by walk on
// exit so the values survive the session even when the YAML was never
// exported. Empty when nothing is edited.
[[nodiscard]] std::string edit_summary(const ExtrinsicEditState & state);

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_EDIT_HPP_
