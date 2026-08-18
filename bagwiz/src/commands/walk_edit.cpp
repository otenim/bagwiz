// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_edit.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/tf/tf_chain.hpp"
#include "bagwiz/core/tf/tf_static_tree_yaml.hpp"
#include "bagwiz/core/tf/tf_transform_format.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <array>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

// Finest to coarsest; see edit_step_preset() in the header for why index 1
// is the default.
constexpr std::array<EditStep, kEditStepPresetCount> kEditStepPresets{
  EditStep{0.001, 0.0005}, EditStep{0.01, 0.005}, EditStep{0.1, 0.05}};

// Frame ids and topic names come from arbitrary bag payload content, and the
// strings this module builds go straight to the terminal (picker rows, the
// info row, the exit summary) — a crafted id must not smuggle an escape
// sequence (ESC/CSI/OSC) through. Replace C0 control bytes and DEL with
// their \xNN spelling; bytes in `keep` pass verbatim (the exit summary keeps
// its newlines). Bytes >= 0x80 are untouched so multi-byte UTF-8 survives.
std::string escape_control_bytes(std::string_view text, std::string_view keep = {})
{
  std::string out;
  out.reserve(text.size());
  for (const char c : text) {
    const auto byte = static_cast<unsigned char>(c);
    if ((byte < 0x20 || byte == 0x7F) && keep.find(c) == std::string_view::npos) {
      out += fmt::format("\\x{:02x}", byte);
    } else {
      out += c;
    }
  }
  return out;
}

}  // namespace

EditPose pose_from_transform(const geometry_msgs::msg::Transform & transform)
{
  const auto rpy = core::quaternion_to_rpy(transform.rotation);
  return {
    transform.translation.x,
    transform.translation.y,
    transform.translation.z,
    rpy.roll,
    rpy.pitch,
    rpy.yaw};
}

geometry_msgs::msg::Transform pose_to_transform(const EditPose & pose)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = pose.x;
  transform.translation.y = pose.y;
  transform.translation.z = pose.z;
  transform.rotation = core::rpy_to_quaternion({pose.roll, pose.pitch, pose.yaw});
  return transform;
}

EditStep edit_step_preset(std::size_t index)
{
  return kEditStepPresets[std::min(index, kEditStepPresets.size() - 1)];
}

bool apply_edit_key(ExtrinsicEditState & state, core::KeyEvent event)
{
  // Step changes act on the state itself, so they need no active edge. At a
  // preset end the key changes nothing and reports unhandled, so the caller
  // skips the repaint.
  if (event == core::KeyEvent::kEditStepUp) {
    if (state.step_index + 1 >= kEditStepPresetCount) {
      return false;
    }
    ++state.step_index;
    return true;
  }
  if (event == core::KeyEvent::kEditStepDown) {
    if (state.step_index == 0) {
      return false;
    }
    --state.step_index;
    return true;
  }

  if (state.edges.empty() || state.active >= state.edges.size()) {
    return false;
  }
  auto & edited = state.edges[state.active].edited;
  const EditStep step = edit_step_preset(state.step_index);
  switch (event) {
    case core::KeyEvent::kEditTransXUp:
      edited.x += step.translation_m;
      return true;
    case core::KeyEvent::kEditTransXDown:
      edited.x -= step.translation_m;
      return true;
    case core::KeyEvent::kEditTransYUp:
      edited.y += step.translation_m;
      return true;
    case core::KeyEvent::kEditTransYDown:
      edited.y -= step.translation_m;
      return true;
    case core::KeyEvent::kEditTransZUp:
      edited.z += step.translation_m;
      return true;
    case core::KeyEvent::kEditTransZDown:
      edited.z -= step.translation_m;
      return true;
    case core::KeyEvent::kEditRollUp:
      edited.roll += step.rotation_rad;
      return true;
    case core::KeyEvent::kEditRollDown:
      edited.roll -= step.rotation_rad;
      return true;
    case core::KeyEvent::kEditPitchUp:
      edited.pitch += step.rotation_rad;
      return true;
    case core::KeyEvent::kEditPitchDown:
      edited.pitch -= step.rotation_rad;
      return true;
    case core::KeyEvent::kEditYawUp:
      edited.yaw += step.rotation_rad;
      return true;
    case core::KeyEvent::kEditYawDown:
      edited.yaw -= step.rotation_rad;
      return true;
    case core::KeyEvent::kEditReset:
      edited = pose_from_transform(state.edges[state.active].original.transform);
      return true;
    default:
      return false;
  }
}

bool is_edited(const EditableEdge & edge)
{
  // Exact comparison is intentional: an untouched or reset pose is an
  // assignment of this very extraction over the same immutable original, so
  // equality is guaranteed; any nudge moves a component by a step orders of
  // magnitude above the double's resolution.
  const EditPose original = pose_from_transform(edge.original.transform);
  return edge.edited.x != original.x || edge.edited.y != original.y ||
         edge.edited.z != original.z || edge.edited.roll != original.roll ||
         edge.edited.pitch != original.pitch || edge.edited.yaw != original.yaw;
}

geometry_msgs::msg::TransformStamped edited_transform(const EditableEdge & edge)
{
  geometry_msgs::msg::TransformStamped ts = edge.original;
  ts.transform = pose_to_transform(edge.edited);
  return ts;
}

void apply_edge_to_buffer(const EditableEdge & edge, tf2::BufferCore & buffer)
{
  // tf2 keeps one slot per static (parent, child) pair, so this replaces the
  // value every later lookup composes with. The authority string only shows
  // up in tf2 diagnostics; name the writer so an edited buffer is
  // recognizable there.
  buffer.setTransform(edited_transform(edge), "bagwiz_edit", /*is_static=*/true);
}

std::vector<EditableEdge> collect_editable_edges(
  const tf2::BufferCore & buffer, const std::vector<std::string> & cloud_frames,
  const std::string & camera_frame, tf2::TimePoint time,
  const std::vector<core::StaticTopicTransforms> & static_topics)
{
  std::vector<EditableEdge> out;
  const auto already_collected = [&out](const std::string & parent, const std::string & child) {
    return std::any_of(out.begin(), out.end(), [&](const EditableEdge & e) {
      return e.original.header.frame_id == parent && e.original.child_frame_id == child;
    });
  };

  for (const auto & cloud_frame : cloud_frames) {
    const auto chain = core::resolve_chain(buffer, cloud_frame, camera_frame, time);
    for (const auto & [parent, child] : core::chain_to_edges(buffer, chain, time)) {
      if (already_collected(parent, child)) {
        continue;
      }
      // Editable only when a static topic carries this exact (parent, child)
      // edge; a chain edge fed by dynamic TF is skipped — it is not tf_static
      // data, so the edit mode cannot fix it.
      for (const auto & st : static_topics) {
        const auto it = std::find_if(
          st.transforms.begin(), st.transforms.end(),
          [&](const geometry_msgs::msg::TransformStamped & t) {
            return t.header.frame_id == parent && t.child_frame_id == child;
          });
        if (it == st.transforms.end()) {
          continue;
        }
        EditableEdge edge;
        edge.topic = st.name;
        edge.original = *it;
        edge.edited = pose_from_transform(it->transform);
        out.push_back(std::move(edge));
        break;
      }
    }
  }
  return out;
}

void carry_over_edits(std::vector<EditableEdge> & fresh, const std::vector<EditableEdge> & previous)
{
  for (const auto & prev : previous) {
    if (!is_edited(prev)) {
      continue;
    }
    const auto it = std::find_if(fresh.begin(), fresh.end(), [&](const EditableEdge & e) {
      return e.topic == prev.topic && e.original.header.frame_id == prev.original.header.frame_id &&
             e.original.child_frame_id == prev.original.child_frame_id;
    });
    if (it != fresh.end()) {
      it->edited = prev.edited;
    } else {
      fresh.push_back(prev);
    }
  }
}

std::vector<geometry_msgs::msg::TransformStamped> edited_transforms(
  const ExtrinsicEditState & state)
{
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  for (const auto & edge : state.edges) {
    if (is_edited(edge)) {
      transforms.push_back(edited_transform(edge));
    }
  }
  return transforms;
}

void commit_edits(ExtrinsicEditState & state)
{
  for (auto & edge : state.edges) {
    if (is_edited(edge)) {
      edge.original = edited_transform(edge);
    }
  }
}

std::string edit_yaml(const ExtrinsicEditState & state, std::string_view source_label)
{
  std::vector<geometry_msgs::msg::TransformStamped> edited;
  for (const auto & edge : state.edges) {
    if (is_edited(edge)) {
      edited.push_back(edited_transform(edge));
    }
  }
  if (edited.empty()) {
    return {};
  }
  return core::emit_static_tf_tree_yaml(
    std::span<const geometry_msgs::msg::TransformStamped>(edited.data(), edited.size()),
    source_label);
}

std::string edit_summary(const ExtrinsicEditState & state)
{
  std::string summary;
  for (const auto & edge : state.edges) {
    if (!is_edited(edge)) {
      continue;
    }
    // The edge parent -> child is the pose of the child expressed in the
    // parent, i.e. of=child ref=parent in format_transform_human's terms.
    summary += core::format_transform_human(
      edited_transform(edge), {edge.original.child_frame_id, edge.original.header.frame_id},
      fmt::format("  (static, topic {})", edge.topic));
  }
  // The frame ids and topic names embedded in the blocks are bag content;
  // only the renderer's own newlines are trusted line structure.
  return escape_control_bytes(summary, "\n");
}

std::string edge_label(const EditableEdge & edge)
{
  std::string label = fmt::format(
    "{} -> {}  [{}]", escape_control_bytes(edge.original.header.frame_id),
    escape_control_bytes(edge.original.child_frame_id), escape_control_bytes(edge.topic));
  if (is_edited(edge)) {
    label += "  (edited)";
  }
  return label;
}

std::string edit_info_text(const ExtrinsicEditState & state)
{
  if (state.edges.empty() || state.active >= state.edges.size()) {
    return "edit: (no editable edge)";
  }
  const auto & edge = state.edges[state.active];
  const EditPose original = pose_from_transform(edge.original.transform);
  constexpr double kRadToDeg = 180.0 / std::numbers::pi;

  std::string text = fmt::format(
    "edit: {}->{}", escape_control_bytes(edge.original.header.frame_id),
    escape_control_bytes(edge.original.child_frame_id));
  // The exact != mirrors is_edited(): an untouched component is an assignment
  // of the same extraction, so equality is guaranteed and no noise delta is
  // shown for it.
  const auto translation = [&text](std::string_view name, double value, double base) {
    text += fmt::format("  {}: {:+.4f}", name, value);
    if (value != base) {
      text += fmt::format(" ({:+.4f})", value - base);
    }
  };
  const auto rotation = [&text, kRadToDeg](std::string_view name, double value, double base) {
    text += fmt::format("  {}: {:+.3f}°", name, value * kRadToDeg);
    if (value != base) {
      text += fmt::format(" ({:+.3f}°)", (value - base) * kRadToDeg);
    }
  };
  translation("x", edge.edited.x, original.x);
  translation("y", edge.edited.y, original.y);
  translation("z", edge.edited.z, original.z);
  rotation("roll", edge.edited.roll, original.roll);
  rotation("pitch", edge.edited.pitch, original.pitch);
  rotation("yaw", edge.edited.yaw, original.yaw);

  const EditStep step = edit_step_preset(state.step_index);
  text += fmt::format("  step: {:.3f}m/{:.3f}°", step.translation_m, step.rotation_rad * kRadToDeg);
  return text;
}

}  // namespace bagwiz::commands
