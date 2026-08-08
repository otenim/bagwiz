// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__TF_STATIC_INJECT_HPP_
#define COMMANDS__TF_STATIC_INJECT_HPP_

#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/tf/tf_static_collect.hpp"
#include "bagwiz/io/bag_open.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

// CLI-internal (not installed): the rewrite passes that write latched static
// TF topics into a bag. `inject_static_tf_pass` writes whole topics from
// scratch, shared by `bagwiz tf static cp` (transforms read from a donor bag)
// and `bagwiz tf static join` (transforms read from a YAML config);
// `edit_static_tf_pass` rewrites a caller-selected set of topics to carry an
// edited transform list each, for `bagwiz tf static drop` and `bagwiz tf static
// update`. Getting the write ORDER wrong is a silent, hard-to-spot bug, so the
// passes live in one place.
// Follows the src-local shared-header idiom of traj_common.hpp /
// pcd_concat_common.hpp.
namespace bagwiz::commands
{

// Per-caller knobs for inject_static_tf_pass().
struct StaticTfInjectOptions
{
  // Logger tag for every diagnostic the pass emits, e.g.
  // "bagwiz.cmd.tf.static.cp".
  const char * logger = "bagwiz.cmd.tf.static";
  // Command name used in the summary line, e.g. "tf static cp".
  std::string label;
  // Passed to core::bag_copy_filtered as its profile_label, which names the
  // per-stage bottleneck report under BAGWIZ_PROFILE. Kept separate from
  // `label` because that report's labels are identifiers, e.g. "tf_static_cp".
  std::string profile_label;
  // Whether a destination topic that already carries messages may have them
  // dropped and replaced. This is the user's -w/--overwrite for `cp` and
  // --force for `join`; without it such a collision aborts the pass.
  bool replace_existing_topic = false;
};

// One full pass over `dst_path`: plan each topic in `topics` against what the
// destination already has, declare topics on the writer, write one latched
// TFMessage per entry stamped at the destination's earliest message time, then
// stream-copy everything else through (suppressing the topics being replaced).
//
// The injected messages are written BEFORE the stream copy. They carry the
// bag's lowest timestamp, so appending them would put them at the highest
// storage position — the only rows whose physical order disagrees with their
// time. A consumer that reads a .db3 in row order rather than by timestamp
// (Foxglove's readers issue their message query without an ORDER BY) would then
// receive the static TF last, after everything it is supposed to precede.
//
// `open_writer` is the factory handed in by core::run_bag_rewrite, so the same
// pass serves both in-place (tmp path) and explicit -o modes. Returns the
// process exit code: 0 on success, 1 after logging any failure (bag could not be
// opened, an unresolved topic/type conflict, a serialize/declare/write failure,
// or an I/O error).
int inject_static_tf_pass(
  const std::filesystem::path & dst_path, const std::vector<core::StaticTopicTransforms> & topics,
  const StaticTfInjectOptions & options, const io::WriterFactory & open_writer);

// One full pass over `dst_path` that rewrites each topic in `touched` to carry
// exactly `transforms` — the edge-granular counterpart of inject_static_tf_pass,
// for `bagwiz tf static drop` and `bagwiz tf static update`. A touched topic
// already in the bag must be a static `tf2_msgs/msg/TFMessage` topic (its
// messages are suppressed and
// replaced by one message holding the given transforms, written BEFORE the
// stream copy for the row-order reason inject_static_tf_pass documents); a
// touched topic absent from the bag is declared new. A touched topic whose
// transforms are empty keeps its declaration but gets no message. Untouched
// topics stream-copy through unchanged. `options.replace_existing_topic` is not
// consulted: the caller selected the touched set explicitly, so there is no
// conflict to force past. Returns the process exit code: 0 on success, 1 after
// logging any failure.
int edit_static_tf_pass(
  const std::filesystem::path & dst_path, const std::vector<core::StaticTopicTransforms> & touched,
  const StaticTfInjectOptions & options, const io::WriterFactory & open_writer);

// The shared tail of `tf static drop` and `tf static update`, which differ only
// in how they mutate `topics` (drop removes edges, update adds/updates them).
// Validates the edited `topics` as a forest (an add/update or a re-parent can
// close a cycle against edges the bag already carries), selects the subset of
// `topics` named in `touched`, and rewrites exactly those via edit_static_tf_pass
// (`input_path`/`output_path`/`overwrite` handed straight to core::run_bag_rewrite).
// `touched` empty is a no-op that writes nothing and returns 0 after logging under
// `options.label`. The caller logs its own edit-count summary on success. Returns
// the process exit code: 0 on success, 1 after logging any failure (the edit broke
// the forest, or the rewrite failed).
int rewrite_touched_static_topics(
  const std::filesystem::path & input_path, const std::vector<core::StaticTopicTransforms> & topics,
  const std::unordered_set<std::string> & touched,
  const std::optional<std::filesystem::path> & output_path, bool overwrite,
  const core::BagRewriteOptions & rewrite_opts, const StaticTfInjectOptions & inject_opts);

}  // namespace bagwiz::commands

#endif  // COMMANDS__TF_STATIC_INJECT_HPP_
