// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TF_STATIC_EDIT_HPP_
#define BAGWIZ__COMMANDS__TF_STATIC_EDIT_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Implements `bagwiz tf static edit -i <input> [--yaml <file>] [--prune <frame>...]
// [-t <topic>] [-o <output>] [-w|--overwrite]`: surgical, edge-granular edits to
// the bag's static TF tree, where `tf static join` only creates or wholesale
// replaces a topic.
//
// Two edit kinds, combinable in one run:
//
//   * --prune <frame> removes the edge whose child is <frame> together with the
//     frame's whole subtree; every dropped edge is logged. The frame must exist
//     as a child in the merged static tree — a typo or a root-only frame aborts
//     the run before anything is written.
//   * --yaml <file> adds or updates edges from a static-transform publisher
//     config (the schema `tf static dump` writes; see
//     core::parse_static_tf_tree_yaml). A child absent from the tree is added
//     under `topic`; a child already present is updated in place, in whichever
//     topic carries it, and a differing parent re-parents the edge (logged).
//
// Prune applies first, so pruning a frame and re-adding it via --yaml in one run
// is a well-defined subtree replacement. The merged result is validated as a
// forest (core::validate_tf_forest) before anything is written.
//
// The write-back preserves the bag's per-topic layout: each touched static topic
// is rewritten as one latched `tf2_msgs/msg/TFMessage` stamped at the bag's
// start time (written ahead of the copied stream, as in inject_static_tf_pass),
// while untouched static topics and every non-TF topic pass through unchanged. A
// topic left with no edges keeps its declaration but carries no message.
//
// When `output_path` is empty, `<input>` is rewritten in place via an atomic
// tmp-swap that preserves its storage format and layout; when it is set,
// `<input>` is left untouched and the result is written there. `overwrite`
// permits replacing an existing `-o`/`--output` path, matching `tf static join`.
//
// Returns the process exit code: 0 on success, 1 on any error (neither edit kind
// given, the YAML could not be read or is invalid, a pruned frame is not a child
// in the tree, the edit would break the forest, a bag could not be opened, a
// topic/type conflict, a serialize failure, or an I/O error).
int run_tf_static_edit(
  const std::filesystem::path & input_path, const std::optional<std::filesystem::path> & yaml_path,
  const std::vector<std::string> & prune_frames, const std::string & topic,
  const std::optional<std::filesystem::path> & output_path, bool overwrite);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TF_STATIC_EDIT_HPP_
