// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TF_STATIC_DROP_HPP_
#define BAGWIZ__COMMANDS__TF_STATIC_DROP_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Implements `bagwiz tf static drop -i <input> --frame <frame>... [-o <output>]
// [-w|--overwrite]`: remove frames from the bag's static TF tree, each together
// with its whole subtree.
//
// Every `--frame` names a _child_ frame: the edge above it is removed along with
// the frame's whole subtree, and every dropped edge is logged. A frame must exist
// as a child in the merged static tree — a typo, an unknown frame, or a root-only
// frame (one that parents edges but has no parent itself) aborts the run before
// anything is written. `--frame` is repeatable, so several subtrees can be dropped
// in one run; the removals are resolved against the tree as loaded, so listing a
// frame and one of its descendants together is well defined.
//
// The write-back preserves the bag's per-topic layout: each touched static topic
// is rewritten as one latched `tf2_msgs/msg/TFMessage` stamped at the bag's start
// time (written ahead of the copied stream, as in inject_static_tf_pass), while
// untouched static topics and every non-TF topic pass through unchanged. A topic
// pruned down to no edges keeps its declaration but carries no message. The merged
// result is validated as a forest before anything is written.
//
// When `output_path` is empty, `<input>` is rewritten in place via an atomic
// tmp-swap that preserves its storage format and layout; when it is set, `<input>`
// is left untouched and the result is written there. `overwrite` permits replacing
// an existing `-o`/`--output` path.
//
// Returns the process exit code: 0 on success, 1 on any error (no frame given, a
// dropped frame is not a child in the tree, a bag could not be opened, a
// topic/type conflict, a serialize failure, or an I/O error).
int run_tf_static_drop(
  const std::filesystem::path & input_path, const std::vector<std::string> & frames,
  const std::optional<std::filesystem::path> & output_path, bool overwrite);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TF_STATIC_DROP_HPP_
