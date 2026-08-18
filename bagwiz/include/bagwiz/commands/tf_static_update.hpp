// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TF_STATIC_UPDATE_HPP_
#define BAGWIZ__COMMANDS__TF_STATIC_UPDATE_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Implements `bagwiz tf static update -i <input> --yaml <file> [-t <topic>]
// [-o <output>] [-w|--overwrite]`: surgical, edge-granular add/update of the bag's
// static TF tree, where `tf static join` only creates or wholesale replaces a
// topic.
//
// `--yaml <file>` is a static-transform publisher config (the schema `tf static
// dump` writes; see core::parse_static_tf_tree_yaml). Each of its edges is applied
// as an upsert: a child already present in the tree is updated in place, in
// whichever topic carries it, and a differing parent re-parents the edge (logged);
// a child absent from the tree is added under `topic`, which is declared when the
// bag does not have it yet. The merged result is validated as a forest
// (core::validate_tf_forest) before anything is written, so an update that would
// close a cycle aborts with the input untouched.
//
// The write-back preserves the bag's per-topic layout: each touched static topic
// is rewritten as one latched `tf2_msgs/msg/TFMessage` stamped at the bag's start
// time (written ahead of the copied stream, as in inject_static_tf_pass), while
// untouched static topics and every non-TF topic pass through unchanged.
//
// When `output_path` is empty, `<input>` is rewritten in place via an atomic
// tmp-swap that preserves its storage format and layout; when it is set, `<input>`
// is left untouched and the result is written there. `overwrite` permits replacing
// an existing `-o`/`--output` path, matching `tf static join`.
//
// Returns the process exit code: 0 on success, 1 on any error (the YAML could not
// be read or is invalid, the edit would break the forest, a bag could not be
// opened, a topic/type conflict, a serialize failure, or an I/O error).
int run_tf_static_update(
  const std::filesystem::path & input_path, const std::filesystem::path & yaml_path,
  const std::string & topic, const std::optional<std::filesystem::path> & output_path,
  bool overwrite);

// Overload taking the edges directly instead of a YAML file, with identical
// upsert/validation/write-back semantics — the YAML overload parses the file
// and delegates here. For in-process callers that already hold the
// transforms (walk's extrinsic edit mode applies its edits through this), so
// nothing round-trips through the YAML emitter's limited precision.
int run_tf_static_update(
  const std::filesystem::path & input_path,
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms, const std::string & topic,
  const std::optional<std::filesystem::path> & output_path, bool overwrite);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TF_STATIC_UPDATE_HPP_
