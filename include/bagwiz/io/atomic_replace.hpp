// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__ATOMIC_REPLACE_HPP_
#define BAGWIZ__IO__ATOMIC_REPLACE_HPP_

#include <filesystem>

namespace bagwiz::io
{

// Replace `target` with `staged` as a single semi-atomic step.
//
// Used by commands that produce a new bag at `staged` and want to swap it
// over the original `target` once writing completes successfully. The two
// paths must live on the same filesystem so the underlying rename is
// atomic on POSIX; the helper validates this and throws std::system_error
// otherwise. If `target` does not yet exist, the call degenerates to a
// plain rename.
//
// Failure semantics:
// - Throws std::system_error on any rename/remove failure.
// - On failure mid-swap (after the original target was moved aside but
//   before the staged path was renamed into place), the original target is
//   restored before the exception propagates, so callers always see either
//   the old contents or the new contents — never a half-applied state.
//
// `staged` may be a directory (rosbag2 directory layout) or a single file
// (`.mcap` / `.db3`); both are handled.
void atomic_replace(const std::filesystem::path & staged, const std::filesystem::path & target);

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__ATOMIC_REPLACE_HPP_
