// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__TOPIC_DECLARE_HPP_
#define COMMANDS__TOPIC_DECLARE_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <cstddef>

// Src-local helper shared by the repack-style commands (`convert format`,
// `compress`) that copy a bag wholesale: declare every topic the reader
// carries on the writer, keeping the output self-describing. Lives in the
// CLI package (not bagwiz_bag) because the schema backfill goes through
// bagwiz_msg's msg_definition_resolver, which bagwiz_bag does not link.
namespace bagwiz::commands
{

// Declare every topic `reader` carries on `writer`. Schema preservation:
//   - reader->populate_schemas() runs first, so multi-shard MCAP inputs load
//     their embedded schemas up front (no-op for single-file MCAP and
//     SQLite3, where schemas are either already loaded or not embedded).
//   - A topic still carrying an empty schema_text (SQLite3 storage on Humble
//     and earlier does not embed message definitions) gets its definition
//     resolved from $AMENT_PREFIX_PATH/share/<pkg>/msg/<Type>.msg before
//     being declared — otherwise an MCAP output loses self-description and
//     breaks strict downstream readers. An unresolvable type is declared
//     without self-description and logged (first 5 at WARN, the rest
//     counted).
//
// A topic whose declare_topic() throws is skipped with a warning rather than
// aborting the run, matching the historical `convert format` behavior.
// Returns the number of topics declared.
std::size_t declare_reader_topics(
  io::BagReader & reader, io::BagWriter & writer, const char * logger);

}  // namespace bagwiz::commands

#endif  // COMMANDS__TOPIC_DECLARE_HPP_
