// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "topic_declare.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/msg_yaml/msg_definition_resolver.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <cstddef>
#include <exception>
#include <utility>

namespace bagwiz::commands
{

std::size_t declare_reader_topics(
  io::BagReader & reader, io::BagWriter & writer, const char * logger)
{
  // Force schema bytes onto the topic list before declaring so MCAP
  // outputs preserve self-description across a repack (one-shot shard
  // open for multi-shard MCAP inputs; no-op for single-file MCAP and
  // SQLite3 where schemas are either already loaded or not embedded).
  reader.populate_schemas();

  std::size_t declared = 0;
  std::size_t resolved_defs = 0;
  std::size_t unresolved_defs = 0;
  for (const auto & t : reader.topics()) {
    io::TopicInfo augmented = t;
    if (augmented.schema_text.empty()) {
      auto resolved = core::resolve_message_definition(augmented.type);
      if (!resolved.text.empty()) {
        augmented.schema_text = std::move(resolved.text);
        augmented.schema_encoding = std::move(resolved.encoding);
        ++resolved_defs;
      } else {
        ++unresolved_defs;
        if (unresolved_defs <= 5) {
          BAGWIZ_LOG_WARN(
            logger,
            "no .msg on disk for type '%s' (topic '%s'); writing MCAP without "
            "self-description for this topic",
            augmented.type.c_str(), augmented.name.c_str());
        }
      }
    }
    try {
      writer.declare_topic(augmented);
      ++declared;
    } catch (const std::exception & e) {
      BAGWIZ_LOG_WARN(
        logger, "declare_topic failed for '%s': %s; skipping topic", t.name.c_str(), e.what());
    }
  }
  if (resolved_defs > 0) {
    BAGWIZ_LOG_INFO(
      logger, "resolved %zu missing message definition(s) from $AMENT_PREFIX_PATH", resolved_defs);
  }
  if (unresolved_defs > 5) {
    BAGWIZ_LOG_WARN(
      logger, "(plus %zu more topic(s) without resolvable .msg)", unresolved_defs - 5);
  }
  return declared;
}

}  // namespace bagwiz::commands
