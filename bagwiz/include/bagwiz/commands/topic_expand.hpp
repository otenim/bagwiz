// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TOPIC_EXPAND_HPP_
#define BAGWIZ__COMMANDS__TOPIC_EXPAND_HPP_

namespace CLI
{
class App;
}  // namespace CLI

namespace bagwiz::commands
{

// Resolve every topic slot declared under `root` on this invocation, in place,
// after parsing and before the command runs.
//
// For each parsed (sub)command carrying slots: the bag named by
// set_topic_input() is opened once and its topic list shared across that
// command's slots. A kGlob slot has its '*' selectors replaced by the matching
// topic names; a kLiteral slot rejects a value containing '*'. Literals in a
// kGlob slot pass through untouched, so the command's own presence and type
// errors stay exactly as they are.
//
// Returns false after logging when a glob matches no topic, when a literal-only
// slot is given a glob, or when a slot's `scope` names an option that is not an
// earlier slot of the same command. An input that cannot be opened as a bag is
// not an error here: expansion is skipped and the command's own validation
// reports it.
[[nodiscard]] bool expand_topic_selectors(const CLI::App & root);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TOPIC_EXPAND_HPP_
