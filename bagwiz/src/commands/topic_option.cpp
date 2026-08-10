// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/topic_option.hpp"

#include "CLI/CLI.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

struct Registration
{
  std::vector<TopicSlot> slots;
  std::filesystem::path * input{};
};

// Process-wide store, keyed by the CLI::App the option was declared on so a
// command with several subcommands keeps them apart. Populated during
// configure() at startup, before any parsing, and read-only afterwards — the
// same single-threaded lifetime as the command Registry in command.hpp.
std::unordered_map<const CLI::App *, Registration> & store()
{
  static std::unordered_map<const CLI::App *, Registration> s;
  return s;
}

// Erases `app`'s entry once nothing references this guard anymore, i.e. when
// the Option carrying it (see arm_guard()) is destroyed. This is a real erase
// in production, not a formality: every top-level CLI::App in this binary
// (see bagwiz/src/main.cpp) is a local in main(), so its destructor — and
// this one — runs when main() returns, while store()'s function-local static
// is still alive. Do not give a CLI::App static or global storage duration:
// function-local statics are destroyed in reverse order of construction at
// program exit, so an App outliving store() would run this destructor after
// store() itself is gone, touching an already-destroyed object.
struct StoreGuard
{
  const CLI::App * app;
  explicit StoreGuard(const CLI::App * owning_app) : app{owning_app} {}
  ~StoreGuard() { store().erase(app); }
};

// Ties a StoreGuard to `option`'s lifetime via its validator list, which is
// how this stays correct without any hook into CLI::App itself: `option` is
// owned by (and destroyed together with) the App it was added to — CLI::App
// keeps its options in a vector<unique_ptr<Option>> — so the guard fires no
// later than `app` does. It can fire earlier: App::remove_option(option), if
// anything ever called it, would destroy this Option (and its guard) while
// `app` and its other options are still alive, erasing `app`'s whole
// Registration — including slots from any other topic option still
// registered on `app` — not just this one's. The validator itself always
// reports "valid"; it exists solely to hold the guard, not to validate
// anything. Constructing the guard
// via make_shared's in-place forwarding matters here: building a StoreGuard
// temporary first and copying it into the shared_ptr would destroy that
// temporary — and thus erase the just-inserted entry — before this function
// even returns.
//
// The two "" arguments are load-bearing, not filler: CLI::Option::check()'s
// third parameter is a name used to look this validator up later, and its
// second is a description that CLI11 folds into the option's --help type
// string (Option::get_type_name() appends ":<description>" for every
// validator whose description is non-empty). Passing anything but "" there
// would both change every topic option's --help output and make this guard
// answer to option->get_validator("") / get_validator(0) — the two lookups
// CLI11 falls back to for an unnamed validator — ahead of whatever real
// validator Tasks 5-9 attach to the same option.
void arm_guard(const CLI::App & app, CLI::Option * option)
{
  const auto guard = std::make_shared<StoreGuard>(&app);
  option->check([guard](const std::string &) { return std::string{}; }, "", "");
}

}  // namespace

CLI::Option * add_topic_option(
  CLI::App & app, std::string flags, std::string & target, std::string description,
  const TopicSlotSpec & spec)
{
  CLI::Option * option = app.add_option(std::move(flags), target, std::move(description));
  // Designated initializers, not positional: the two overloads differ only in
  // which of multi_target/single_target is set, and a future reorder of
  // TopicSlot's fields must not be able to silently swap them.
  store()[&app].slots.push_back(
    TopicSlot{.option = option, .single_target = &target, .spec = spec});
  arm_guard(app, option);
  return option;
}

CLI::Option * add_topic_option(
  CLI::App & app, std::string flags, std::vector<std::string> & target, std::string description,
  const TopicSlotSpec & spec)
{
  CLI::Option * option = app.add_option(std::move(flags), target, std::move(description));
  store()[&app].slots.push_back(TopicSlot{.option = option, .multi_target = &target, .spec = spec});
  arm_guard(app, option);
  return option;
}

void set_topic_input(CLI::App & app, std::filesystem::path & input_target)
{
  store()[&app].input = &input_target;
}

std::span<const TopicSlot> topic_slots_of(const CLI::App & app)
{
  const auto it = store().find(&app);
  if (it == store().end()) {
    return {};
  }
  return it->second.slots;
}

const std::filesystem::path * topic_input_of(const CLI::App & app)
{
  const auto it = store().find(&app);
  if (it == store().end()) {
    return nullptr;
  }
  return it->second.input;
}

}  // namespace bagwiz::commands
