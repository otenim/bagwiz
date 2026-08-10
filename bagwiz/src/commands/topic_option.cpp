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

// Erases `app`'s entry once nothing references this guard anymore. A real
// CLI::App is constructed once at startup and lives for the process, so in
// production this destructor only ever runs at process exit, after `store()`
// itself is gone — a no-op. It matters for tests: a short-lived CLI::App goes
// out of scope at the end of every TEST(), and the allocator routinely hands
// the next TEST()'s CLI::App (and its Options) the very same addresses. A
// bare pointer-keyed map cannot tell those two App instances apart on its
// own, so without this guard a dead App's slots would leak into whichever
// later App happens to reuse its address.
struct StoreGuard
{
  const CLI::App * app;
  explicit StoreGuard(const CLI::App * owning_app) : app{owning_app} {}
  ~StoreGuard() { store().erase(app); }
};

// Ties a StoreGuard to `option`'s lifetime via its validator list, which is
// how this stays correct without any hook into CLI::App itself: `option` is
// owned by (and destroyed together with) the App it was added to — CLI::App
// keeps its options in a vector<unique_ptr<Option>> — so the guard fires
// exactly when `app` does. The validator itself always reports "valid"; it
// exists solely to hold the guard, not to validate anything. Constructing the
// guard via make_shared's in-place forwarding matters here: building a
// StoreGuard temporary first and copying it into the shared_ptr would destroy
// that temporary — and thus erase the just-inserted entry — before this
// function even returns.
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
  store()[&app].slots.push_back(TopicSlot{option, nullptr, &target, spec});
  arm_guard(app, option);
  return option;
}

CLI::Option * add_topic_option(
  CLI::App & app, std::string flags, std::vector<std::string> & target, std::string description,
  const TopicSlotSpec & spec)
{
  CLI::Option * option = app.add_option(std::move(flags), target, std::move(description));
  store()[&app].slots.push_back(TopicSlot{option, &target, nullptr, spec});
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
