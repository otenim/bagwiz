// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/logging.hpp"

#include <algorithm>
#include <exception>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
constexpr const char * kVersion = "0.1.0";
constexpr const char * kMainLogger = "bagwiz.main";

bool is_help_flag(const std::string & token)
{
  return token == "-h" || token == "--help";
}

std::optional<std::vector<std::string>> build_forced_help_argv(
  int argc, char ** argv, const bagwiz::commands::Registry & registry)
{
  bool requested_help = false;
  for (int i = 1; i < argc; ++i) {
    if (is_help_flag(argv[i])) {
      requested_help = true;
      break;
    }
  }
  if (!requested_help) {
    return std::nullopt;
  }

  std::unordered_set<std::string> top_level_names;
  top_level_names.reserve(registry.all().size());
  for (const auto & cmd : registry.all()) {
    top_level_names.insert(std::string(cmd->name()));
  }
  const std::unordered_map<std::string, std::unordered_set<std::string>> nested = {
    {"convert", {"1to2", "2to1", "storage"}},
    {"traj", {"dump"}},
    {"tf", {"tree", "walk", "inject-static"}}};

  std::vector<std::string> out;
  out.reserve(4);
  out.emplace_back(argv[0]);

  std::string parent;
  for (int i = 1; i < argc; ++i) {
    const std::string token(argv[i]);
    if (is_help_flag(token)) {
      break;
    }
    if (token.empty() || token.front() == '-') {
      continue;
    }
    if (parent.empty()) {
      if (top_level_names.find(token) != top_level_names.end()) {
        parent = token;
        out.push_back(token);
      }
      continue;
    }
    const auto nested_it = nested.find(parent);
    if (nested_it != nested.end() && nested_it->second.find(token) != nested_it->second.end()) {
      out.push_back(token);
    }
    break;
  }

  out.emplace_back("--help");
  return out;
}
}  // namespace

int main(int argc, char ** argv)
{
  bagwiz::core::init_logging();

  CLI::App app{"bagwiz - Fast CLI for analyzing and processing ROS 2 rosbags"};
  app.set_version_flag("--version", kVersion);
  app.require_subcommand(1);

  auto & registry = bagwiz::commands::Registry::instance();
  // Selected is set by the top-level subcommand callback; nested subcommands
  // store their own state on the Command instance. run() fires once after
  // parsing completes so parent and child callbacks can both observe args
  // before the command executes.
  bagwiz::commands::Command * selected = nullptr;
  for (const auto & cmd : registry.all()) {
    auto * sub = app.add_subcommand(std::string(cmd->name()), std::string(cmd->description()));
    cmd->configure(*sub);
    sub->callback([&selected, raw = cmd.get()]() { selected = raw; });
  }

  if (const auto forced_help = build_forced_help_argv(argc, argv, registry)) {
    std::vector<char *> raw_argv;
    raw_argv.reserve(forced_help->size());
    for (const auto & s : *forced_help) {
      raw_argv.push_back(const_cast<char *>(s.c_str()));
    }
    int forced_argc = static_cast<int>(raw_argv.size());
    CLI11_PARSE(app, forced_argc, raw_argv.data());
    return 0;
  }

  try {
    CLI11_PARSE(app, argc, argv);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_FATAL(kMainLogger, "Unhandled exception during argument parsing: %s", e.what());
    return 1;
  }

  if (!selected) {
    // CLI11 handled --help/--version or required_subcommand already printed
    // an error; nothing further to do.
    return 0;
  }

  try {
    return selected->run();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_FATAL(kMainLogger, "Command failed: %s", e.what());
    return 1;
  }
}
