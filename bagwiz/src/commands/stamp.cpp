// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/stamp_sync.hpp"
#include "bagwiz/core/base/logging.hpp"

#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.stamp";
}  // namespace

// `bagwiz stamp` is a command group for message-timestamp edits. Ships `sync`
// (overwrite each message's header.stamp with its receive time); the group
// leaves room for further stamp operations (offset shifts, the reverse sync).
class StampCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "stamp"; }
  [[nodiscard]] std::string_view description() const override { return "Edit message timestamps"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_sync(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kSync:
        return run_stamp_sync(sync_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kSync };
  Subcommand selected_ = Subcommand::kNone;

  StampSyncArgs sync_args_;

  void configure_sync(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "sync",
      "Overwrite each message's header.stamp with its receive (log) time. Applies to every topic "
      "whose type leads with a std_msgs/Header (the ROS 2 stamped-message convention); messages "
      "on other topics are copied verbatim.");
    sub->add_option("-i,--input", sync_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option(
      "-o,--output", sync_args_.output_path,
      "Write the result to this new bag instead of rewriting <input> in place.");
    sub->add_flag(
      "-w,--overwrite", sync_args_.overwrite,
      "Replace an existing -o/--output path. Without it, an existing output path stops the run.");
    sub->callback([this]() { selected_ = Subcommand::kSync; });
  }
};

BAGWIZ_REGISTER_COMMAND(StampCommand)

}  // namespace bagwiz::commands
