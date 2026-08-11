// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// `bagwiz walk` is TTY-interactive: its only behaviour reachable from a test
// process is the non-interactive early exit, pinned here (exit code + guard
// message) so the decomposition cannot silently drop it. stdin/stdout are
// forced onto pipes so the guard triggers deterministically even when the
// test binary itself runs on a terminal.

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/commands/topic_types.hpp"
#include "topic_slot_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <string>
#include <vector>

namespace
{

// RAII: replace stdin/stdout/stderr with pipes for the guarded scope.
// stderr's read end is exposed so the test can inspect logged output.
class StdFdsToPipes
{
public:
  StdFdsToPipes()
  {
    // Plain calls: a failure here is an OS-level resource exhaustion, not a
    // test condition, and surfaces as a failed read downstream anyway.
    if (pipe(in_pipe_) != 0 || pipe(out_pipe_) != 0 || pipe(err_pipe_) != 0) {
      return;
    }
    saved_stdin_ = dup(STDIN_FILENO);
    saved_stdout_ = dup(STDOUT_FILENO);
    saved_stderr_ = dup(STDERR_FILENO);
    dup2(in_pipe_[0], STDIN_FILENO);
    dup2(out_pipe_[1], STDOUT_FILENO);
    dup2(err_pipe_[1], STDERR_FILENO);
    fcntl(err_pipe_[0], F_SETFL, O_NONBLOCK);
  }

  StdFdsToPipes(const StdFdsToPipes &) = delete;
  StdFdsToPipes & operator=(const StdFdsToPipes &) = delete;

  ~StdFdsToPipes()
  {
    dup2(saved_stdin_, STDIN_FILENO);
    dup2(saved_stdout_, STDOUT_FILENO);
    dup2(saved_stderr_, STDERR_FILENO);
    close(saved_stdin_);
    close(saved_stdout_);
    close(saved_stderr_);
    close(in_pipe_[0]);
    close(in_pipe_[1]);
    close(out_pipe_[0]);
    close(out_pipe_[1]);
    close(err_pipe_[0]);
    close(err_pipe_[1]);
  }

  // Drain whatever has been written to stderr so far (non-blocking).
  std::string take_stderr()
  {
    fflush(nullptr);  // push any libc-buffered stderr into the pipe
    std::string out;
    char buf[4096];
    for (;;) {
      const ssize_t n = read(err_pipe_[0], buf, sizeof(buf));
      if (n <= 0) {
        break;
      }
      out.append(buf, static_cast<std::size_t>(n));
    }
    return out;
  }

private:
  // Initialized to -1 so that if a pipe() call in the constructor fails, the
  // destructor's dup2/close calls hit only invalid fds (harmless EBADF
  // no-ops) instead of indeterminate ones.
  int in_pipe_[2] = {-1, -1};
  int out_pipe_[2] = {-1, -1};
  int err_pipe_[2] = {-1, -1};
  int saved_stdin_ = -1;
  int saved_stdout_ = -1;
  int saved_stderr_ = -1;
};

bagwiz::commands::Command * find_walk_command()
{
  for (const auto & cmd : bagwiz::commands::Registry::instance().all()) {
    if (cmd->name() == "walk") {
      return cmd.get();
    }
  }
  return nullptr;
}

TEST(WalkCommand, NonTtyInvocationExitsWithGuardMessage)
{
  bagwiz::commands::Command * walk = find_walk_command();
  ASSERT_NE(walk, nullptr);

  StdFdsToPipes pipes;
  CLI::App app;
  walk->configure(app);
  // stdin/stdout are pipes here, so the TTY guard must fire before any bag
  // access: the input path does not need to exist for this check, but CLI11
  // validates ExistingPath at parse time, so use a real one.
  const std::string argv0 = "bagwiz";
  const std::string argv1 = "-i";
  const std::string argv2 = "/tmp";
  const std::string argv3 = "-t";
  const std::string argv4 = "/topic";
  const char * argv[] = {argv0.c_str(), argv1.c_str(), argv2.c_str(), argv3.c_str(), argv4.c_str()};
  ASSERT_NO_THROW(app.parse(5, argv));

  EXPECT_EQ(walk->run(), 1);
  EXPECT_NE(
    pipes.take_stderr().find("walk requires an interactive terminal (stdin+stdout must be TTY)"),
    std::string::npos);
}

// Exercises the real WalkCommand::configure() — reached through the
// process-wide command registry that walk.cpp's BAGWIZ_REGISTER_COMMAND
// registrar populates — rather than a hand-mirrored copy of its wiring.
// Dropping `.mode = kLiteral` from either declaration fails this test
// directly (and, per topic_expand.cpp's assign_slot_result(), would also
// leave -t/--topic silently keeping only the first match of a glob).
TEST(WalkCommand, TopicOptionsAreLiteralOnly)
{
  bagwiz::commands::Command * walk = find_walk_command();
  ASSERT_NE(walk, nullptr);

  CLI::App app;
  walk->configure(app);

  const auto slots = bagwiz::commands::topic_slots_of(app);
  ASSERT_EQ(slots.size(), 2U);  // -t/--topic, --cam-info

  const auto * topic_slot = bagwiz::test::slot_for(slots, "topic");
  ASSERT_NE(topic_slot, nullptr);
  EXPECT_EQ(topic_slot->spec.mode, bagwiz::commands::TopicSelectorMode::kLiteral);
  EXPECT_TRUE(topic_slot->spec.allowed_types.empty());
  EXPECT_TRUE(topic_slot->option->get_required());

  const auto * cam_info_slot = bagwiz::test::slot_for(slots, "cam-info");
  ASSERT_NE(cam_info_slot, nullptr);
  EXPECT_EQ(cam_info_slot->spec.mode, bagwiz::commands::TopicSelectorMode::kLiteral);
  ASSERT_EQ(cam_info_slot->spec.allowed_types.size(), 1U);
  EXPECT_EQ(cam_info_slot->spec.allowed_types[0], "sensor_msgs/msg/CameraInfo");
}

}  // namespace
