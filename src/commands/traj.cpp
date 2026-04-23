// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/introspection_loader.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/message_deserializer.hpp"
#include "bagwiz/core/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <fmt/core.h>

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.traj";
constexpr const char * kFormatTum = "tum";

}  // namespace

// `bagwiz traj <input> <topic> <output> [-f tum]` extracts the pose
// trajectory of one topic and writes it to disk in a trajectory exchange
// format. Only TUM is implemented today; the `-f` flag is plumbed so more
// formats (KITTI, bag2, ...) can be added without changing the CLI shape.
//
// Supported input types are anything for which core::extract_pose can
// find a (header, pose|transform) pair via introspection —
// PoseStamped / PoseWithCovarianceStamped / Odometry / TransformStamped
// and close cousins. Array-shaped types like tf2_msgs/msg/TFMessage are
// rejected with a clear error (they would need a frame-pair selector).
class TrajCommand : public Command
{
public:
  std::string_view name() const override { return "traj"; }
  std::string_view description() const override
  {
    return "Extract a topic's pose trajectory and save it (currently TUM format)";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("input", input_path_, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    app.add_option("topic", topic_, "Topic name to extract poses from")->required();
    app.add_option("output", output_path_, "Output file path")->required();
    app.add_option("-f,--format", format_, "Output format")
      ->default_val(kFormatTum)
      ->check(CLI::IsMember({kFormatTum}));
  }

  int run() override
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(input_path_);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", input_path_.c_str(), e.what());
      return 1;
    }

    const io::TopicInfo * topic_info = nullptr;
    for (const auto & t : reader->topics()) {
      if (t.name == topic_) {
        topic_info = &t;
        break;
      }
    }
    if (topic_info == nullptr) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is not present in %s", topic_.c_str(), input_path_.c_str());
      return 1;
    }
    const std::string type_name = topic_info->type;

    io::ReadFilter filter;
    filter.topics.push_back(topic_);
    reader->set_filter(filter);

    const core::IntrospectionLoad introspection = core::load_introspection(type_name);
    if (!introspection.ok()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Message type '%s' could not be loaded (tried %s; error: %s). Source a workspace that "
        "provides the package and re-run.",
        type_name.c_str(), introspection.library_name.c_str(), introspection.error.c_str());
      return 1;
    }

    std::vector<core::TrajectoryPose> poses;
    std::int64_t decoded = 0;
    std::int64_t skipped = 0;
    bool shape_logged = false;

    io::RawMessage raw;
    while (true) {
      try {
        if (!reader->next(raw)) {
          break;
        }
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "read error: %s", e.what());
        return 1;
      }
      ++decoded;
      try {
        const core::DeserializedMessage msg(introspection, raw.payload);
        const auto pose = core::extract_pose(msg.members(), msg.data());
        if (pose) {
          poses.push_back(*pose);
        } else {
          ++skipped;
          if (!shape_logged) {
            BAGWIZ_LOG_ERROR(
              kLogger,
              "Topic '%s' of type '%s' does not expose a (header, pose|transform) pair that can "
              "be turned into a trajectory sample.",
              topic_.c_str(), type_name.c_str());
            shape_logged = true;
          }
        }
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "decode error on message #%ld: %s", decoded, e.what());
        return 1;
      }
    }

    if (poses.empty()) {
      if (skipped > 0) {
        return 1;  // shape error was already logged
      }
      BAGWIZ_LOG_WARN(
        kLogger, "No messages found on topic '%s'; writing an empty trajectory.", topic_.c_str());
    }

    if (format_ != kFormatTum) {
      // Shouldn't be reachable (CLI::IsMember filter above), but be
      // defensive in case the check is ever relaxed without extending
      // this switch.
      BAGWIZ_LOG_ERROR(kLogger, "Unsupported format '%s'", format_.c_str());
      return 1;
    }

    std::ofstream out(output_path_, std::ios::out | std::ios::trunc);
    if (!out) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Failed to open output path %s for writing", output_path_.c_str());
      return 1;
    }
    core::write_tum(out, poses);
    out.close();

    BAGWIZ_LOG_INFO(
      kLogger, "Wrote %zu poses (from %ld messages) to %s in %s format", poses.size(), decoded,
      output_path_.c_str(), format_.c_str());
    return 0;
  }

private:
  std::filesystem::path input_path_;
  std::string topic_;
  std::filesystem::path output_path_;
  std::string format_;
};

BAGWIZ_REGISTER_COMMAND(TrajCommand)

}  // namespace bagwiz::commands
