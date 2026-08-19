// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/commands/topic_types.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "calib_cam_lidar_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.calib";
}  // namespace

// `bagwiz calib` is a command group for sensor-extrinsic calibration tools.
// Its actions are:
//   cam-lidar  - Refine one static-TF edge on a camera's chain by registering
//                the bag's LiDAR clouds (accumulated through the bag's pose
//                topic) against the bag's images (NID). Writes a
//                static-TF-tree YAML that `bagwiz tf static update` applies.
class CalibCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "calib"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Sensor-extrinsic calibration tools";
  }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_cam_lidar(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kCamLidar:
        return run_calib_cam_lidar(cam_lidar_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kCamLidar };
  Subcommand selected_ = Subcommand::kNone;

  CalibCamLidarArgs cam_lidar_args_;

  void configure_cam_lidar(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "cam-lidar",
      "Refine one static-TF edge on a camera's chain by registering the bag's "
      "LiDAR clouds against its images (NID). The clouds are accumulated into a "
      "map through the bag's own pose topic; writes a YAML that `tf static "
      "update` applies.");
    sub->add_option("-i,--input", cam_lidar_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    // --pcd, --pose, --cam, and --cam-info are declared as topic slots (not
    // plain options) so shell completion offers the bag's topics of each
    // accepted type, exactly as `walk`'s equivalents do.
    set_topic_input(*sub, cam_lidar_args_.input_path);
    add_topic_option(
      *sub, "--pcd", cam_lidar_args_.pcd_topic,
      "PointCloud2 topic to accumulate into the calibration map (needs an intensity field)",
      TopicSlotSpec{.allowed_types = kPointCloud2Type, .mode = TopicSelectorMode::kLiteral})
      ->required();
    add_topic_option(
      *sub, "--pose", cam_lidar_args_.pose_topic,
      "Self-position topic (TFMessage / Odometry / PoseStamped / "
      "PoseWithCovarianceStamped) the map and the image samples are placed by",
      TopicSlotSpec{.allowed_types = kUndistortPoseTopicTypes, .mode = TopicSelectorMode::kLiteral})
      ->required();
    add_topic_option(
      *sub, "--cam", cam_lidar_args_.cam_topic, "Image topic to calibrate against",
      TopicSlotSpec{.allowed_types = kImageTopicTypes, .mode = TopicSelectorMode::kLiteral})
      ->required();
    sub->add_option(
      "--of", cam_lidar_args_.of_frame,
      "Frame the --pose trajectory tracks; anchors the static TF chain to the camera and the "
      "per-cloud extrinsic (default: base_link)");
    sub->add_option(
      "--ref", cam_lidar_args_.ref_frame,
      "Frame the --pose trajectory is expressed in; the map is accumulated in it (default: map)");
    sub
      ->add_option(
        "--parent", cam_lidar_args_.parent_frame, "Parent frame of the static edge to refine")
      ->required();
    sub
      ->add_option(
        "--child", cam_lidar_args_.child_frame, "Child frame of the static edge to refine")
      ->required();
    auto * cam_info_opt = add_topic_option(
      *sub, "--cam-info", cam_lidar_args_.cam_info_topic,
      "CameraInfo topic (auto-resolved from the image topic when omitted)",
      TopicSlotSpec{.allowed_types = kCameraInfoType, .mode = TopicSelectorMode::kLiteral});
    sub->add_option(
      "-o,--output", cam_lidar_args_.output_path,
      "Output YAML path (default: <bag>_calib_cam_lidar.yaml)");
    sub->add_option(
      "--samples", cam_lidar_args_.samples, "Image samples to use (default 8, min 3)");
    sub->add_option(
      "--fix", cam_lidar_args_.fix_axes,
      "Comma list of axes to hold at the bag value (x,y,z,roll,pitch,yaw)");
    sub->add_option(
      "--max-trans", cam_lidar_args_.max_trans,
      "Trust region: max translation delta in meters (default 0.2)");
    sub->add_option(
      "--max-rot", cam_lidar_args_.max_rot_deg,
      "Trust region: max rotation delta in degrees (default 2.0)");
    sub->add_option("--nid-bins", cam_lidar_args_.nid_bins, "NID histogram bins (default 16)");
    sub->add_option(
      "--keyframe-dist", cam_lidar_args_.keyframe_dist,
      "Pose-gated keyframe sampling: open a new keyframe interval each time the interpolated "
      "pose moves this many meters, then use each picked interval's sharpest frame (default 0 = "
      "plain even time spacing)");
    sub->add_option(
      "--keyframe-rot", cam_lidar_args_.keyframe_rot_deg,
      "Rotation half of the keyframe gate, in degrees (default 0 = disabled)");
    sub->add_option(
      "--min-depth", cam_lidar_args_.min_depth,
      "Nearest projected point depth in meters (default 2)");
    sub->add_option(
      "--max-depth", cam_lidar_args_.max_depth,
      "Farthest projected point depth in meters (default 150)");
    sub->add_flag("--json", cam_lidar_args_.json, "Emit the stdout summary as JSON");
    sub->add_flag(
      "-w,--overwrite", cam_lidar_args_.overwrite, "Replace an existing -o/--output path");
    sub->callback([this, cam_info_opt]() {
      cam_lidar_args_.cam_info_given = cam_info_opt->count() > 0;
      selected_ = Subcommand::kCamLidar;
    });
  }
};

BAGWIZ_REGISTER_COMMAND(CalibCommand)

}  // namespace bagwiz::commands
