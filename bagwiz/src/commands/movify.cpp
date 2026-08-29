// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/movify_video.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/commands/topic_types.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/property.hpp"

#include <map>
#include <string>
#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.movify";
}  // namespace

// `bagwiz movify` renders a rosbag to video. Its only subcommand is `cam`,
// which renders image topic(s) to a video file.
class MovifyCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "movify"; }
  [[nodiscard]] std::string_view description() const override { return "Render a rosbag to video"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_cam(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kCam:
        return run_movify_video(video_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kCam };
  Subcommand selected_ = Subcommand::kNone;
  MovifyVideoArgs video_args_;

  void configure_cam(CLI::App & app)
  {
    auto * sub = app.add_subcommand("cam", "Render image topic(s) from a rosbag to a video file.");
    sub->add_option("-i,--input", video_args_.input_path, "Input ROS 2 rosbag (file or directory).")
      ->required()
      ->check(CLI::ExistingPath);
    set_topic_input(*sub, video_args_.input_path);
    add_topic_option(
      *sub, "-t,--topic", video_args_.topics,
      "Image topic(s) to render. Supported types: sensor_msgs/msg/Image (bgr8, rgb8) and "
      "sensor_msgs/msg/CompressedImage (JPEG/PNG). A literal name or a '*' glob; a glob's "
      "matches expand in lexicographic (topic-name) order. List several to arrange them in "
      "a grid (left to right, top to bottom — see --grid); the first topic drives the frame "
      "rate and output timing, and its frame size fixes the cell size. Repeatable.",
      TopicSlotSpec{.allowed_types = kImageTopicTypes})
      ->required()
      ->expected(-1);
    sub
      ->add_option(
        "-o,--output", video_args_.output_path,
        "Output video path. Extension selects container/codec: .mp4/.mkv/.mov -> H.264, "
        ".avi -> MJPEG.")
      ->required();
    sub->add_flag(
      "-w,--overwrite", video_args_.overwrite,
      "Replace an existing <output>. Without it, an existing output path stops the run.");
    sub
      ->add_option(
        "--grid", video_args_.grid,
        "Grid layout for multiple -t topics, as <cols>x<rows> (e.g. 2x2). Must hold at least "
        "as many cells as topics; extra cells stay black. When omitted, a near-square grid is "
        "derived from the topic count.")
      ->check([](const std::string & grid) {
        if (grid.empty()) {
          return std::string{"grid must not be empty; omit --grid for the automatic layout"};
        }
        return std::string{};
      });
    add_topic_option(
      *sub, "--cam-info", video_args_.camera_info_entries,
      "CameraInfo topic for rectification and --pcd: a bare <info_topic> applies to every "
      "view, an <image_topic>=<info_topic> entry overrides one view. Both halves are literal "
      "topic names. Views without an entry derive it from the image topic name following the "
      "standard /camera_info suffix rules. Repeatable.",
      TopicSlotSpec{
        .allowed_types = kImageTopicTypes,
        .mode = TopicSelectorMode::kLiteral,
        .pair_value = true,
        .pair_optional = true})
      ->check([](const std::string & topic) {
        if (topic.empty()) {
          return std::string{"cam-info topic must not be empty"};
        }
        return std::string{};
      })
      ->expected(-1);
    sub->add_flag(
      "!--no-rectify", video_args_.rectify,
      "Disable distortion correction (default on): each frame is otherwise rectified using "
      "the view's resolved CameraInfo. Applies to --pcd views too, whose points then "
      "project onto the raw image with the lens distortion applied. A view whose "
      "camera-info topic cannot be derived renders unrectified with a warning — pass "
      "--cam-info to name it explicitly. Point-cloud projection always requires a "
      "camera-info topic.");
    sub
      ->add_option(
        "--resize", video_args_.resize_scale,
        "Scale the cell width and height by this factor while preserving aspect ratio. "
        "1.0 keeps the original size, 0.5 halves both dimensions, 2.0 doubles them. "
        "Single-view: scales the output directly.")
      ->default_val(1.0f)
      ->check(CLI::Range(0.01f, 10.0f));
    auto * width_opt =
      sub
        ->add_option(
          "--width", video_args_.width,
          "Fix the composed output width in pixels: the cell width is the width split across "
          "the grid columns, and the cell height follows the primary frame's aspect ratio "
          "(both rounded down to even, so the output can be a few pixels narrower). Mutually "
          "exclusive with --resize.")
        ->check(CLI::PositiveNumber);
    width_opt->excludes("--resize");
    add_topic_option(
      *sub, "--pcd", video_args_.pointcloud_topics,
      "PointCloud2 topic(s) to project onto the frames: a bare value (a literal name or a "
      "'*' glob) projects onto every view, an <image_topic>=<pcd_selector> entry projects "
      "onto that view only. Repeatable. Points project onto the rectified image, or onto "
      "the raw image with lens distortion applied when --no-rectify is given. Requires a "
      "CameraInfo topic and a TF chain from each cloud frame to the camera frame.",
      TopicSlotSpec{
        .allowed_types = kPointCloud2Type, .pair_value = true, .pair_selector_rhs = true})
      ->check([](const std::string & topic) {
        if (topic.empty()) {
          return std::string{"pcd topic must not be empty"};
        }
        return std::string{};
      })
      ->expected(-1);
    const std::map<std::string, core::pointcloud::PointCloudProperty> property_map = {
      {"x", core::pointcloud::PointCloudProperty::kX},
      {"y", core::pointcloud::PointCloudProperty::kY},
      {"z", core::pointcloud::PointCloudProperty::kZ},
      {"distance", core::pointcloud::PointCloudProperty::kDistance},
      {"intensity", core::pointcloud::PointCloudProperty::kIntensity}};
    sub
      ->add_option(
        "--field", video_args_.property,
        "Point-cloud field used for coloring: x, y, z, distance, intensity.")
      ->transform(CLI::CheckedTransformer{property_map})
      ->default_val(core::pointcloud::PointCloudProperty::kDistance);
    sub
      ->add_option(
        "--min", video_args_.property_min, "Manual minimum value for field normalization.")
      ->capture_default_str();
    sub
      ->add_option(
        "--max", video_args_.property_max, "Manual maximum value for field normalization.")
      ->capture_default_str();
    const std::map<std::string, core::pointcloud::ColorScheme> scheme_map = {
      {"viridis", core::pointcloud::ColorScheme::kViridis},
      {"turbo", core::pointcloud::ColorScheme::kTurbo},
      {"jet", core::pointcloud::ColorScheme::kJet},
      {"plasma", core::pointcloud::ColorScheme::kPlasma},
      {"inferno", core::pointcloud::ColorScheme::kInferno},
      {"magma", core::pointcloud::ColorScheme::kMagma},
      {"rainbow", core::pointcloud::ColorScheme::kRainbow}};
    sub->add_option("--scheme", video_args_.colorscheme, "Color scheme for point coloring.")
      ->transform(CLI::CheckedTransformer{scheme_map})
      ->default_val(core::pointcloud::ColorScheme::kViridis);
    sub
      ->add_option(
        "--point-size", video_args_.point_size, "Side length of drawn square points in pixels.")
      ->default_val(2U)
      ->check(CLI::Range(1U, 64U));
    sub->add_option("--alpha", video_args_.alpha, "Point overlay opacity.")
      ->default_val(1.0f)
      ->check(CLI::Range(0.0f, 1.0f));
    sub->footer(
      "Frames stream straight to the encoder (no large temp files); the output is written\n"
      "atomically and a failed run leaves no partial file behind.");
    sub->callback([this]() { selected_ = Subcommand::kCam; });
  }
};

BAGWIZ_REGISTER_COMMAND(MovifyCommand)

}  // namespace bagwiz::commands
