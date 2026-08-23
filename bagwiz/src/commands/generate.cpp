// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/generate_video.hpp"
#include "bagwiz/commands/generate_video_pcd_scan.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/commands/topic_types.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/property.hpp"
#include "bagwiz/core/pointcloud/scan_pattern.hpp"

#include <map>
#include <string>
#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.generate";
}  // namespace

// `bagwiz generate` is a command group for producing non-rosbag *media* from a
// rosbag (rosbag -> media, not rosbag -> rosbag). Its first subcommand group
// `video` holds video renderers: `video cam` renders an image topic to a video
// file, `video scan` renders a point-cloud topic's scan pattern (points
// appearing in firing order) to a video file. The nesting leaves room for
// further media generators (image sequences, GIFs, ...) and further video
// sources.
class GenerateCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "generate"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Generate non-rosbag media from a rosbag";
  }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    auto * video = app.add_subcommand("video", "Video rendering");
    video->require_subcommand(1);
    configure_cam(*video);
    configure_pcd_scan(*video);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kCam:
        return run_generate_video(video_args_);
      case Subcommand::kPcdScan:
        return run_generate_video_pcd_scan(pcd_scan_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kCam, kPcdScan };
  Subcommand selected_ = Subcommand::kNone;
  GenerateVideoArgs video_args_;
  GenerateVideoPcdScanArgs pcd_scan_args_;

  void configure_cam(CLI::App & video)
  {
    auto * sub =
      video.add_subcommand("cam", "Render image topic(s) from a rosbag to a video file.");
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
      "CameraInfo topic for --rectify and --pcd: a bare <info_topic> applies to every view, "
      "an <image_topic>=<info_topic> entry overrides one view. Both halves are literal topic "
      "names. Views without an entry derive it from the image topic name following the "
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
    sub
      ->add_flag(
        "--rectify,!--no-rectify", video_args_.rectify,
        "Apply distortion correction to each frame using each view's resolved CameraInfo "
        "(on by default; --no-rectify opts out). A view whose camera-info topic cannot be "
        "derived renders unrectified with a warning — pass --cam-info to name it "
        "explicitly. Point-cloud projection always requires a camera-info topic.")
      ->default_val(true);
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
      "onto that view only. Repeatable. Projecting implies distortion correction for that "
      "view and requires a CameraInfo topic and a TF chain from each cloud frame to the "
      "camera frame.",
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

  void configure_pcd_scan(CLI::App & video)
  {
    auto * sub = video.add_subcommand(
      "scan",
      "Render a point-cloud topic's scan pattern from a rosbag to a video file. Within each "
      "sweep the points appear in firing order, read from the cloud's per-point time field "
      "('t', 'time', 'time_stamp', or 'timestamp'), colored by their sweep-relative time.");
    sub
      ->add_option(
        "-i,--input", pcd_scan_args_.input_path, "Input ROS 2 rosbag (file or directory).")
      ->required()
      ->check(CLI::ExistingPath);
    set_topic_input(*sub, pcd_scan_args_.input_path);
    add_topic_option(
      *sub, "-t,--topic", pcd_scan_args_.topic,
      "PointCloud2 topic to render. A per-point time field is required; clouds without one "
      "stop the run.",
      TopicSlotSpec{.allowed_types = kPointCloud2Type, .mode = TopicSelectorMode::kLiteral})
      ->required();
    sub
      ->add_option(
        "-o,--output", pcd_scan_args_.output_path,
        "Output video path. Extension selects container/codec: .mp4/.mkv/.mov -> H.264, "
        ".avi -> MJPEG.")
      ->required();
    sub->add_flag(
      "-w,--overwrite", pcd_scan_args_.overwrite,
      "Replace an existing <output>. Without it, an existing output path stops the run.");
    const std::map<std::string, core::pointcloud::ScanPatternProjection> view_map = {
      {"bev", core::pointcloud::ScanPatternProjection::kBev},
      {"3d", core::pointcloud::ScanPatternProjection::kPerspective}};
    sub
      ->add_option(
        "--view", pcd_scan_args_.view,
        "Projection space: 'bev' is a top-down XY view centered on the sensor, '3d' is a "
        "perspective view from a fixed camera (--elev/--azim/--dist) looking at the sensor.")
      ->transform(CLI::CheckedTransformer{view_map})
      ->default_val(core::pointcloud::ScanPatternProjection::kPerspective);
    sub->add_option("--width", pcd_scan_args_.width, "Output width in pixels. Must be even.")
      ->default_val(1280U)
      ->check(CLI::Range(2U, 7680U));
    sub->add_option("--height", pcd_scan_args_.height, "Output height in pixels. Must be even.")
      ->default_val(720U)
      ->check(CLI::Range(2U, 4320U));
    sub
      ->add_option(
        "--fps", pcd_scan_args_.fps,
        "Output frame rate in fps. Each sweep spans round(--fps / (cloud rate x --speed)) "
        "video frames (at least 1), so a higher value gives a smoother animation.")
      ->default_val(60U)
      ->check(CLI::Range(1U, 240U));
    sub
      ->add_option(
        "--speed", pcd_scan_args_.speed,
        "Playback speed as a fraction of real time: 1.0 plays each sweep in its recorded "
        "duration, 0.1 slows the animation to one tenth.")
      ->default_val(0.1)
      ->check(CLI::Range(0.001, 100.0));
    sub
      ->add_option(
        "--range", pcd_scan_args_.range_m,
        "BEV half-extent in meters: the BEV view spans +-range on both ground axes. Not used "
        "by the 3D view. Default: the largest finite XY distance in the first cloud.")
      ->check(CLI::PositiveNumber);
    sub
      ->add_option(
        "--elev", pcd_scan_args_.elev_deg,
        "3D view: camera elevation above the XY plane in degrees.")
      ->default_val(20.0)
      ->check(CLI::Range(-89.0, 89.0));
    sub
      ->add_option(
        "--azim", pcd_scan_args_.azim_deg,
        "3D view: camera azimuth around the +z axis in degrees, measured from +x. "
        "180 looks at the scene from behind the sensor.")
      ->default_val(180.0);
    sub
      ->add_option(
        "--dist", pcd_scan_args_.dist_m, "3D view: camera distance from the sensor in meters.")
      ->default_val(30.0)
      ->check(CLI::PositiveNumber);
    const std::map<std::string, core::pointcloud::ColorScheme> scheme_map = {
      {"viridis", core::pointcloud::ColorScheme::kViridis},
      {"turbo", core::pointcloud::ColorScheme::kTurbo},
      {"jet", core::pointcloud::ColorScheme::kJet},
      {"plasma", core::pointcloud::ColorScheme::kPlasma},
      {"inferno", core::pointcloud::ColorScheme::kInferno},
      {"magma", core::pointcloud::ColorScheme::kMagma},
      {"rainbow", core::pointcloud::ColorScheme::kRainbow}};
    sub
      ->add_option(
        "--scheme", pcd_scan_args_.colorscheme,
        "Color scheme for the sweep-relative time coloring.")
      ->transform(CLI::CheckedTransformer{scheme_map})
      ->default_val(core::pointcloud::ColorScheme::kViridis);
    sub
      ->add_option(
        "--point-size", pcd_scan_args_.point_size, "Side length of drawn square points in pixels.")
      ->default_val(2U)
      ->check(CLI::Range(1U, 64U));
    sub->footer(
      "Frames stream straight to the encoder (no large temp files); the output is written\n"
      "atomically and a failed run leaves no partial file behind.");
    sub->callback([this]() { selected_ = Subcommand::kPcdScan; });
  }
};

BAGWIZ_REGISTER_COMMAND(GenerateCommand)

}  // namespace bagwiz::commands
