// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/movify.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/commands/topic_types.hpp"
#include "bagwiz/core/pointcloud/cloud_view.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/property.hpp"

#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::commands
{

// `bagwiz movify` renders a rosbag to video: the image topics named with
// --cam and the point clouds named with --pcd become the panels of one grid,
// one output frame per message of the clock topic, with point clouds
// optionally projected onto the camera panels (--cam-pcd). Every input is a role selector — there
// is no general topic operand, because no single topic is "the" topic of a composed video.
class MovifyCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "movify"; }
  [[nodiscard]] std::string_view description() const override { return "Render a rosbag to video"; }

  void configure(CLI::App & app) override
  {
    app.add_option("-i,--input", args_.input_path, "Input ROS 2 rosbag (file or directory).")
      ->required()
      ->check(CLI::ExistingPath);
    set_topic_input(app, args_.input_path);
    add_topic_option(
      app, "--cam", args_.cam_topics,
      "Image topic(s) to render as camera panels, in grid order (left to right, top to "
      "bottom — see --grid). Supported types: sensor_msgs/msg/Image (bgr8, rgb8) and "
      "sensor_msgs/msg/CompressedImage (JPEG/PNG). A literal name or a '*' glob; a glob's "
      "matches expand in lexicographic (topic-name) order. At least one --cam, --pcd or "
      "--gnss topic is required. Repeatable.",
      TopicSlotSpec{.allowed_types = kImageTopicTypes})
      ->expected(-1);
    add_topic_option(
      app, "--pcd", args_.pcd_topics,
      "PointCloud2 topic(s) to render as point-cloud panels: every listed topic is drawn "
      "into one panel per --view, each cloud transformed into the --frame frame at its own "
      "stamp. A literal name or a '*' glob. Repeatable.",
      TopicSlotSpec{.allowed_types = kPointCloud2Type})
      ->expected(-1);
    add_topic_option(
      app, "--gnss", args_.gnss_topic,
      "NavSatFix topic to render as a map panel, after the point-cloud panels: the vehicle's "
      "track in a local East-North-Up plan view with the current fix marked. A literal topic "
      "name.",
      TopicSlotSpec{.allowed_types = kNavSatFixType, .mode = TopicSelectorMode::kLiteral});
    app
      .add_option(
        "-o,--output", args_.output_path,
        "Output video path. Extension selects container/codec: .mp4/.mkv/.mov -> H.264, "
        ".avi -> MJPEG.")
      ->required();
    app.add_flag(
      "-w,--overwrite", args_.overwrite,
      "Replace an existing <output>. Without it, an existing output path stops the run.");
    const std::map<std::string, core::video::H264Backend> encoder_map{
      {"auto", core::video::H264Backend::kAuto},
      {"x264", core::video::H264Backend::kX264},
      {"nvenc", core::video::H264Backend::kNvenc}};
    app
      .add_option(
        "--encoder", args_.encoder,
        "H.264 encoder for .mp4/.mkv/.mov outputs: 'auto' uses NVIDIA NVENC for outputs "
        "larger than 1080p when this build and a GPU support it, else libx264; 'x264' and "
        "'nvenc' force one. .avi (MJPEG) ignores it. Default: auto.")
      ->transform(CLI::CheckedTransformer{encoder_map})
      ->default_val(core::video::H264Backend::kAuto);
    app
      .add_option(
        "--preset", args_.preset,
        "H.264 speed/quality preset, by libx264's names (ultrafast, superfast, veryfast, "
        "faster, fast, medium, slow, slower, veryslow); NVENC maps them onto its p1-p7. "
        "Default: medium.")
      ->check(
        CLI::IsMember(
          std::vector<std::string>(
            core::video::kH264Presets.begin(), core::video::kH264Presets.end())))
      ->default_val("medium");
    add_topic_option(
      app, "--clock", args_.clock,
      "Topic whose messages define the output frames: each message becomes one frame, its "
      "message rate sets the frame rate, and its frame size fixes the grid's cell size. Must "
      "be one of the --cam, --pcd or --gnss topics. Default: the first --cam topic, else the "
      "first --pcd topic, else the --gnss topic.",
      TopicSlotSpec{.allowed_types = kMovifyClockTopicTypes, .mode = TopicSelectorMode::kLiteral});
    app
      .add_option(
        "--grid", args_.grid,
        "Grid layout for the panels, as <cols>x<rows> (e.g. 2x2). Must hold at least as many "
        "cells as panels; extra cells stay black. When omitted, a near-square grid is derived "
        "from the panel count.")
      ->check([](const std::string & grid) {
        if (grid.empty()) {
          return std::string{"grid must not be empty; omit --grid for the automatic layout"};
        }
        return std::string{};
      });
    add_topic_option(
      app, "--cam-info", args_.camera_info_entries,
      "CameraInfo topic for rectification and --cam-pcd: a bare <info_topic> applies to every "
      "camera panel, an <image_topic>=<info_topic> entry overrides one panel. Both halves are "
      "literal topic names. Panels without an entry derive it from the image topic name "
      "following the standard /camera_info suffix rules. Repeatable.",
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
    app.add_flag(
      "!--no-rectify", args_.rectify,
      "Disable distortion correction (default on): each frame is otherwise rectified using "
      "the panel's resolved CameraInfo. Applies to --cam-pcd panels too, whose points then "
      "project onto the raw image with the lens distortion applied. A panel whose "
      "camera-info topic cannot be derived renders unrectified with a warning — pass "
      "--cam-info to name it explicitly. Point-cloud projection always requires a "
      "camera-info topic.");
    app
      .add_option(
        "--resize", args_.resize_scale,
        "Scale the clock panel's frame by this factor while preserving aspect ratio, which "
        "sets the cell size. 1.0 keeps the original size, 0.5 halves both dimensions, 2.0 "
        "doubles them.")
      ->default_val(1.0f)
      ->check(CLI::Range(0.01f, 10.0f));
    auto * width_opt =
      app
        .add_option(
          "--width", args_.width,
          "Fix the composed output width in pixels: the cell width is the width split across "
          "the grid columns, and the cell height follows the clock panel's aspect ratio "
          "(both rounded down to even, so the output can be a few pixels narrower). Mutually "
          "exclusive with --resize.")
        ->check(CLI::PositiveNumber);
    width_opt->excludes("--resize");
    add_topic_option(
      app, "--cam-pcd", args_.cam_pcd_entries,
      "PointCloud2 topic(s) to project onto the camera panels: a bare value (a literal name "
      "or a '*' glob) projects onto every panel, an <image_topic>=<pcd_selector> entry "
      "projects onto that panel only. Repeatable. Points project onto the rectified image, "
      "or onto the raw image with lens distortion applied when --no-rectify is given. "
      "Requires a CameraInfo topic and a TF chain from each cloud frame to the camera "
      "frame.",
      TopicSlotSpec{
        .allowed_types = kPointCloud2Type, .pair_value = true, .pair_selector_rhs = true})
      ->check([](const std::string & topic) {
        if (topic.empty()) {
          return std::string{"cam-pcd topic must not be empty"};
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
    app
      .add_option(
        "--field", args_.property,
        "Point-cloud field used for coloring: x, y, z, distance, intensity.")
      ->transform(CLI::CheckedTransformer{property_map})
      ->default_val(core::pointcloud::PointCloudProperty::kDistance);
    app.add_option("--min", args_.property_min, "Manual minimum value for field normalization.")
      ->capture_default_str();
    app.add_option("--max", args_.property_max, "Manual maximum value for field normalization.")
      ->capture_default_str();
    const std::map<std::string, core::pointcloud::ColorScheme> scheme_map = {
      {"viridis", core::pointcloud::ColorScheme::kViridis},
      {"turbo", core::pointcloud::ColorScheme::kTurbo},
      {"jet", core::pointcloud::ColorScheme::kJet},
      {"plasma", core::pointcloud::ColorScheme::kPlasma},
      {"inferno", core::pointcloud::ColorScheme::kInferno},
      {"magma", core::pointcloud::ColorScheme::kMagma},
      {"rainbow", core::pointcloud::ColorScheme::kRainbow}};
    app.add_option("--scheme", args_.colorscheme, "Color scheme for point coloring.")
      ->transform(CLI::CheckedTransformer{scheme_map})
      ->default_val(core::pointcloud::ColorScheme::kViridis);
    app
      .add_option("--point-size", args_.point_size, "Side length of drawn square points in pixels.")
      ->default_val(2U)
      ->check(CLI::Range(1U, 64U));
    app.add_option("--alpha", args_.alpha, "Point overlay opacity.")
      ->default_val(1.0f)
      ->check(CLI::Range(0.0f, 1.0f));
    const std::map<std::string, core::pointcloud::CloudProjection> view_map = {
      {"3d", core::pointcloud::CloudProjection::kPerspective},
      {"bev", core::pointcloud::CloudProjection::kBev}};
    app
      .add_option(
        "--view", args_.views,
        "Projection(s) of the point-cloud panels, one panel each: '3d' is a perspective view "
        "from a virtual camera (--elev/--azim/--dist) looking at the --frame origin, 'bev' a "
        "top-down view of its XY plane (up is +x, left is +y). Default: 3d.")
      ->transform(CLI::CheckedTransformer{view_map})
      ->expected(-1);
    app.add_option(
      "--frame", args_.frame,
      "TF frame the point-cloud panels draw in; every cloud is transformed into it at its own "
      "stamp. Default: the first --pcd topic's frame.");
    app
      .add_option(
        "--range", args_.range_m,
        "BEV half-extent in meters: the bev view spans +-range on both ground axes. Default: "
        "the 95th percentile of the first cloud's point distances from the frame origin, "
        "so a few far returns do not shrink the scene.")
      ->check(CLI::PositiveNumber);
    app
      .add_option(
        "--elev", args_.elev_deg, "3d view: camera elevation above the XY plane in degrees.")
      ->default_val(20.0)
      ->check(CLI::Range(-89.0, 89.0));
    app
      .add_option(
        "--azim", args_.azim_deg,
        "3d view: camera azimuth around the +z axis in degrees, measured from +x. 180 looks "
        "at the scene from behind the sensor.")
      ->default_val(180.0);
    app
      .add_option(
        "--dist", args_.dist_m, "3d view: camera distance from the --frame origin in meters.")
      ->default_val(30.0)
      ->check(CLI::PositiveNumber);
    app
      .add_option(
        "--map-range", args_.map_range_m,
        "Map panel: follow the current fix, the panel spanning +-range meters around it. "
        "Default: the whole track fitted into the panel.")
      ->check(CLI::PositiveNumber);
    app.footer(
      "Frames stream straight to the encoder (no large temp files); the output is written\n"
      "atomically and a failed run leaves no partial file behind.");
  }

  int run() override { return run_movify(args_); }

private:
  MovifyArgs args_;
};

BAGWIZ_REGISTER_COMMAND(MovifyCommand)

}  // namespace bagwiz::commands
