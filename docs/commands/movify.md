# `bagwiz movify`

Render a rosbag to video: the image topics named with `--cam`, the point
clouds named with `--pcd` (drawn in 3D and/or from above) and the GNSS track
named with `--gnss` (a plan view of the vehicle's position) become the
panels of one grid — a single view, or a multi-view grid — with point clouds
optionally projected onto the camera panels. One topic is the clock: each of
its messages becomes one output frame, its message rate sets the frame rate,
and its panel's size fixes the grid's cell size. The container/codec is
chosen from the `<output>` extension.

## Usage

```text
bagwiz movify -i <input> [--cam <topic>...] [--pcd <topic>...] [--gnss <topic>] [--pose <topic>] -o <output> [OPTIONS]
```

## Examples

```bash
# Render a camera topic to an MP4 (H.264).
bagwiz movify -i drive.mcap --cam /sensing/camera/image_raw -o out.mp4

# Render to MJPEG AVI, replacing an existing file.
bagwiz movify -i drive_dir/ --cam /sensing/camera/image_raw -o clip.avi -w

# Render with distortion correction (on by default) using an explicit
# CameraInfo topic.
bagwiz movify -i drive.mcap --cam /sensing/camera/image_raw -o out.mp4 \
  --cam-info /sensing/camera/camera_info

# Render the raw frames, skipping distortion correction.
bagwiz movify -i drive.mcap --cam /sensing/camera/image_raw -o out.mp4 --no-rectify

# Project a point cloud onto the camera panel, colored by distance.
bagwiz movify -i drive.mcap --cam /sensing/camera/image_raw/compressed -o out.mp4 \
  --cam-pcd /sensing/lidar/front/points --field distance --scheme turbo --point-size 3 --alpha 0.8

# Project several point clouds onto the same camera panel.
bagwiz movify -i drive.mcap --cam /sensing/camera/image_raw/compressed -o out.mp4 \
  --cam-pcd /sensing/lidar/front/points \
            /sensing/lidar/rear/points \
  --field distance --scheme turbo --point-size 3 --alpha 0.8

# Project every lidar point cloud under /sensing/lidar, via a glob (quoted so
# the shell doesn't expand it).
bagwiz movify -i drive.mcap --cam /sensing/camera/image_raw/compressed -o out.mp4 \
  --cam-pcd '/sensing/lidar/*/points'

# Project the point cloud onto the raw (unrectified) frame: the camera's lens
# distortion is applied to the projected points, keeping them aligned with the
# distorted image.
bagwiz movify -i drive.mcap --cam /sensing/camera/image_raw/compressed -o out.mp4 \
  --cam-pcd /sensing/lidar/front/points --no-rectify

# Render a lidar alone: a 3D view from a virtual camera behind the sensor,
# one frame per sweep at the sweep rate.
bagwiz movify -i drive.mcap --pcd /sensing/lidar/top/points -o lidar.mp4

# The same lidar from above (bird's-eye view), the extent fixed to +-80 m.
bagwiz movify -i drive.mcap --pcd /sensing/lidar/top/points -o bev.mp4 --view bev --range 80

# Both views side by side, with the virtual camera tuned.
bagwiz movify -i drive.mcap --pcd /sensing/lidar/top/points -o lidar.mp4 \
  --view 3d bev --elev 35 --azim 150 --dist 60

# Four lidars merged into one panel in the vehicle frame (through the bag's
# TF), colored by intensity.
bagwiz movify -i drive.mcap -o surround.mp4 --frame base_link --field intensity \
  --pcd '/sensing/lidar/*/points'

# A camera next to the lidar: the camera drives the frames, the lidar panel
# fills a cell of the camera's size.
bagwiz movify -i drive.mcap -o cam_lidar.mp4 \
  --cam /sensing/camera/front/image_raw/compressed --pcd /sensing/lidar/top/points

# The lidar as the clock instead: one frame per sweep, the camera following.
bagwiz movify -i drive.mcap -o cam_lidar.mp4 \
  --cam /sensing/camera/front/image_raw/compressed --pcd /sensing/lidar/top/points \
  --clock /sensing/lidar/top/points

# The lidar next to the vehicle's GNSS track: a plan view of the whole drive
# with the current position marked.
bagwiz movify -i drive.mcap -o lidar_map.mp4 \
  --pcd /sensing/lidar/top/points --gnss /sensing/gnss/nav_sat_fix

# The map following the vehicle instead, 100 m around it.
bagwiz movify -i drive.mcap -o lidar_map.mp4 \
  --pcd /sensing/lidar/top/points --gnss /sensing/gnss/nav_sat_fix --map-range 100

# The vehicle's trajectory (its odometry) drawn over the camera and the
# lidar: the ten seconds behind and ahead of every frame.
bagwiz movify -i drive.mcap -o drive.mp4 \
  --cam /sensing/camera/front/image_raw/compressed --pcd /sensing/lidar/top/points \
  --pose /localization/odometry

# Render at half resolution to reduce output file size.
bagwiz movify -i drive.mcap --cam /sensing/camera/image_raw/compressed -o out.mp4 --resize 0.5

# Multi-view: two cameras side by side (auto grid), the front camera driving
# the frame rate.
bagwiz movify -i drive.mcap -o front_rear.mp4 \
  --cam /sensing/camera/front/image_raw/compressed \
        /sensing/camera/rear/image_raw/compressed

# Multi-view with the rear camera as the clock instead of the first panel.
bagwiz movify -i drive.mcap -o front_rear.mp4 \
  --cam /sensing/camera/front/image_raw/compressed \
        /sensing/camera/rear/image_raw/compressed \
  --clock /sensing/camera/rear/image_raw/compressed

# Multi-view from a glob (quoted so the shell doesn't expand it): every
# matching camera topic, expanded in topic-name order, at a fixed width.
bagwiz movify -i drive.mcap -o all_cams.mp4 --width 3840 \
  --cam '/sensing/camera/camera*/image_raw/compressed'

# Multi-view at a fixed output width: three cameras on an auto 2x2 grid, the
# composed video exactly 1920 px wide (cells 960x540 for 16:9 inputs).
bagwiz movify -i drive.mcap -o surround.mp4 --width 1920 \
  --cam /sensing/camera/front/image_raw/compressed \
        /sensing/camera/rear/image_raw/compressed \
        /sensing/camera/left/image_raw/compressed

# Multi-view with an explicit 2x2 grid (three cameras; the fourth cell stays
# black).
bagwiz movify -i drive.mcap -o surround.mp4 --grid 2x2 \
  --cam /sensing/camera/front/image_raw/compressed \
        /sensing/camera/rear/image_raw/compressed \
        /sensing/camera/left/image_raw/compressed

# Multi-view with a point cloud projected onto every panel (bare --cam-pcd value).
bagwiz movify -i drive.mcap -o overlay_all.mp4 \
  --cam /sensing/camera/front/image_raw/compressed \
        /sensing/camera/rear/image_raw/compressed \
  --cam-pcd /sensing/lidar/top/points

# Multi-view with per-panel point-cloud bindings: each camera gets its own
# lidar's projection.
bagwiz movify -i drive.mcap -o overlay_each.mp4 \
  --cam /sensing/camera/front/image_raw/compressed \
        /sensing/camera/rear/image_raw/compressed \
  --cam-pcd /sensing/camera/front/image_raw/compressed=/sensing/lidar/front/points \
            /sensing/camera/rear/image_raw/compressed=/sensing/lidar/rear/points
```

## Options

| Flag                      | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`   | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| `--cam <topic>...`        | Image topic(s) to render as camera panels, in grid order (left to right, top to bottom). Supported types: `sensor_msgs/msg/Image` (`bgr8`, `rgb8`) and `sensor_msgs/msg/CompressedImage` (JPEG/PNG). A literal topic name or a `*` glob (see [Topic selectors](topic.md#topic-selectors)); a glob's matches expand in lexicographic (topic-name) order, so grid placement stays deterministic. At least one `--cam`, `--pcd` or `--gnss` topic is required. Long-form only. Repeatable.                                                                                                                                |
| `--pcd <topic>...`        | `sensor_msgs/msg/PointCloud2` topic(s) to render as point-cloud panels, after the camera panels in the grid: every listed topic is drawn into one panel per `--view`, each cloud transformed into the `--frame` frame at its own stamp — see [Point-cloud panels](#point-cloud-panels). A literal topic name or a `*` glob. Long-form only. Repeatable.                                                                                                                                                                                                                                                                |
| `--gnss <topic>`          | `sensor_msgs/msg/NavSatFix` topic to render as a map panel, after the point-cloud panels: the vehicle's track in a local East-North-Up plan view with the current fix marked — see [Map panel](#map-panel). A literal topic name. Long-form only.                                                                                                                                                                                                                                                                                                                                                                      |
| `--pose <topic>`          | Pose topic (`nav_msgs/msg/Odometry`, `geometry_msgs/msg/PoseStamped` or `geometry_msgs/msg/PoseWithCovarianceStamped`) whose trajectory every camera and point-cloud panel draws — see [Trajectory overlay](#trajectory-overlay). A literal topic name. Long-form only.                                                                                                                                                                                                                                                                                                                                                |
| `--pose-of <frame>`       | The frame the `--pose` trajectory is of. Default: an Odometry message's `child_frame_id`, else `base_link` (the pose topics do not name their body). The bag's static TF must know this frame (and an Odometry's `child_frame_id`, through which another frame is reached). Long-form only.                                                                                                                                                                                                                                                                                                                            |
| `--pose-window <s>`       | Seconds of the `--pose` trajectory drawn on each side of a frame. Default: 10. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `--pose-width <m>`        | Width of the plates the panels lay along the `--pose` trajectory (the vehicle's width, say). Default: 2. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| `-o`, `--output <output>` | **Required.** Output video path. Extension selects the container/codec: `.mp4`/`.mkv`/`.mov` -> H.264, `.avi` -> MJPEG.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| `--encoder <encoder>`     | H.264 encoder for `.mp4`/`.mkv`/`.mov` outputs: `auto` uses NVIDIA NVENC for outputs larger than 1080p (by pixel count, 1920x1080) when this build and a GPU support it, else libx264; `x264` and `nvenc` force one (a forced `nvenc` without a usable GPU stops the run). `.avi` (MJPEG) ignores it. Default: `auto`. Long-form only.                                                                                                                                                                                                                                                                                 |
| `--preset <preset>`       | H.264 speed/quality preset, by libx264's names: `ultrafast`, `superfast`, `veryfast`, `faster`, `fast`, `medium`, `slow`, `slower`, `veryslow`; NVENC maps them onto its `p1`-`p7`. Default: `medium`. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                 |
| `--clock <topic>`         | The panel whose messages define the output frames: each message becomes one frame, its message rate sets the frame rate, and its panel's size (a camera frame after `--resize` / `--width`, or a point-cloud or map panel's 1280x720, or the `--width` split across the columns at 16:9) fixes the grid's cell size — see [The clock panel](#the-clock-panel). Must be one of the `--cam`, `--pcd` or `--gnss` topics. A literal topic name, not a glob. Default: the first `--cam` topic, else the first `--pcd` topic, else the `--gnss` topic. Long-form only.                                                      |
| `--grid <cols>x<rows>`    | Grid layout for the panels (e.g. `2x2`). Must provide at least as many cells as panels; extra cells stay black. When omitted, a near-square grid is derived from the panel count (2 panels -> 2x1, 3-4 -> 2x2, 5-6 -> 3x2, ...). Long-form only.                                                                                                                                                                                                                                                                                                                                                                       |
| `--cam-info <topic>`      | `sensor_msgs/msg/CameraInfo` topic for rectification and `--cam-pcd`: a bare `<info_topic>` applies to every camera panel, an `<image_topic>=<info_topic>` entry overrides one panel. Panels without an entry derive it from the image topic name (`/image_raw`, `/image_raw/compressed`, `/image_rect_color`, and `/image_rect_color/compressed` map their prefix to `/camera_info`). Literal topic names, not globs. Long-form only. Repeatable.                                                                                                                                                                     |
| `--no-rectify`            | Disable rectification. Each frame is otherwise rectified (lens-distortion correction applied) using the panel's resolved CameraInfo — there is no opt-in flag, since that is the default. `--no-rectify` also covers `--cam-pcd` panels, whose points then project onto the raw image with the camera's lens distortion applied. A panel whose camera-info topic cannot be derived renders unrectified with a warning (name it with `--cam-info`); point-cloud projection still requires one. Long-form only.                                                                                                          |
| `--cam-pcd <topic>...`    | `sensor_msgs/msg/PointCloud2` topic selector(s) to project onto the camera panels — a bare value (a literal name or a `*` glob, see [Topic selectors](topic.md#topic-selectors)) projects onto every panel, an `<image_topic>=<pcd_selector>` entry projects onto that panel only. Repeatable. Every resolved topic is drawn with the same field, color scheme, point size, and alpha. Points project onto the rectified image, or onto the raw image with lens distortion applied when `--no-rectify` is given. Requires a CameraInfo topic and a TF chain from each cloud frame to the camera frame. Long-form only. |
| `--field <field>`         | Point-cloud field used for coloring: `x`, `y`, `z`, `distance`, `intensity`. Default: `distance`. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `--min <value>`           | Manual minimum value for field normalization. Default: auto-computed from the point-cloud span. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `--max <value>`           | Manual maximum value for field normalization. Default: auto-computed from the point-cloud span. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `--scheme <scheme>`       | Color scheme for point coloring: `viridis`, `turbo`, `jet`, `plasma`, `inferno`, `magma`, `rainbow`. Default: `viridis`. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| `--point-size <px>`       | Side length of drawn square points in pixels (range: 1-64). Default: 2. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| `--alpha <alpha>`         | Opacity of the point clouds projected onto camera panels, 0.0-1.0. Default: 1.0. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| `--view <view>...`        | Projection(s) of the point-cloud panels, one panel each, in this order: `3d` is a perspective view from a virtual camera on a sphere around the `--frame` origin (`--elev`, `--azim`, `--dist`), `bev` a top-down view of its XY plane spanning `--range` (up is +x/forward, left is +y). Each may appear once. Default: `3d`. Long-form only.                                                                                                                                                                                                                                                                         |
| `--frame <frame_id>`      | TF frame the point-cloud panels draw in; every cloud is transformed into it at its own `header.stamp` through the bag's TF, a cloud already in that frame as is. Default: the first `--pcd` topic's own frame. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                         |
| `--range <m>`             | `bev` half-extent in meters: the view spans +-range on both ground axes. Not used by the `3d` view. Default: the 95th percentile of the ground distances of the points in the first cloud of the first `--pcd` topic, so a few far returns do not shrink the scene (the run logs the value). Long-form only.                                                                                                                                                                                                                                                                                                           |
| `--elev <deg>`            | `3d` view: camera elevation above the XY plane in degrees (range: -89 to 89). Default: 20. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| `--azim <deg>`            | `3d` view: camera azimuth around the +z axis in degrees, measured from +x. 180 looks at the scene from behind the sensor. Default: 180. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| `--dist <m>`              | `3d` view: camera distance from the `--frame` origin in meters. Default: 30. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| `--map-range <m>`         | Map panel: follow the current fix, the panel's shorter axis spanning +-range meters around it (the longer axis shows proportionally more). Default: the whole track fitted into the panel. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                             |
| `--resize <factor>`       | Scale the clock panel's frame by this factor while preserving aspect ratio, which sets the cell size. 1.0 keeps the original size, 0.5 halves both dimensions, 2.0 doubles them. Camera intrinsics are scaled accordingly so rectification and `--cam-pcd` stay aligned (range: 0.01-10.0). Applies only when the clock panel is a camera: a point-cloud or map clock renders at a fixed size and ignores it (use `--width` instead). Default: 1.0. Long-form only. Mutually exclusive with `--width`.                                                                                                                 |
| `--width <px>`            | Fix the composed output width in pixels: the cell width is the width split across the grid columns, and the cell height follows the clock panel's aspect ratio (both rounded down to even, so the output can be a few pixels narrower). Mutually exclusive with `--resize`. Long-form only.                                                                                                                                                                                                                                                                                                                            |
| `-w`, `--overwrite`       | Replace an existing `<output>`. Without it, an existing output path stops the run.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

## The clock panel

One panel drives the output: the first `--cam` topic — the first `--pcd`
topic when there is no camera, the `--gnss` topic when there is neither — or
the one named with `--clock`.

- The output's frame rate and frame count come from the clock topic's message
  timestamps. Each output frame shows, for every other panel, that topic's
  message whose bag record time is nearest the clock frame's (a frame is
  simply repeated while its topic is slower, and a tick before that topic's
  first message shows that first message).
- The cell size is the clock frame's size after `--resize` — or, when
  `--width` is given, the size derived from the output width and the grid
  columns; a point-cloud or map clock panel renders at 1280x720, or at the
  `--width` split across the columns at 16:9. Every other camera panel is
  scaled uniformly to fit the cell, preserving aspect ratio, and centered with
  black bars when the aspect ratios differ; a point-cloud or map panel fills
  the cell.
- The clock topic's frame size must not change mid-bag (a change aborts the
  run); other panels re-fit automatically.
- A topic that carries no messages at all stops the run with an error, rather
  than silently rendering a black cell.

## Point-cloud panels

Every `--pcd` topic is drawn into one panel per `--view`: a `3d` panel is a
perspective view from a virtual camera on a sphere around the view frame's
origin (`--elev`, `--azim`, `--dist`), a `bev` panel a top-down view of its XY
plane spanning `--range` on both axes (up is +x, left is +y). Each frame,
every topic's cloud whose bag record time is nearest the clock message's is
transformed into the view frame — `--frame`, or the first topic's own frame
— at the cloud's own `header.stamp`, through the bag's TF; a cloud already
in that frame needs no TF, and a cloud whose chain to the frame does not
resolve stops the run. Points are colored by `--field` with `--scheme` over
the range of every point-cloud topic in the run (or `--min`/`--max`) and
drawn as `--point-size` squares with a depth test: where points overlap,
the `3d` panel keeps the one nearest the camera and the `bev` panel the
highest one.

## Map panel

`--gnss` draws a `sensor_msgs/msg/NavSatFix` topic as a map: every fix with
a position (`STATUS_NO_FIX` messages are skipped) is projected into a local
East-North-Up plane anchored at the first fix, and the panel shows the whole
track dimmed, the part driven so far highlighted, and the fix whose bag
record time is nearest the clock message's as a marker pointing along its
direction of travel (read off the last half meter of movement, so a vehicle
at rest keeps its heading). A north arrow, a scale bar, and the current
latitude, longitude and altitude are drawn over it. By default the whole
track is fitted into the panel, over at least 20 m on each axis so a
stationary vehicle's GNSS noise is not blown up to fill it; `--map-range <m>`
follows the vehicle instead, the panel's shorter axis spanning +-m around it
(the longer axis shows proportionally more). No map tiles are drawn: the
panel is an offline plan view. A topic whose messages all lack a position
stops the run.

## Trajectory overlay

`--pose` reads a pose topic whole — the body's trajectory in the messages'
own frame (`header.frame_id`), the body being an Odometry message's
`child_frame_id` or the frame `--pose-of` names, bridged through the bag's
static TF when the two differ — and draws it over every camera and
point-cloud panel: the stretch driven up to the frame in grey, the stretch
ahead in orange, `--pose-window` seconds each. For each frame the pose at
that frame's time is interpolated, the trajectory is moved into the body's
frame at that time and on into the panel's frame (a camera's optical frame,
or a point-cloud panel's `--frame`) through the static TF, then projected like the panel's own content: as plates laid on the ground along the path — every 2 m, 1.2 m long, `--pose-width` wide across the path, translucent and fading with distance, as end-to-end driving demos show a planned path — a camera panel projecting them through the camera model (rectified or raw, like `--cam-pcd`), a point-cloud panel through its `3d` / `bev` view. A camera panel needs its camera
info, as `--cam-pcd` does; a bag without static TF draws only in the body's
own frame, and a body frame the static TF cannot take to a panel's frame
stops the run. A body frame the bag's static TF does not know at all — an
Odometry's `child_frame_id` (an INS's own link that was never published,
say), or the frame `--pose-of` names or the `base_link` a pose topic is
taken as — stops the run before any frame is rendered: add the frame's
static transform to the bag (`bagwiz tf static update`) first. The map
panel does not draw it, and `--pose` cannot be the clock.

## Multi-view grids

With several panels, each occupies one grid cell in order — the `--cam`
topics as given, then one point-cloud panel per `--view`, then the map panel
(left to right, top to bottom), the clock panel included. The composed
size is the cell size multiplied by the grid, so it grows faster than the
topic count suggests: nine 1080p cameras on a 3x3 grid compose a 5760x3240
video. A run reports a composed size above 4K — see
[Oversized outputs](#oversized-outputs).

## Frame rate

The frame rate is derived from the clock topic's message timestamps; a topic
with fewer than two distinct timestamps falls back to 10 fps.

## Point-cloud overlay time alignment

Each frame is paired with the point cloud whose `header.stamp` (sensor capture
time) is nearest the image's own `header.stamp`, rather than the bag record time
— so overlays stay aligned even when recording latency differs between the
camera and lidar. If either the camera frame or the point-cloud topic leaves
`header.stamp` unset, that pairing falls back to matching by bag record time on
both sides, so the two are always compared on the same clock rather than mixing
capture time with record time. A topic falls back as a whole: one message
without a stamp makes its timestamp axis a mix of two clocks, which cannot be
searched meaningfully. TF is evaluated at whichever time was matched on, which
keeps the transform lookup on the same clock the TF messages themselves are
stamped with — so a cloud published in a moving frame is projected with that
frame's pose at capture time, not with the last pose in the bag.

In a multi-view grid each panel pairs its clouds against its own selected
frame's stamps, following the same rule.

`bagwiz walk`'s interactive overlay applies exactly the same rule, so the
preview and the encoded video agree frame for frame; see
[walk.md](walk.md#point-cloud-overlay).

## Geometry and encoding

Geometry and encoding are locked to the first frame: a later clock frame with
a different resolution stops the run (other panels re-fit into their cell
instead). Dimensions must be even (the 4:2:0 pixel formats these codecs use
require it).

H.264 outputs larger than 1080p (by pixel count, 1920x1080) are encoded by
NVIDIA NVENC when the FFmpeg
build has it and a GPU accepts the stream, else by libx264 — the run logs
which. Below that size libx264 is used outright: the GPU's setup and
per-frame cost outweigh what it saves on small frames, while at 4K libx264
dominates the whole run. `--encoder` forces one, and `--preset` trades speed
for quality and size (libx264's names, mapped onto NVENC's `p1`-`p7`).
NVENC's H.264 encoder tops out at 4096x4096 on most GPUs, so a composed
output larger than that (a 2x2 grid of 4K cameras) falls back to libx264
under `auto`, and stops the run under a forced `--encoder nvenc`.
Both keep B-frames off, so the output plays on hardware decoders that choke
on negative timestamps.

A single JPEG camera shown as decoded — one `--cam` topic of
`CompressedImage`, `--no-rectify` (or no camera info to rectify with), no
`--resize` / `--width`, no `--cam-pcd`, no `--grid` larger than `1x1`, and no
other panel — streams each
frame's decoded YUV planes straight to the encoder, decoding a few frames
ahead on worker threads, instead of composing a frame through packed BGR
and back; the run logs "streaming ... as decoded". Every other layout
composes its frames — with a camera as the clock, the loop reads a few
messages ahead and decodes them on worker threads while the current frame
composes and encodes.

## Output

Frames stream to the encoder as they are composed — the panels' work on
worker threads, a clock camera's decodes a few frames ahead — and nothing is
buffered whole; the video is written to a temporary file and atomically
moved into place on success. A failed run leaves no partial output or
leftover temporary file.

## Oversized outputs

The output size is not capped, but one large enough to be worth a second
look is reported: when the output exceeds 3840x2160 (4K UHD, 8,294,400
pixels), the run logs a warning naming the actual size and continues.

```text
[WARN] output is 5760x3240 (3x3 grid of 1920x1080 cells, 18.7 Mpx), larger than
3840x2160; encoding will be slow and the output file large. Pass --width to cap
the output width, or --resize to scale the cells down.
```

The test is on the pixel product, not on either dimension, so a tall, narrow
output is judged by what it actually costs to encode. A 2x2 grid of 1080p views
lands exactly on 4K and stays quiet.

The size is not known until the first clock frame fixes the cell size, so the
warning appears once decoding has begun.

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
