# `bagwiz generate`

Generate non-rosbag **media** from a rosbag. Unlike `convert` or `topic` (which
read a bag and write another bag), `generate` reads a bag and produces a
different kind of artifact. Subcommands:

| Subcommand                                  | What it does                                            |
| ------------------------------------------- | ------------------------------------------------------- |
| [`video cam`](#bagwiz-generate-video-cam)   | Render image topic(s) to a video file (single or grid). |
| [`video scan`](#bagwiz-generate-video-scan) | Render a point-cloud topic's scan pattern to video.     |

---

## `bagwiz generate video cam`

Render one or more image topics from a rosbag to a video file. With several
`-t` topics the views are arranged in a grid, producing a multi-view video.
The frame rate is derived from the first topic's message timestamps, and the
container/codec is chosen from the `<output>` extension.

### Usage

```text
bagwiz generate video cam -i <input> -t <topic>... -o <output> [OPTIONS]
```

### Examples

```bash
# Render a camera topic to an MP4 (H.264).
bagwiz generate video cam -i drive.mcap -t /sensing/camera/image_raw -o out.mp4

# Render to MJPEG AVI, replacing an existing file.
bagwiz generate video cam -i drive_dir/ -t /sensing/camera/image_raw -o clip.avi -w

# Render with distortion correction.
bagwiz generate video cam -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 --rectify

# Render with distortion correction using an explicit CameraInfo topic.
bagwiz generate video cam -i drive.mcap -t /sensing/camera/image_raw -o out.mp4 \
  --rectify --cam-info /sensing/camera/camera_info

# Render with a point-cloud overlay colored by distance.
bagwiz generate video cam -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 \
  --pcd /sensing/lidar/front/points --field distance --scheme turbo --point-size 3 --alpha 0.8

# Render with multiple point-cloud overlays in the same camera view.
bagwiz generate video cam -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 \
  --pcd /sensing/lidar/front/points \
        /sensing/lidar/rear/points \
  --field distance --scheme turbo --point-size 3 --alpha 0.8

# Render with every lidar point cloud under /sensing/lidar overlaid, via a
# glob (quoted so the shell doesn't expand it).
bagwiz generate video cam -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 \
  --pcd '/sensing/lidar/*/points'

# Render at half resolution to reduce output file size.
bagwiz generate video cam -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 --resize 0.5

# Multi-view: two cameras side by side (auto grid), front camera driving the
# frame rate.
bagwiz generate video cam -i drive.mcap -o front_rear.mp4 \
  -t /sensing/camera/front/image_raw/compressed \
     /sensing/camera/rear/image_raw/compressed

# Multi-view from a glob (quoted so the shell doesn't expand it): every
# matching camera topic, expanded in topic-name order, at a fixed width.
bagwiz generate video cam -i drive.mcap -o all_cams.mp4 --width 3840 \
  -t '/sensing/camera/camera*/image_raw/compressed'

# Multi-view at a fixed output width: three cameras on an auto 2x2 grid, the
# composed video exactly 1920 px wide (cells 960x540 for 16:9 inputs).
bagwiz generate video cam -i drive.mcap -o surround.mp4 --width 1920 \
  -t /sensing/camera/front/image_raw/compressed \
     /sensing/camera/rear/image_raw/compressed \
     /sensing/camera/left/image_raw/compressed

# Multi-view with an explicit 2x2 grid (three cameras; the fourth cell stays
# black).
bagwiz generate video cam -i drive.mcap -o surround.mp4 --grid 2x2 \
  -t /sensing/camera/front/image_raw/compressed \
     /sensing/camera/rear/image_raw/compressed \
     /sensing/camera/left/image_raw/compressed

# Multi-view with a point cloud projected onto every view (bare --pcd value).
bagwiz generate video cam -i drive.mcap -o overlay_all.mp4 \
  -t /sensing/camera/front/image_raw/compressed \
     /sensing/camera/rear/image_raw/compressed \
  --pcd /sensing/lidar/top/points

# Multi-view with per-view point-cloud bindings: each camera gets its own
# lidar's projection.
bagwiz generate video cam -i drive.mcap -o overlay_each.mp4 \
  -t /sensing/camera/front/image_raw/compressed \
     /sensing/camera/rear/image_raw/compressed \
  --pcd /sensing/camera/front/image_raw/compressed=/sensing/lidar/front/points \
        /sensing/camera/rear/image_raw/compressed=/sensing/lidar/rear/points
```

### Options

| Flag                       | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| -------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`    | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `-t`, `--topic <topic>...` | **Required.** Image topic(s) to render, in grid order (left to right, top to bottom). Supported types: `sensor_msgs/msg/Image` (`bgr8`, `rgb8`) and `sensor_msgs/msg/CompressedImage` (JPEG/PNG). A literal topic name or a `*` glob (see [Topic selectors](topic.md#topic-selectors)); a glob's matches expand in lexicographic (topic-name) order, so grid placement stays deterministic. The first topic is primary: it drives the frame rate and output timing, and its frame size (after `--resize`) fixes the grid's cell size. Repeatable. |
| `-o`, `--output <output>`  | **Required.** Output video path. Extension selects the container/codec: `.mp4`/`.mkv`/`.mov` -> H.264, `.avi` -> MJPEG.                                                                                                                                                                                                                                                                                                                                                                                                                           |
| `--grid <cols>x<rows>`     | Grid layout for multiple topics (e.g. `2x2`). Must provide at least as many cells as topics; extra cells stay black. When omitted, a near-square grid is derived from the topic count (2 topics -> 2x1, 3-4 -> 2x2, 5-6 -> 3x2, ...). Long-form only.                                                                                                                                                                                                                                                                                             |
| `--cam-info <topic>`       | `sensor_msgs/msg/CameraInfo` topic for `--rectify` and `--pcd`: a bare `<info_topic>` applies to every view, an `<image_topic>=<info_topic>` entry overrides one view. Views without an entry derive it from the image topic name (`/image_raw`, `/image_raw/compressed`, `/image_rect_color`, and `/image_rect_color/compressed` map their prefix to `/camera_info`). Literal topic names, not globs. Long-form only. Repeatable.                                                                                                                |
| `--rectify`                | Rectify each frame (apply lens-distortion correction) using each view's resolved CameraInfo. **On by default.** A view whose camera-info topic cannot be derived renders unrectified with a warning (name it with `--cam-info`); point-cloud projection still requires one. Long-form only.                                                                                                                                                                                                                                                       |
| `--no-rectify`             | Disable rectification even when a CameraInfo is available. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `--pcd <topic>...`         | `sensor_msgs/msg/PointCloud2` topic selector(s) to project onto the frames — a bare value (a literal name or a `*` glob, see [Topic selectors](topic.md#topic-selectors)) projects onto every view, an `<image_topic>=<pcd_selector>` entry projects onto that view only. Repeatable. Every resolved topic is drawn with the same field, color scheme, point size, and alpha. Projecting implies distortion correction for that view and requires a CameraInfo topic and a TF chain from each cloud frame to the camera frame. Long-form only.    |
| `--field <field>`          | Point-cloud field used for coloring: `x`, `y`, `z`, `distance`, `intensity`. Default: `distance`. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| `--min <value>`            | Manual minimum value for field normalization. Default: auto-computed from the point-cloud span. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `--max <value>`            | Manual maximum value for field normalization. Default: auto-computed from the point-cloud span. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `--scheme <scheme>`        | Color scheme for point coloring: `viridis`, `turbo`, `jet`, `plasma`, `inferno`, `magma`, `rainbow`. Default: `viridis`. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                          |
| `--point-size <px>`        | Side length of drawn square points in pixels (range: 1-64). Default: 2. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| `--alpha <alpha>`          | Point overlay opacity, 0.0-1.0. Default: 1.0. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| `--resize <factor>`        | Scale the cell width and height by this factor while preserving aspect ratio. 1.0 keeps the original size, 0.5 halves both dimensions, 2.0 doubles them. Camera intrinsics are scaled accordingly so `--rectify` and `--pcd` stay aligned (range: 0.01-10.0). Default: 1.0. Long-form only. Mutually exclusive with `--width`.                                                                                                                                                                                                                    |
| `--width <px>`             | Fix the composed output width in pixels: the cell width is the width split across the grid columns, and the cell height follows the primary frame's aspect ratio (both rounded down to even, so the output can be a few pixels narrower). Mutually exclusive with `--resize`. Long-form only.                                                                                                                                                                                                                                                     |
| `-w`, `--overwrite`        | Replace an existing `<output>`. Without it, an existing output path stops the run.                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

### Multi-view grids

With several `-t` topics, each topic occupies one grid cell in argument order
(left to right, top to bottom). The first topic is primary:

- The output's frame rate and frame count come from the primary topic's
  message timestamps. Each output frame shows, for every other view, that
  topic's message whose bag record time is nearest the primary frame's (a
  frame is simply repeated while its topic is slower, and a topic that has not
  produced a message yet renders as a black cell).
- The cell size is the primary topic's frame size after `--resize` — or, when
  `--width` is given, the size derived from the output width and the grid
  columns. Every other view is scaled uniformly to fit the cell, preserving
  aspect ratio, and centered with black bars when the aspect ratios differ.
- The primary topic's frame size must not change mid-bag (a change aborts the
  run, as in the single-view case); secondary views re-fit automatically.
- A topic that carries no messages at all stops the run with an error, rather
  than silently rendering a black cell.

### Frame rate

The frame rate is derived from the primary (first) topic's message timestamps;
a topic with fewer than two distinct timestamps falls back to 10 fps.

### Point-cloud overlay time alignment

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

In a multi-view grid each view pairs its clouds against its own selected
frame's stamps, following the same rule.

`bagwiz walk`'s interactive overlay applies exactly the same rule, so the
preview and the encoded video agree frame for frame; see
[walk.md](walk.md#point-cloud-overlay).

### Geometry and encoding

Geometry and encoding are locked to the first frame: a later primary frame
with a different resolution stops the run (secondary views re-fit into their
cell instead). Dimensions must be even (the 4:2:0 pixel formats these codecs
use require it).

### Output

Frames are decoded and encoded one at a time; the video is written to a
temporary file and atomically moved into place on success. A failed run leaves
no partial output or leftover temporary file.

---

## `bagwiz generate video scan`

Render the scan pattern of a point-cloud topic to a video file: within each
sweep the points appear one by one in firing order, colored by their
sweep-relative time. This makes the sensor's firing sequence visible — the
rotating sweep of a spinning lidar, or a non-repetitive pattern — and helps
spot timestamp irregularities and motion-distortion behavior.

### Usage

```text
bagwiz generate video scan -i <input> -t <topic> -o <output> [OPTIONS]
```

### Examples

```bash
# Render a lidar's scan pattern as a 3D animation (the default view).
bagwiz generate video scan -i drive.mcap -t /sensing/lidar/top/points -o scan.mp4

# Tune the 3D viewpoint: elevation, azimuth, and camera distance.
bagwiz generate video scan -i drive.mcap -t /sensing/lidar/top/points -o scan.mp4 \
  --elev 35 --azim 180 --dist 120

# Top-down (BEV) animation instead.
bagwiz generate video scan -i drive.mcap -t /sensing/lidar/top/points -o scan.mp4 --view bev

# Coarser animation (and a smaller file): 30 fps output instead of 60.
bagwiz generate video scan -i drive.mcap -t /sensing/lidar/top/points -o scan.mp4 --fps 30

# Play in real time instead of the default one-tenth speed.
bagwiz generate video scan -i drive.mcap -t /sensing/lidar/top/points -o scan.mp4 --speed 1.0

# Fix the view extent to +-80 m instead of auto-fitting the first cloud.
bagwiz generate video scan -i drive.mcap -t /sensing/lidar/top/points -o scan.mp4 --range 80
```

### Options

| Flag                      | Description                                                                                                                                                                                               |
| ------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`   | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                                                                                                                                  |
| `-t`, `--topic <topic>`   | **Required.** `sensor_msgs/msg/PointCloud2` topic to render. A literal topic name, not a glob. The topic must carry a per-point time field (see below).                                                   |
| `-o`, `--output <output>` | **Required.** Output video path. Extension selects the container/codec: `.mp4`/`.mkv`/`.mov` -> H.264, `.avi` -> MJPEG.                                                                                   |
| `--view <view>`           | Projection space: `3d` (perspective view from a fixed camera looking at the sensor) or `bev` (top-down XY view centered on the sensor; up is +x/forward, left is +y). Default: `3d`. Long-form only.      |
| `--width <px>`            | Output width in pixels (range: 2-7680). Must be even. Default: 1280. Long-form only.                                                                                                                      |
| `--height <px>`           | Output height in pixels (range: 2-4320). Must be even. Default: 720. Long-form only.                                                                                                                      |
| `--fps <f>`               | Output frame rate in fps (range: 1-240). Each sweep spans round(`--fps` / (cloud rate x `--speed`)) video frames (at least 1), so a higher value gives a smoother animation. Default: 60. Long-form only. |
| `--speed <x>`             | Playback speed as a fraction of real time: 1.0 plays each sweep in its recorded duration, 0.1 slows the animation to one tenth (range: 0.001-100). Default: 0.1. Long-form only.                          |
| `--range <m>`             | BEV half-extent in meters: the BEV view spans +-range on both ground axes. Not used by the 3D view. Default: auto — the largest finite XY distance in the first cloud. Long-form only.                    |
| `--elev <deg>`            | 3D view: camera elevation above the XY plane in degrees (range: -89 to 89). Default: 20. Long-form only.                                                                                                  |
| `--azim <deg>`            | 3D view: camera azimuth around the +z axis in degrees, measured from +x. 180 looks at the scene from behind the sensor. Default: 180. Long-form only.                                                     |
| `--dist <m>`              | 3D view: camera distance from the sensor in meters. Default: 30. Long-form only.                                                                                                                          |
| `--scheme <scheme>`       | Color scheme for the sweep-relative time coloring: `viridis`, `turbo`, `jet`, `plasma`, `inferno`, `magma`, `rainbow`. Default: `viridis`. Long-form only.                                                |
| `--point-size <px>`       | Side length of drawn square points in pixels (range: 1-64). Default: 2. Long-form only.                                                                                                                   |
| `-w`, `--overwrite`       | Replace an existing `<output>`. Without it, an existing output path stops the run.                                                                                                                        |

### Per-point time field

The firing order is read from the cloud's per-point time field: the first
field named `t`, `time`, `time_stamp`, or `timestamp` with element count 1 and
datatype UINT32 (nanoseconds) or FLOAT32 / FLOAT64 (seconds). A topic without
one stops the run — there is no array-order fallback, because array order does
not reliably match firing order across drivers. Whether the values are
sweep-relative or epoch-absolute does not matter: each sweep is normalized by
its own earliest and latest point time. Points with a non-finite time never
appear.

### Animation model

Each sweep clears the canvas and re-accumulates its points in firing order
over round(`--fps` / (cloud rate x `--speed`)) video frames; the last frame of
a sweep shows the complete cloud. The output frame rate is `--fps` itself (a
10 Hz lidar with the default 60 fps and the default speed 0.1 gives each sweep
60 frames), so the animation plays at `--speed` times real time. If `--fps` is
below the cloud rate times `--speed`, each sweep gets exactly one frame with a
warning (the animation then plays slower than the requested speed). A sweep in
which no point carries a finite time contributes the same number of blank
frames, so the video timeline is not disturbed. Dimensions must be even (the
4:2:0 pixel formats these codecs use require it).

### Output

Clouds are parsed and encoded one at a time; the video is written to a
temporary file and atomically moved into place on success. A failed run leaves
no partial output or leftover temporary file.

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
