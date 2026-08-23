# `bagwiz generate`

Generate non-rosbag **media** from a rosbag. Unlike `convert` or `topic` (which
read a bag and write another bag), `generate` reads a bag and produces a
different kind of artifact. Subcommands:

| Subcommand                                | What it does                                            |
| ----------------------------------------- | ------------------------------------------------------- |
| [`video cam`](#bagwiz-generate-video-cam) | Render image topic(s) to a video file (single or grid). |

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

| Flag                       | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| -------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`    | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| `-t`, `--topic <topic>...` | **Required.** Image topic(s) to render, in grid order (left to right, top to bottom). Supported types: `sensor_msgs/msg/Image` (`bgr8`, `rgb8`) and `sensor_msgs/msg/CompressedImage` (JPEG/PNG). Literal topic names, not globs. The first topic is primary: it drives the frame rate and output timing, and its frame size (after `--resize`) fixes the grid's cell size. Repeatable.                                                                                                                                                        |
| `-o`, `--output <output>`  | **Required.** Output video path. Extension selects the container/codec: `.mp4`/`.mkv`/`.mov` -> H.264, `.avi` -> MJPEG.                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `--grid <cols>x<rows>`     | Grid layout for multiple topics (e.g. `2x2`). Must provide at least as many cells as topics; extra cells stay black. When omitted, a near-square grid is derived from the topic count (2 topics -> 2x1, 3-4 -> 2x2, 5-6 -> 3x2, ...). Long-form only.                                                                                                                                                                                                                                                                                          |
| `--cam-info <topic>`       | `sensor_msgs/msg/CameraInfo` topic for `--rectify` and `--pcd`: a bare `<info_topic>` applies to every view, an `<image_topic>=<info_topic>` entry overrides one view. Views without an entry derive it from the image topic name (`/image_raw`, `/image_raw/compressed`, `/image_rect_color`, and `/image_rect_color/compressed` map their prefix to `/camera_info`). Literal topic names, not globs. Long-form only. Repeatable.                                                                                                             |
| `--rectify`                | Rectify each frame (apply lens-distortion correction) using each view's resolved CameraInfo. Requires a camera-info topic per view. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                            |
| `--pcd <topic>...`         | `sensor_msgs/msg/PointCloud2` topic selector(s) to project onto the frames — a bare value (a literal name or a `*` glob, see [Topic selectors](topic.md#topic-selectors)) projects onto every view, an `<image_topic>=<pcd_selector>` entry projects onto that view only. Repeatable. Every resolved topic is drawn with the same field, color scheme, point size, and alpha. Projecting implies distortion correction for that view and requires a CameraInfo topic and a TF chain from each cloud frame to the camera frame. Long-form only. |
| `--field <field>`          | Point-cloud field used for coloring: `x`, `y`, `z`, `distance`, `intensity`. Default: `distance`. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                              |
| `--min <value>`            | Manual minimum value for field normalization. Default: auto-computed from the point-cloud span. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                |
| `--max <value>`            | Manual maximum value for field normalization. Default: auto-computed from the point-cloud span. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                |
| `--scheme <scheme>`        | Color scheme for point coloring: `viridis`, `turbo`, `jet`, `plasma`, `inferno`, `magma`, `rainbow`. Default: `viridis`. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                       |
| `--point-size <px>`        | Side length of drawn square points in pixels (range: 1-64). Default: 2. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `--alpha <alpha>`          | Point overlay opacity, 0.0-1.0. Default: 1.0. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `--resize <factor>`        | Scale the cell width and height by this factor while preserving aspect ratio. 1.0 keeps the original size, 0.5 halves both dimensions, 2.0 doubles them. Camera intrinsics are scaled accordingly so `--rectify` and `--pcd` stay aligned (range: 0.01-10.0). Default: 1.0. Long-form only.                                                                                                                                                                                                                                                    |
| `-w`, `--overwrite`        | Replace an existing `<output>`. Without it, an existing output path stops the run.                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

### Multi-view grids

With several `-t` topics, each topic occupies one grid cell in argument order
(left to right, top to bottom). The first topic is primary:

- The output's frame rate and frame count come from the primary topic's
  message timestamps. Each output frame shows, for every other view, that
  topic's message whose bag record time is nearest the primary frame's (a
  frame is simply repeated while its topic is slower, and a topic that has not
  produced a message yet renders as a black cell).
- The cell size is the primary topic's frame size after `--resize`. Every
  other view is scaled uniformly to fit the cell, preserving aspect ratio, and
  centered with black bars when the aspect ratios differ.
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

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
