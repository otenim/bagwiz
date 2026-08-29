# `bagwiz movify`

Render a rosbag to video. Subcommands:

| Subcommand                  | What it does                                            |
| --------------------------- | ------------------------------------------------------- |
| [`cam`](#bagwiz-movify-cam) | Render image topic(s) to a video file (single or grid). |

---

## `bagwiz movify cam`

Render one or more image topics from a rosbag to a video file. With several
`-t` topics the views are arranged in a grid, producing a multi-view video.
The frame rate is derived from the first topic's message timestamps, and the
container/codec is chosen from the `<output>` extension.

### Usage

```text
bagwiz movify cam -i <input> -t <topic>... -o <output> [OPTIONS]
```

### Examples

```bash
# Render a camera topic to an MP4 (H.264).
bagwiz movify cam -i drive.mcap -t /sensing/camera/image_raw -o out.mp4

# Render to MJPEG AVI, replacing an existing file.
bagwiz movify cam -i drive_dir/ -t /sensing/camera/image_raw -o clip.avi -w

# Render with distortion correction (on by default) using an explicit
# CameraInfo topic.
bagwiz movify cam -i drive.mcap -t /sensing/camera/image_raw -o out.mp4 \
  --cam-info /sensing/camera/camera_info

# Render the raw frames, skipping distortion correction.
bagwiz movify cam -i drive.mcap -t /sensing/camera/image_raw -o out.mp4 --no-rectify

# Render with a point-cloud overlay colored by distance.
bagwiz movify cam -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 \
  --pcd /sensing/lidar/front/points --field distance --scheme turbo --point-size 3 --alpha 0.8

# Render with multiple point-cloud overlays in the same camera view.
bagwiz movify cam -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 \
  --pcd /sensing/lidar/front/points \
        /sensing/lidar/rear/points \
  --field distance --scheme turbo --point-size 3 --alpha 0.8

# Render with every lidar point cloud under /sensing/lidar overlaid, via a
# glob (quoted so the shell doesn't expand it).
bagwiz movify cam -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 \
  --pcd '/sensing/lidar/*/points'

# Project the point cloud onto the raw (unrectified) frame: the camera's lens
# distortion is applied to the projected points, keeping them aligned with the
# distorted image.
bagwiz movify cam -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 \
  --pcd /sensing/lidar/front/points --no-rectify

# Render at half resolution to reduce output file size.
bagwiz movify cam -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 --resize 0.5

# Multi-view: two cameras side by side (auto grid), front camera driving the
# frame rate.
bagwiz movify cam -i drive.mcap -o front_rear.mp4 \
  -t /sensing/camera/front/image_raw/compressed \
     /sensing/camera/rear/image_raw/compressed

# Multi-view from a glob (quoted so the shell doesn't expand it): every
# matching camera topic, expanded in topic-name order, at a fixed width.
bagwiz movify cam -i drive.mcap -o all_cams.mp4 --width 3840 \
  -t '/sensing/camera/camera*/image_raw/compressed'

# Multi-view at a fixed output width: three cameras on an auto 2x2 grid, the
# composed video exactly 1920 px wide (cells 960x540 for 16:9 inputs).
bagwiz movify cam -i drive.mcap -o surround.mp4 --width 1920 \
  -t /sensing/camera/front/image_raw/compressed \
     /sensing/camera/rear/image_raw/compressed \
     /sensing/camera/left/image_raw/compressed

# Multi-view with an explicit 2x2 grid (three cameras; the fourth cell stays
# black).
bagwiz movify cam -i drive.mcap -o surround.mp4 --grid 2x2 \
  -t /sensing/camera/front/image_raw/compressed \
     /sensing/camera/rear/image_raw/compressed \
     /sensing/camera/left/image_raw/compressed

# Multi-view with a point cloud projected onto every view (bare --pcd value).
bagwiz movify cam -i drive.mcap -o overlay_all.mp4 \
  -t /sensing/camera/front/image_raw/compressed \
     /sensing/camera/rear/image_raw/compressed \
  --pcd /sensing/lidar/top/points

# Multi-view with per-view point-cloud bindings: each camera gets its own
# lidar's projection.
bagwiz movify cam -i drive.mcap -o overlay_each.mp4 \
  -t /sensing/camera/front/image_raw/compressed \
     /sensing/camera/rear/image_raw/compressed \
  --pcd /sensing/camera/front/image_raw/compressed=/sensing/lidar/front/points \
        /sensing/camera/rear/image_raw/compressed=/sensing/lidar/rear/points
```

### Options

| Flag                       | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| -------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`    | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `-t`, `--topic <topic>...` | **Required.** Image topic(s) to render, in grid order (left to right, top to bottom). Supported types: `sensor_msgs/msg/Image` (`bgr8`, `rgb8`) and `sensor_msgs/msg/CompressedImage` (JPEG/PNG). A literal topic name or a `*` glob (see [Topic selectors](topic.md#topic-selectors)); a glob's matches expand in lexicographic (topic-name) order, so grid placement stays deterministic. The first topic is primary: it drives the frame rate and output timing, and its frame size (after `--resize`) fixes the grid's cell size. Repeatable.                                                             |
| `-o`, `--output <output>`  | **Required.** Output video path. Extension selects the container/codec: `.mp4`/`.mkv`/`.mov` -> H.264, `.avi` -> MJPEG.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| `--grid <cols>x<rows>`     | Grid layout for multiple topics (e.g. `2x2`). Must provide at least as many cells as topics; extra cells stay black. When omitted, a near-square grid is derived from the topic count (2 topics -> 2x1, 3-4 -> 2x2, 5-6 -> 3x2, ...). Long-form only.                                                                                                                                                                                                                                                                                                                                                         |
| `--cam-info <topic>`       | `sensor_msgs/msg/CameraInfo` topic for rectification and `--pcd`: a bare `<info_topic>` applies to every view, an `<image_topic>=<info_topic>` entry overrides one view. Views without an entry derive it from the image topic name (`/image_raw`, `/image_raw/compressed`, `/image_rect_color`, and `/image_rect_color/compressed` map their prefix to `/camera_info`). Literal topic names, not globs. Long-form only. Repeatable.                                                                                                                                                                          |
| `--no-rectify`             | Disable rectification. Each frame is otherwise rectified (lens-distortion correction applied) using the view's resolved CameraInfo — there is no opt-in flag, since that is the default. `--no-rectify` also covers `--pcd` views, whose points then project onto the raw image with the camera's lens distortion applied. A view whose camera-info topic cannot be derived renders unrectified with a warning (name it with `--cam-info`); point-cloud projection still requires one. Long-form only.                                                                                                        |
| `--pcd <topic>...`         | `sensor_msgs/msg/PointCloud2` topic selector(s) to project onto the frames — a bare value (a literal name or a `*` glob, see [Topic selectors](topic.md#topic-selectors)) projects onto every view, an `<image_topic>=<pcd_selector>` entry projects onto that view only. Repeatable. Every resolved topic is drawn with the same field, color scheme, point size, and alpha. Points project onto the rectified image, or onto the raw image with lens distortion applied when `--no-rectify` is given. Requires a CameraInfo topic and a TF chain from each cloud frame to the camera frame. Long-form only. |
| `--field <field>`          | Point-cloud field used for coloring: `x`, `y`, `z`, `distance`, `intensity`. Default: `distance`. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| `--min <value>`            | Manual minimum value for field normalization. Default: auto-computed from the point-cloud span. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| `--max <value>`            | Manual maximum value for field normalization. Default: auto-computed from the point-cloud span. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| `--scheme <scheme>`        | Color scheme for point coloring: `viridis`, `turbo`, `jet`, `plasma`, `inferno`, `magma`, `rainbow`. Default: `viridis`. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `--point-size <px>`        | Side length of drawn square points in pixels (range: 1-64). Default: 2. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| `--alpha <alpha>`          | Point overlay opacity, 0.0-1.0. Default: 1.0. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| `--resize <factor>`        | Scale the cell width and height by this factor while preserving aspect ratio. 1.0 keeps the original size, 0.5 halves both dimensions, 2.0 doubles them. Camera intrinsics are scaled accordingly so rectification and `--pcd` stay aligned (range: 0.01-10.0). Default: 1.0. Long-form only. Mutually exclusive with `--width`.                                                                                                                                                                                                                                                                              |
| `--width <px>`             | Fix the composed output width in pixels: the cell width is the width split across the grid columns, and the cell height follows the primary frame's aspect ratio (both rounded down to even, so the output can be a few pixels narrower). Mutually exclusive with `--resize`. Long-form only.                                                                                                                                                                                                                                                                                                                 |
| `-w`, `--overwrite`        | Replace an existing `<output>`. Without it, an existing output path stops the run.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |

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
- The composed size is the cell size multiplied by the grid, so it grows faster
  than the topic count suggests: nine 1080p cameras on a 3x3 grid compose a
  5760x3240 video. A run reports a composed size above 4K — see
  [Oversized outputs](#oversized-outputs).

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

## Oversized outputs

`cam` does not cap the output size, but it reports one large enough to be
worth a second look: when the output exceeds 3840x2160 (4K UHD, 8,294,400
pixels), the run logs a warning naming the actual size and continues.

```text
[WARN] output is 5760x3240 (3x3 grid of 1920x1080 cells, 18.7 Mpx), larger than
3840x2160; encoding will be slow and the output file large. Pass --width to cap
the output width, or --resize to scale the cells down.
```

The test is on the pixel product, not on either dimension, so a tall, narrow
output is judged by what it actually costs to encode. A 2x2 grid of 1080p views
lands exactly on 4K and stays quiet.

`cam` cannot know its size until the first frame fixes the cell size, so the
warning appears once decoding has begun.

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
