# `bagwiz generate`

Generate non-rosbag **media** from a rosbag. Unlike `convert` or `topic` (which
read a bag and write another bag), `generate` reads a bag and produces a
different kind of artifact. Subcommands:

| Subcommand                                          | What it does                                        |
| --------------------------------------------------- | --------------------------------------------------- |
| [`video cam`](#bagwiz-generate-video-cam)           | Render an image topic to a video file.              |
| [`video pcd-scan`](#bagwiz-generate-video-pcd-scan) | Render a point-cloud topic's scan pattern to video. |

---

## `bagwiz generate video cam`

Render an image topic from a rosbag to a video file. The frame rate is derived
from message timestamps, and the container/codec is chosen from the `<output>`
extension.

### Usage

```text
bagwiz generate video cam -i <input> -t <topic> -o <output> [OPTIONS]
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
```

### Options

| Flag                      | Description                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| ------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`   | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                                                                                                                                                                                                                                                                                                                                                                     |
| `-t`, `--topic <topic>`   | **Required.** Image topic to render. Supported types: `sensor_msgs/msg/Image` (`bgr8`, `rgb8`) and `sensor_msgs/msg/CompressedImage` (JPEG/PNG). A literal topic name, not a glob.                                                                                                                                                                                                                                                           |
| `-o`, `--output <output>` | **Required.** Output video path. Extension selects the container/codec: `.mp4`/`.mkv`/`.mov` -> H.264, `.avi` -> MJPEG.                                                                                                                                                                                                                                                                                                                      |
| `--cam-info <topic>`      | `sensor_msgs/msg/CameraInfo` topic for `--rectify` and `--pcd`. When omitted, bagwiz derives it from `<topic>` (`/image_raw`, `/image_raw/compressed`, `/image_rect_color`, and `/image_rect_color/compressed` map their prefix to `/camera_info`). A literal topic name, not a glob. Long-form only.                                                                                                                                        |
| `--rectify`               | Rectify each frame (apply lens-distortion correction) using the resolved CameraInfo. Requires a camera-info topic. Long-form only.                                                                                                                                                                                                                                                                                                           |
| `--pcd <topic>...`        | `sensor_msgs/msg/PointCloud2` topic selector(s) to project onto each frame — a literal name or a `*` glob (see [Topic selectors](topic.md#topic-selectors)). Repeatable; every resolved topic is projected into the camera frame and drawn with the same field, color scheme, point size, and alpha. Implies distortion correction and requires a CameraInfo topic and a TF chain from each cloud frame to the camera frame. Long-form only. |
| `--field <field>`         | Point-cloud field used for coloring: `x`, `y`, `z`, `distance`, `intensity`. Default: `distance`. Long-form only.                                                                                                                                                                                                                                                                                                                            |
| `--min <value>`           | Manual minimum value for field normalization. Default: auto-computed from the point-cloud span. Long-form only.                                                                                                                                                                                                                                                                                                                              |
| `--max <value>`           | Manual maximum value for field normalization. Default: auto-computed from the point-cloud span. Long-form only.                                                                                                                                                                                                                                                                                                                              |
| `--scheme <scheme>`       | Color scheme for point coloring: `viridis`, `turbo`, `jet`, `plasma`, `inferno`, `magma`, `rainbow`. Default: `viridis`. Long-form only.                                                                                                                                                                                                                                                                                                     |
| `--point-size <px>`       | Side length of drawn square points in pixels (range: 1-64). Default: 2. Long-form only.                                                                                                                                                                                                                                                                                                                                                      |
| `--alpha <alpha>`         | Point overlay opacity, 0.0-1.0. Default: 1.0. Long-form only.                                                                                                                                                                                                                                                                                                                                                                                |
| `--resize <factor>`       | Scale the output width and height by this factor while preserving aspect ratio. 1.0 keeps the original size, 0.5 halves both dimensions, 2.0 doubles them. Camera intrinsics are scaled accordingly so `--rectify` and `--pcd` stay aligned (range: 0.01-10.0). Default: 1.0. Long-form only.                                                                                                                                                |
| `-w`, `--overwrite`       | Replace an existing `<output>`. Without it, an existing output path stops the run.                                                                                                                                                                                                                                                                                                                                                           |

### Frame rate

The frame rate is derived from the topic's message timestamps; a topic with
fewer than two distinct timestamps falls back to 10 fps.

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

`bagwiz walk`'s interactive overlay applies exactly the same rule, so the
preview and the encoded video agree frame for frame; see
[walk.md](walk.md#point-cloud-overlay).

### Geometry and encoding

Geometry and encoding are locked to the first frame: a later frame with a
different resolution or pixel encoding stops the run. Dimensions must be even
(the 4:2:0 pixel formats these codecs use require it).

### Output

Frames are decoded and encoded one at a time; the video is written to a
temporary file and atomically moved into place on success. A failed run leaves
no partial output or leftover temporary file.

---

## `bagwiz generate video pcd-scan`

Render the scan pattern of a point-cloud topic to a video file: within each
sweep the points appear one by one in firing order, colored by their
sweep-relative time. This makes the sensor's firing sequence visible — the
rotating sweep of a spinning lidar, or a non-repetitive pattern — and helps
spot timestamp irregularities and motion-distortion behavior.

### Usage

```text
bagwiz generate video pcd-scan -i <input> -t <topic> -o <output> [OPTIONS]
```

### Examples

```bash
# Render a lidar's scan pattern as a top-down (BEV) animation.
bagwiz generate video pcd-scan -i drive.mcap -t /sensing/lidar/top/points -o scan.mp4

# Same, from a fixed 3D viewpoint behind and above the sensor.
bagwiz generate video pcd-scan -i drive.mcap -t /sensing/lidar/top/points -o scan.mp4 \
  --view 3d --elev 35 --azim 180 --dist 120

# Finer time resolution within each sweep: 20 video frames per sweep instead of 10.
bagwiz generate video pcd-scan -i drive.mcap -t /sensing/lidar/top/points -o scan.mp4 --steps 20

# Fix the view extent to +-80 m instead of auto-fitting the first cloud.
bagwiz generate video pcd-scan -i drive.mcap -t /sensing/lidar/top/points -o scan.mp4 --range 80
```

### Options

| Flag                      | Description                                                                                                                                                                                                                      |
| ------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`   | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                                                                                                                                                         |
| `-t`, `--topic <topic>`   | **Required.** `sensor_msgs/msg/PointCloud2` topic to render. A literal topic name, not a glob. The topic must carry a per-point time field (see below).                                                                          |
| `-o`, `--output <output>` | **Required.** Output video path. Extension selects the container/codec: `.mp4`/`.mkv`/`.mov` -> H.264, `.avi` -> MJPEG.                                                                                                          |
| `--view <view>`           | Projection space: `bev` (top-down XY view centered on the sensor; up is +x/forward, left is +y) or `3d` (perspective view from a fixed camera looking at the sensor). Default: `bev`. Long-form only.                            |
| `--width <px>`            | Output width in pixels (range: 2-7680). Must be even. Default: 1280. Long-form only.                                                                                                                                             |
| `--height <px>`           | Output height in pixels (range: 2-4320). Must be even. Default: 720. Long-form only.                                                                                                                                             |
| `--steps <n>`             | Video frames rendered per sweep (range: 1-100). The output frame rate is the cloud rate times this value, so the animation plays in real time. Default: 10. Long-form only.                                                      |
| `--range <m>`             | BEV half-extent in meters: the BEV view spans +-range on both ground axes. In the 3D view it only sets the default `--dist` (2.5x the range). Default: auto — the largest finite XY distance in the first cloud. Long-form only. |
| `--elev <deg>`            | 3D view: camera elevation above the XY plane in degrees (range: -89 to 89). Default: 30. Long-form only.                                                                                                                         |
| `--azim <deg>`            | 3D view: camera azimuth around the +z axis in degrees, measured from +x. 180 looks at the scene from behind the sensor. Default: 180. Long-form only.                                                                            |
| `--dist <m>`              | 3D view: camera distance from the sensor in meters. Default: 2.5x the range. Long-form only.                                                                                                                                     |
| `--scheme <scheme>`       | Color scheme for the sweep-relative time coloring: `viridis`, `turbo`, `jet`, `plasma`, `inferno`, `magma`, `rainbow`. Default: `viridis`. Long-form only.                                                                       |
| `--point-size <px>`       | Side length of drawn square points in pixels (range: 1-64). Default: 2. Long-form only.                                                                                                                                          |
| `-w`, `--overwrite`       | Replace an existing `<output>`. Without it, an existing output path stops the run.                                                                                                                                               |

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
over `--steps` video frames; the last frame of a sweep shows the complete
cloud. A sweep in which no point carries a finite time contributes `--steps`
blank frames, so the video timeline is not disturbed. The output frame rate is
the cloud rate times `--steps` (a 10 Hz lidar with the default 10 steps yields
a 100 fps video), so the animation plays in real time. If that product would
exceed 240 fps, the step count is reduced with a warning. Dimensions must be
even (the 4:2:0 pixel formats these codecs use require it).

### Output

Clouds are parsed and encoded one at a time; the video is written to a
temporary file and atomically moved into place on success. A failed run leaves
no partial output or leftover temporary file.

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
