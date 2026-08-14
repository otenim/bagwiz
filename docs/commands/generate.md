# `bagwiz generate`

Generate non-rosbag **media** from a rosbag. Unlike `convert` or `topic` (which
read a bag and write another bag), `generate` reads a bag and produces a
different kind of artifact. Subcommands:

| Subcommand                        | What it does                           |
| --------------------------------- | -------------------------------------- |
| [`video`](#bagwiz-generate-video) | Render an image topic to a video file. |

---

## `bagwiz generate video`

Render an image topic from a rosbag to a video file. The frame rate is derived
from message timestamps, and the container/codec is chosen from the `<output>`
extension.

### Usage

```text
bagwiz generate video -i <input> -t <topic> -o <output> [OPTIONS]
```

### Examples

```bash
# Render a camera topic to an MP4 (H.264).
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw -o out.mp4

# Render to MJPEG AVI, replacing an existing file.
bagwiz generate video -i drive_dir/ -t /sensing/camera/image_raw -o clip.avi -w

# Render with distortion correction.
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 --rectify

# Render with distortion correction using an explicit CameraInfo topic.
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw -o out.mp4 \
  --rectify --cam-info /sensing/camera/camera_info

# Render with a point-cloud overlay colored by distance.
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 \
  --pcd /sensing/lidar/front/points --field distance --scheme turbo --point-size 3 --alpha 0.8

# Render with multiple point-cloud overlays in the same camera view.
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 \
  --pcd /sensing/lidar/front/points \
        /sensing/lidar/rear/points \
  --field distance --scheme turbo --point-size 3 --alpha 0.8

# Render with every lidar point cloud under /sensing/lidar overlaid, via a
# glob (quoted so the shell doesn't expand it).
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 \
  --pcd '/sensing/lidar/*/points'

# Render at half resolution to reduce output file size.
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 --resize 0.5
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

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
