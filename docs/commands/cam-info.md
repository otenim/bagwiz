# `bagwiz cam-info`

Operations on `sensor_msgs/msg/CameraInfo` topics. Subcommands:

| Subcommand                                    | What it does                                                                                                |
| --------------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| [`replace`](#bagwiz-cam-info-replace)         | Overwrite one or more CameraInfo topics' calibration with the values from YAML files (shared or per-topic). |
| [`recompute-p`](#bagwiz-cam-info-recompute-p) | Recompute the projection matrix from the intrinsics, in a YAML file or in a bag.                            |
| [`dump`](#bagwiz-cam-info-dump)               | Write a CameraInfo topic's calibration out to a YAML file, verbatim.                                        |

ROS 1 `*.bag` inputs are not supported.

---

## `bagwiz cam-info replace`

Replace the calibration carried by one or more `sensor_msgs/msg/CameraInfo`
topics with the values from standard ROS camera calibration YAML files — the
kind produced by the `camera_calibration` package and consumed by
`camera_info_manager`. This is the offline equivalent of re-recording the bag
with a corrected calibration: useful when a bag was captured with a wrong or
placeholder calibration.

Each `-t` entry is either a bare `<topic>` — rewritten from the shared
`--yaml` — or `<topic>=<yaml>`, giving that topic its own calibration file.
The two forms can be mixed: bare entries share one calibration (for example a
`/camera_info` and a republished `/camera_info_throttled`) while `=` entries
each carry their own (for example one YAML per camera of a rig), all in a
single pass. `<topic>` is a topic selector — a literal name or a `*` glob
restricted to `sensor_msgs/msg/CameraInfo` topics (see
[Topic selectors](topic.md#topic-selectors)). The glob, not the whole
`<topic>[=<yaml>]` entry, is what expands: `<glob>` alone expands to a bare
`<topic>` per match (sharing `--yaml`), and `<glob>=<yaml>` expands to
`<topic>=<yaml>` for every matched topic, applying that one YAML to all of
them.

For every message on the chosen topic(s), the calibration fields are taken from
that topic's YAML while each message's own `header` timestamp, `header.frame_id`
(unless `--frame-id` is given), `binning_x` / `binning_y`, and `roi` are
preserved. Every other topic in the bag is copied verbatim.

### Usage

```text
bagwiz cam-info replace -i <input> [--yaml <yaml>] -t|--topics <topic>[=<yaml>]... [OPTIONS]
```

### Examples

```bash
# Fix a camera's intrinsics in place.
bagwiz cam-info replace -i drive.mcap --yaml left_camera.yaml -t /camera/left/camera_info

# Write a corrected copy and also relabel the frame, leaving the input untouched.
bagwiz cam-info replace -i drive.mcap --yaml left.yaml -t /camera/left/camera_info \
  --frame-id camera_left_optical_frame -o drive_fixed.mcap

# Apply one calibration to several CameraInfo topics in a single pass.
bagwiz cam-info replace -i drive.mcap --yaml shared.yaml \
  -t /camera/camera_info /camera/camera_info_throttled

# Give each camera its own calibration in a single pass.
bagwiz cam-info replace -i drive.mcap \
  -t /camera/left/camera_info=left.yaml /camera/right/camera_info=right.yaml

# Mix both forms: the bare topic falls back to --yaml.
bagwiz cam-info replace -i drive.mcap --yaml shared.yaml \
  -t /camera/camera_info /camera/wide/camera_info=wide.yaml

# Apply one calibration to every CameraInfo topic under /camera/ via a glob
# (quoted so the shell doesn't expand it).
bagwiz cam-info replace -i drive.mcap -t '/camera/*=shared.yaml'
```

### Options

| Flag                             | Description                                                                                                                                                                                                                                                                                                           |
| -------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`          | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                                                                                                                                                                                                                                              |
| `--yaml <yaml>`                  | Camera calibration YAML in the `camera_calibration` / `camera_info_manager` format, applied to every bare `<topic>` entry of `-t`. Required when at least one entry is bare; rejected when none is.                                                                                                                   |
| `-t`, `--topics <t>[=<yaml>]...` | **Required.** One or more CameraInfo topic selectors to rewrite, each as `<topic>` (uses `--yaml`) or `<topic>=<yaml>` (its own file), where `<topic>` is a literal name or a `*` glob (see [Topic selectors](topic.md#topic-selectors)). Each type must be `sensor_msgs/msg/CameraInfo`. The two forms can be mixed. |
| `--frame-id <id>`                | Override `header.frame_id` on the rewritten messages. When omitted, each message keeps its frame_id.                                                                                                                                                                                                                  |
| `-o`, `--output <p>`             | Write the result to a new bag instead of rewriting `<input>` in place.                                                                                                                                                                                                                                                |
| `-w`, `--overwrite`              | Replace an existing `-o` path. Without it, an existing output path stops the run. No effect in-place.                                                                                                                                                                                                                 |

### YAML format and field mapping

The input is the standard camera calibration YAML. Each matrix block is a
mapping of `rows`, `cols`, and a flat `data` sequence in row-major order:

```yaml
image_width: 640
image_height: 480
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.01, -0.02, 0.003, 0.004, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
```

| YAML key                  | CameraInfo field   | Required size        |
| ------------------------- | ------------------ | -------------------- |
| `image_width`             | `width`            | scalar               |
| `image_height`            | `height`           | scalar               |
| `distortion_model`        | `distortion_model` | string               |
| `distortion_coefficients` | `d`                | any non-empty `data` |
| `camera_matrix`           | `k`                | 9 (`data`, 3×3)      |
| `rectification_matrix`    | `r`                | 9 (`data`, 3×3)      |
| `projection_matrix`       | `p`                | 12 (`data`, 3×4)     |

`camera_name`, if present, is informational only — it is not a CameraInfo field
and is ignored. Each block's declared `rows * cols` must match its `data` length,
and `k` / `r` / `p` must yield exactly 9 / 9 / 12 values; otherwise the run stops
with an error before the bag is touched.

### Topic selection

One or more entries are given via `-t`/`--topics`, each as `<topic>` or
`<topic>=<yaml>`, where `<topic>` is a topic selector — a literal name or a
`*` glob restricted to `sensor_msgs/msg/CameraInfo` topics (see
[Topic selectors](topic.md#topic-selectors)).

- Only the resolved topics are rewritten; their message type is unchanged, so
  the bag's other topics and metadata are preserved exactly.
- A bare `<topic>` receives the shared `--yaml`; `<topic>=<yaml>` gives that
  topic its own calibration file. Topic names cannot contain `=`, so the split
  is at the first `=` — a YAML path containing `=` is fine. The split happens
  before glob expansion, so `<glob>=<yaml>` applies that one YAML to every
  topic the glob matches.
- `--yaml` is required when at least one entry is bare, and rejected when every
  entry carries its own `=<yaml>` (an unused `--yaml` is a likely mistake).
- Listing the same topic with two different YAMLs is an error. Repeating the
  exact same entry is harmless — duplicates are de-duplicated, and a listed
  topic that carries no messages is reported with a warning.

### In-place vs `-o`

- `<input>` doubles as the write-side target: without `-o` the bag is rewritten
  in place, mirroring `bagwiz traj join`.
- In-place mode replaces the input atomically via a sibling temporary bag, in the
  same storage backend and layout as the input.
- The output bag is always written uncompressed (re-compress later with
  `ros2 bag convert` if needed).

---

## `bagwiz cam-info recompute-p`

Recompute a projection matrix from the intrinsics it belongs to. `p` is derived
as:

```text
p = [ cv::getOptimalNewCameraMatrix(k, d, (width, height), alpha) | 0 ]
```

so `k`, `d`, and the image size are the **inputs** — everything else in the file
or message is preserved. This is the same computation the `camera_calibration`
package performs when it writes a monocular calibration, so it reconstructs a
`projection_matrix` that is missing, was hand-edited to something wrong, or has
gone stale after `k` changed. For a fisheye (`equidistant`) calibration the new
camera matrix comes from `cv::fisheye::estimateNewCameraMatrixForUndistortRectify`
instead — see
[Supported `distortion_model` values](#supported-distortion_model-values).

Note `p` is **not** `[k | 0]`: rectification re-maps pixels, so the rectified
image needs its own focal length and principal point. `alpha` chooses how.

### Usage

```text
bagwiz cam-info recompute-p -i <input> [-t|--topics <topics>...] [OPTIONS]
```

### Examples

```bash
# Fix a calibration file's projection matrix in place.
bagwiz cam-info recompute-p -i camera_info.yaml

# Write a corrected copy, keeping all source pixels.
bagwiz cam-info recompute-p -i camera_info.yaml --alpha 1.0 -o fixed.yaml

# Compose with `replace` to push a corrected calibration into a bag.
bagwiz cam-info recompute-p -i camera_info.yaml -o fixed.yaml
bagwiz cam-info replace -i drive.mcap --yaml fixed.yaml -t /camera/camera_info

# Recompute `p` directly on a bag's CameraInfo topics.
bagwiz cam-info recompute-p -i drive.mcap -t /camera/camera_info -o drive_fixed.mcap

# Recompute `p` on every CameraInfo topic under /camera/ via a glob (quoted
# so the shell doesn't expand it).
bagwiz cam-info recompute-p -i drive.mcap -t '/camera/*' -o drive_fixed.mcap

# Pull a bag's calibration out as a YAML — that is `cam-info dump`; `recompute-p`
# then fixes its `p` if you want.
bagwiz cam-info dump -i drive.mcap -t /camera/camera_info -o camera_info.yaml
bagwiz cam-info recompute-p -i camera_info.yaml
```

### Options

| Flag                    | Description                                                                                                                                                                                                                    |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `-i`, `--input <input>` | **Required.** Calibration YAML (`camera_calibration` / `camera_info_manager` format) **or** an input ROS 2 rosbag. Must exist.                                                                                                 |
| `-t`, `--topics <t>...` | Bag input only: one or more `sensor_msgs/msg/CameraInfo` topic selectors whose `p` to recompute — a literal name or a `*` glob (see [Topic selectors](topic.md#topic-selectors)). **Required** for a bag, rejected for a YAML. |
| `-a`, `--alpha <a>`     | OpenCV free-scaling parameter in `[0, 1]`; for a fisheye calibration it is passed to `cv::fisheye` as its `balance`. Default: `0`.                                                                                             |
| `-o`, `--output <p>`    | Write the result to a new path instead of rewriting `<input>` in place.                                                                                                                                                        |
| `-w`, `--overwrite`     | Replace an existing `-o` path. Without it, an existing output path stops the run. No effect in-place.                                                                                                                          |

### Input modes and output shape

`<input>` says where the calibration comes from, and decides whether `--topics`
applies:

| `<input>`               | Source                         | `--topics`   |
| ----------------------- | ------------------------------ | ------------ |
| a `.yaml` / `.yml` file | The file's own calibration.    | **Rejected** |
| anything else (bag/dir) | The named CameraInfo topic(s). | **Required** |

The result always has the same shape as `<input>` — a YAML in, a YAML out; a bag
in, a bag out. `-o` only says **where** it goes, never what it is. To pull a
bag's calibration out as a YAML instead, use
[`cam-info dump`](#bagwiz-cam-info-dump).

For a bag, `-o`'s extension does pick the **storage format**:

| `-o`          | Result                                                      |
| ------------- | ----------------------------------------------------------- |
| `out.mcap`    | A single-file MCAP bag, converting if `<input>` is SQLite3. |
| `out.db3`     | A single-file SQLite3 bag, converting if `<input>` is MCAP. |
| anything else | A directory bag in `<input>`'s own format.                  |

Without `-o` the input is rewritten in place, mirroring `cam-info replace`.

`--topics` is required for a bag rather than defaulting to every CameraInfo
topic: rewriting topics you did not name would be a guess, and `<input>` doubles
as the in-place target. Because the requirement depends on what `<input>` turns
out to be, it is reported when the run starts rather than by the argument parser:

```console
$ bagwiz cam-info recompute-p -i drive.mcap
[ERROR] 'drive.mcap' is a bag, so --topics is required to say which CameraInfo topic's p to
recompute. Pass -t/--topics <topic>..., or pass a .yaml calibration file as <input>.
```

Tab completion offers the bag's CameraInfo topics — and only those — at every
`--topics` value slot.

**YAML mode is a re-emit, not an edit.** Values are preserved (including
`camera_name`), but comments, key order, and incidental formatting are
normalized. Use `-o` to keep the original file untouched.

### Per-message recomputation

- `--topics` accepts several topics at once
  (`--topics /cam1/camera_info /cam2/camera_info`); each is recomputed from its
  own intrinsics, so unlike `replace` they need not share a calibration.
- In bag mode each message's `p` is recomputed from that **same message's** own
  `k` / `d` / `width` / `height`, so a stream whose calibration changes mid-bag is
  handled correctly. A constant stream is the common case and is memoized, so
  OpenCV is consulted once rather than per message.
- Only the named topics are rewritten. Each message's `header` (stamp and
  `frame_id`), `binning_x` / `binning_y`, `roi`, and every other topic are copied
  verbatim — only `p` changes.

### Supported `distortion_model` values

| `distortion_model`        | Supported | Behavior                                                                                                                |
| ------------------------- | --------- | ----------------------------------------------------------------------------------------------------------------------- |
| `plumb_bob`               | ✅        | Brown–Conrady, 5 coefficients. The ROS default. `cv::getOptimalNewCameraMatrix`.                                        |
| `rational_polynomial`     | ✅        | The same model with 8 coefficients.                                                                                     |
| `equidistant` / `fisheye` | ✅        | Fisheye: `cv::fisheye::estimateNewCameraMatrixForUndistortRectify`, with `--alpha` passed as its `balance` — see below. |
| `""` (empty) / `none`     | ✅        | Declares no lens distortion, so `p` is `[k \| 0]` whatever `d` holds.                                                   |
| anything else             | ❌        | Error.                                                                                                                  |

An unsupported model **stops the run with an error** rather than producing a
best-effort `p`. The model is validated _before_ `d` is examined, so an
unsupported model is refused even when its coefficients happen to be all zero:

```console
$ bagwiz cam-info recompute-p -i ds_cam.yaml
[ERROR] Cannot recompute p for 'ds_cam.yaml': distortion_model 'double_sphere' is not supported; p
can be recomputed for 'plumb_bob', 'rational_polynomial', or the fisheye models
'equidistant'/'fisheye' (an empty model or 'none' is treated as distortion-free)
```

For a fisheye calibration the projection matrix comes from
`cv::fisheye::estimateNewCameraMatrixForUndistortRectify`, which is
parameterized by a `balance` rather than an `alpha`. The two knobs mean the
same thing to a user — `0` crops to valid pixels, `1` keeps every source pixel
— so `--alpha`'s value is passed through as the `balance`. `cv::fisheye` takes
exactly four coefficients, so the first four of `d` are used and missing ones
are treated as zero, matching the point-cloud projector's `equidistant`
handling.
Nothing is written when a model is rejected in YAML mode or in place; with
`-o`, messages streamed before the first offending CameraInfo may already have
been written, leaving a partial output bag.

### Choosing `alpha`

`alpha` trades black borders against cropping in the rectified image:

| `alpha` | Meaning                                                                                                            |
| ------- | ------------------------------------------------------------------------------------------------------------------ |
| `0`     | Keep only valid pixels — zoom until no black border remains. The `camera_calibration` default, and this command's. |
| `1`     | Retain every source pixel — nothing is cropped, but the edges show black borders.                                  |
| between | A linear trade-off between the two.                                                                                |

For a fisheye calibration the same value is passed to `cv::fisheye` as its
`balance`, which implements the same trade-off.

### Sub-pixel changes are expected

`cv::getOptimalNewCameraMatrix` gives slightly different answers across OpenCV
versions — between 4.5.4 and 4.13.0 the result moves by up to **0.77 px** on a
1920×1280 `plumb_bob` calibration — and bagwiz builds against each ROS distro's
own OpenCV. So recomputing a `p` that an older `camera_calibration` wrote
**changes it slightly** rather than reproducing it exactly, and the same input
can give marginally different output on different distros.

This is benign: the recomputed `p` is consistent with the OpenCV that this binary
will later feed it to (`movify cam --rectify`, the point-cloud overlay).
The run reports how far `p` moved so a small change is legible as version drift
rather than a correction:

```text
p changed by at most 0.767 px (alpha=0.00, OpenCV 4.13.0). A sub-pixel change like this is
cv::getOptimalNewCameraMatrix differing across OpenCV versions, not a corrected calibration.
```

A genuinely wrong `p` is off by tens or hundreds of pixels, so the two are easy
to tell apart.

### When it refuses

Recomputing `p` from `k` is **wrong**, not merely imprecise, in these cases, so
the run stops with an error. In YAML mode and in place nothing is written; with
`-o`, messages streamed before the first offending CameraInfo may already have
been written, leaving a partial output bag:

| Condition                              | Why                                                                                                                                                                                                                         |
| -------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `r` is a genuine non-identity rotation | The camera is stereo-rectified: its `p` comes from `cv::stereoRectify` against the paired camera. Recomputing would break rectification. An all-zero (unset) `r` is treated as identity, matching the rectification helper. |
| `p[3]` or `p[7]` is non-zero           | `p` carries a stereo baseline (`p[3] = -fx · baseline`); `[newK \| 0]` would zero it and lose the extrinsic.                                                                                                                |
| `distortion_model` is unsupported      | Only `plumb_bob`, `rational_polynomial`, and an empty / `none` model can be recomputed — see [Supported `distortion_model` values](#supported-distortion_model-values).                                                     |
| `width` or `height` is 0               | No image size means no valid new camera matrix.                                                                                                                                                                             |
| `k` is degenerate or non-finite        | `fx`/`fy` must be positive and every entry finite.                                                                                                                                                                          |

When the model is Brown–Conrady but `d` is empty or all-zero there is no distortion to
correct, so the result is exactly `[k | 0]`.

### In-place vs `-o`

- In-place mode replaces the input atomically: a bag via a sibling temporary bag,
  a YAML via a sibling temporary file.
- The output bag is always written uncompressed (re-compress later with
  `ros2 bag convert` if needed).

---

## `bagwiz cam-info dump`

Write the calibration carried by a `sensor_msgs/msg/CameraInfo` topic out as a
standard ROS camera calibration YAML — the inverse of
[`replace`](#bagwiz-cam-info-replace), and the format
[`replace`](#bagwiz-cam-info-replace) consumes. Useful to inspect what a bag
actually recorded, or to lift a calibration out and edit it before pushing a
corrected copy back in.

The dump is **verbatim**: `height`, `width`, `distortion_model`, `d`, `k`, `r`,
and `p` are copied exactly as recorded. In particular `p` is **not** recomputed —
[`recompute-p`](#bagwiz-cam-info-recompute-p) is the command for that, and the
two compose.

### Usage

```text
bagwiz cam-info dump -i <input> -t <topic> [OPTIONS]
```

### Examples

```bash
# Look at what a bag recorded.
bagwiz cam-info dump -i drive.mcap -t /camera/camera_info

# Save it to a file.
bagwiz cam-info dump -i drive.mcap -t /camera/camera_info -o camera_info.yaml

# Compose with `recompute-p` to fix a stale `p`, then push it back with `replace`.
bagwiz cam-info dump -i drive.mcap -t /camera/camera_info -o camera_info.yaml
bagwiz cam-info recompute-p -i camera_info.yaml
bagwiz cam-info replace -i drive.mcap --yaml camera_info.yaml -t /camera/camera_info
```

### Options

| Flag                    | Description                                                                                                                       |
| ----------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>` | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                                                          |
| `-t`, `--topic <topic>` | The CameraInfo topic whose calibration to write. Its type must be `sensor_msgs/msg/CameraInfo`. A literal topic name, not a glob. |
| `-o`, `--output <p>`    | Write the YAML to this path instead of stdout.                                                                                    |
| `-w`, `--overwrite`     | Replace an existing `-o` path. Without it, an existing output path stops the run. No effect without `-o`.                         |

Exactly one `<topic>` is taken, since a camera calibration YAML holds exactly one
calibration. Tab completion offers the bag's CameraInfo topics — and only those.

### YAML format

The output is the standard camera calibration YAML documented under
[`replace`](#yaml-format-and-field-mapping), minus `camera_name`: it is not a
CameraInfo field, so the bag cannot supply one. The key is optional, and
inventing a name from the topic or `frame_id` would be a guess.

### Which message is dumped

The **first** message's calibration is used. A topic whose calibration is not
constant across the bag is reported with a warning saying so — one YAML cannot
represent a stream that changes mid-bag.

### Output

Without `-o` the YAML goes to **stdout** and every diagnostic goes to stderr, so
`bagwiz cam-info dump -i drive.mcap -t /camera/camera_info > camera_info.yaml` is
pipe-clean. The bag is opened read-only and is never modified.

### Errors

| Situation                                                                       | Outcome                            |
| ------------------------------------------------------------------------------- | ---------------------------------- |
| The topic is missing, is not a `sensor_msgs/msg/CameraInfo` topic, or is empty. | The run stops; nothing is written. |
| The `-o` path already exists and `-w` / `--overwrite` is not given.             | The run stops; nothing is written. |

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
