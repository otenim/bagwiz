# `bagwiz calib`

Sensor-extrinsic calibration tools.

| Subcommand                             | What it does                                                                                                                                               |
| -------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [`cam-lidar`](#bagwiz-calib-cam-lidar) | Refine one static-TF edge on a camera's chain by registering the bag's LiDAR map (from `map slam`) against its images; writes YAML for `tf static update`. |

---

## `bagwiz calib cam-lidar`

Automatically refines one static-TF edge on a camera's chain by registering
the bag's dense LiDAR map (from a prior
[`bagwiz map slam`](map.md#bagwiz-map-slam) run) against the bag's own
images, minimizing the normalized information distance (NID) between
projected map intensity and image intensity. The refined edge is written as a
[`static dump`](tf.md#bagwiz-tf-static-dump)-schema YAML that
[`static update`](tf.md#bagwiz-tf-static-update) applies; `--input` itself is only
ever read, never modified.

### Usage

```text
bagwiz calib cam-lidar -i <input> --map <map.pcd> --traj <traj.tum> \
  --traj-frame <frame> -t|--topic <topic> --parent <frame> --child <frame> \
  [--cam-info <topic>] [-o <output>] [--samples <n>] [--fix <axes>] \
  [--keyframe-dist <m>] [--keyframe-rot <deg>] \
  [--max-trans <m>] [--max-rot <deg>] [--nid-bins <n>] [--min-depth <m>] \
  [--max-depth <m>] [--json] [-w|--overwrite]
```

### Example

```bash
# 1. Build a map + trajectory of the bag, in the frame the edited edge's
#    chain starts from.
bagwiz map slam -i capture.mcap --pcd /sensing/lidar/concatenated/pointcloud \
  -o out/ --frame base_link

# 2. Refine the camera mount edge against that map.
bagwiz calib cam-lidar -i capture.mcap --map out/map.pcd --traj out/traj.tum \
  --traj-frame base_link -t /sensing/camera/camera1/image_raw/compressed \
  --parent truck_cabin_base_link --child top_front_narrow/camera_link

# 3. Apply the refined edge back into the bag's static TF.
bagwiz tf static update -i capture.mcap --yaml capture_calib_cam_lidar.yaml
```

### Options

| Flag                    | Description                                                                                                                                                                                                                                                                                           |
| ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>` | **Required.** ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`, `*.db3.zstd`).                                                                                                                                                                                                                 |
| `--map <path>`          | **Required.** Dense map PCD from `map slam` — needs an `intensity` field, which NID compares against image gray. Long-form only.                                                                                                                                                                      |
| `--traj <path>`         | **Required.** TUM trajectory from `map slam`. Long-form only.                                                                                                                                                                                                                                         |
| `--traj-frame <frame>`  | **Required.** Frame the trajectory poses express — must be the same `--frame` `map slam` was run with. Long-form only.                                                                                                                                                                                |
| `-t`, `--topic <topic>` | **Required.** Image topic to calibrate against. Supported types: `sensor_msgs/msg/Image` (`bgr8`, `rgb8`) and `sensor_msgs/msg/CompressedImage` (JPEG/PNG). A literal topic name, not a glob.                                                                                                         |
| `--parent <frame>`      | **Required.** Parent frame of the static edge to refine. Long-form only.                                                                                                                                                                                                                              |
| `--child <frame>`       | **Required.** Child frame of the static edge to refine. Long-form only.                                                                                                                                                                                                                               |
| `--cam-info <topic>`    | CameraInfo topic. When omitted, resolved from `-t`/`--topic` using the same auto-resolution rules as [`generate video`'s `--cam-info`](generate.md#bagwiz-generate-video). A literal topic name, not a glob. Long-form only.                                                                          |
| `-o`, `--output <path>` | Output YAML path. Default: `<input>`'s filename stem plus `_calib_cam_lidar.yaml`, written in the current working directory (not necessarily beside `--input`).                                                                                                                                       |
| `--samples <n>`         | Image samples to pick, evenly spread across the trajectory span (or across keyframe intervals — see `--keyframe-dist`). Default `8`, minimum `3`. Long-form only.                                                                                                                                     |
| `--keyframe-dist <m>`   | Pose-gated keyframe sampling: a new keyframe interval opens each time the interpolated pose moves this many meters, samples spread over the intervals instead of over time, and each picked interval contributes its sharpest frame. `0` (the default) keeps plain even time spacing. Long-form only. |
| `--keyframe-rot <deg>`  | Rotation half of the keyframe gate: an interval also opens after this much rotation from the interval's first frame, so a platform turning in place keeps contributing new viewpoints. `0` (the default) disables the rotation test. Long-form only.                                                  |
| `--fix <axes>`          | Comma list of axes to hold at the bag's value instead of optimizing (`x,y,z,roll,pitch,yaw`, any subset). Fixing all six is rejected — nothing would be left to refine. Long-form only.                                                                                                               |
| `--max-trans <m>`       | Trust region: max translation delta from the bag's value, in meters. Default `0.2`. Long-form only.                                                                                                                                                                                                   |
| `--max-rot <deg>`       | Trust region: max rotation delta from the bag's value, in degrees. Default `2.0`. Long-form only.                                                                                                                                                                                                     |
| `--nid-bins <n>`        | NID intensity/gray histogram bins, `4`–`256`. Default `16`. Long-form only.                                                                                                                                                                                                                           |
| `--min-depth <m>`       | Nearest projected map-point depth kept, in meters. Default `2`. Long-form only.                                                                                                                                                                                                                       |
| `--max-depth <m>`       | Farthest projected map-point depth kept, in meters. Default `150`. Long-form only.                                                                                                                                                                                                                    |
| `--json`                | Emit the stdout summary as JSON instead of the human table. The YAML is written either way. Long-form only.                                                                                                                                                                                           |
| `-w`, `--overwrite`     | Replace an existing `-o`/`--output` path.                                                                                                                                                                                                                                                             |

### The map/trajectory handshake

`--map` and `--traj` are `map slam`'s own outputs (`map.pcd` and `traj.tum`
under its `-o` root) — run `map slam` over this bag first. Two constraints tie
them back to it:

- `--traj-frame` must be the exact frame `map slam --frame` used. The
  trajectory's poses are expressed in that frame, and `cam-lidar` needs to
  know which frame that is to resolve the static-TF chain from it to the
  image topic's camera-optical frame.
- `--map` must carry an `intensity` field, since NID compares projected map
  intensity against image gray; `map slam`'s own output has it, but a
  hand-built or re-processed PCD without one is rejected before any refining
  starts.

`--parent`/`--child` must name an edge that is both on the resolved chain
from `--traj-frame` to the camera's optical frame, and recorded directly on a
static TF topic (e.g. `/tf_static`) — an edge only reachable through dynamic
`/tf` is not something `static update` can rewrite later, so `cam-lidar`
rejects it up front.

### Sample selection

By default samples are spread evenly over the trajectory's **time** span
(minus a 3 s margin at each end so every pick can be interpolated). Even time
spacing is only even _viewpoint_ spacing at constant speed: a stop at a light
turns several picks into near-duplicates of one scene, which both wastes
samples and over-weights that scene in the NID sum. With `--keyframe-dist`
(and optionally `--keyframe-rot`), the eligible frames are first partitioned
into **keyframe intervals** — a new interval opens once the pose has moved or
rotated enough since the interval's first frame, the same gate
[`map slam --color-min-dist`](map.md#camera-colorization---color) applies —
and `--samples` intervals are picked evenly instead. Each picked interval
then contributes its **sharpest** member (highest mean image gradient, the
`--color-keyframe-blur` policy): blur is what actually weakens NID, whether it
came from a turn, a bump, or exposure, so sharpness is gated directly rather
than by any motion proxy. A recording whose gate finds fewer than 3 intervals
(a near-stationary platform) falls back to plain even time spacing with a
warning. `--keyframe-dist 1.0 --keyframe-rot 10` is a reasonable starting
point for driving data.

### Method

For each picked sample, `cam-lidar` interpolates the trajectory's pose at the
image's `header.stamp` (falling back to the message's bag record time when
unset, warning once for the whole run rather than per image), projects the
map's points into the camera through the current extrinsic estimate, and
scores the alignment as the NID between the projected points' intensity
histogram and the image's grayscale patch — lower is better. A two-pass
Nelder-Mead search over the free axes (everything not named by `--fix`)
minimizes the mean NID across all samples, confined to the trust region
around the edge's bag value (`--max-trans`, `--max-rot`), so a bad initial
mount value or an unconstrained axis cannot wander to an unrelated optimum.

The six numbers the search moves are the edge's own `x, y, z, roll, pitch,
yaw` — the scalars [`static dump`](tf.md#bagwiz-tf-static-dump) writes — and the
delta is added to them axis by axis. The value the cost was evaluated at, the
`refined value` column of the report, and the transform in the output YAML are
therefore the same arithmetic, `before + delta` per axis, and cannot describe
different edges. `--fix <axis>` holds an axis by forcing its delta to zero,
which leaves the bag's own scalar for that axis in the output verbatim even
when the edge's rotation does not commute (an optical-convention mount, say).

### Observability report

After refining, each of the six axes is probed independently around the
optimum and reported as `strong`, `weak`, `degenerate`, or (for an axis named
by `--fix`) `fixed`. The probe is a symmetric second difference of the mean
NID along that one axis: a cost surface that curves sharply around the optimum
reads `strong`, a flat one `degenerate`. `degenerate` means the sampled views
could not pin that axis down at all — its reported delta is essentially
whatever the optimizer's flat cost surface happened to land on, not a real
correction, and emphatically not the bag's own value — and a warning
recommends re-running with `--fix <axis>` to hold it at the bag's value
outright, rather than trusting a delta the data never actually constrained.
A single forward-looking, narrow-field-of-view (telephoto) camera commonly
cannot observe its own forward translation, and cannot tell lateral
translation apart from yaw (both shift image content the same way from that
vantage point), so those are the two common degenerate cases; a wider-angle
or multi-view rig sees them better.

**What the classification does not tell you.** It is a coarse screen, not a
covariance estimate, and it is worth reading with three caveats in mind:

- It probes one axis at a time, so it only detects _hard, single-axis_
  degeneracies. A pairwise trade-off — the lateral-translation/yaw valley
  above is the standard example — leaves both axes curving along their own
  probe directions and so reads `strong` on both, even though only their
  combination is determined.
- Its thresholds are absolute constants calibrated against synthetic scenes,
  not scaled to the scene, the sample count, or the NID's own noise floor. A
  recording whose NID is flatter or noisier overall shifts every axis's
  reading together.
- Consequently, on real recordings all six axes can come back `strong` while a
  span analysis over repeated runs (varying the samples, or sweeping one axis
  and watching where the NID actually moves) shows several of them only weakly
  determined. Treat `degenerate` as a reliable "definitely not observable" and
  `strong` as no more than "not obviously unobservable"; for a number you are
  going to ship, corroborate it against a second run over different samples.

```text
calib cam-lidar: truck_cabin_base_link -> top_front_narrow/camera_link
axis        bag value  refined value          delta  observability
x            0.180000       0.183214       0.003214  degenerate
y           -0.050000      -0.047850       0.002150  degenerate
z            1.420000       1.447213       0.027213  strong
roll         0.000000      -0.008421      -0.008421  strong
pitch        0.000000       0.014732       0.014732  strong
yaw          0.000000       0.009481       0.009481  degenerate

nid: 0.412887 -> 0.276541
samples used: 8
warning: x is not observable from this data; the delta shown is unconstrained — re-run with --fix x to hold the bag value
warning: y is not observable from this data; the delta shown is unconstrained — re-run with --fix y to hold the bag value
warning: yaw is not observable from this data; the delta shown is unconstrained — re-run with --fix yaw to hold the bag value

apply with: bagwiz tf static update -i capture.mcap --yaml capture_calib_cam_lidar.yaml
```

Rotations in the human table are shown in degrees; `--json` reports the same
`before`/`after`/`delta` per axis in radians instead, alongside the `parent`,
`child`, `nid_before`, `nid_after`, and `samples` fields.

### Failures

Beyond the codes in [Exit status](#exit-status) below, exit `1` covers: an
invalid flag combination (`--samples` under 3, `--fix` naming an unknown axis
or all six, a non-positive `--max-trans`/`--max-rot`/`--min-depth`, `--nid-bins`
outside `4`–`256`, or `--max-depth` at or below `--min-depth`), an unreadable
or intensity-less `--map`, a `--traj` with fewer than two poses, a missing or
wrong-typed `-t`/`--topic` or an unresolvable CameraInfo, a `--parent`/`--child`
edge that is not on the static chain (or not directly on a static topic), too
few usable image samples surviving the trajectory-span and pre-cull filtering,
or a refinement failure (e.g. no sample projects enough map points at the
initial estimate).

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
