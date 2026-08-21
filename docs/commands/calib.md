# `bagwiz calib`

Sensor-extrinsic calibration tools.

| Subcommand                             | What it does                                                                                                                                |
| -------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| [`cam-lidar`](#bagwiz-calib-cam-lidar) | Refine one static-TF edge on a camera's chain by registering the bag's LiDAR clouds against its images; writes YAML for `tf static update`. |

---

## `bagwiz calib cam-lidar`

Automatically refines one static-TF edge on a camera's chain by registering
the bag's LiDAR point clouds against the bag's own images, minimizing the
normalized information distance (NID) between projected cloud intensity and
image intensity. Everything is read from the one bag: the clouds are
accumulated into a map through the bag's own self-position topic, and the
image samples are placed by the same trajectory. The refined edge is written
as a [`static dump`](tf.md#bagwiz-tf-static-dump)-schema YAML that
[`static update`](tf.md#bagwiz-tf-static-update) applies; `--input` itself is
only ever read, never modified.

### Usage

```text
bagwiz calib cam-lidar -i <input> --pcd <topic> --pose <topic> --cam <topic> \
  [--of <frame>] [--ref <frame>] --parent <frame> --child <frame> \
  [--cam-info <topic>] [-o <output>] [--samples <n>] [--fix <axes>] \
  [--keyframe-dist <m>] [--keyframe-rot <deg>] \
  [--max-trans <m>] [--max-rot <deg>] [--nid-bins <n>] [--min-depth <m>] \
  [--max-depth <m>] [--voxel <m>] [--skip-start <dur>] [--skip-end <dur>] \
  [--cam-offset <dur>] [--json] [-w|--overwrite]
```

### Example

```bash
# 1. Refine the camera mount edge against the bag's own LiDAR and pose topics.
bagwiz calib cam-lidar -i capture.mcap \
  --pcd /sensing/lidar/concatenated/pointcloud \
  --pose /localization/pose \
  --cam /sensing/camera/camera1/image_raw/compressed \
  --parent truck_cabin_base_link --child top_front_narrow/camera_link

# 2. Apply the refined edge back into the bag's static TF.
bagwiz tf static update -i capture.mcap --yaml capture_calib_cam_lidar.yaml
```

### Options

| Flag                    | Description                                                                                                                                                                                                                                                                                                |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>` | **Required.** ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`, `*.db3.zstd`).                                                                                                                                                                                                                      |
| `--pcd <topic>`         | **Required.** PointCloud2 topic accumulated into the calibration map — needs an `intensity` field, which NID compares against image gray. A literal topic name, not a glob. Long-form only.                                                                                                                |
| `--pose <topic>`        | **Required.** Self-position topic the map and the image samples are placed by. Supported types: `tf2_msgs/msg/TFMessage`, `nav_msgs/msg/Odometry`, `geometry_msgs/msg/PoseStamped`, `geometry_msgs/msg/PoseWithCovarianceStamped` (the same set [`pcd undistort --pose`](pcd.md) accepts). Long-form only. |
| `--cam <topic>`         | **Required.** Image topic to calibrate against. Supported types: `sensor_msgs/msg/Image` (`bgr8`, `rgb8`) and `sensor_msgs/msg/CompressedImage` (JPEG/PNG). A literal topic name, not a glob. Long-form only.                                                                                              |
| `--of <frame>`          | Frame the `--pose` trajectory tracks. Anchors the static-TF chain to the camera's optical frame and each cloud's extrinsic. Default `base_link`. Long-form only.                                                                                                                                           |
| `--ref <frame>`         | Frame the `--pose` trajectory is expressed in; the map is accumulated in it. Default `map`. Long-form only.                                                                                                                                                                                                |
| `--parent <frame>`      | **Required.** Parent frame of the static edge to refine. Long-form only.                                                                                                                                                                                                                                   |
| `--child <frame>`       | **Required.** Child frame of the static edge to refine. Long-form only.                                                                                                                                                                                                                                    |
| `--cam-info <topic>`    | CameraInfo topic. When omitted, resolved from `--cam` using the same auto-resolution rules as [`generate video`'s `--cam-info`](generate.md#bagwiz-generate-video). A literal topic name, not a glob. Long-form only.                                                                                      |
| `-o`, `--output <path>` | Output YAML path. Default: `<input>`'s filename stem plus `_calib_cam_lidar.yaml`, written in the current working directory (not necessarily beside `--input`).                                                                                                                                            |
| `--samples <n>`         | Image samples to pick, evenly spread across the trajectory span (or across keyframe intervals — see `--keyframe-dist`). Default `8`, minimum `3`. Long-form only.                                                                                                                                          |
| `--keyframe-dist <m>`   | Pose-gated keyframe sampling: a new keyframe interval opens each time the interpolated pose moves this many meters, samples spread over the intervals instead of over time, and each picked interval contributes its sharpest frame. `0` (the default) keeps plain even time spacing. Long-form only.      |
| `--keyframe-rot <deg>`  | Rotation half of the keyframe gate: an interval also opens after this much rotation from the interval's first frame, so a platform turning in place keeps contributing new viewpoints. `0` (the default) disables the rotation test. Long-form only.                                                       |
| `--fix <axes>`          | Comma list: axes to hold at the bag value (`x,y,z,roll,pitch,yaw`), plus `auto` — also hold every direction the data cannot constrain — and `none` — hold nothing. A manual axis list alone switches `auto` off; fixing all six is rejected. Default `auto`. Long-form only.                               |
| `--max-trans <m>`       | Trust region: max translation delta from the bag's value, in meters. Default `0.2`. Long-form only.                                                                                                                                                                                                        |
| `--max-rot <deg>`       | Trust region: max rotation delta from the bag's value, in degrees. Default `2.0`. Long-form only.                                                                                                                                                                                                          |
| `--nid-bins <n>`        | NID intensity/gray histogram bins, `4`–`256`. Default `16`. Long-form only.                                                                                                                                                                                                                                |
| `--min-depth <m>`       | Nearest projected map-point depth kept, in meters. Default `2`. Long-form only.                                                                                                                                                                                                                            |
| `--max-depth <m>`       | Farthest projected map-point depth kept, in meters. Default `150`. Long-form only.                                                                                                                                                                                                                         |
| `--voxel <m>`           | Edge length of the grid the accumulated map is collapsed onto, in meters. Default `0.1`; `0` keeps every point of every cloud. See [How the map is built](#how-the-map-is-built). Long-form only.                                                                                                          |
| `--skip-start <dur>`    | Exclude this duration, measured from the bag's start, from the estimation (e.g. `30s`; a unit suffix is required: `ns`/`us`/`ms`/`s`). See [Sample selection](#sample-selection). Long-form only.                                                                                                          |
| `--skip-end <dur>`      | Exclude this duration, measured from the bag's end, from the estimation. Same duration grammar as `--skip-start`. Long-form only.                                                                                                                                                                          |
| `--cam-offset <dur>`    | Signed duration added to every image stamp before the `--pose` lookup, so the image stamped $t$ is placed at the pose of $t + \text{offset}$; a camera clock that stamps late is corrected with a negative value (e.g. `-42ms`; same grammar as `--skip-start`). See [Method](#method). Long-form only.    |
| `--json`                | Emit the stdout summary as JSON instead of the human table. The YAML is written either way. Long-form only.                                                                                                                                                                                                |
| `-w`, `--overwrite`     | Replace an existing `-o`/`--output` path.                                                                                                                                                                                                                                                                  |

### How the map is built

The trajectory is the pose of `--of` expressed in `--ref`, sampled from the
`--pose` topic exactly as [`pcd undistort`](pcd.md) builds it: a TFMessage
topic's `--of` → `--ref` path is replayed and sampled, while an Odometry /
PoseStamped / PoseWithCovarianceStamped topic contributes its own poses,
bridged into `--ref` (and an Odometry child frame into `--of`) through the
bag's static TF when the frames differ.

Every cloud on the `--pcd` topic is then placed into that frame as
`T_ref_of(header.stamp) * T_of_cloud`, the extrinsic coming from the bag's
static TF, and accumulated into a single map. Four behaviors are worth
knowing:

- The map covers only what the picked image samples can look at: a point
  that falls outside every sample's view (the union of the sample frusta,
  padded like the per-sample pre-cull and widened for distortion) is dropped
  as it arrives, so neither the voxel grid nor memory pays for the rest of
  the scene. For a narrow camera against a 360° lidar this is most of the
  recording; the log reports the culled count. The cull is a superset
  filter: the exact per-sample projection predicate still runs at candidate
  assembly, and the NID bins are equalized over those candidates (see
  [Method](#method)), so it does not move the calibration.
- A cloud whose per-point time field is usable **and not uniform** holds a
  real sweep, so it is deskewed to its own `header.stamp` first (the same
  deskew `pcd undistort` applies). A uniform field — all zeros, or the
  constant field a cloud carries after `pcd undistort` rewrote it — means no
  sweep motion and the cloud is accumulated as-is.
- A cloud stamped outside the trajectory's time span is skipped (with a
  warning count), never clamped to an endpoint pose — clamping would smear
  the map.
- The topic must carry an `intensity` field, since NID compares projected
  cloud intensity against image gray; a topic without one is rejected before
  any refining starts.

Points are collapsed onto a voxel grid as they arrive — one point per
occupied voxel, at the centroid of everything that landed in it and carrying
their mean intensity — so the map costs memory in proportion to the
**surface** it covers rather than to the number of points recorded. This
matters because a driving platform re-measures the same surface once per
sweep, hundreds of times over a bag, and NID reads the map as a statistical
sample of (intensity, gray) pairs that those duplicates do not enrich. The
run logs both counts, e.g. `Map: 2100000 point(s) on a 0.100 m voxel grid
from 118 cloud(s) ... (37223852 point(s) read, 0 deskewed)`.

`--voxel 0` turns the grid off and keeps every point of every cloud, at the
memory and per-iteration cost the raw density implies — worth it only for a
short recording, or a `--pcd` topic that is already a sparse or
pre-downsampled map. The emitted map is ordered by voxel index either way, so
two runs over the same clouds build the same map.

`--parent`/`--child` must name an edge that is both on the resolved chain
from `--of` to the camera's optical frame, and recorded directly on a
static TF topic (e.g. `/tf_static`) — an edge only reachable through dynamic
`/tf` is not something `static update` can rewrite later, so `cam-lidar`
rejects it up front.

### Sample selection

`--skip-start` / `--skip-end` first shrink what "the trajectory's time span"
means below: each drops the poses lying inside its skipped range — measured
from the bag's start / end, not from the pose topic's own span — and the map
accumulation, the sample eligibility, and the deskew clamping all follow the
trimmed span, so clouds and images in the skipped ranges never enter the
estimation. Skips that together cover the whole bag, or that leave fewer than
two trajectory poses, are rejected.

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
histogram and the image's grayscale patch — lower is better. The intensity
axis of the joint histogram is histogram-equalized over the union of all
samples' projected candidate points, so the binning is decided by exactly
what NID scores and does not move with the map's coverage (the frustum cull
above, the voxel size, or where the map came from). A two-pass
Nelder-Mead search over the free axes (everything not named by `--fix`)
minimizes the mean NID across all samples, confined to the trust region
around the edge's bag value (`--max-trans`, `--max-rot`), so a bad initial
mount value or an unconstrained axis cannot wander to an unrelated optimum.

`--cam-offset` shifts that lookup: every image stamp has the offset added the
moment it is read, so sample eligibility and picking, the keyframe gate, the
map's frustum cull, the pre-cull and each sample's pose all see the same
shifted time and the image stamped $t$ is placed at the trajectory's pose of
$t + \text{offset}$. Clouds and poses are never shifted. The sign is literal:
when the camera's clock stamps later than the `--pose` clock (an image stamped
$t$ was really taken when the trajectory was at $t - 42\,\text{ms}$), pass
`--cam-offset -42ms`. A constant offset of a few tens of milliseconds is an
ordinary property of a multi-sensor rig — at highway speed it is a displacement
of the order of a meter plus the rotation swept in that time at every sample —
and is not something the NID search can absorb, so it has to be measured and
passed in. The run logs the applied offset, the human report prints it as
`camera stamp offset` when non-zero, and `--json` always carries it as
`cam_offset_ns`.

The six numbers the search moves are the edge's own `x, y, z, roll, pitch,
yaw` — the scalars [`static dump`](tf.md#bagwiz-tf-static-dump) writes — and the
delta is added to them axis by axis. The value the cost was evaluated at, the
`refined value` column of the report, and the transform in the output YAML are
therefore the same arithmetic, `before + delta` per axis, and cannot describe
different edges. `--fix <axis>` holds an axis by forcing its delta to zero,
which leaves the bag's own scalar for that axis in the output verbatim even
when the edge's rotation does not commute (an optical-convention mount, say).
The default `--fix auto` generalizes this from axes to directions — see the
next section.

### Observability report and automatic holding

After refining, each of the six axes is probed around the optimum and
reported as `strong`, `weak`, `degenerate`, or (for an axis named by `--fix`)
`fixed`. The probe is a symmetric second difference of the mean NID along
that axis, estimated per sample: an axis's curvature is the mean of the
per-sample second differences, and it counts as measured at all only when
that mean both clears a small absolute floor and stands out of its own
standard error across samples — 97.7% confidence it is positive (the normal
two-sigma tail), `degenerate` otherwise, and `strong` at 99.9% confidence,
`weak` in between. The multiplier is the Student-t quantile with samples−1
degrees of freedom rather than the normal one, so the stated confidence
holds at `--samples 8` and not only asymptotically (at 8 samples the two
boundaries sit at 2.43 and 4.79 standard errors, converging on 2.0 and 3.1
as the count grows). The per-sample pairing is what keeps the reading
meaningful across scenes: a flat but quiet surface and a curved but noisy
one do not classify the same. The `curv/se` column shows the number behind
each verdict — the mean curvature in units of its own standard error — so a
borderline label reads as the graded quantity it is rather than as
categorical; `--json` carries the same evidence per axis as `curvature`,
`std_error`, and their ratio.

With the default `--fix auto`, the same measurement also runs on the
eigen-directions of the full 6x6 curvature matrix rather than per axis, so a
degenerate _combination_ — the classic forward-camera valley in which
lateral translation and yaw shift the image the same way — is caught even
though each of its axes curves on its own. Every _clearly_ unobservable
direction — mean curvature within one standard error of zero, strictly
inside the significance band — is then held at the bag value (its delta
component forced to zero) and the remaining directions are re-optimized,
repeating until nothing clearly unobservable is left. A borderline
direction, insignificant but not clearly so, is left free rather than
pinned on a noisy reading. The report lists each held direction as an axis
mixture (`held at bag value (auto): 0.99y + 0.17yaw`), and `--json` echoes
them under `held`. An axis-aligned held direction is exactly a `--fix
<axis>` you did not have to write. When every direction of the edge is
unobservable, the command fails instead of writing a YAML the data cannot
justify.

**Under `--fix auto`, `degenerate` means held.** The table's axis probe and
the auto-hold decision apply the same test along _different_ directions —
the six raw axes in the table, the six eigen-directions of the curvature
matrix for the holding — and because each direction carries its own sample
noise, a borderline axis can fail its own probe while every eigen-direction
passes. Such an axis is reported `weak`, with its `curv/se` ratio left
visible, rather than `degenerate`: under `auto` the `degenerate` label is
reserved for axes whose content a held direction covers, so the label can
never contradict the held set below the table.

`--fix none` switches all of this off: every free axis is optimized, the
classification is report-only, and a degenerate axis warns that its delta is
unconstrained, recommending a manual `--fix <axis>` re-run.

**What the classification does not tell you.**

- It is a local probe at the found optimum, not a global guarantee. If the
  search settles into a wrong but locally curved basin (a projectively
  near-degenerate scene can do that to any 6-axis run), every direction
  reads `strong` there while the result is simply off. Auto-holding removes
  flat directions; it does not rescue the search from a bad basin.
- It is a screening scale, not a covariance estimate. Treat `degenerate` as
  a reliable "definitely not observable" and `strong` as no more than "not
  obviously unobservable"; for a number you are going to ship, corroborate
  it against a second run over different samples.
- On a real recording the absolute floor is usually inert — every direction
  curves far above it — and the standard-error test decides every verdict on
  its own. `degenerate` then means "the samples do not agree well enough to
  call this measured", not "the cost surface is flat here". Since that
  standard error is itself estimated from `--samples` numbers, a verdict
  sitting near the boundary can flip between runs: read the `curv/se` ratio
  before reading much into the label, and raise `--samples` when a decision
  rides on it.

```text
calib cam-lidar: truck_cabin_base_link -> top_front_narrow/camera_link
axis        bag value  refined value          delta  curv/se  observability
x            0.180000       0.180000       0.000000     1.83  degenerate
y           -0.050000      -0.050000       0.000000     1.90  degenerate
z            1.420000       1.447213       0.027213    12.40  strong
roll         0.000000      -0.008421      -0.008421     9.85  strong
pitch        0.000000       0.014732       0.014732     7.31  strong
yaw          0.000000       0.009481       0.009481     1.72  weak

nid: 0.412887 -> 0.276541
samples used: 8
held at bag value (auto): 1.00x
held at bag value (auto): 1.00y

apply with: bagwiz tf static update -i capture.mcap --yaml capture_calib_cam_lidar.yaml
```

Rotations in the human table are shown in degrees; `--json` reports the same
`before`/`after`/`delta` per axis in radians instead, alongside the `parent`,
`child`, `nid_before`, `nid_after`, `samples`, `cam_offset_ns` (the applied
`--cam-offset` in nanoseconds, `0` when omitted), and `held` fields. Each axis
also carries its curvature evidence: `curvature` and `std_error` as measured,
`curvature_ratio` as their quotient — `null` for an axis that was never
probed (a `--fix`-named one), the ratio additionally `null` when the estimate
has no spread.

### Failures

Beyond the codes in [Exit status](#exit-status) below, exit `1` covers: an
invalid flag combination (`--samples` under 3, `--fix` naming an unknown token,
combining `none` with other tokens, or naming all six axes, a non-positive `--max-trans`/`--max-rot`/`--min-depth`, `--nid-bins`
outside `4`–`256`, `--max-depth` at or below `--min-depth`, an empty
`--of`/`--ref`, or a negative `--keyframe-dist`/`--keyframe-rot`, or an
unparseable or negative `--skip-start`/`--skip-end`, or an unparseable
`--cam-offset`), a missing or
wrong-typed `--pcd`/`--pose`/`--cam` topic, a `--pcd` topic without an
`intensity` field, an unparseable or big-endian cloud, a `--pcd`/`--cam`
topic with no messages, every cloud falling outside the trajectory span, a
pose topic yielding fewer than two poses, skips that cover the whole bag or
leave fewer than two trajectory poses inside the window, an unresolvable
CameraInfo, a cloud frame with no static-TF path from `--of`, a
`--parent`/`--child` edge that is not on the static chain (or not directly on
a static topic), too few usable image samples surviving the trajectory-span
and pre-cull filtering, or a refinement failure (e.g. no sample projects
enough map points at the initial estimate, or no direction of the edge is
observable at all).

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
