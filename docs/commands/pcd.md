# `bagwiz pcd`

PointCloud2 topic processing.

| Subcommand                           | What it does                                                              |
| ------------------------------------ | ------------------------------------------------------------------------- |
| [`concat`](#bagwiz-pcd-concat)       | Merge multiple PointCloud2 topics into one new topic.                     |
| [`undistort`](#bagwiz-pcd-undistort) | Motion-deskew one or more PointCloud2 topics from an external pose topic. |

---

## `bagwiz pcd concat`

Merge multiple `sensor_msgs/msg/PointCloud2` topics into one new topic. Each
input topic is rigidly transformed into a common `--frame` using the bag's
static TF, and messages are paired against the first `--pcd` topic — the
reference — nearest-in-time within a tolerance. The result is written as a new
rosbag with `-o`, or the input bag is rewritten in place when `-o` is omitted.

This is the offline (rosbag) counterpart of Autoware's point-cloud concatenate
node. It does **not** apply motion compensation — points from different sensor
sweeps are placed by the static extrinsic only. Run `pcd undistort` beforehand
if you need per-cloud deskew; per-point timestamps are preserved so a downstream
undistort still sees correct absolute times.

### Usage

```text
bagwiz pcd concat -i <input> -t <output_topic> --pcd <topic>... [OPTIONS]
```

### Examples

```bash
# Concatenate four Seyond LiDARs (front/rear/left/right) into base_link,
# with the left/right sensors triggering ~50 ms early.
bagwiz pcd concat -i drive.mcap -t /sensing/lidar/concatenated/points \
  --frame base_link \
  --pcd /sensing/lidar/front/seyond_points /sensing/lidar/rear/seyond_points \
        /sensing/lidar/left/seyond_points  /sensing/lidar/right/seyond_points \
  --stamp-offset /sensing/lidar/left/seyond_points=50ms \
                 /sensing/lidar/right/seyond_points=50ms \
  -o concatenated.mcap
```

### Options

| Flag                           | Description                                                                                                                                                                                                                                                                                                                    |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `-i`, `--input <input>`        | **Required.** Input bag (file or directory).                                                                                                                                                                                                                                                                                   |
| `-t`, `--topic <output_topic>` | **Required.** Name of the new concatenated PointCloud2 topic.                                                                                                                                                                                                                                                                  |
| `--pcd <t...>`                 | **Required.** PointCloud2 topics to concatenate (2 or more). The first topic is the reference; concatenation order follows this list.                                                                                                                                                                                          |
| `--frame <frame>`              | Target frame all clouds are transformed into. Default: `base_link`. Required when the default is not reachable from every `--pcd` frame via the bag's static TF.                                                                                                                                                               |
| `-o`, `--output <path>`        | Output bag. When omitted, the input bag is rewritten in place (atomic tmp swap).                                                                                                                                                                                                                                               |
| `--tolerance <val>`            | Nearest-match tolerance for pairing the other topics to the first `--pcd` topic. Takes an optional unit `ns`/`us`/`ms`/`s` (no unit = ms), e.g. `50ms`. Default: half the first topic's median header-stamp period, or 50 ms when that period cannot be measured (fewer than two reference messages, or a zero median period). |
| `--stamp-offset <topic>=<val>` | Per-topic offset **added to `header.stamp` for matching only** (the real stamp and per-point times are never rewritten). `<val>` takes an optional unit `ns`/`us`/`ms`/`s` (no unit = ms), signed, e.g. `=-50ms`, `=500ns`. Repeatable. Use it when a sensor triggers early/late relative to the first topic.                  |
| `--drop-inputs`                | Drop the source `--pcd` topics from the output. Default: keep them.                                                                                                                                                                                                                                                            |
| `--force`                      | Proceed even if `<output_topic>` already exists in the bag (replaces that topic).                                                                                                                                                                                                                                              |
| `-w`, `--overwrite`            | Overwrite an existing `-o/--output` path.                                                                                                                                                                                                                                                                                      |
| `-j`, `--threads <N>`          | Number of worker threads. Default: `8`. Accepts `0`–`256`; `0` uses `std::thread::hardware_concurrency()`; `1` forces the synchronous path; in-range values above hardware concurrency are capped to it.                                                                                                                       |

### Field layout

All `--pcd` topics must share an identical PointField layout (fields and
`point_step`); a mismatch is an error.

### Missing sensors (partial emit)

If a topic has no message within tolerance for a given reference message, it is
skipped for that output message and the remaining topics are still
concatenated.

### Per-point time preservation

Each point's original absolute acquisition time is preserved. A header-relative
FLOAT time field is re-based in place by `(source_header_stamp - output_stamp)`;
a header-relative UINT32 time field is emitted as FLOAT32 seconds (same 4-byte
width) and re-based (its value may go negative when an input is earlier than the
output stamp); an absolute-encoded field is copied verbatim.

### Output

The output header's `frame_id` is `--frame` and its `stamp` is the reference
message's real stamp. The merged cloud is unorganized (`height = 1`).

The output inherits the input's format/layout, but MCAP output is written
**uncompressed** (`compression: none`), unlike `pcd undistort`, which keeps the
zstd default. Expect a larger output file than the input.

### Determinism

CPU-only, no GLIM — deterministic output for a given input. When `--threads` is
greater than 1, group assembly runs in parallel but a single collector thread
serializes output, so bag message order — and the output itself — is identical
to the synchronous path. Message order is outside the numeric tolerance
contract in AGENTS.md "Numerical Reproducibility": it is held strictly, at any
thread count.

---

## `bagwiz pcd undistort`

Motion-deskew (undistort) one or more `sensor_msgs/msg/PointCloud2` topics
using an external pose or twist (vehicle-velocity) topic as the motion source.
`pcd undistort` never estimates motion itself — no SLAM, no scan matching — it
only reads a pose/trajectory/velocity topic that is already in the bag (or
joined into it beforehand, e.g. with `traj join`) and moves every point back
to one reference time per scan. Given the same input, it always produces the
same output.

This is the per-cloud counterpart to `pcd concat` above, which does not
compensate for motion at all. Since deskew rewrites only xyz and per-point
time, running `undistort` before `concat` still leaves per-point timestamps
intact for the downstream merge.

### Usage

```text
bagwiz pcd undistort -i <input> (--pose <pose_topic> | --twist <twist_topic>) --pcd <topic>... [OPTIONS]
```

### Examples

To deskew against SLAM-derived poses rather than an existing localization
topic, compose it from `map slam` and `traj join`: generate a trajectory,
embed it into the bag as a topic, then point `pcd undistort` at that topic
(see the third example below).

```bash
# Deskew one topic in place, using an existing localization pose (Odometry).
bagwiz pcd undistort -i drive.mcap --pose /localization/kinematic_state \
  --pcd /sensing/lidar/top/pointcloud

# Deskew multiple topics against the same pose, into a new bag.
bagwiz pcd undistort -i drive.mcap --pose /localization/kinematic_state \
  --pcd /sensing/lidar/top/pointcloud /sensing/lidar/left/pointcloud \
  -o undistorted.mcap

# Deskew from a vehicle-velocity topic (TwistWithCovarianceStamped) instead of
# poses: the velocity is dead-reckoned into the relative motion.
bagwiz pcd undistort -i drive.mcap --twist /vehicle/status/velocity_status \
  --pcd /sensing/lidar/top/pointcloud

# Composition workflow: derive a trajectory with SLAM, embed it as a topic,
# then deskew against it.
bagwiz map slam -i drive.mcap --pcd /points -o out/                 # -> out/traj.tum
bagwiz traj join -i drive.mcap --traj out/traj.tum -t /slam/tf --ref map --of base_link
bagwiz pcd undistort -i drive.mcap --pose /slam/tf --pcd /points -o undistorted.mcap
```

### Options

| Flag                          | Description                                                                                                                                                                                                                                                                                                          |
| ----------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`       | **Required.** Input bag (file or directory).                                                                                                                                                                                                                                                                         |
| `--pose <pose_topic>`         | Self-position source topic already in the bag. Type must be one of `tf2_msgs/msg/TFMessage`, `nav_msgs/msg/Odometry`, `geometry_msgs/msg/PoseStamped`, `geometry_msgs/msg/PoseWithCovarianceStamped`. Exactly one of `--pose` / `--twist` is required.                                                               |
| `--twist <twist_topic>`       | Vehicle-velocity source topic already in the bag, integrated (dead-reckoned) into the deskew motion. Type must be one of `geometry_msgs/msg/Twist`, `geometry_msgs/msg/TwistStamped`, `geometry_msgs/msg/TwistWithCovarianceStamped`. Exactly one of `--pose` / `--twist` is required.                               |
| `--pcd <topic>`               | **Required.** PointCloud2 topic(s) to deskew. Variadic and repeatable — `--pcd /a /b` and `--pcd /a --pcd /b` are equivalent. At least one is required.                                                                                                                                                              |
| `--ref <frame>`               | Reference frame the trajectory is resolved in (same convention as `traj dump`). Default: `map`. Has no effect with `--twist` (a velocity source carries only relative motion).                                                                                                                                       |
| `--of <frame>`                | Tracked body frame. The trajectory is obtained as `T_ref_of` (e.g. `T_map_base_link`). Default: `base_link`.                                                                                                                                                                                                         |
| `-o`, `--output <path>`       | Output bag. When omitted, `<input>` is rewritten in place (atomic tmp swap).                                                                                                                                                                                                                                         |
| `-w`, `--overwrite`           | Replace `-o/--output` if it already exists. Has no effect in in-place mode.                                                                                                                                                                                                                                          |
| `-j`, `--threads <N>`         | Number of worker threads for Pass 2. Default: `8`. Accepts `0`–`256`; `0` uses `std::thread::hardware_concurrency()`; `1` forces the synchronous path; in-range values above hardware concurrency are capped to it.                                                                                                  |
| `--no-extrap`                 | Do not extrapolate the motion trajectory beyond its samples. Clouds outside the trajectory's time span are deskewed against the clamped endpoint poses (with a warning), instead of being covered by extrapolation. Mutually exclusive with `--max-extrap-duration`.                                                 |
| `--max-extrap-duration <val>` | Per-side cap on the trajectory extrapolation. Default: `1s`; `0` is equivalent to `--no-extrap`. Takes an optional unit `ns`/`us`/`ms`/`s` (no unit = ms), e.g. `500ms`. If covering the `--pcd` topics' first clouds needs more extrapolation than this on either side, the run errors out before writing anything. |
| `--compression <codec>`       | MCAP chunk codec for the output: `zstd` (the default), `lz4`, or `none`. Compression is roughly half the command's CPU on typical LiDAR bags, so `lz4` (several times cheaper to encode, larger output) or `none` (no encode cost, largest output) trade output size for speed. Errors on non-mcap outputs.          |
| `--compression-level <level>` | Encoder effort for the chosen (or default) codec: `fastest`, `fast`, `default`, `slow`, or `slowest`. When unset, zstd uses `default` and lz4 uses `fastest` — mcap maps lz4's `default` onto LZ4-HC, which measures several times slower than zstd for a larger output. Not combinable with `--compression none`.   |

### Trajectory resolution

Pass 1 resolves the trajectory. `<input>` must have a `...tf_static` topic —
it is loaded together with the motion-source topic to resolve `--ref` → `--of`
(with `--pose`) or the twist frame (with `--twist`). Only the motion-source
topic and the bag's static TF feed the trajectory; no other topic
(e.g. a bag's own dynamic `/tf`) is read automatically. With `--pose`, the
composition
mirrors `traj dump`'s: for `TFMessage`, the `--ref` → `--of` chain is
resolved against tf_static plus the edges carried on `<pose_topic>` itself,
then sampled at every stamp published on that chain; for `Odometry` /
`PoseStamped` / `PoseWithCovarianceStamped`, each message's own pose is
bridged into `--ref`/`--of` via the bag's static TF when its frames don't
already match (`T_ref_of = T_ref_header · T_header_body · T_body_of`). One
difference from `traj dump`: an unresolvable bridge is fatal here, where
`traj dump` would just skip that one sample. An unresolvable `--ref` → `--of`
overall is likewise fatal — checked before anything is written.

With `--twist`, Pass 1 instead dead-reckons the velocity samples into a
relative trajectory: starting from identity at the first sample, each interval
between consecutive samples is advanced with a zero-order-hold constant-twist
model (exact axis-angle rotation and SO(3) left-Jacobian translation under the
sample's linear/angular velocity). Deskew only consumes the relative motion
`T(t_ref)⁻¹·T(t_i)`, so the arbitrary integration origin cancels and `--ref`
plays no role. For the stamped types (`TwistStamped`,
`TwistWithCovarianceStamped`; the covariance is ignored) the samples are
stamped with `header.stamp`, and a `header.frame_id` that is neither empty nor
`--of` is rotated into `--of` via the bag's static TF (an unresolvable chain
is fatal). A bare `Twist` has no header: samples are stamped with the bag's
message reception (log) time — so accuracy additionally depends on the
publisher→recorder latency being small — and the velocity is assumed to be
expressed in the `--of` frame. Either way, accuracy follows the twist topic's
rate and quality: where `--pose` deskews against measured poses, `--twist`
deskews against integrated velocities, and any velocity bias accumulates over
the scan duration.

Whichever source built it, the raw trajectory spans only the motion topic's
first-to-last sample, and a cloud (or point) outside that span would otherwise
be deskewed against the clamped endpoint poses — for a cloud wholly outside,
that means no correction at all. To avoid that, Pass 1 by default extends the
trajectory by constant-velocity extrapolation: it continues the endpoint
velocity (linear translation, SLERP-extrapolated rotation of the first/last
interval) backwards to the earliest first-cloud timestamp and forwards by one
sweep (the longest first-cloud sweep over the `--pcd` topics) past the last
sample, so leading/trailing clouds are deskewed against plausible motion
instead of a frozen pose. The extension assumes the endpoint velocity holds
over the extrapolated interval, so it is only trustworthy near the endpoint:
if covering the first clouds this way needs more than `--max-extrap-duration`
(default `1s`) on either side, the run errors out before anything is written —
raise the cap to extrapolate that far, or pass `--no-extrap` to turn the
extension off entirely. Clouds that still fall outside the extended span
during the rewrite (e.g. past a longer tail gap than one sweep, or a mid-bag
motion-source dropout) are not an error: they are deskewed against the clamped
endpoint poses with an out-of-span warning.

### Per-topic extrinsics and time fields

For every `--pcd` topic, the sensor extrinsic `E = T_of_C` (`C` = that topic's
cloud `frame_id`) is resolved from the same frame sources as `--ref` → `--of`:
the bag's `*/tf_static`, plus the motion-source topic itself when it is a
`TFMessage` topic (identity when `C == --of`). For a statically-mounted
sensor — the
normal case — this extrinsic comes from `tf_static` alone. A missing chain is
fatal. Each topic's first message must also already carry a per-point time
field (checked by name, count, and datatype: one of `t`, `time`, `time_stamp`,
or `timestamp` with `count == 1` and a `UINT32`, `FLOAT32`, or `FLOAT64`
datatype, sized to fit within `point_step`); a topic without one is fatal,
since undistort cannot deskew without per-point time.

### Deskew rewrite

In Pass 2, every message is copied through unchanged except `--pcd` topics.
For each cloud on a `--pcd` topic:

- per-point time is normalized to an absolute timestamp per point
  (relative-to-header vs. absolute is auto-detected);
- each point's xyz is moved from its own timestamp's pose to the pose at
  `t_ref = header.stamp` via
  `p' = E⁻¹·(T_ref_of(t_ref)⁻¹·T_ref_of(t_i))·E·p`, interpolating the
  trajectory (SLERP + lerp). Points or reference stamps that still fall
  outside the (by default extrapolated) trajectory's time span are deskewed
  against the nearest endpoint pose and reported with an out-of-span
  warning;
- the per-point time field is rewritten to the `t_ref`-equivalent value
  (`0` for a relative field, `header.stamp` for an absolute one), so a
  later `undistort` or `concat` run can't double-deskew the cloud;
- non-finite (NaN/Inf) points are left byte-for-byte unchanged;
- every other field, the point count, `point_step`/`row_step` (organized
  clouds included), and `frame_id` are unchanged — points never leave
  their own cloud's frame;
- `FLOAT32` and `FLOAT64` xyz are both supported (computed internally as
  `double`). A malformed cloud — big-endian, missing/misshapen x/y/z, or an
  inconsistent point/row layout — aborts the whole run rather than being
  skipped; a cloud that merely fails to parse as `PointCloud2` is instead
  copied through unchanged with a warning.
- a cloud that reaches Pass 2 with no usable per-point time field is **not**
  an error — it is written through un-deskewed and reported with a
  `had nothing deskewed … passed through un-deskewed` warning (the upfront
  fatal check only inspects each topic's first message). A reference stamp
  outside the trajectory's time span is deskewed against the nearest
  endpoint pose, just like out-of-span points above.
- The trajectory is built once and shared by every `--pcd` topic; only the
  extrinsic `E` changes per topic, so sensors with different mount points
  can be deskewed together in one run.

### Output

`-o` writes a new bag inheriting `<input>`'s storage format; omitting it
rewrites `<input>` in place through a tmp file and an atomic swap, so a
mid-pass failure leaves the original bag untouched.

### Determinism

No SLAM is involved, and the same input always produces the same output. When
`--threads` is greater than 1, deskew work runs in parallel but a single
collector thread serializes output, so bag message order is preserved. Message
order is outside the numeric tolerance contract in AGENTS.md "Numerical
Reproducibility": it is held strictly, at any thread count.

### Errors

| Situation                                                                                                                 | Result                                                                                                    |
| ------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| No `--pcd` given                                                                                                          | Error.                                                                                                    |
| Neither `--pose` nor `--twist` given, or both                                                                             | Error.                                                                                                    |
| The motion-source topic is absent from `<input>`, or not one of its supported types                                       | Error.                                                                                                    |
| A `--pcd` topic absent from `<input>`, or not `PointCloud2`                                                               | Error.                                                                                                    |
| `<input>` has no `...tf_static` topic                                                                                     | Fatal — needed to resolve `--ref` → `--of` and every `--pcd` topic's extrinsic.                           |
| `--ref` → `--of` cannot be resolved from `pose_topic` + the bag's static TF                                               | Fatal.                                                                                                    |
| A `--twist` topic's `header.frame_id` has no static TF chain to `--of`                                                    | Fatal.                                                                                                    |
| A `--pcd` topic's first message has no per-point time field                                                               | Fatal.                                                                                                    |
| A later `--pcd` cloud has no usable per-point time field                                                                  | Warning; cloud passed through un-deskewed.                                                                |
| A point's or reference stamp's time falls outside the (possibly extrapolated) trajectory's time span                      | Warning; deskewed against the clamped endpoint pose.                                                      |
| Covering the `--pcd` clouds would need more than `--max-extrap-duration` (default `1s`) of trajectory extrapolation       | Error before anything is written; raise the cap or pass `--no-extrap`.                                    |
| `--of` → a `--pcd` topic's cloud frame is not reachable via `*/tf_static` + `<pose_topic>`                                | Fatal.                                                                                                    |
| A cloud reaching the rewrite step is malformed (big-endian, missing/misshapen x/y/z, or an inconsistent point/row layout) | Aborts the run (a cloud that merely fails to _parse_ is copied through unchanged with a warning instead). |
| `-o` output path already exists without `-w`/`--overwrite`                                                                | Error.                                                                                                    |

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
