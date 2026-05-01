# `bagwiz traj`

Trajectory-shaped operations on a ROS 2 rosbag. Currently ships a single
subcommand:

| Subcommand                  | Purpose                                                         |
| --------------------------- | --------------------------------------------------------------- |
| [`dump`](#bagwiz-traj-dump) | Dump the `--to → --from` trajectory from a TF stream as a file. |

ROS 1 `*.bag` inputs are not supported — convert them first with
[`bagwiz convert 1to2`](convert.md#bagwiz-convert-1to2).

---

## `bagwiz traj dump`

Dump the trajectory of frame `--to` expressed in frame `--from`,
sampled at every TF update on the chain between them that arrives on
the input topic.

The only supported input is `tf2_msgs/msg/TFMessage`. A static
counterpart in the same bag (any topic whose name ends with
`tf_static`) is loaded automatically and used to compose the chain.

### Usage

```text
bagwiz traj dump [OPTIONS] <input> <output> <topic> --from <FRAME> --to <FRAME>
```

### Positional arguments

| Name     | Description                                                                                                                       |
| -------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `input`  | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`).                                                                         |
| `output` | Output file path. Will be truncated if it already exists.                                                                         |
| `topic`  | The dynamic `tf2_msgs/msg/TFMessage` topic (typically `/tf`). The topic determines the sampling cadence of the output trajectory. |

### Options

| Flag                 | Default      | Description                                                    |
| -------------------- | ------------ | -------------------------------------------------------------- |
| `--from <FRAME>`     | _(required)_ | Reference (fixed) frame the output trajectory is expressed in. |
| `--to <FRAME>`       | _(required)_ | Tracked (moving) frame whose pose each sample represents.      |
| `-f`, `--format <F>` | `tum`        | Output format. Currently only `tum` is supported.              |

### How sampling works

1. The bag is scanned once. Every `tf2_msgs/msg/TFMessage` topic is
   loaded into a single TF buffer; topics whose name ends with
   `tf_static` are inserted as static transforms, the rest as dynamic.
2. The chain `--from → … → --to` is resolved against the buffer (a
   stable topology is assumed; resolution happens once).
3. While reading the input topic, every `TransformStamped` whose
   `(frame_id, child_frame_id)` lies on the chain contributes its
   `header.stamp` to the sample-time set.
4. Sample times are sorted and de-duplicated.
5. For each sample time `t`, `lookupTransform(--from, --to, t)` runs
   against the (static + dynamic) TF buffer and the result is written
   to the output file. Lookups that fail (extrapolation, out-of-range)
   are counted in the summary.

### Output: TUM format

A whitespace-separated text file, one pose per line:

```text
timestamp tx ty tz qx qy qz qw
```

`timestamp` is in seconds (with fractional nanoseconds). The file is
sorted by timestamp.

### Examples

```bash
# Trajectory of base_link in map, using /tf as the dynamic source.
bagwiz traj dump capture.mcap traj.tum /tf --from map --to base_link

# Trajectory of an IMU mounted on base_link, in map. The sensor offset
# (base_link → imu_link) typically lives on /tf_static; it is composed
# automatically.
bagwiz traj dump capture.mcap imu.tum /tf --from map --to imu_link
```

### Errors

| Situation                                                                        | Result                                                                     |
| -------------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| `--from` and `--to` are equal                                                    | Error.                                                                     |
| Topic absent / wrong type / static topic given as `<topic>`                      | Error.                                                                     |
| No path between `--from` and `--to` in the TF tree                               | Error.                                                                     |
| Path exists but no chain edge is published on `<topic>` (e.g. fully-static path) | Error: traj dump needs at least one dynamic chain edge on the input topic. |
| Some sample lookups fail (extrapolation, etc.)                                   | Skipped and counted in the summary; remaining poses are written.           |

### Exit status

| Code | Meaning                                               |
| ---- | ----------------------------------------------------- |
| `0`  | At least one pose was written to the output file.     |
| `1`  | Any of the error conditions above, or an I/O failure. |
