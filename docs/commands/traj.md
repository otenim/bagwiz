# `bagwiz traj`

Trajectory-shaped operations on a ROS 2 rosbag. Currently ships a single
subcommand:

| Subcommand                  | Purpose                                                         |
| --------------------------- | --------------------------------------------------------------- |
| [`dump`](#bagwiz-traj-dump) | Dump a sampled trajectory to a TUM file from a supported topic. |

ROS 1 `*.bag` inputs are not supported — convert them first with
[`bagwiz convert 1to2`](convert.md#bagwiz-convert-1to2).

---

## `bagwiz traj dump`

Samples poses from one bag topic and writes TUM. Supported message types:

| Message type                                  | `--from` | `--to`         |
| --------------------------------------------- | -------- | -------------- |
| `tf2_msgs/msg/TFMessage`                      | Required | Required       |
| `geometry_msgs/msg/PoseStamped`               | Optional | Ignored if set |
| `geometry_msgs/msg/PoseWithCovarianceStamped` | Optional | Ignored if set |

For TF topics, output is the trajectory of frame `--to` expressed in frame
`--from`, sampled at TF updates on the chain between them that arrive on the
input topic (same behavior as before).

For `PoseStamped` and `PoseWithCovarianceStamped`, each row uses that
message’s pose. With no `--from`, values are written as they appear in the bag
(the implicit reference frame is each sample’s `header.frame_id`). With
`--from <FRAME>`, each pose is transformed from `header.frame_id` into
`<FRAME>` using all `tf2_msgs/msg/TFMessage` topics in the bag (including
topics whose name ends with `tf_static`). Covariance is not written to TUM.

If `--to` is passed for a pose topic, it is ignored and a warning is logged.

### Usage

```text
bagwiz traj dump [OPTIONS] <input> <output> <topic>
```

### Positional arguments

| Name     | Description                                                                                       |
| -------- | ------------------------------------------------------------------------------------------------- |
| `input`  | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`).                                         |
| `output` | Output file path. Will be truncated if it already exists.                                         |
| `topic`  | Topic whose type selects processing (`TFMessage`, `PoseStamped`, or `PoseWithCovarianceStamped`). |

### Options

| Flag                 | Default      | Description                                                                                                                                        |
| -------------------- | ------------ | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--from <FRAME>`     | _(optional)_ | TF topics: required reference frame. Pose topics: optional; omit to keep each sample in `header.frame_id`, or set to remap into this frame via TF. |
| `--to <FRAME>`       | _(optional)_ | TF topics: required tracked frame. Pose topics: ignored (warning if set).                                                                          |
| `-f`, `--format <F>` | `tum`        | Output format. Currently only `tum` is supported.                                                                                                  |

### TF topic: how sampling works

1. The bag is scanned once. Every `tf2_msgs/msg/TFMessage` topic is loaded into
   a single TF buffer; topics whose name ends with `tf_static` are inserted as
   static transforms, the rest as dynamic.
2. The chain `--from → … → --to` is resolved against the buffer (a stable
   topology is assumed; resolution happens once).
3. While reading the input topic, every `TransformStamped` whose
   `(frame_id, child_frame_id)` lies on the chain contributes its
   `header.stamp` to the sample-time set.
4. Sample times are sorted and de-duplicated.
5. For each sample time `t`, `lookupTransform(--from, --to, t)` runs against
   the buffer and the result is written to the output file.

### Pose topics: how sampling works

1. Messages are read from the chosen topic in bag order (one output row per
   message that decodes successfully).
2. If `--from` is set, TF messages from the same bag are merged in timestamp
   order so lookups can resolve before each pose.
3. For each pose, `lookupTransform(--from, header.frame_id, t)` supplies the
   remap when `--from` is set.

### Output: TUM format

A whitespace-separated text file, one pose per line:

```text
timestamp tx ty tz qx qy qz qw
```

`timestamp` is in seconds (with fractional nanoseconds). The file is sorted by
timestamp only when the TF path sorts sample times; pose streams follow bag
message order.

### Examples

```bash
# TF: trajectory of base_link in map, using /tf as the dynamic source.
bagwiz traj dump capture.mcap traj.tum /tf --from map --to base_link

# Pose topic: use poses as stored (reference frame is each header.frame_id).
bagwiz traj dump capture.mcap pose.tum /localization/pose

# Pose topic: express poses in map using TF from the bag.
bagwiz traj dump capture.mcap pose_map.tum /localization/pose --from map
```

### Errors

| Situation                                                                        | Result                                                           |
| -------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| TF topic: `--from` or `--to` missing or empty                                    | Error.                                                           |
| `--from` and `--to` equal (TF topics)                                            | Error.                                                           |
| Pose topic: `--from` set but empty                                               | Error.                                                           |
| Topic absent / unsupported type / static TF topic given as `<topic>` for TF path | Error.                                                           |
| TF path: no path between `--from` and `--to`                                     | Error.                                                           |
| TF path: path exists but no chain edge on `<topic>`                              | Error.                                                           |
| Pose remap: no TF topics in bag                                                  | Error.                                                           |
| Some lookups fail                                                                | Skipped and counted; remaining poses are written if any succeed. |

### Exit status

| Code | Meaning                                               |
| ---- | ----------------------------------------------------- |
| `0`  | At least one pose was written to the output file.     |
| `1`  | Any of the error conditions above, or an I/O failure. |
