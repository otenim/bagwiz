# `bagwiz traj`

Trajectory-shaped operations on a rosbag. Currently ships a single
subcommand:

| Subcommand                      | Purpose                                                  |
| ------------------------------- | -------------------------------------------------------- |
| [`export`](#bagwiz-traj-export) | Extract a topic's pose trajectory and save it to a file. |

---

## `bagwiz traj export`

Extract a pose-bearing topic from a bag and write the resulting trajectory
to a file in a tool-friendly format.

### Usage

```text
bagwiz traj export [OPTIONS] <input> <output> <topic>
```

### Positional arguments

| Name     | Description                                               |
| -------- | --------------------------------------------------------- |
| `input`  | Bag path (rosbag2 directory, `*.mcap`, `*.db3`, `*.bag`). |
| `output` | Output file path. Will be truncated if it already exists. |
| `topic`  | Topic name to extract poses from.                         |

### Options

| Flag                 | Default                     | Description                                                                                                                                                                                                       |
| -------------------- | --------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-f`, `--format <F>` | `tum`                       | Output format. Currently only `tum` (TUM trajectory format) is supported.                                                                                                                                         |
| `--from <FRAME>`     | _topic's `header.frame_id`_ | Reference (fixed) frame the output trajectory is expressed in. Setting this to a non-default frame requires TF in the bag.                                                                                        |
| `--to <FRAME>`       | _topic's `child_frame_id`_  | Tracked (moving) frame whose pose each sample represents. Only valid for types that carry a `child_frame_id`.                                                                                                     |
| `--edge <SRC:DST>`   | _(none)_                    | TF edge to extract from a `tf2_msgs/msg/TFMessage` input, in `SRC:DST` form. Required for `TFMessage`; rejected for other types. After filtering, `--from` / `--to` apply uniformly (same as `TransformStamped`). |

### Supported message types

Stamped (sample timestamp comes from `header.stamp`):

- `geometry_msgs/msg/PoseStamped`
- `geometry_msgs/msg/PoseWithCovarianceStamped`
- `geometry_msgs/msg/TransformStamped`
- `nav_msgs/msg/Odometry`

Unstamped (sample timestamp comes from the bag's log time, i.e. recorder
receive time — a one-shot warning is logged):

- `geometry_msgs/msg/Pose`
- `geometry_msgs/msg/Transform`

Multi-edge (requires `--edge SRC:DST`):

- `tf2_msgs/msg/TFMessage` — the chosen edge becomes the input "topic";
  `--from` / `--to` then apply the same way as for `TransformStamped`.

`--to` is rejected for types that have no `child_frame_id` (i.e.
`PoseStamped`, `PoseWithCovarianceStamped`, `Pose`, `Transform`).
`--from` / `--to` are rejected for the unstamped types since they have
no `header.frame_id` to anchor on.

### Frame composition (`--from` / `--to`)

Each input message produces one or more `(frame_id, child_frame_id, pose,
stamp)` candidates. `--edge` (when applicable) pre-filters them down to
the labeled edge you asked for. After that, the output pose `P` is
composed as:

```text
P  := T_from_source @ message_pose @ T_tracked_to
```

…where the left/right multiplications are skipped when the requested
frame already matches the candidate's frame. In particular, when only
`--edge` is given on a `TFMessage` input (no `--from` / `--to`), no TF
lookup is performed at all.

When `--from` or `--to` differs from the candidate's natural frame, the
TF tree is loaded from the bag:

- All `tf2_msgs/msg/TFMessage` topics are scanned.
- Topics whose name ends in `tf_static` are inserted as static
  transforms; everything else as dynamic transforms.
- Per-message lookups that fall outside the published TF range are
  skipped and counted in the final summary.

### Output: TUM format

A whitespace-separated text file, one pose per line:

```text
timestamp tx ty tz qx qy qz qw
```

`timestamp` is in seconds (with fractional nanoseconds). The file is
sorted by timestamp using a stable sort, so adjacent samples with equal
stamps preserve the bag's read order. Empty trajectories (zero messages
on the topic) are written as an empty file with a warning.

### Examples

```bash
# Stamped pose, defaults.
bagwiz traj export capture.mcap traj.tum /localization/pose

# Odometry expressed in a different reference frame (requires TF in the bag).
bagwiz traj export capture.mcap traj.tum /odom --from map

# Track a different child frame.
bagwiz traj export capture.mcap traj.tum /odom --from map --to base_footprint

# Extract a single edge from /tf, keep its native frames.
bagwiz traj export capture.mcap map_to_base.tum /tf --edge map:base_link

# Extract a /tf edge, then re-anchor to a different reference frame.
bagwiz traj export capture.mcap traj.tum /tf --edge map:base_link --from world
```

### Exit status

| Code | Meaning                                                                                                                                                                                                                                                                       |
| ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | At least one pose was written, or the topic was empty (in which case an empty file is produced and a warning is logged).                                                                                                                                                      |
| `1`  | Bag could not be opened, the topic is absent, the type is unsupported, the flag combination is invalid, the decoder failed, the TF buffer could not be loaded, all messages were skipped because none composed to (`--from`, `--to`), or the output file could not be opened. |
