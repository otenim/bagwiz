# `bagwiz ls`

List the topics contained in a single ROS 2 rosbag, with per-topic message
counts and average frequencies. ROS 1 `*.bag` inputs are not supported —
convert them first with [`bagwiz convert 1to2`](convert.md#bagwiz-convert-1to2).

## Usage

```text
bagwiz ls [OPTIONS] <input>
```

## Positional arguments

| Name    | Description                                                                 |
| ------- | --------------------------------------------------------------------------- |
| `input` | ROS 2 rosbag path: a rosbag2 directory or a single-file `*.mcap` / `*.db3`. |

## Options

| Flag                  | Default  | Description                                                                                                                                                                                                                                  |
| --------------------- | -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-p`, `--pattern <P>` | _(none)_ | Keep only rows whose key field matches `<P>`. Plain text is matched as a substring. If the pattern contains `*` or `?` it is matched as a shell-style glob anchored at both ends (`*` spans any characters, `?` matches a single character). |
| `-k`, `--key <FIELD>` | `topic`  | Field the pattern matches against. One of `topic` or `type`.                                                                                                                                                                                 |

## Output

A four-column table written to `stdout`, sorted by topic name:

```text
TOPIC    TYPE    COUNT    HZ
```

- `COUNT` is the total number of messages on that topic in the bag.
- `HZ` is the average publish rate, computed as
  `(count - 1) / (last_stamp - first_stamp)` over the entire bag's
  message-time range. Topics with `count <= 1` or a zero-duration bag
  print `0.00`.
- Column widths are computed from the actual data, so long topic /
  type names do not push later columns out of alignment.

## Examples

```bash
# List every topic in a directory-layout rosbag2.
bagwiz ls path/to/rosbag2_2025_01_01-12_00_00/

# Single-file MCAP.
bagwiz ls capture.mcap

# Only sensor topics.
bagwiz ls capture.mcap -p '/sensors/*'

# Only PointCloud2 streams, anywhere.
bagwiz ls capture.mcap -k type -p 'sensor_msgs/msg/PointCloud2'

# Substring match (no glob characters).
bagwiz ls capture.mcap -p lidar
```

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success (including empty match set). |
| `1`  | Failed to open `<input>`.            |
