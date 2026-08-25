# `bagwiz du`

Report each topic's total serialized payload size in a single ROS 2 rosbag,
in the spirit of du(1): one row per topic sorted by size descending, plus a
closing `total` row. ROS 1 `*.bag` inputs are not supported.

## Usage

```text
bagwiz du -i <input> [OPTIONS]
```

## Examples

```bash
# Every topic, raw byte counts, largest first.
bagwiz du -i capture.mcap

# Human-readable sizes (1024-based), like `du -h`.
bagwiz du -i capture.mcap -h

# Restrict the report to selected topics (globs quoted against the shell).
bagwiz du -i capture.mcap -t '/sensing/*' /tf_static
```

## Options

| Flag                        | Description                                                                                                                                                                                                                                                                                 |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`     | **Required.** ROS 2 rosbag path: a rosbag2 directory or a single-file `*.mcap` / `*.db3`. zstd-compressed `*.db3.zstd` inputs are also accepted.                                                                                                                                            |
| `-t`, `--topics <topic>...` | Topic selector(s) to report: a literal topic name or a `*` glob. Repeat for several. Omit to report every topic in the bag. A selector that matches no topic is an error. Selecting fewer topics also narrows the message scan (see Performance).                                           |
| `-h`, `--human`             | Print sizes in human-readable units (1024-based, one decimal and a `K`/`M`/`G`/`T` suffix, e.g. `4.0K`, `1.2M`; values below 1 KiB stay raw bytes) instead of raw byte counts. Note: unlike every other bagwiz command, `-h` here is NOT help — `du` follows du(1)'s binding; use `--help`. |

## Output

A table written to `stdout`, sorted by size descending (ties broken by topic
name), with a `total` row last:

```text
      SIZE  TOPIC
 805306368  /sensing/lidar
 104857600  /sensing/camera
       512  /tf_static
 909455360  total
```

- `SIZE` is the sum of the topic's uncompressed serialized payload bytes —
  the logical message size, not the on-disk footprint. Per-topic chunk
  compression makes the latter unrecoverable, so a compressed bag's reported
  total can exceed its file size.
- Topics declared in the bag but carrying no messages are listed with `0`.
- Column widths are computed from the actual data, so long sizes / topic
  names do not push later columns out of alignment.

## Performance

Computing the sizes requires a full scan of the bag's messages, on every
storage format: neither the MCAP summary nor `metadata.yaml` records
per-topic byte totals, so there is no summary shortcut like `ls -l`'s counts.
On a large bag, expect a runtime comparable to `bagwiz convert`. Passing
`-t/--topics` pushes the selection down into the storage layer, so a narrow
selection scans only the matching messages.

## Exit status

| Code | Meaning                                   |
| ---- | ----------------------------------------- |
| `0`  | Success (including a bag with no topics). |
| `1`  | Failed — check stderr for the cause.      |
