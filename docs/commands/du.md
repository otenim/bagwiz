# `bagwiz du`

Report each topic's total serialized payload size in a single ROS 2 rosbag,
in the spirit of du(1): one row per topic sorted by size descending, each
row carrying that size's share of the reported total, plus a closing `total`
row. Sizes print in 1024-based human-readable units by default (`-b` for raw
byte counts). ROS 1 `*.bag` inputs are not supported.

## Usage

```text
bagwiz du -i <input> [OPTIONS]
```

## Examples

```bash
# Every topic, human-readable sizes, largest first.
bagwiz du -i capture.mcap

# Raw byte counts, like `du -b`.
bagwiz du -i capture.mcap -b

# Restrict the report to selected topics (globs quoted against the shell).
bagwiz du -i capture.mcap -t '/sensing/*' /tf_static

# Aggregate by topic-name depth, like `du --max-depth`: group under the
# first name component (/sensing, /perception, ...).
bagwiz du -i capture.mcap -d 1
```

## Options

| Flag                        | Description                                                                                                                                                                                                                                                                                           |
| --------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`     | **Required.** ROS 2 rosbag path: a rosbag2 directory or a single-file `*.mcap` / `*.db3`. zstd-compressed `*.db3.zstd` inputs are also accepted.                                                                                                                                                      |
| `-t`, `--topics <topic>...` | Topic selector(s) to report: a literal topic name or a `*` glob. Repeat for several. Omit to report every topic in the bag. A selector that matches no topic is an error. Selecting fewer topics also narrows the work (see Performance).                                                             |
| `-b`, `--bytes`             | Print sizes as raw byte counts instead of the default human-readable units (1024-based, one decimal and a `K`/`M`/`G`/`T` suffix, e.g. `4.0K`, `1.2M`; values below 1 KiB stay raw bytes).                                                                                                            |
| `-d`, `--depth <n>`         | Aggregate topics by their first `<n>` name components, du(1) `--max-depth` style: `-d 1` groups `/sensing/lidar/points` under `/sensing`. A topic already at or above the depth keeps its full name. `-d 0` prints only the `total` row. Combines with `-t`: grouping applies to the selected topics. |

## Output

A table written to `stdout`, sorted by size descending (ties broken by topic
name), with a `total` row last:

```text
  SIZE      % TOPIC
768.0M  88.5% /sensing/lidar
100.0M  11.5% /sensing/camera
   512   0.0% /tf_static
868.0M 100.0% total
```

- `SIZE` is the sum of the topic's uncompressed serialized payload bytes —
  the logical message size, not the on-disk footprint. Per-topic chunk
  compression makes the latter unrecoverable, so a compressed bag's reported
  total can exceed its file size.
- `%` is the row's share of the reported `total`, to one decimal. The
  denominator is the total actually reported, so `-t/--topics` narrows it
  too and the selected topics still add up to `100.0%`. Rounding is per row,
  so the column need not sum to exactly `100.0%`. A selection that reports
  nothing has no total to divide by: every share then reads `0.0%`, the
  `total` row included.
- Topics declared in the bag but carrying no messages are listed with `0`.
- Column widths are computed from the actual data, so long sizes / topic
  names do not push later columns out of alignment.

## Performance

`du` does not read message payloads. Neither the MCAP summary nor
`metadata.yaml` records per-topic byte totals — there is no summary shortcut
like `ls -l`'s counts — but both storage formats record every message's own
length, and reading only those lengths is far cheaper than reading the bytes
they describe:

- SQLite3: `LENGTH(data)` is answered from the row header, so the payload's
  overflow pages stay unread. Disjoint rowid ranges are scanned in parallel
  (`BAGWIZ_READ_THREADS` workers, 8 by default).
- MCAP: the chunk and message indexes already say where every message record
  starts, so only each record's own length prefix is read. A chunk stored
  uncompressed is addressed in place, so those few bytes per message are all
  that is read of it; a compressed chunk still has to be read and
  decompressed to reach its record headers, so a fully compressed bag stays
  closer to the cost of a full read.

So the shape of the bag decides the size of the win. Measured cold-cache on
one NVMe host: a 12.3 GB uncompressed-chunk MCAP went from 13.1s to 0.47s, a
12.9 GB `.db3` from 20.5s to 2.4s, and an 8.0 GB MCAP whose chunks are mostly
zstd from 11.6s to 5.2s.

Two bag shapes cannot be answered this way and fall back to a full message
scan, with identical output: bags recorded with `compression_mode: MESSAGE`
(where the stored length is the compressed one) and MCAPs written without a
chunk index (unchunked, or never finalized).

Passing `-t/--topics` narrows the work further. The selection is pushed down
into the storage layer, and on MCAP a chunk that holds none of the selected
topics is not read at all.

## Exit status

| Code | Meaning                                   |
| ---- | ----------------------------------------- |
| `0`  | Success (including a bag with no topics). |
| `1`  | Failed — check stderr for the cause.      |
