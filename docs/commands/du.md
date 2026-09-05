# `bagwiz du`

Report each topic's on-disk size in a single ROS 2 rosbag, in the spirit of
du(1): one row per topic sorted by size descending, each row carrying that
size's share of the reported total, plus a closing `total` row. Sizes print
in 1024-based human-readable units by default (`-b` for raw byte counts).
ROS 1 `*.bag` inputs are not supported.

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

- `SIZE` is what the topic's messages occupy on disk, so compressing a bag
  shrinks its report the way it shrinks the file. See "What is counted" for
  exactly which bytes each storage format charges to a topic, and where the
  figure is a proportional estimate rather than an exact count.
- `%` is the row's share of the reported `total`, to one decimal. The
  denominator is the total actually reported, so `-t/--topics` narrows it
  too and the selected topics still add up to `100.0%`. Rounding is per row,
  so the column need not sum to exactly `100.0%`. A selection that reports
  nothing has no total to divide by: every share then reads `0.0%`, the
  `total` row included.
- Topics declared in the bag but carrying no messages are listed with `0`.
- Column widths are computed from the actual data, so long sizes / topic
  names do not push later columns out of alignment.

## What is counted

A topic is charged the bytes its own messages occupy in the bag's files.
The parts of a file that belong to no topic in particular — file headers,
schema and channel declarations, the MCAP summary section, SQLite3 page and
row overhead, `metadata.yaml` — are charged to none, so the `total` row
falls a little short of the file size. Exactly what is charged depends on
the storage format:

- MCAP: each message record in full (its 31-byte framing plus the payload)
  and the topic's own message index records. In a compressed chunk the
  records of several topics are compressed together, so no exact per-topic
  byte count exists on disk: the chunk's compressed bytes are split among
  its topics in proportion to their uncompressed record bytes, to the
  nearest byte. That split assumes every topic in the chunk compressed
  equally well, so a topic of already-compressed data (JPEG images, say)
  sharing chunks with compressible neighbours is credited some of their
  shrinkage. An uncompressed chunk is charged exactly. A bag written with
  rosbag2's `compression_mode: MESSAGE` over MCAP storage is charged its
  compressed records as they are.
- SQLite3: each row's `data` BLOB as stored — the plain payload, or the zstd
  frame a `compression_mode: MESSAGE` bag keeps in its place. A
  `compression_mode: FILE` bag wraps the whole database in one zstd
  envelope, so each topic's BLOB bytes are scaled by the envelope's
  compression ratio (envelope bytes over decompressed database bytes), to
  the nearest byte.

One approximation is deliberate on MCAP. `du` never reads records, only the
indexes, and a record's size is the gap to the next record in its chunk.
libmcap-based writers (rosbag2 included) emit a channel's schema and channel
declarations into the chunk that first carries one of its messages, between
other records; those bytes are invisible to the index and are charged to the
topic of the message ahead of them. The error is bounded by the size of the
bag's declarations — kilobytes against gigabytes.

## Performance

`du` does not read message payloads. Neither the MCAP summary nor
`metadata.yaml` records per-topic byte totals — there is no summary shortcut
like `ls -l`'s counts — but both storage formats carry enough framing to
answer without reading the bytes it describes:

- SQLite3: `LENGTH(data)` is answered from the row header, so the payload's
  overflow pages stay unread. Disjoint rowid ranges are scanned in parallel
  (`BAGWIZ_READ_THREADS` workers, 8 by default).
- MCAP: the chunk index says what every chunk compressed from and to, and
  the message indexes say where every message record starts inside it.
  Those indexes are all that is read — well under 0.1 % of a bag — and no
  chunk is decompressed, so a compressed bag costs the same as an
  uncompressed one.

Measured cold-cache on one NVMe host, against a 60 s capture of 929 k
messages over 805 topics stored as a 20.6 GB MCAP with uncompressed chunks
and as its 11.0 GB zstd-compressed twin: reading only the indexes answers
the uncompressed bag in 2.6-2.9s where reading each record's length prefix
took 8.7-11.1s, and the compressed bag in 3.7s where decompressing every
chunk to reach those prefixes took 33.8s. Warm, both answer in about 0.9s,
most of it spent opening the bag. A 12.9 GB `.db3` reads its row headers in
2.4s cold-cache against 20.5s for a full scan.

One bag shape cannot be answered this way and falls back to a full message
scan: an MCAP carrying no chunk index — written with chunking off, or never
finalized. The scan charges each message its payload bytes.

Passing `-t/--topics` narrows the work further. The selection is pushed down
into the storage layer, and on MCAP a chunk that holds none of the selected
topics is not read at all.

## Exit status

| Code | Meaning                                   |
| ---- | ----------------------------------------- |
| `0`  | Success (including a bag with no topics). |
| `1`  | Failed — check stderr for the cause.      |
