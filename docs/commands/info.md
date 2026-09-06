# `bagwiz info`

Show the bag-level metadata of a single ROS 2 rosbag: its layout and storage
backend, how it is compressed, how many files it spans and what they occupy
on disk, whether it carries a `metadata.yaml`, and how many topics and
messages it holds between which times. Every value is read from the bag's own
summaries and the file system, never from its message records, so the command
answers in the time it takes to open the files whatever the bag's size.
Per-topic detail belongs to [`ls`](ls.md) and [`du`](du.md). ROS 1 `*.bag`
inputs are not supported.

## Usage

```text
bagwiz info -i <input> [OPTIONS]
```

## Examples

```bash
# The summary block of a directory-layout rosbag2.
bagwiz info -i path/to/rosbag2_2025_01_01-12_00_00/

# Single-file MCAP.
bagwiz info -i capture.mcap

# Add one row per storage file (shard): size, message count, start, duration.
bagwiz info -i path/to/rosbag2_2025_01_01-12_00_00/ -l

# Sizes as raw byte counts, like `du -b`.
bagwiz info -i capture.mcap -b

# Pick one field in a script.
bagwiz info -i capture.mcap | grep '^Messages:'
```

## Options

| Flag                    | Description                                                                                                                                                                                                                                    |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>` | **Required.** ROS 2 rosbag path: a rosbag2 directory or a single-file `*.mcap` / `*.db3`. A bare `*.db3.zstd` envelope is accepted too, but only its file-level facts are reported (see "What stays unknown").                                 |
| `-l`, `--long`          | Append one row per storage file (shard) after the summary block, with the columns `SIZE`, `MESSAGES`, `START`, `DURATION` and `PATH`. A value the shard's summary does not state prints `-`.                                                   |
| `-b`, `--bytes`         | Print sizes as raw byte counts instead of the default human-readable units (1024-based, one decimal and a `K`/`M`/`G`/`T` suffix, e.g. `4.0K`, `1.2M`; values below 1 KiB stay raw bytes). Applies to `Size` and to the `SIZE` column of `-l`. |

## Output

One `Key: value` line per field, written to `stdout` with the values aligned
in one column, always in this order:

```text
Path:         drive_2026_01_01/
Layout:       directory
Storage:      mcap
Compression:  zstd chunks (2.4x)
Files:        3
Size:         1.9G
Metadata:     metadata.yaml version 5, ros_distro jazzy
Topics:       42 (17 types)
Messages:     1234567
Start:        2026-01-01 03:00:00.123456789 UTC (1767236400.123456789)
End:          2026-01-01 03:20:34.690000000 UTC (1767237634.690000000)
Duration:     20m 34.567s
```

- `Path` is the input path as given.
- `Layout` is `directory` for a rosbag2 directory and `single-file` for a
  bare `*.mcap`, `*.db3` or `*.db3.zstd`.
- `Storage` is `mcap` or `sqlite3`: `metadata.yaml`'s `storage_identifier`
  for a directory, the file's magic bytes for a single file (the inner
  extension for a `*.db3.zstd` envelope). `unknown` when neither identifies
  it.
- `Compression` speaks the vocabulary of [`compress`](compress.md): `none`;
  `<codec> chunks (<ratio>x)` for MCAP chunk compression, where the ratio is
  the chunk index's uncompressed bytes over its compressed bytes and the codec
  reads `lz4+zstd` when the chunks mix codecs; `<codec> per-message (rosbag2
MESSAGE mode)`; `<codec> whole-file envelope (rosbag2 FILE mode)`. The
  rosbag2 modes are what `metadata.yaml` (or a bare `.db3`'s own `metadata`
  row) declares; MCAP chunk compression is read from the chunk index, since
  `metadata.yaml` never records it.
- `Files` counts the bag's storage files (shards), one for a single-file bag.
  `Size` is what those files occupy on disk, summed; `metadata.yaml` itself is
  not counted.
- `Metadata` describes a directory bag's `metadata.yaml`: its declared
  `version`, and `ros_distro` when the recorder wrote one. When the file is
  missing the bag is still described, from the shards found by listing the
  directory, and the line reads
  `metadata.yaml missing (reconstructed from the directory)`. A single file
  reads `none (single file)`.
- `Topics` is the number of topics and, in parentheses, of distinct message
  types among them.
- `Messages`, `Start`, `End` and `Duration` come from the bag's summary.
  Times print in UTC with nanosecond precision, followed by the epoch seconds
  in parentheses; the duration prints as hours, minutes and seconds with
  millisecond precision (`2.500s`, `1m 02.500s`, `1h 02m 03.456s`). An empty
  bag prints `0` messages and `-` for the three extent fields.

With `-l`, a blank line and one row per storage file follow, in play order:

```text
  SIZE MESSAGES START                                                DURATION PATH
668.1M   411522 2026-01-01 03:00:00.123456789 UTC (1767236400.123456789) 6m 51.523s drive_2026_01_01_0.mcap
```

The row's `MESSAGES`, `START` and `DURATION` are the shard's own, from the
`files:` entry of `metadata.yaml` (metadata version 4 and later) or from the
shard's MCAP summary; `-` where neither states them.

## What stays unknown

`info` never reads message records, so a field the bag does not summarise is
reported as `unknown` rather than computed:

| Bag                                                                                                                       | Unknown fields                                                                                                   |
| ------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| An MCAP with no summary section (a recording that was never finalized).                                                   | `Compression`, `Topics`, `Messages`, `Start`, `End`, `Duration`.                                                 |
| A `*.db3` without a `metadata` row: recorded by rosbag2 humble. Iron and later, and every bagwiz writer, fill the row in. | `Messages`. `Topics` still come from the `topics` table and the extent from the two ends of the timestamp index. |
| A bare `*.db3.zstd` FILE-mode envelope opened on its own. Its directory bag reports everything from `metadata.yaml`.      | `Topics`, `Messages`, `Start`, `End`, `Duration`. Decompressing the envelope is the only way to read them.       |

`Messages` is reported as the summary states it. Unlike `ls -l`, it is not
cross-checked against the message table, which is what keeps the command
instant on a large `*.db3`.

## Performance

Each answer comes from a summary structure or a `stat(2)` call: `metadata.yaml`,
the MCAP summary section (one footer read per shard, which is also where the
chunk codec lives), a `*.db3`'s `metadata` row, its `topics` table and the two
ends of its timestamp index. No chunk is decompressed and no message row is
read, so the cost does not grow with the bag's size. A sqlite3 directory bag
whose `metadata.yaml` carries the topic list and summary is answered without
opening a shard at all.

Measured cold-cache on one NVMe host, excluding process startup: a 20.6 GB
single-file MCAP with uncompressed chunks and its 11.0 GB zstd-compressed twin
(929 k messages over 805 topics) each answer in about 0.3 s, and a 13 GB
single-file `*.db3` (625 k messages over 547 topics) in under 0.1 s.

## Exit status

| Code | Meaning                                                                          |
| ---- | -------------------------------------------------------------------------------- |
| `0`  | Success.                                                                         |
| `1`  | Failed — check stderr for the cause (missing path, unparseable `metadata.yaml`). |
