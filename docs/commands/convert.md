# `bagwiz convert`

Cross-format bag conversion. Subcommands:

| Subcommand                         | What it does                                                                       |
| ---------------------------------- | ---------------------------------------------------------------------------------- |
| [`format`](#bagwiz-convert-format) | ROS 2 rosbag2 repack between MCAP and SQLite3 storage and/or file/directory layout |

---

## `bagwiz convert format`

Repack a ROS 2 rosbag2. The subcommand handles two independent
conversions in one pass — choose the target storage backend (MCAP ↔
SQLite3) via `--storage`, and choose the on-disk layout
(single-file ↔ directory) via the shape of `<output>`. Messages are
copied verbatim; no deserialization or type conversion is performed, and
the input's compression is carried over to the output (see
[Compression handling](#compression-handling)).

### Usage

```text
bagwiz convert format -i <input> -o <output> [OPTIONS]
```

### Examples

```bash
# MCAP file -> SQLite3 file (extension picks the backend).
bagwiz convert format -i drive.mcap -o drive.db3

# SQLite3 directory -> directory-layout MCAP.
bagwiz convert format -i drive_dir/ -o drive_mcap_dir/ --storage mcap

# Layout change without storage change: single-file MCAP -> directory MCAP.
# --storage is optional here — the directory output inherits MCAP from the input.
bagwiz convert format -i drive.mcap -o drive_dir/

# MESSAGE-mode SQLite3 directory -> single-file MCAP: the per-message compression is carried over as chunk compression.
bagwiz convert format -i drive_compressed_dir/ -o drive.mcap
```

### Options

| Flag                      | Description                                                                                                                                                                                                                                                                                        |
| ------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`   | **Required.** Input ROS 2 rosbag2 (directory or single-file). Must exist.                                                                                                                                                                                                                          |
| `-o`, `--output <output>` | **Required.** Output rosbag2 directory or single-file (`*.mcap` / `*.db3`). The input's compression is carried over and translated to the output storage; a single-file `*.db3` cannot carry compression and is written plain, with a warning (see [Compression handling](#compression-handling)). |
| `--storage <S>`           | Target storage backend. One of `mcap`, `sqlite3`. Default: inferred from the output extension; otherwise inherited from the input bag's storage (see [Storage backend resolution](#storage-backend-resolution)). Long-form only.                                                                   |
| `-w`, `--overwrite`       | Replace `<output>` if it already exists. Without this flag, any pre-existing entry at `<output>` (file or directory) stops the run with a clear log line. Supported by every `bagwiz` subcommand that writes a file or directory output.                                                           |

### Storage backend resolution

The target storage backend is resolved in this order (first match wins):

1. `--storage` if given.
2. Output path's extension (`.mcap` → MCAP, `.db3` → SQLite3).
3. Input bag's detected storage backend. Directory-layout outputs
   without `--storage` therefore inherit the input's backend — handy
   for a pure file ↔ directory layout change.

### Format detection

- Same-storage + same-layout repacks (e.g. MCAP file → MCAP file) are
  rejected; a plain `cp` is what you actually want. Format detection
  uses magic bytes (single-file inputs) or `metadata.yaml` (directory
  layouts), so a renamed `.mcap` / `.db3` input is still classified
  correctly. The one exception: for a single-file zstd envelope
  (`.mcap.zstd` / `.db3.zstd`) the magic sniff cannot see past the
  compression, so the inner storage is resolved from the extension.

### Layout conversion

- Layout transitions (file ↔ directory) are supported on either
  storage backend. The output layout is derived from `<output>`: a
  path ending in `.mcap` or `.db3` produces a single-file bag, any
  other path produces a directory-layout bag with the canonical
  `metadata.yaml`.

### Compression handling

A repack changes the container, not how the bytes are compressed: the
input's compression is carried over and translated to the output storage,
following the shared [output bag shape](../../README.md#subcommands) rule.
The input's compression is read the way `bagwiz info` reads it — from
`metadata.yaml` for a directory bag, from the file's own `metadata` table
for a single `.db3` (which is how a shard lifted out of a MESSAGE-mode
directory bag still reads correctly), or from the chunk index of an MCAP —
and it is read before `<output>` is claimed, so under `-w` the existing
path is removed only once the run is ready to write. An exact carry-over
is logged only at `debug` level; a translation between compression shapes
is logged at `[INFO]`.

What the output carries, by input:

- MCAP `zstd` or `lz4` chunks: the same codec as chunk compression on an
  MCAP output; rosbag2 `MESSAGE` mode (`zstd`) on a SQLite3 directory
  output — `lz4` has no SQLite3 counterpart, and the `[INFO]` line says so.
  An MCAP whose chunks mix `lz4` and `zstd` is written with `zstd`
  throughout.
- `MESSAGE` + `zstd`: payloads are transparently decompressed on read; an
  `[INFO]` line announces the path. An MCAP output takes `zstd` chunk
  compression; a SQLite3 directory output keeps `MESSAGE` mode.
- `MESSAGE` + non-zstd: rejected with a clear error (only `zstd` is
  implemented today).
- `FILE` on MCAP: a declaration bagwiz never writes — MCAP keeps its
  chunk compression inside the shard, and naming it in
  `metadata.yaml` instead would make rosbag2 expand the shard as a
  whole-file envelope and fail to open the bag — but does accept on
  input, since the shard is an ordinary MCAP whose chunks libmcap
  decompresses transparently. No extra work needed; the output carries
  whatever chunk compression the shards actually use.
- `FILE` + `zstd` on SQLite3: a whole-database `.db3.zstd` envelope.
  Each shard is stream-decompressed to a temporary `.db3` on read (an
  `[INFO]` line announces the path) and removed when the reader
  closes; reading needs free temp space roughly the size of the
  decompressed database. A bare single-file `.db3.zstd` is accepted
  the same way. An MCAP output takes `zstd` chunk compression; a SQLite3
  directory output reproduces the `FILE`-mode envelope.
- `FILE` + non-zstd on SQLite3: rejected with a clear error (only
  `zstd` is implemented today).
- Plain input (`NONE` / empty declaration, uncompressed MCAP chunks):
  stays plain on every output.

A single-file `.db3` output cannot carry compression: rosbag2 reads the
mode from `metadata.yaml`, which only a directory bag has. A compressed
input written to a `*.db3` path comes out plain, and a `[WARN]` line says
so — name a directory output to keep it. No bag records the encoder level
it was written with, so the codec's default level is used. When the
input's compression cannot be read without scanning it (an MCAP with no
summary section, typically a recording that was never finalized), a
`[WARN]` line says so and the output is written plain — a repack never
adds compression its input may not have had.

MCAP chunk compression is never declared in `metadata.yaml` (see `FILE` on
MCAP above): a directory-layout MCAP output keeps its `compression_mode` /
`compression_format` fields empty even when its chunks are compressed. To
change compression rather than keep it, use
[`bagwiz compress`](compress.md).

### Self-description preservation

- For multi-shard MCAP inputs, schemas are loaded eagerly before
  declaring topics so the output preserves self-description.
- SQLite3 inputs from Humble and earlier carry no embedded message
  definitions. bagwiz resolves each missing definition from
  `$AMENT_PREFIX_PATH/share/<pkg>/msg/<Type>.msg` before declaring
  the topic so the resulting MCAP keeps self-description for strict
  downstream readers.

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
