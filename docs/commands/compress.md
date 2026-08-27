# `bagwiz compress`

Compress or decompress a rosbag by re-encoding it. MCAP outputs use chunk
compression (zstd or lz4); SQLite3 directory outputs use rosbag2's
MESSAGE-mode (per-message zstd) or FILE-mode (whole-shard `.db3.zstd`
envelope). `--mode none` reverses any of these back to plain storage.
Messages are re-encoded wholesale; no topic selection or time windowing is
applied.

## Usage

```text
bagwiz compress -i <input> -o <output> [OPTIONS]
```

## Examples

```bash
# Compress an MCAP bag with zstd chunk compression (the default for MCAP).
bagwiz compress -i drive_dir/ -o drive_zstd/

# Compress with lz4 chunks instead (MCAP only), at the fastest effort.
bagwiz compress -i drive_dir/ -o drive_lz4/ --codec lz4 --level fastest

# Compress a SQLite3 bag per message (the default for SQLite3 outputs).
bagwiz compress -i drive_sqlite/ -o drive_msg/ --storage sqlite3

# Wrap a SQLite3 bag's shard in a whole-database .db3.zstd envelope.
bagwiz compress -i drive_sqlite/ -o drive_file/ --storage sqlite3 --mode file

# Decompress: back to plain, uncompressed storage (either backend).
bagwiz compress -i drive_zstd/ -o drive_plain/ --mode none
```

## Options

| Flag                      | Description                                                                                                                                                                                                                                                                                                                                                      |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`   | **Required.** Input ROS 2 rosbag2 (directory or single-file). Must exist. Compressed inputs of every supported shape (MCAP chunk compression, MESSAGE-mode, FILE-mode `.db3.zstd` envelope) are read transparently, including a bare `.db3` lifted out of a MESSAGE-mode directory bag — its own `metadata` table carries the declaration.                       |
| `-o`, `--output <output>` | **Required.** Output rosbag2 directory or single-file (`*.mcap` / `*.db3`). SQLite3 compression (`--mode file` / `message`) requires a directory output: rosbag2 only decompresses when a `metadata.yaml` declares the mode, so a single `.db3` would read back as raw zstd frames without an error.                                                             |
| `--mode <M>`              | Compression mode. One of `auto`, `file`, `message`, `none`. `file`: MCAP chunk compression, or the whole-shard `.db3.zstd` envelope for SQLite3. `message`: per-message zstd frames (SQLite3 only; rejected for MCAP, where rosbag2 defines no per-message mode). `none`: decompress to plain storage. Default: `auto` — `file` for MCAP, `message` for SQLite3. |
| `--codec <C>`             | Compression codec. One of `zstd`, `lz4`. `lz4` is valid only for MCAP chunk compression; rosbag2 defines zstd alone for SQLite3 storage. Nothing is encoded under `--mode none`, so naming a codec there is rejected. Default: `zstd`. Long-form only.                                                                                                           |
| `--level <L>`             | Encoder effort. One of `fastest`, `fast`, `default`, `slow`, `slowest`. Maps onto the codec's effort scale (for SQLite3 zstd: 1, 2, the library default, 9, 19 respectively). Default: the codec's own default. Long-form only.                                                                                                                                  |
| `--storage <S>`           | Target storage backend. One of `mcap`, `sqlite3`. Default: inferred from the output extension; otherwise inherited from the input bag's storage — the same resolution order as [`convert format`](convert.md#storage-backend-resolution). Long-form only.                                                                                                        |
| `-w`, `--overwrite`       | Replace `<output>` if it already exists. Without this flag, any pre-existing entry at `<output>` (file or directory) stops the run with a clear log line.                                                                                                                                                                                                        |

## Compression modes

| Storage | `file`                                                                             | `message`                                                                    | `none`              |
| ------- | ---------------------------------------------------------------------------------- | ---------------------------------------------------------------------------- | ------------------- |
| MCAP    | Chunk compression (zstd or lz4), recorded inside the `.mcap` itself                | Not supported (rejected)                                                     | Uncompressed chunks |
| SQLite3 | Whole-shard `.db3.zstd` envelope (`compression_mode: FILE`), directory layout only | Per-message zstd frames (`compression_mode: MESSAGE`), directory layout only | Plain `.db3`        |

Both SQLite3 modes produce the byte shapes rosbag2 writes for the same
settings (rosbag2_compression_zstd, reached with `ros2 bag record
--compression-mode message|file --compression-format zstd`, or the matching
keys in a `ros2 bag convert` output config): MESSAGE-mode payloads are
bare zstd frames, and FILE-mode wraps each finished shard in a single zstd
frame and points `metadata.yaml` at the `.db3.zstd` name.

MCAP is different, because its compression is part of the container: the
codec is recorded on each chunk inside the `.mcap`, and `metadata.yaml`'s
`compression_format` / `compression_mode` stay empty. Those two fields name
rosbag2's own compression layer, which expands the file they point at as a
whole-file envelope before the storage plugin sees it — so declaring chunk
compression there yields a bag that `ros2 bag info` still summarises but
`ros2 bag play` and `ros2 bag convert` cannot open. rosbag2's own MCAP writer
leaves the pair empty for the same reason, and every output shape this
command writes is checked against a live rosbag2 by
`scripts/check-rosbag2-compat.sh`.

## Performance

The re-encode streams every message through the decoded pipeline; nothing is
chunk-copied, since copying chunks would preserve the input's compression —
the exact thing this command changes. MCAP chunk compression parallelizes
across write threads when available. SQLite3 FILE-mode reads best on
machines with free temp space roughly the size of the decompressed database
(readers expand the envelope to a temporary `.db3`).

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
