# `bagwiz convert`

Cross-format bag conversion. Three subcommands:

| Subcommand                           | Direction                                      |
| ------------------------------------ | ---------------------------------------------- |
| [`1to2`](#bagwiz-convert-1to2)       | ROS 1 `*.bag` → ROS 2 rosbag2 (MCAP / SQLite3) |
| [`2to1`](#bagwiz-convert-2to1)       | ROS 2 rosbag2 → ROS 1 `*.bag`                  |
| [`storage`](#bagwiz-convert-storage) | ROS 2 rosbag2 repack between MCAP and SQLite3  |

## Common notes

- For `1to2` / `2to1`, only standard message types from a built-in
  whitelist are converted; topics with unsupported types are skipped with
  a warning. The supported set is fixed at build time — see
  [supported types](#supported-types-for-1to2--2to1) below.
- For `1to2` and `storage`, when `--storage` is omitted the storage
  backend is inferred from the output path's extension (`.mcap` →
  MCAP, `.db3` → SQLite3). Output paths that do not carry one of those
  extensions (e.g. a directory) require an explicit `--storage`.
- `mcap` outputs are written without chunk compression. Re-compress
  afterwards with `ros2 bag convert` if needed.
- Per-message conversion / write failures are reported as warnings (rate-
  limited to the first 3 per topic) and counted in the per-topic summary.
  A bad message never aborts the bag.

---

## `bagwiz convert 1to2`

Convert a ROS 1 `*.bag` to a ROS 2 rosbag2.

### Usage

```text
bagwiz convert 1to2 [OPTIONS] <input> <output>
```

### Positional arguments

| Name     | Description                                                   |
| -------- | ------------------------------------------------------------- |
| `input`  | ROS 1 `*.bag` file (must exist).                              |
| `output` | Output rosbag2 directory or single-file (`*.mcap` / `*.db3`). |

### Options

| Flag                  | Description                                                                                    |
| --------------------- | ---------------------------------------------------------------------------------------------- |
| `-s`, `--storage <S>` | Output storage backend. One of `mcap`, `sqlite3`. Default: inferred from the output extension. |

### Behavior

- ROS 1 message types are mapped to their ROS 2 equivalents via the
  built-in whitelist (e.g. `nav_msgs/Odometry` → `nav_msgs/msg/Odometry`).
  Topics whose ROS 1 type is not in the whitelist are skipped with a
  warning.
- The same topic appearing under multiple ROS 1 connections (one
  publisher per chunk, etc.) is declared once on the writer side.
- The MCAP writer is configured with `compression=none`.

### Example

```bash
bagwiz convert 1to2 drive.bag drive.mcap
bagwiz convert 1to2 drive.bag drive_dir/ --storage sqlite3
```

---

## `bagwiz convert 2to1`

Convert a ROS 2 rosbag2 to a ROS 1 `*.bag` file.

### Usage

```text
bagwiz convert 2to1 <input> <output>
```

### Positional arguments

| Name     | Description                                                  |
| -------- | ------------------------------------------------------------ |
| `input`  | ROS 2 rosbag2 (directory, `*.mcap`, or `*.db3`; must exist). |
| `output` | Output ROS 1 `*.bag` file.                                   |

### Behavior

- The output is a non-compressed ROS 1 bag v2.0.
- rosbag2-layer compression on the input
  (`compression_mode: FILE` / `MESSAGE` in `metadata.yaml`) is **not**
  supported and rejected with a clear error. Decompress the input first
  with `ros2 bag convert`.
  - Note: MCAP chunk-level compression on a single-file MCAP input is
    transparent to bagwiz (libmcap handles it), and is therefore
    accepted.
- Topics whose ROS 2 type is not in the whitelist, or whose ROS 1
  counterpart has no message-definition entry, are skipped with a
  warning.

### Example

```bash
bagwiz convert 2to1 drive.mcap drive.bag
bagwiz convert 2to1 rosbag2_2025_01_01/ drive.bag
```

---

## `bagwiz convert storage`

Repack a ROS 2 rosbag2 between MCAP and SQLite3 storage backends. Messages
are copied verbatim — no deserialization or type conversion.

### Usage

```text
bagwiz convert storage [OPTIONS] <input> <output>
```

### Positional arguments

| Name     | Description                                                   |
| -------- | ------------------------------------------------------------- |
| `input`  | Input ROS 2 rosbag2 (directory or single-file). Must exist.   |
| `output` | Output rosbag2 directory or single-file (`*.mcap` / `*.db3`). |

### Options

| Flag                  | Description                                                                                    |
| --------------------- | ---------------------------------------------------------------------------------------------- |
| `-s`, `--storage <S>` | Target storage backend. One of `mcap`, `sqlite3`. Default: inferred from the output extension. |

### Behavior

- Same-storage repacks (e.g. MCAP → MCAP) are rejected; a plain `cp` is
  what you actually want. Format detection uses magic bytes
  (single-file inputs) or `metadata.yaml` (directory layouts), never
  the file extension, so a renamed input is still classified
  correctly.
- Inputs that use rosbag2-layer compression are rejected the same way
  as `2to1`.
- For multi-shard MCAP inputs, schemas are loaded eagerly before
  declaring topics so the output preserves self-description.
- The MCAP writer is configured with `compression=none`.

### Example

```bash
# MCAP -> SQLite3 (extension picks the backend).
bagwiz convert storage drive.mcap drive.db3

# SQLite3 -> directory-layout MCAP.
bagwiz convert storage drive_dir/ drive_mcap_dir/ --storage mcap
```

---

## Supported types (for `1to2` / `2to1`)

The whitelist is fixed at build time. Anything outside this list is
skipped with a warning during conversion.

- `std_msgs`: `Bool`, `Header`, `String`, `Float32`, `Float64`, `Int32`,
  `Int64`, `UInt32`, `UInt64`
- `geometry_msgs`: `Vector3`(`Stamped`), `Point`(`Stamped`),
  `Quaternion`(`Stamped`), `Pose`(`Stamped`),
  `PoseWithCovariance`(`Stamped`), `Transform`(`Stamped`),
  `Twist`(`Stamped`), `TwistWithCovariance`(`Stamped`), `Accel`(`Stamped`)
- `nav_msgs`: `Odometry`, `Path`
- `sensor_msgs`: `Imu`, `Image`, `CompressedImage`, `CameraInfo`,
  `PointCloud2`, `PointField`, `NavSatFix`, `NavSatStatus`, `LaserScan`,
  `Range`, `Temperature`, `FluidPressure`, `MagneticField`
- `diagnostic_msgs`: `DiagnosticArray`, `DiagnosticStatus`, `KeyValue`
- `tf`/`tf2_msgs`: `tf/tfMessage`, `tf2_msgs/TFMessage` →
  `tf2_msgs/msg/TFMessage`
- `builtin_interfaces`: `Time`, `Duration`

## Exit status

| Code | Meaning                                                                                                                                                                                                             |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Conversion finished. Per-topic skip / failure tallies are logged on stderr.                                                                                                                                         |
| `1`  | Argument resolution failed (bad `--storage`, ambiguous output path), the input could not be opened or used a rejected compression mode, the output could not be opened, or a fatal read/write/close error occurred. |
