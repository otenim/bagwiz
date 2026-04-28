# bagwiz

Fast CLI for analyzing, processing, and extracting data from ROS 2
rosbags. Reads MCAP, SQLite3, and ROS 1 `*.bag` inputs through a unified
backend; ships focused subcommands for listing, scrubbing, exporting, and
converting bag content without spinning up a ROS graph.

## Highlights

- **Unified bag I/O** — directory-layout rosbag2, single-file `*.mcap`
  / `*.db3`, and ROS 1 `*.bag` all open through the same reader API.
- **Schema-driven decoding** — for MCAP shards that embed `ros2msg`
  schemas, no per-package typesupport `.so` is needed at runtime; the
  introspection fallback handles legacy MCAPs, SQLite3, and ROS 1.
- **Interactive scrubbing** — `walk` and `tf walk` provide pager-style
  TUIs over messages and TF lookups.
- **Cross-format conversion** — `convert` covers ROS 1 ↔ ROS 2 and
  storage-only repacks (MCAP ↔ SQLite3) for the standard message
  whitelist.
- **TUM trajectory export** — `traj export` lifts pose-bearing topics
  (and arbitrary `/tf` edges) into a tool-friendly file.

## Installation

bagwiz is an `ament_cmake` package, built with `colcon`. Tested on ROS 2
Humble.

### Prerequisites

- A working ROS 2 installation (e.g. `/opt/ros/humble`).
- `colcon`.
- The system / rosdep packages declared in [`package.xml`](package.xml):
  `geometry_msgs`, `libsqlite3-dev`, `mcap_vendor`, `rcutils`, `rmw`,
  `rmw_implementation`, `rosidl_runtime_c`, `rosidl_runtime_cpp`,
  `rosidl_typesupport_introspection_cpp`, `tf2`, `yaml_cpp_vendor`.
- `CLI11` (`v2.4.2`) and `fmt` (`11.0.2`) are fetched automatically at
  configure time via CMake `FetchContent` — no system install needed.

If the listed deps are not yet on the machine, run from the workspace
root that contains this repository:

```bash
rosdep install --from-paths . --ignore-src -r -y
```

### Build

From a colcon workspace that contains `bagwiz` as a package (e.g.
`<ws>/src/bagwiz`):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bagwiz \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

A convenience wrapper with the same flags lives at
[`build.sh`](build.sh).

### Run

After the build, source the workspace's overlay and the `bagwiz`
executable is on `PATH`:

```bash
source install/setup.bash
bagwiz --version
bagwiz --help
```

### Tests

```bash
colcon test --packages-select bagwiz --event-handlers console_direct+
colcon test-result --verbose
```

## Subcommands

`bagwiz` is a single executable that dispatches to one subcommand per
invocation. Click through for full usage, options, and examples:

| Command                                      | What it does                                                 |
| -------------------------------------------- | ------------------------------------------------------------ |
| [`bagwiz ls`](docs/commands/ls.md)           | List topics in a rosbag with counts and average frequencies. |
| [`bagwiz walk`](docs/commands/walk.md)       | Interactively walk a topic's messages as decoded YAML.       |
| [`bagwiz convert`](docs/commands/convert.md) | Convert between ROS 1 / ROS 2 and repack MCAP / SQLite3.     |
| [`bagwiz traj`](docs/commands/traj.md)       | Extract a topic's pose trajectory (TUM format).              |
| [`bagwiz tf`](docs/commands/tf.md)           | Step through the TF between two frames over time.            |

`bagwiz <subcommand> --help` is always available and reflects the same
options documented in the per-command pages.

## Contributing

See [`AGENTS.md`](AGENTS.md) for repository-wide contribution
conventions (commit message format, branch naming, pre-commit hooks,
CI etiquette).

## License

Apache-2.0. See [`LICENSE`](LICENSE).
