# bagwiz

Fast CLI for analyzing, processing, and extracting data from ROS 2
rosbags. Reads MCAP, SQLite3, and ROS 1 `*.bag` inputs through a unified
backend; ships focused subcommands for listing, scrubbing, exporting, and
converting bag content without spinning up a ROS graph.

## Installation

Install the ROS 2 / system dependencies declared in
[`package.xml`](package.xml) via rosdep, from the workspace root that
contains this repository:

```bash
rosdep install --from-paths . --ignore-src -r -y
```

`CLI11` and `fmt` are fetched automatically at CMake configure time, so
no extra system install is needed for them.

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
