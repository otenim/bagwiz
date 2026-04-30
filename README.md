# bagwiz

Fast CLI for analyzing, processing, and extracting data from ROS 2
rosbags. The inspection and export subcommands (`ls`, `walk`, `tf`,
`traj`) read rosbag2 inputs — directory layouts and single-file `*.mcap`
/ `*.db3` — through a unified backend. The `convert` subcommand
additionally bridges to and from ROS 1 `*.bag` for cross-format work.
All of this happens without spinning up a ROS graph.

## Installation

Bagwiz bundles a few ROS message packages (declared in
[`bagwiz.repos`](bagwiz.repos)) that are imported into `dependencies/`
at setup time and built alongside bagwiz. After cloning, source your
ROS environment and run `setup.bash` to import those sources and
install the ROS / system dependencies declared in
[`package.xml`](package.xml):

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
./setup.bash
```

`setup.bash` uses [vcstool](https://github.com/dirk-thomas/vcstool) and
[rosdep](https://docs.ros.org/en/independent/api/rosdep/html/), so make
sure both are installed (`sudo apt install python3-vcstool python3-rosdep`)
and that rosdep has been initialised (`sudo rosdep init && rosdep update`).

`CLI11` and `fmt` are fetched automatically at CMake configure time, so
no extra system install is needed for them.

## Subcommands

`bagwiz` is a single executable that dispatches to one subcommand per
invocation. Click through for full usage, options, and examples:

| Command                                      | What it does                                                             |
| -------------------------------------------- | ------------------------------------------------------------------------ |
| [`bagwiz ls`](docs/commands/ls.md)           | List topics in a ROS 2 rosbag with counts and average frequencies.       |
| [`bagwiz walk`](docs/commands/walk.md)       | Interactively walk a ROS 2 topic's messages as decoded YAML.             |
| [`bagwiz convert`](docs/commands/convert.md) | Convert between ROS 1 and ROS 2, or repack ROS 2 between MCAP / SQLite3. |
| [`bagwiz traj`](docs/commands/traj.md)       | Extract a ROS 2 topic's pose trajectory (TUM format).                    |
| [`bagwiz tf`](docs/commands/tf.md)           | Step through the TF between two frames in a ROS 2 rosbag.                |

`bagwiz <subcommand> --help` is always available and reflects the same
options documented in the per-command pages.

## Contributing

See [`AGENTS.md`](AGENTS.md) for repository-wide contribution
conventions (commit message format, branch naming, pre-commit hooks,
CI etiquette).

## License

Apache-2.0. See [`LICENSE`](LICENSE).
