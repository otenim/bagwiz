# bagwiz

A fast CLI for analyzing, processing, and extracting data from ROS 2 rosbags
entirely offline — without spinning up a ROS graph. rosbag2 inputs are read
through a unified backend that spans three independent dimensions:

- **Storage format** — MCAP (`*.mcap`) or SQLite3 (`*.db3`)
- **Layout** — a rosbag2 directory or a bare single file
- **Compression** — uncompressed, rosbag2 `compression_mode: MESSAGE`, MCAP
  per-chunk compression, or whole-database `compression_mode: FILE` zstd
  envelopes (`*.db3.zstd`)

Any combination of these is accepted transparently.

## Installation

bagwiz is built and run through [pixi](https://pixi.sh) — no system ROS 2
install needed.

1. Install pixi once, then reopen your shell so `pixi` is on `PATH`:

   ```bash
   curl -fsSL https://pixi.sh/install.sh | bash
   ```

2. Build bagwiz for a distro. bagwiz supports ROS 2 Humble and Jazzy; each
   has CPU and CUDA environments.

   ```bash
   pixi run -e humble build        # basic features
   pixi run -e humble build-full   # includes advanced features such as `map`
   ```

   Use `build-full` only when you need features like `bagwiz map`. The default
   environment is Jazzy, so `pixi run build` is equivalent to
   `pixi run -e jazzy build`.

   Builds compile through ccache and Ninja (both provided by the pixi
   environment), so repeated builds — including the first build in a fresh git
   worktree — reuse previously compiled objects. To cap build parallelism on
   memory-constrained hosts, append the worker count after the build type, e.g.
   `pixi run build Release 8`; it is forwarded to colcon's
   `--parallel-workers` (default: half the physical CPU cores). Both knobs have
   env-var forms too: `BAGWIZ_BUILD_PARALLELISM=8` and `BAGWIZ_BUILD_TYPE=Debug`.

3. Install a `bagwiz` launcher on your `PATH` so you can run it from anywhere:

   ```bash
   pixi run -e humble install   # installs ~/.local/bin/bagwiz
   ```

   This also installs shell completion for your current shell. It does not
   build; run the build from step 2 first. Use the same `-e <distro>` you built
   with. To switch distros, run `pixi run -e <distro> install` again.

4. Verify the install:

   ```bash
   bagwiz --help
   ```

   If the command is not found, make sure the install directory (default
   `~/.local/bin`) is on your `PATH`.

### Cleaning builds

Each pixi environment has its own build/install directory. To remove the
artifacts for a single environment only:

```bash
pixi run -e humble clean       # removes build/humble and install/humble
pixi run -e humble-cuda clean  # removes build/humble-cuda and install/humble-cuda
```

A bare `pixi run clean` targets the default environment (`jazzy`), so it only
removes `build/jazzy` and `install/jazzy`. To wipe every distro's build/install
artifacts plus all logs, use:

```bash
pixi run clean-all
```

### Using your own message packages (overlays)

Bags whose topics use message types beyond the standard stack need the matching
ROS 2 message packages available at run time. Build those packages in your own
colcon workspace and source its `install/setup.bash` before running bagwiz:

```bash
source /path/to/my_msgs_ws/install/setup.bash
bagwiz walk -i my.mcap -t /topic
```

Sourcing the overlay sets `AMENT_PREFIX_PATH` and `LD_LIBRARY_PATH`, so bagwiz
finds the `msg/*.msg` definitions and can dlopen() the introspection typesupport
at runtime without a rebuild. Build overlays against the same distro so their
libraries stay ABI compatible with bagwiz.

This works through the installed `bagwiz` launcher as well: it re-reads your
`AMENT_PREFIX_PATH` on every invocation, layering the pixi environment's own
prefixes in front of yours so it keeps priority on package-name collisions.
The one invocation path where a sourced overlay does not survive is
`pixi run -e <distro> run -- ...`, because pixi's own activation rebuilds
`AMENT_PREFIX_PATH` before bagwiz starts — there, enter
`pixi shell -e <distro>` first and source the overlay inside that shell.

## Subcommands

`bagwiz` is a single executable that dispatches to one subcommand per
invocation. Click through for full usage, options, and examples:

| Command                                        | What it does                                                                                                                                                                                                                                                                                                                                                                    |
| ---------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [`bagwiz ls`](docs/commands/ls.md)             | List topics in a ROS 2 rosbag (add `-l` for per-topic counts and average frequencies).                                                                                                                                                                                                                                                                                          |
| [`bagwiz du`](docs/commands/du.md)             | Report per-topic serialized payload sizes in a ROS 2 rosbag, du(1)-style: sorted by size descending, each row showing its share of the total, with a `total` row (human-readable sizes by default, `-b` for raw bytes, `-d` for topic-name depth grouping).                                                                                                                     |
| [`bagwiz walk`](docs/commands/walk.md)         | Interactively walk a ROS 2 topic's messages as decoded YAML.                                                                                                                                                                                                                                                                                                                    |
| [`bagwiz convert`](docs/commands/convert.md)   | Repack a ROS 2 rosbag between storage backends/layouts.                                                                                                                                                                                                                                                                                                                         |
| [`bagwiz compress`](docs/commands/compress.md) | Compress or decompress a ROS 2 rosbag: MCAP chunk compression (zstd/lz4), rosbag2 MESSAGE/FILE-mode for SQLite3, or back to plain storage with `--mode none`.                                                                                                                                                                                                                   |
| [`bagwiz topic`](docs/commands/topic.md)       | Keep (`keep`), drop (`drop`), or rename (`rename`) topics in a ROS 2 rosbag.                                                                                                                                                                                                                                                                                                    |
| [`bagwiz trim`](docs/commands/trim.md)         | Trim a ROS 2 rosbag to a time window given as offsets or message counts from the bag start (e.g. `--start 5s --end 90s`, `--duration 30s`, `--both 50msg`), or align it to the time span of selected topics (`--align`). `--keep` exempts topics from the window so they are copied whole. Windows are evaluated on `header.stamp` by default (`--stamp recv` for record time). |
| [`bagwiz stamp`](docs/commands/stamp.md)       | Edit message timestamps: overwrite each message's `header.stamp` with its receive (log) time (`sync`), on every topic whose type leads with a `std_msgs/Header`.                                                                                                                                                                                                                |
| [`bagwiz calib`](docs/commands/calib.md)       | Sensor-extrinsic calibration: refine a camera-LiDAR static-TF edge (`cam-lidar`) against the bag's own point clouds and pose topic.                                                                                                                                                                                                                                             |
| [`bagwiz cam-info`](docs/commands/cam-info.md) | Replace (`replace`) one or more CameraInfo topics' calibration with values from camera_calibration YAML files (shared or per-topic), recompute (`recompute-p`) the projection matrix from the intrinsics, or dump (`dump`) a topic's calibration back out to YAML.                                                                                                              |
| [`bagwiz movify`](docs/commands/movify.md)     | Render a rosbag to video — image topic(s) to a video file (`cam`), as a single view or a multi-view grid with per-view point-cloud overlays.                                                                                                                                                                                                                                    |
| [`bagwiz traj`](docs/commands/traj.md)         | Dump a topic's pose trajectory to TUM, or join a trajectory file back into a bag.                                                                                                                                                                                                                                                                                               |
| [`bagwiz tf`](docs/commands/tf.md)             | Inspect and edit TF in a ROS 2 rosbag: frame tree, static-transform resolution, static-TF copy, edge-level static-tree edits (`drop` frames, `update` edges), and dumping the static tree to a publisher-config YAML or embedding one back in.                                                                                                                                  |
| [`bagwiz pcd`](docs/commands/pcd.md)           | PointCloud2 topic processing: concatenate multiple LiDAR topics into one (`concat`) via static TF + time sync, or motion-deskew one or more topics from an external pose or twist topic (`undistort`).                                                                                                                                                                          |
| [`bagwiz map`](docs/commands/map.md)           | LiDAR map generation and viewing: `map slam`, `map viewer`. Optional build.                                                                                                                                                                                                                                                                                                     |
| [`bagwiz complete`](docs/commands/complete.md) | Generate a shell completion script (`bash`, `zsh`, `fish`).                                                                                                                                                                                                                                                                                                                     |

`bagwiz <subcommand> --help` is always available and reflects the same
options documented in the per-command pages.

Many topic-valued flags — `topic drop -t`, `pcd concat --pcd`, `map slam
--color`, and others — accept a `'*'` glob in addition to a literal topic
name; some flags only accept a literal. See
[Topic selectors](docs/commands/topic.md#topic-selectors) for the shared
rules and each command's page for which flags accept a glob.

Every subcommand that writes to an `-o`/`--output` path shares one clobber
rule: an existing path stops the run unless `-w`/`--overwrite` is passed. The
collision is reported before the input bag is read, so a wrong `-o` costs a
single stat rather than a full pass over the bag — or, for
[`bagwiz calib cam-lidar`](docs/commands/calib.md), a full refinement. Under
`-w` the existing path is removed only when the command is ready to write, so
a run that fails partway leaves it intact.

## Environment variables

bagwiz reads a handful of **optional** environment variables to override
defaults — logging verbosity (`BAGWIZ_LOG_LEVEL`), decoder backend
(`BAGWIZ_DECODER`), color output (`NO_COLOR`), message-package overlays
(`AMENT_PREFIX_PATH`), and the installed launcher's distro
(`BAGWIZ_DEFAULT_DISTRO`), among others. None are required for normal use. See
the full reference in [docs/environment.md](docs/environment.md).

## License

Apache-2.0. See [`LICENSE`](LICENSE).
