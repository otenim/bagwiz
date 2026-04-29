# `bagwiz tf`

TF inspection on a ROS 2 rosbag. Currently ships a single subcommand:

| Subcommand                | Purpose                                                              |
| ------------------------- | -------------------------------------------------------------------- |
| [`walk`](#bagwiz-tf-walk) | Step through the TF between two frames at each dynamic `/tf` update. |

ROS 1 `*.bag` inputs are not supported — convert them first with
[`bagwiz convert 1to2`](convert.md#bagwiz-convert-1to2).

---

## `bagwiz tf walk`

Step one-at-a-time through the TF chain between `<from>` and `<to>` at
every dynamic `/tf` update in the bag. Each step renders the lookup result
at that exact stamp, so you can scrub a recorded TF tree the same way
you would scrub a YAML message stream with `bagwiz walk`.

### Usage

```text
bagwiz tf walk [OPTIONS] <input> <from> <to>
```

### Positional arguments

| Name    | Description                                                                        |
| ------- | ---------------------------------------------------------------------------------- |
| `input` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`).                          |
| `from`  | Reference (fixed) frame — the output expresses `<to>` in this frame's coordinates. |
| `to`    | Tracked (moving) frame to sample.                                                  |

### Options

| Flag                | Default | Description                                                                                             |
| ------------------- | ------- | ------------------------------------------------------------------------------------------------------- |
| `-r`, `--rot <FMT>` | `quat`  | Rotation format. One of `quat`, `euler`, `euler_rad`, `euler_deg`. `euler` is an alias for `euler_rad`. |

### Behavior

- All `tf2_msgs/msg/TFMessage` topics are read in a single pass:
  - Topics whose name ends in `tf_static` are inserted as static
    transforms; everything else as dynamic transforms.
  - The set of distinct timestamps emitted by dynamic `/tf` messages
    (i.e. excluding `tf_static`) becomes the walk's timeline. Each one
    is a moment at which the TF tree observably changed.
- Before entering the interactive view, bagwiz probes
  `lookupTransform(<from>, <to>, timeline.front())`:
  - If the chain is structurally broken (frame absent, no connecting
    edges) the command exits non-zero.
  - If the chain simply hasn't been published yet at the bag's first
    dynamic stamp (typical when sensor `/tf` precedes the localizer),
    the timeline is cropped forward to the earliest stamp at which the
    chain is queryable.
  - If the chain is never queryable for any timeline stamp, the command
    exits non-zero.
- Bags whose only TF topic is `/tf_static` produce a single-step
  timeline, which `tf walk` rejects (a one-step walk is not useful).

### Header per step

```text
[STEP <i> / <N>]  YYYY-MM-DD HH:MM:SS.nnnnnnnnn UTC (<seconds>.<nanos> s)
TF: <from>  ->  <to>

translation:
  x: ...
  y: ...
  z: ...
rotation (...):
  ...
```

The body shows the lookup result at the current step. If a mid-bag gap
or a chain that ceases to publish before the bag ends causes a lookup to
fail, `tf2`'s error text is shown inline (`⚠  Lookup failed at this
step: …`) instead of crashing the walk.

### Rotation formats

| `--rot`               | Output                         |
| --------------------- | ------------------------------ |
| `quat`                | Quaternion `(x, y, z, w)`.     |
| `euler` / `euler_rad` | Roll / pitch / yaw in radians. |
| `euler_deg`           | Roll / pitch / yaw in degrees. |

### Keys

| Key            | Action                                     |
| -------------- | ------------------------------------------ |
| `→` / `Space`  | Next step (wraps from last back to first). |
| `←` / `b`      | Previous step.                             |
| `g`            | Jump to the first step.                    |
| `G`            | Jump to the last step.                     |
| `q` / `Ctrl-C` | Quit.                                      |

Scroll keys (`↑`/`↓`/`Home`/`End`) are accepted but ignored — the body
fits in a normal-sized terminal.

### Requirements

- `tf walk` is interactive — both stdin and stdout must be a TTY.
- The bag must contain at least one `tf2_msgs/msg/TFMessage` topic.

### Examples

```bash
# Track base_link in the map frame, default quaternion output.
bagwiz tf walk capture.mcap map base_link

# Same chain, Euler in degrees.
bagwiz tf walk capture.mcap map base_link --rot euler_deg
```

### Exit status

| Code | Meaning                                                                                                                                                                                                              |
| ---- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Quit cleanly via `q` / `Ctrl-C`.                                                                                                                                                                                     |
| `1`  | Bag could not be opened, no TFMessage topic in the bag, no dynamic `/tf` updates, the TF chain is structurally broken or never resolvable inside the bag's window, the decoder failed, or stdin/stdout is not a TTY. |
