# `bagwiz walk`

Walk the messages of a single topic in a ROS 2 rosbag one at a time and
render each payload as YAML, mirroring `ros2 topic echo`. Designed for
interactive inspection: the view is a pager with vim-style scroll keys,
so even a multi-kilobyte message stays anchored at the top with a
navigable body. ROS 1 `*.bag` inputs are not supported — convert them
first with [`bagwiz convert 1to2`](convert.md#bagwiz-convert-1to2).

## Usage

```text
bagwiz walk <input> <topic>
```

## Positional arguments

| Name    | Description                                               |
| ------- | --------------------------------------------------------- |
| `input` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`). |
| `topic` | Topic name to inspect. Must exist in the bag.             |

## Behavior

- Decoding goes through bagwiz's unified decoder factory. For MCAP shards
  with a non-empty `ros2msg` schema embedded for the topic's type the
  schema-driven backend is used directly. Otherwise (legacy MCAP / SQLite3
  inputs) bagwiz falls back to the rosidl introspection typesupport, which
  **requires the message package to be installed and on
  `AMENT_PREFIX_PATH` at runtime**. If the typesupport library is missing,
  bagwiz reports the package name to install and exits non-zero.
- Messages are loaded lazily and cached as you advance, so `prev` is
  always `O(1)` for anything you have already seen. Only `G` (jump to
  last) can trigger a full-remaining scan.
- Pressing `→` / `Space` past the last message wraps back to the first
  with a `(wrapped to first)` status hint.
- Pressing `s` saves the **currently displayed** message body (the same
  YAML string shown in the pager, not including the header lines) to a
  file. The command prompts for an output path; press Enter with an empty
  line to write under the process current working directory using the name
  `<topic>_<index>.yaml`, where `<topic>` is the ROS topic with each `/`
  replaced by `__`, and `<index>` is the same **zero-based** message index
  as the first number in the header line `[<index> / <last>[+]]` (see the
  Header section for `<last>`).

## Header

Each redraw shows a three-line header:

```text
[<index> / <last>[+]]  <topic>  <type>
timestamp: YYYY-MM-DD HH:MM:SS.nnnnnnnnn UTC (<seconds>.<nanos>)
size:      <bytes> bytes
```

`<last>` is the index of the last message currently loaded in the cache
(equivalently, the count of loaded messages minus one). The trailing `+`
after `<last>` means the bag has more messages after that index that have
not been read into the cache yet (they get pulled in on demand).

## Keys

| Key            | Action                                                             |
| -------------- | ------------------------------------------------------------------ |
| `→` / `Space`  | Next message (wraps from last back to first).                      |
| `←` / `b`      | Previous message.                                                  |
| `↑` / `k`      | Scroll body up one line.                                           |
| `↓` / `j`      | Scroll body down one line.                                         |
| `Home` / `H`   | Jump body scroll to the head.                                      |
| `End` / `T`    | Jump body scroll to the tail.                                      |
| `g`            | Jump to the first message.                                         |
| `G`            | Jump to the last message (forces a full-remaining scan).           |
| `s`            | Save as yaml - writes the current message body (prompts for path). |
| `q` / `Ctrl-C` | Quit.                                                              |

When the body is taller than the visible window, a `lines X-Y of N`
indicator is shown above the key legend.

## Requirements

- `walk` is interactive — both stdin and stdout must be a TTY. Piping the
  output (`bagwiz walk … | less`) exits with an error.

## Example

```bash
bagwiz walk capture.mcap /sensing/imu/data
```

## Exit status

| Code | Meaning                                                                                                           |
| ---- | ----------------------------------------------------------------------------------------------------------------- |
| `0`  | Quit cleanly via `q` / `Ctrl-C`, or the topic had no messages.                                                    |
| `1`  | Bag could not be opened, the topic is absent, the decoder could not be initialized, or stdin/stdout is not a TTY. |
