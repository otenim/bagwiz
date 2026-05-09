# `bagwiz join`

Copy an existing ROS 2 rosbag to a new path and insert a single message,
read from a YAML file, onto a topic that already exists in the source bag.

The YAML is checked against that topic's message type before any bytes are
written. Serialization uses the same introspection-backed path as other
bagwiz tooling and the active RMW implementation.

## Usage

```text
bagwiz join [OPTIONS] <input> <output> <topic> <msg> <at>
```

## Positional arguments

- `input`: Source rosbag: rosbag2 directory or single-file `*.mcap` / `*.db3` (must exist).
- `output`: Destination path (must not exist yet). Storage format follows the extension / layout inferred from `input` (same rules as other bagwiz writers).
- `topic`: Fully qualified topic name; must appear in the input bag's topic list so the message type is known.
- `msg`: Path to a YAML mapping describing the payload (similar shape to ROS 2 `ros2 topic echo -f yaml`).
- `at`: Receive-time selector (`head`, `tail`, or POSIX epoch seconds); see [At](#at).

## Options

- `--sync-msg-stamp`: Overwrite top-level `header.stamp` in the input YAML
  so it matches resolved `<at>` before validation and serialization.
  Current scope is only top-level `header.stamp` (not nested arrays such as
  `transforms[].header.stamp`).

## At

Argument `at` selects the inserted message timestamp in nanoseconds (`int64`
wire time used by bagwiz writers):

- `head`: Same timestamp as the first message time span in the bag (`start_ns` from bag stats).
- `tail`: Same as the last message time (`end_ns`).
- POSIX epoch seconds: Decimal seconds since 1970-01-01 (optional fractional part); converted with truncation toward zero to nanoseconds (`static_cast<int64_t>(sec * 1e9)`).

Matching is ASCII case-insensitive for `head` / `tail` after trimming spaces.

If the bag contains no recorded messages (`total_messages == 0`), both
`head` and `tail` resolve to timestamp `0`. That still allows producing a
bag whose only content is the inserted message plus declared topics.

## Merge order

Messages are streamed from the input bag in encounter order. The inserted
message is written once before the next original message whose timestamp is
strictly greater than `insert_ns`, using the predicate `insert_ns <= next_ts`
where `next_ts` is the next message time (or maximum `int64_t` after EOF).
Therefore the new message sorts at or before ties with the following original
stamp.

## Prerequisites

Types used in the bag must be resolvable (`AMENT_PREFIX_PATH` and embedded
schemas as for other commands). Embedded schema text missing on a topic is
filled from the message-definition resolver where possible before writing.

## Examples

```bash
# Append after natural order using the bag's last sample time.
bagwiz join in.mcap out.mcap /chatter msg.yaml tail

# Fixed wall time (whole seconds).
bagwiz join ./bag_dir ./bag_dir_copy /gps/fix odom.yaml 1700000000

# Fractional POSIX time.
bagwiz join a.db3 b.db3 /tf static_tf.yaml 1700000000.5
```

## Exit status

| Code | Meaning  |
| ---- | -------- |
| `0`  | Success. |
| `1`  | Failure. |

Code `1` includes CLI usage errors, bag/topic/schema resolution failures,
YAML parse/type mismatch, stamp-sync shape/type failures when
`--sync-msg-stamp` is enabled, serialization failures, and unrecoverable
read/write/close errors.

When code `1` happens mid-copy, partial output may exist; remove the
output path and retry.
