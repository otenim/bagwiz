# `bagwiz join`

Insert a single message, read from a YAML file, onto a topic that already
exists in a ROS 2 rosbag. By default the input bag is edited **in place**;
pass `-o <output>` to write the result to a new bag and leave the input
untouched.

The YAML is checked against that topic's message type before any bytes are
written. Serialization uses the same introspection-backed path as other
bagwiz tooling and the active RMW implementation.

## Usage

```text
bagwiz join [OPTIONS] <input> <topic> <msg> <at>
```

## Positional arguments

- `input`: Source rosbag: rosbag2 directory or single-file `*.mcap` / `*.db3` (must exist).
- `topic`: Fully qualified topic name. If the topic does not already exist
  in the input bag, pass `-t <ros2_type>` to create it as a new topic.
- `msg`: Path to a YAML mapping describing the payload (similar shape to ROS 2 `ros2 topic echo -f yaml`).
- `at`: Receive-time selector (`head`, `tail`, or POSIX epoch seconds); see [At](#at).

## Options

- `-o, --output <path>`: Destination bag path. When given, `<input>` is left
  untouched and the modified bag is written here; the path must not exist
  yet. When omitted, `<input>` is edited in place: bagwiz writes the
  rewritten bag to a sibling staging path and, on success, atomically
  swaps it over `<input>`. If the operation fails, the staging path is
  removed and `<input>` is left unchanged.
- `-t, --type <ros2_type>`: ROS 2 message type (e.g. `std_msgs/msg/String`)
  used to **create `<topic>` when it does not already exist** in the input
  bag. Without `-t`, a missing topic is a fatal error and bagwiz prints the
  required flag in the message. When `<topic>` already exists in the bag,
  `-t` is ignored except that a mismatch with the bag's recorded type is
  reported as a warning. See [Creating a new topic](#creating-a-new-topic).
- `--sync-msg-stamp`: Overwrite top-level `header.stamp` in the input YAML
  so it matches resolved `<at>` before validation and serialization.
  Current scope is only top-level `header.stamp` (not nested arrays such as
  `transforms[].header.stamp`).

## Creating a new topic

When `<topic>` is not present in the input bag, bagwiz can register it on
the fly using the type provided via `-t/--type`. The flow is:

1. The `.msg` schema for `<ros2_type>` is resolved from
   `$AMENT_PREFIX_PATH/share/<pkg>/msg/<Type>.msg` (the same lookup used
   for existing topics with missing schema text).
2. The YAML payload is validated against that schema and serialized to
   CDR.
3. A new topic entry with name `<topic>`, type `<ros2_type>`, and
   `serialization_format = "cdr"` is declared in the output bag. For
   MCAP, the resolved schema text is embedded; for SQLite3, the topic
   row is inserted with empty QoS / type-description-hash (both are
   optional fields).

Requirements:

- The package providing `<ros2_type>` must be installed and visible via
  `AMENT_PREFIX_PATH` so the schema text can be resolved.
- If the topic already exists and `-t` is given with a different type,
  bagwiz keeps using the bag's recorded type and emits a warning.

## In-place semantics

When `-o` is omitted, `<input>` is replaced atomically:

1. The rewritten bag is staged at a sibling path of `<input>`
   (e.g. `.<name>.bagwiz-staged-<pid>-<nanos>`), on the same filesystem.
2. After the writer closes cleanly, bagwiz renames `<input>` aside,
   renames the staging path into `<input>`'s name, and removes the
   aside copy.
3. If anything fails before the swap, the staging path is removed and
   `<input>` is left untouched. If the swap itself fails partway, the
   original `<input>` is restored.

Practical implications:

- Disk space: peak usage is roughly twice the size of `<input>` during
  the operation.
- The input and staging path must live on the same filesystem (they do
  by construction, since bagwiz places the staging path next to
  `<input>`).
- Other readers that have `<input>` open at the moment of the swap will
  continue to see the pre-swap bytes on POSIX; new opens see the new
  bag.

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
# In-place: edit the bag at in.mcap and leave the result at in.mcap.
bagwiz join in.mcap /chatter msg.yaml tail

# Write a copy to out.mcap, leaving in.mcap untouched.
bagwiz join in.mcap /chatter msg.yaml tail -o out.mcap

# In-place edit on a directory bag, using a fixed wall time.
bagwiz join ./bag_dir /gps/fix odom.yaml 1700000000

# Copy on a SQLite3 bag, using fractional POSIX time.
bagwiz join a.db3 /tf static_tf.yaml 1700000000.5 -o b.db3

# Create a brand-new topic in the bag (in place).
bagwiz join in.mcap /new_topic hello.yaml head -t std_msgs/msg/String

# Same idea but produce a copy at out.mcap.
bagwiz join in.mcap /new_topic hello.yaml head -t std_msgs/msg/String -o out.mcap
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

In-place mode is crash-safe at the swap boundary: a failure before the
swap leaves `<input>` untouched. With `-o`, a failure mid-write may
leave a partial bag at the output path; remove it and retry.
