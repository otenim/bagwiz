# `bagwiz stamp`

Message-timestamp edits. Subcommands:

| Subcommand                   | What it does                                                     |
| ---------------------------- | ---------------------------------------------------------------- |
| [`sync`](#bagwiz-stamp-sync) | Overwrite each message's `header.stamp` with its receive time.   |

---

## `bagwiz stamp sync`

Overwrite each message's `header.stamp` with the time the message was recorded
into the bag (its receive / log time). Useful when the publishers' clocks and
the recording clock disagree — e.g. sensors stamped by an unsynchronized
source clock — and downstream tooling should see one consistent timeline.

The rewrite applies to every topic whose message type leads with a
`std_msgs/Header` — the ROS 2 stamped-message convention, and the same
classification `trim --stamp header` uses. Messages on every other topic are
copied verbatim: a type without a leading header (e.g. `tf2_msgs/msg/TFMessage`,
whose headers sit inside the `transforms` array) has no top-level
`header.stamp` to rewrite. Receive times themselves are never changed — only
the `header.stamp` bytes inside the payloads.

### Usage

```text
bagwiz stamp sync -i <input> [OPTIONS]
```

### Examples

```bash
# Sync header.stamp to the receive time on every headered topic, in place.
bagwiz stamp sync -i drive.mcap

# Write the synced bag to a new path, leaving the input untouched.
bagwiz stamp sync -i drive.mcap -o drive_synced.mcap

# Re-run onto an output path that already exists.
bagwiz stamp sync -i drive_dir/ -o synced_dir/ -w
```

### Options

| Flag                    | Description                                                                                           |
| ----------------------- | ----------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>` | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                              |
| `-o`, `--output <p>`    | Write the result to a new bag instead of rewriting `<input>` in place.                                |
| `-w`, `--overwrite`     | Replace an existing `-o` path. Without it, an existing output path stops the run. No effect in-place. |

### Topic classification

- A topic is rewritten when its message type's top-level definition declares a
  leading `std_msgs/Header header` field. The type's definition comes from the
  schema embedded in the bag; a type without one (SQLite3 bags do not embed
  definitions) is resolved from `.msg` files on `$AMENT_PREFIX_PATH`.
- A type that cannot be classified either way (no embedded schema and no
  resolvable `.msg`) is copied verbatim, after a warning naming the count and
  an example type.
- When the bag has no headered topic at all, the run stops with an error and
  leaves the input untouched — there is nothing to sync.
- A headered message whose `header.stamp` cannot be written — a payload
  truncated below the 12 bytes of CDR encapsulation plus stamp, or a receive
  time outside the `builtin_interfaces/Time` range (negative, or seconds
  overflowing int32) — is copied verbatim; the run succeeds and logs a warning
  with the count.

### In-place vs `-o`

- Without `-o`, `<input>` is rewritten via an atomic tmp-swap that preserves
  its storage format and layout. With `-o`, `<input>` is left untouched and
  the result is written to that path; the output's storage follows the output
  extension (`.mcap` / `.db3` pick a single-file backend) or, for a directory
  output, resolves from the output path.
- In-place rewriting requires an uncompressed input. A directory bag whose
  `metadata.yaml` declares `compression_mode: file` is rejected with `could
not detect storage format of input bag`; pass an explicit `-o` output for
  those.
- The bag is re-encoded message by message (nearly every message changes, so
  there is no chunk pass-through), and an MCAP output is written with
  `compression=none`; re-compress afterwards with `ros2 bag convert` if
  needed.

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
