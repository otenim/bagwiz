# `bagwiz walk`

Walk the messages of a single topic in a ROS 2 rosbag one at a time and
render each payload as YAML, mirroring `ros2 topic echo`. Designed for
interactive inspection: the view is a pager with vim-style scroll keys,
backed by the reusable TUI SDK (`bagwiz::core::tui`). ROS 1 `*.bag`
inputs are not supported.

## Usage

```text
bagwiz walk -i <input> -t <topic> [OPTIONS]
```

## Examples

```bash
# Walk an IMU topic, paging through its messages one at a time.
bagwiz walk -i capture.mcap -t /sensing/imu/data
```

## Options

| Flag                    | Description                                                                                                                                                                                                                                                                                    |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>` | **Required.** ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`, `*.db3.zstd`). Must exist.                                                                                                                                                                                              |
| `-t`, `--topic <topic>` | **Required.** Topic name to inspect. Must exist in the bag. A literal topic name, not a glob.                                                                                                                                                                                                  |
| `--cam-info <topic>`    | Long-form only. Explicit `sensor_msgs/msg/CameraInfo` topic for the preview's rectify toggle and the PointCloud2 projection overlay. A literal topic name, not a glob. When omitted, bagwiz auto-resolves it from `<topic>` using the rules documented for `bagwiz generate video --cam-info`. |

An explicit empty value (`--cam-info ""`) is a real value, not "omitted": it
skips auto-resolution and is then rejected as not naming a CameraInfo topic,
the same as any other absent topic. Only leaving the flag off entirely
triggers auto-resolution.

## Decoding

Decoding goes through bagwiz's unified decoder factory. For MCAP shards
with a non-empty `ros2msg` schema embedded for the topic's type the
schema-driven backend is used directly. Otherwise (legacy MCAP / SQLite3
inputs) bagwiz falls back to the rosidl introspection typesupport, which
**requires the message package to be installed and on
`AMENT_PREFIX_PATH` at runtime**. If the typesupport library is missing,
bagwiz reports the package name to install and exits non-zero.

## Loading and caching

Messages are loaded lazily and cached as you advance, so `prev` is
always `O(1)` for anything you have already seen. Only `G` (jump to
last) always triggers a full-remaining scan; `.` / `>` read ahead only
as far as the target timestamp, which reaches the end of the topic when
no later message exists.

## Array expansion

By default, primitive arrays with more than 32 elements (e.g. the byte
buffer behind `sensor_msgs/Image.data` or `sensor_msgs/PointCloud2.data`)
are summarized as `[<N items>]` to keep the pager view scannable. Pressing
`a` toggles **full array expansion** for the rest of the walk session.
When expanded, long arrays render as a YAML block sequence (one element
per line under a `-` marker) so the output stays within the terminal
width and remains valid YAML. Short arrays (≤ 32 elements) keep their
inline `[a, b, c]` form either way. The toggle affects both on-screen
rendering and the YAML written by `S`, so saving while expanded produces
a full-fidelity dump of every element. Press `a` again to return to the
summarized view.

## Saving the current message

Pressing `S` saves the **currently displayed** message body (the same
YAML string shown in the pager, not including the header lines) to a
file. The command prompts for an output path; press Enter with an empty
line to write under the process current working directory using the name
`<topic>_<index>.yaml`, where `<topic>` is the ROS topic with each `/`
replaced by `__`, and `<index>` is the same **zero-based** message index
as the first number in the footer line `[<index> / <last>[+]]` (see the
Footer section for `<last>`). Entering an existing directory (or any path
ending in `/`) writes the default name inside it; missing parent directories
are created. Aborting the prompt reports `(save cancelled)`.

## Image preview

For image topics (`sensor_msgs/msg/Image` and
`sensor_msgs/msg/CompressedImage`), pressing `i` toggles an in-terminal
image preview that decodes the current message and draws the real image
in the terminal instead of the YAML byte array. Navigation stays live
in the preview — `→`/`Space` (next), `←`/`b` (prev), `.` (+1s), `,` (-1s), `>`
(+10s), `<` (-10s), `g` (first), `G` (last) re-decode and re-render the new
frame — and the view redraws on resize. Press `q` to return to the YAML view.

- **Supported encodings mirror `bagwiz generate video`:** raw
  `sensor_msgs/msg/Image` in `bgr8`/`rgb8`, and `sensor_msgs/msg/CompressedImage`
  carrying JPEG or PNG (decoded via FFmpeg). Other encodings show a short
  "cannot decode" note in place of the image and you can keep navigating.
- **Supported terminals are graphics-protocol only.** The preview uses the
  **Kitty graphics protocol** (kitty, Ghostty, WezTerm) or **DEC Sixel** (foot,
  Konsole, `xterm` built with sixel support, WezTerm); when a terminal supports
  both, Kitty is preferred. There is **no half-block / ASCII fallback** — on a
  terminal that speaks neither protocol the `[i]` hint is hidden and the YAML
  view is the only view. Capability is detected once at startup (a Kitty
  graphics query plus the Primary Device Attributes reply, where capability `4`
  signals Sixel). Inside `tmux`/`screen` the graphics queries are swallowed (no
  passthrough yet), so preview is reported as unavailable.
- The image is scaled aspect-preserved to fit the body region with a fixed
  padding margin (it never fills edge-to-edge) and is centered. Decoded frames
  are cached (LRU, 16 frames) and shared by the on-screen preview and the `S`
  save, so revisiting a nearby frame reuses the decode; only the
  rectify/overlay compositing is redone per repaint. Frames beyond the cache
  are re-decoded on demand.
- Pressing `u` toggles **rectification** (lens-distortion correction) when a CameraInfo topic was resolved or
  explicitly provided. The rectified frame is rendered and saved by `S`. If no
  CameraInfo is available, `u` shows `rectify: no camera_info` in the status
  line and leaves the original image on screen.
- Pressing `p` toggles a **PointCloud2 projection overlay** on the image
  preview. The first time it is enabled, bagwiz shows a checkbox list of the
  PointCloud2 topics in the bag; you can select one or more topics to project.
  The overlay projects the nearest clouds onto the image using TF and colors
  each point by the selected property. See
  [Point-cloud overlay](#point-cloud-overlay) below for the controls.
- With the overlay active, pressing `e` enters a **static-extrinsic edit
  mode** that nudges a static TF edge on the cloud→camera chain while the
  overlay re-projects live — a visual fixer for camera-lidar
  miscalibration. See
  [Editing static extrinsics](#editing-static-extrinsics).
- The overlay follows the current rectify state: with **rectify off** the
  points are projected onto the raw image using the camera's lens distortion
  (`plumb_bob`/`rational_polynomial`, or `equidistant` for fisheye lenses), and
  with **rectify on** they are projected onto the rectified image. Pressing `u`
  re-aims the overlay accordingly.
- Pressing `i` on a non-image topic shows `(not an image topic)`; pressing it on
  an image topic in an unsupported terminal shows
  `(image preview not supported in this terminal)`.

## Point-cloud overlay

When the image preview is active, `p` toggles a PointCloud2 projection overlay.
The overlay finds the nearest PointCloud2 message to the current image timestamp,
transforms it into the camera frame using the bag's TF data, and projects points
onto the image. Points are colored by the selected property using the current
color scheme and alpha-blended on top of the frame.

The first time the overlay is enabled in a session, bagwiz opens a checkbox
list of every `sensor_msgs/msg/PointCloud2` topic in the bag. Use `↑/↓` (or
`k`/`j`) to move the cursor, `g`/`G` to jump to the first/last topic, `Space`
(or `→`) to check/uncheck a topic, and `Enter` to confirm. You can select any
number of topics; their projected points are drawn together. Press `q` or `Esc`
to cancel without changing the current selection. Confirming a selection
identical to the one already active is treated the same way — the status line
reports `(topic selection unchanged)` and no rescan is started. Confirming an
empty selection from `t` turns the overlay off instead of rescanning. The
selected topics are remembered for the rest of the walk session.

After the selection is confirmed, bagwiz initializes the overlay in the
background: a single pass over the bag decodes the TF topics and reads each
selected cloud's leading `header.stamp` (only that stamp — the point data is
never decoded). While that
pass runs, the preview stays fully usable and the status line shows
`loading pcd overlay ... N%`; the overlay switches on automatically when the
pass finishes. A topic with no messages fails the initialization and is
reported on the status line.

Each frame is paired with the point cloud whose `header.stamp` (sensor capture
time) is nearest the image's own `header.stamp`, rather than the bag record time
— so overlays stay aligned even when recording latency differs between the
camera and lidar, and match what `bagwiz generate video --pcd` renders for the
same frame. If either the camera frame or the point-cloud topic leaves
`header.stamp` unset, that pairing falls back to matching by bag record time on
both sides, so the two are always compared on the same clock rather than mixing
capture time with record time. A topic falls back as a whole: one message
without a stamp makes its timestamp axis a mix of two clocks, which cannot be
searched meaningfully. TF is evaluated at whichever time was matched on, which
keeps the transform lookup on the same clock the TF messages themselves are
stamped with.

In automatic range mode the min/max of the active
property is computed from the clouds actually displayed so far — parsing
every cloud in the bag up front would make initialization expensive — so the
colors can shift during the first frames and then stabilize; `r` pins a
manual range at any time.

| Key             | Action                                                                                                                    |
| --------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `p`             | Toggle the point-cloud overlay on/off. Points project onto the raw or rectified image to match the current rectify state. |
| `t`             | Open the PointCloud2 topic picker again to change the selected topics.                                                    |
| `f`             | Cycle the visualized property: `distance` → `intensity` (when the topic has intensity) → `x` → `y` → `z` → `distance`.    |
| `c`             | Cycle the color scheme: `jet` → `viridis` → `turbo` → `plasma` → `inferno` → `magma` → `rainbow` → `jet`.                 |
| `r`             | Toggle between automatic min/max range and a manual range prompt.                                                         |
| `=` / `+` / `-` | Increase / decrease point size.                                                                                           |
| `]` / `[`       | Increase / decrease overlay alpha (transparency).                                                                         |

Defaults on first enable are: property `distance`, scheme `viridis`, range `auto`,
point size `2`, alpha `1.0`. The info row at the top of the preview (directly
under the topic name) shows the current property, scheme, range mode, point
size, and alpha, alongside the rectify state and any transient status message.
While the overlay is on it also reports how the current frame was paired:
`match: header` for capture time, `match: record` when no capture time was
available on either side, and `match: header->record` when a selected topic
could not supply stamps and forced the whole frame down to record time. Next to
it, `Δ` is the signed gap between the displayed cloud's capture time and the
frame's — the residual misalignment, in milliseconds, of the worst-aligned
selected topic (`n/a` when either side left its stamp unset). Values within
roughly half a lidar period are as tight as the pairing can get; a `Δ` of
several hundred milliseconds means the two sensors' stamps genuinely disagree.
Once a topic is selected, the preview's key legend also lists these adjustment
keys (`f`/`c`/`r`/`=`/`-`/`[`/`]`) and the extrinsic-edit entry (`e`) so they
are discoverable without leaving the TUI.

The overlay is applied to both the on-screen preview and the image saved by `S`.
If no TF data is available, no CameraInfo was resolved, or the selected topic has
no messages near the current frame, the status line reports the failure and the
image is shown without the overlay.

## Editing static extrinsics

With the point-cloud overlay active, pressing `e` enters an interactive
static-extrinsic edit mode: nudge a static TF edge on the chain between the
cloud and the camera while the overlay re-projects live, which turns walk
into a visual fixer for the miscalibration of a camera-lidar extrinsic. The
mode edits a preview-only copy of the TF tree — **the bag is never
modified**; the result is exported as a YAML that
[`bagwiz tf static update`](tf.md#bagwiz-tf-static-update) applies.

The editable candidates are the `(parent, child)` edges on the TF chains
between each selected cloud topic's frame and the camera frame, resolved at
the current frame's TF-lookup time, that a static TF topic (`*/tf_static`)
actually carries — a chain link fed by dynamic TF is not editable, since it
is not tf_static data. With more than one candidate the first `e` opens a
picker (`↑`/`↓`/`k`/`j` move, `g`/`G` jump, `Enter` confirms, `Esc`/`q`
cancels); `E` re-opens it at any time to switch edges, also re-deriving the
candidates so a changed topic selection is picked up. Leaving the mode with
`e` keeps the edits applied to the preview, and re-entering resumes the same
edge. Changing the overlay's topic selection re-reads the bag's TF, and the
edited values are re-applied to the fresh tree automatically.

Edits are expressed in the six scalars of the static-transform-publisher
schema — `x`, `y`, `z` in meters and `roll`, `pitch`, `yaw` in radians (tf2
fixed-axis), the same parametrization
[`tf static dump`](tf.md#bagwiz-tf-static-dump) writes. Each keypress moves
one component by the active step preset; `m`/`M` switch between the three
presets — `0.001 m / 0.0005 rad`, `0.01 m / 0.005 rad` (the default), and
`0.1 m / 0.05 rad`. While the mode is on, the info row shows the active
edge, all six current values (rotations in degrees), the delta from the bag
value for every nudged component, and the step.

| Key                       | Action                                                                     |
| ------------------------- | -------------------------------------------------------------------------- |
| `e`                       | Enter/leave the edit mode (edits stay applied to the preview).             |
| `E`                       | Open the edge picker to choose or switch the edited static TF edge.        |
| `x`/`X`, `y`/`Y`, `z`/`Z` | Nudge that translation component up/down one step.                         |
| `l`/`L`                   | Nudge roll up/down (ro**ll** — `r` is taken by the range toggle).          |
| `n`/`N`                   | Nudge pitch up/down (a **n**od is a pitch — `p` is taken by the overlay).  |
| `w`/`W`                   | Nudge yaw up/down (ya**w** — `y` is taken by the y translation).           |
| `m`/`M`                   | Coarser / finer step preset.                                               |
| `0`                       | Reset the edited edge to the value recorded in the bag.                    |
| `D`                       | Export every edited edge as static-TF YAML (prompts for a path, like `S`). |

All other preview keys stay live while editing — navigate frames to check the
fix elsewhere in the recording, toggle rectification with `u`, or adjust the
overlay's point size and alpha.

### Persisting the fix

`D` writes the edited edges in the nested
`parent -> child -> {x, y, z, roll, pitch, yaw}` YAML that
[`tf static dump`](tf.md#bagwiz-tf-static-dump) produces. The path prompt
follows the same rules as `S`; the default name is
`<bag>_tf_static_edit.yaml`. Apply it to the bag afterwards:

```bash
bagwiz tf static update -i capture.mcap --yaml capture_tf_static_edit.yaml
```

Quitting walk with unexported edits is safe: every edited edge is printed to
stdout after the TUI closes (the same rendering
[`tf static calc`](tf.md#bagwiz-tf-static-calc) uses), together with the
`tf static update` command line that applies it.

## Layout

The visible viewport is split into three pinned regions — the header and
footer are pinned in place and only the body region scrolls:

```text
┌─────────────────────────────────────────────────┐
│ timestamp: ...                                  │ ← header (≥ 3 rows)
│ size:      N bytes                              │
│                                                 │
│ <decoded YAML body — scrolls>                   │ ← body
│ ...                                             │
│                                                 │
│   [i / n+]  /topic  Type    lines X-Y of M      │ ← footer (≥ 4 rows)
│   [keys legend ...]                             │
│   <status hint or blank>                        │
└─────────────────────────────────────────────────┘
```

Header and footer rows are sized to the wrapped content, so on narrow
terminals the key legend or other long lines occupy multiple rows and
the body region shrinks accordingly. The status row is always reserved
(blank when there is no message) so the body never grows or shrinks
underfoot when transient messages like `saved /tmp/x.yaml` or
`(wrapped to first)` appear.

Any body line that does not fit the terminal width is wrapped onto
continuation lines, which inherit the original line's leading whitespace
so YAML nesting stays visually intact. The view also redraws cleanly on
terminal resize.

## Header

Each redraw shows a two-line header (plus a blank separator before the
body):

```text
timestamp: YYYY-MM-DD HH:MM:SS.nnnnnnnnn UTC (<seconds>.<nanos>)
size:      <bytes> bytes
```

## Footer

The footer carries the message index, the topic, the type, the scroll
hint, the key legend, and a status row:

```text
  [<index> / <last>[+]]  <topic>  <type>    lines <X>-<Y> of <M>
  [→/Space] next   [←/b] prev   ...   [q] quit
  <status hint or blank>
```

`<last>` is the index of the last message currently loaded in the cache
(equivalently, the count of loaded messages minus one). The trailing `+`
after `<last>` means the bag has more messages after that index that have
not been read into the cache yet (they get pulled in on demand).

## Keys

| Key                                     | Action                                                                                                                                                                                                                          |
| --------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `→` / `Space`                           | Next message (wraps from last back to first).                                                                                                                                                                                   |
| `←` / `b`                               | Previous message.                                                                                                                                                                                                               |
| `.`                                     | Jump forward to the next message at least one second after the current one.                                                                                                                                                     |
| `,`                                     | Jump backward to the previous message at least one second before the current one.                                                                                                                                               |
| `>`                                     | Jump forward to the next message at least ten seconds after the current one.                                                                                                                                                    |
| `<`                                     | Jump backward to the previous message at least ten seconds before the current one.                                                                                                                                              |
| `↑` / `k`                               | Scroll body up one line.                                                                                                                                                                                                        |
| `↓` / `j`                               | Scroll body down one line.                                                                                                                                                                                                      |
| `Home` / `H`                            | Jump body scroll to the head.                                                                                                                                                                                                   |
| `End` / `T`                             | Jump body scroll to the tail.                                                                                                                                                                                                   |
| `g`                                     | Jump to the first message.                                                                                                                                                                                                      |
| `G`                                     | Jump to the last message (forces a full-remaining scan).                                                                                                                                                                        |
| `S`                                     | Save as yaml - writes the current message body (prompts for path). In the image preview, `S` saves the decoded image including any rectification or point-cloud overlay.                                                        |
| `a`                                     | Toggle full expansion of long primitive arrays (default off).                                                                                                                                                                   |
| `i`                                     | Toggle in-terminal image preview (image topics on a Kitty- or Sixel-capable terminal; hidden otherwise). See [Image preview](#image-preview).                                                                                   |
| `u`                                     | Toggle rectification (lens-distortion correction) in the image preview (when CameraInfo is available). Also re-aims the point-cloud overlay: off projects onto the raw (distorted) image, on projects onto the rectified image. |
| `p`                                     | Toggle PointCloud2 projection overlay in the image preview. See [Point-cloud overlay](#point-cloud-overlay).                                                                                                                    |
| `t`                                     | Open the PointCloud2 topic picker to select or change the overlay topics.                                                                                                                                                       |
| `f`                                     | Cycle the point-cloud overlay property.                                                                                                                                                                                         |
| `c`                                     | Cycle the point-cloud overlay color scheme.                                                                                                                                                                                     |
| `r`                                     | Toggle auto/manual value range for the overlay.                                                                                                                                                                                 |
| `=` / `+` / `-`                         | Increase / decrease overlay point size.                                                                                                                                                                                         |
| `]` / `[`                               | Increase / decrease overlay alpha.                                                                                                                                                                                              |
| `e`                                     | Enter/leave the static-extrinsic edit mode in the image preview (needs the overlay). See [Editing static extrinsics](#editing-static-extrinsics).                                                                               |
| `E`                                     | Open the picker that chooses the edited static TF edge.                                                                                                                                                                         |
| `x`/`X`, `y`/`Y`, `z`/`Z`               | Edit mode: nudge that translation component up/down one step.                                                                                                                                                                   |
| `l`/`L`, `n`/`N`, `w`/`W`               | Edit mode: nudge roll / pitch / yaw up/down one step.                                                                                                                                                                           |
| `m` / `M`                               | Edit mode: coarser / finer nudge step.                                                                                                                                                                                          |
| `0`                                     | Edit mode: reset the edited edge to the bag's value.                                                                                                                                                                            |
| `D`                                     | Export the edited static TF edges as YAML (prompts for a path).                                                                                                                                                                 |
| `q` / `Q` / `Esc` / `Ctrl-C` / `Ctrl-D` | Quit (in the image preview, returns to the YAML view).                                                                                                                                                                          |

When the body is taller than the visible window, a `lines X-Y of N`
indicator is shown above the key legend. Wrapping past the last message
back to the first shows a `(wrapped to first)` status hint.

## Requirements

- `walk` is interactive — both stdin and stdout must be a TTY. Piping the
  output (`bagwiz walk … | less`) exits with an error.

## Environment

These variables affect any bagwiz command that decodes messages (`walk`,
`tf tree`, `convert format`, `traj …`); they are documented here
because `walk` is the most decoder-centric command.

- `BAGWIZ_DECODER`: decoder backend override. When set to `introspection`,
  forces the runtime introspection backend and skips the schema-driven path.
  Any other value (and the default of unset) selects the schema-first
  auto-policy, which falls back to introspection only when the schema
  backend cannot decode a topic. See `bagwiz_msg/src/core/decoder/decoder_factory.cpp`.
- `BAGWIZ_LOG_LEVEL`: minimum severity for diagnostic log lines on stderr,
  one of `debug`, `info`, `warn`, `error`, `fatal` (case-insensitive). Unset
  defaults to `info`. Set it to `debug` to surface the lower-level diagnostics
  that are otherwise suppressed — for example the per-topic decoder backend
  selection (`backend=schema` / `backend=introspection`), which the interactive
  `walk` preview would otherwise never print. An unrecognised value is ignored
  with a warning. See `bagwiz_base/src/core/base/logging.cpp`.

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
