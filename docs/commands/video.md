# `bagwiz video`

Convert image topics to video topics and back. Subcommands:

| Subcommand                       | What it does                                                                                             |
| -------------------------------- | -------------------------------------------------------------------------------------------------------- |
| [`encode`](#bagwiz-video-encode) | Encode `Image` / `CompressedImage` topics into `foxglove_msgs/msg/CompressedVideo` topics (H.264/H.265). |
| [`decode`](#bagwiz-video-decode) | Decode `CompressedVideo` topics back into `CompressedImage` (jpeg/png) or `Image` (bgr8) topics.         |

Both subcommands rewrite the bag: every topic that is not converted is copied
verbatim, and each converted topic is replaced by its output by default
(`--keep-inputs` keeps the source next to it).

---

## `bagwiz video encode`

Encodes each frame of an image topic into one `foxglove_msgs/msg/CompressedVideo`
message, in the shape Foxglove plays back as video: H.264 or H.265 in Annex B
form, one message per frame, every keyframe carrying its parameter sets and no
B-frames, so any message decodes on its own from the last keyframe. A camera
topic typically shrinks by one to two orders of magnitude against raw frames
and by several times against JPEG, while staying viewable in Foxglove.

### Usage

```text
bagwiz video encode -i <input> -t <topic>... [OPTIONS]
```

### Examples

```bash
# Replace a raw camera topic by an H.264 video topic named
# /sensing/camera/front/image_raw/video, written to a new bag.
bagwiz video encode -i drive.mcap -t /sensing/camera/front/image_raw -o drive_video.mcap

# Every camera in one go (quoted so the shell does not expand the glob), in
# place: each /sensing/camera/*/image_raw/compressed becomes its own
# .../image_raw/compressed/video topic.
bagwiz video encode -i drive.mcap -t '/sensing/camera/*/image_raw/compressed'

# Keep the JPEG topic next to the video, pick the video topic's name, and
# use H.265 with a smaller quality target.
bagwiz video encode -i drive.mcap -t /cam/image_raw/compressed --as /cam/video \
  --keep-inputs --codec h265 --crf 26 -o out/

# 4K frames: let the GPU encode when NVENC is available, with a keyframe
# every second at 10 fps.
bagwiz video encode -i drive.mcap -t /cam/image_raw --encoder auto --gop 10 -o out/
```

### Options

| Flag                        | Description                                                                                                                                                                                                                                                                                                                                                                                   |
| --------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`     | **Required.** Input ROS 2 rosbag (file or directory). Must exist.                                                                                                                                                                                                                                                                                                                             |
| `-t`, `--topics <topic>...` | **Required.** The image topics to encode: `sensor_msgs/msg/Image` (encodings `bgr8` and `rgb8`) or `sensor_msgs/msg/CompressedImage` (formats `jpeg` and `png`); each a literal name or a `*` glob resolved against the bag. A topic that is missing, has another type, or carries a frame the encoder cannot take (another encoding, odd dimensions, a size change mid-topic) stops the run. |
| `--as <topic>`              | Name of the video topic to create. Only with a single source topic; with several, drop it and let each output take its derived name. Must not name a topic that survives the rewrite (it may reuse the source's own name when the source is replaced). Long-form only. Default: `<source>/video`.                                                                                             |
| `-o`, `--output <path>`     | Output bag path. When omitted, the input bag is rewritten in place (atomic tmp swap). See [In-place vs `-o`](#in-place-vs--o).                                                                                                                                                                                                                                                                |
| `-w`, `--overwrite`         | Overwrite an existing `-o/--output` path. Has no effect in in-place mode.                                                                                                                                                                                                                                                                                                                     |
| `--keep-inputs`             | Keep the source image topics in the output next to the video topics. Default: replace them.                                                                                                                                                                                                                                                                                                   |
| `--codec <codec>`           | `h264` (widest playback support) or `h265` (smaller at the same quality; Foxglove needs a browser or desktop build with HEVC decoding, see [Playback in Foxglove](#playback-in-foxglove)). Default: `h264`.                                                                                                                                                                                   |
| `--encoder <encoder>`       | `auto` uses NVIDIA NVENC for frames larger than 1080p when this build and a GPU support it, else the CPU encoder (libx264 / libx265); `cpu` and `nvenc` force one. A forced `nvenc` that cannot open stops the run; under `auto` the fallback is logged as a warning. Default: `auto`.                                                                                                        |
| `--preset <preset>`         | Speed/quality preset, by libx264's names: `ultrafast`, `superfast`, `veryfast`, `faster`, `fast`, `medium`, `slow`, `slower`, `veryslow`. libx265 takes the same names; NVENC maps them onto its `p1` (fastest) to `p7` (slowest). Default: `medium`.                                                                                                                                         |
| `--crf <n>`                 | Constant-quality target, `0` (best) to `51` (smallest); NVENC uses it as its constant-quality level. Default: `23` for `h264`, `28` for `h265` (whose scale sits about five points lower for a similar picture).                                                                                                                                                                              |
| `--gop <n>`                 | Keyframe interval in frames: every `n`-th frame is a keyframe a player can start or seek from. A smaller value seeks faster and costs bytes. Must be at least 1. Default: `30`.                                                                                                                                                                                                               |
| `-j`, `--threads <n>`       | Encoder threads, `1` to `256`. Default: the encoder's own choice.                                                                                                                                                                                                                                                                                                                             |

### Frames, stamps and names

- Each source message becomes exactly one video message. The video message's
  `timestamp` and `frame_id` are the source `header.stamp` and
  `header.frame_id` (a source whose `header.stamp` is unset takes its record
  time), and it is written at the source message's record time, so the
  output keeps the input's message order.
- The first frame of a topic fixes the stream: its width and height (both
  must be even, as 4:2:0 video requires) and, for JPEG sources, its color
  range. A later frame of another size stops the run.
- JPEG sources are decoded to their 4:2:0 planes and handed to the encoder
  without a detour through RGB; the stream is then tagged full-range so the
  JPEG levels survive. Raw frames and PNG sources are converted from BGR into
  a limited-range stream.
- The nominal frame rate handed to the encoder's rate control is estimated
  from the topic's message count over the bag's time span; it does not
  affect the stamps written.
- A selected topic that carries no messages is declared in the output
  (empty) and reported with a warning.

### Playback in Foxglove

The output follows the requirements `foxglove_msgs/msg/CompressedVideo`
states for its `format` values, so Foxglove's Image panel plays the topic
as video:

- `h264` / `h265`: Annex B byte streams; each message holds the NAL units of
  exactly one frame; a keyframe message also carries its SPS/PPS (and VPS for
  H.265); no B-frames.
- The topic is declared with the message definition, resolved from an
  installed `foxglove_msgs` package when one is on `$AMENT_PREFIX_PATH` and
  otherwise from a copy embedded in bagwiz, so MCAP outputs stay
  self-describing without the package.
- H.264 plays everywhere Foxglove runs. H.265 depends on the platform's HEVC
  decoder (Safari and most desktop builds have one; Chrome and Firefox on
  Linux commonly do not), so prefer `h264` for bags that other people will
  open.

### In-place vs `-o`

- Without `-o`, `<input>` is rewritten via an atomic tmp-swap that preserves
  its storage format and layout. With `-o`, `<input>` is left untouched and
  the result is written to that path; the output's storage follows the output
  extension (`.mcap` / `.db3` pick a single-file backend) or, for a directory
  output, inherits the input bag's storage backend.
- In-place rewriting requires an uncompressed input. A directory bag whose
  `metadata.yaml` declares `compression_mode: file` is rejected with `could
not detect storage format of input bag`; pass an explicit `-o` output for
  those.
- MCAP output is written uncompressed (`compression: none`); re-compress
  afterwards with [`bagwiz compress`](compress.md) if needed. Embedded
  message schemas are preserved for the copied topics.

---

## `bagwiz video decode`

Decodes each message of a `foxglove_msgs/msg/CompressedVideo` topic (`h264` or
`h265`) back into one image message, as `sensor_msgs/msg/CompressedImage`
(`jpeg` by default, or `png`) or as `sensor_msgs/msg/Image` (`bgr8`), for tools
that read images rather than video.

### Usage

```text
bagwiz video decode -i <input> -t <topic>... [OPTIONS]
```

### Examples

```bash
# Replace a video topic by JPEG frames; /cam/image_raw/video becomes
# /cam/image_raw again.
bagwiz video decode -i drive_video.mcap -t /cam/image_raw/video -o drive_jpeg.mcap

# Raw bgr8 frames under a chosen name, keeping the video topic too.
bagwiz video decode -i drive_video.mcap -t /cam/video --as /cam/image_raw \
  --format raw --keep-inputs -o out/

# Every video topic in place, as smaller JPEGs.
bagwiz video decode -i drive_video.mcap -t '*/video' --quality 80
```

### Options

| Flag                        | Description                                                                                                                                                                                                                                                                                                           |
| --------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`     | **Required.** Input ROS 2 rosbag (file or directory). Must exist.                                                                                                                                                                                                                                                     |
| `-t`, `--topics <topic>...` | **Required.** The `foxglove_msgs/msg/CompressedVideo` topics to decode; each a literal name or a `*` glob resolved against the bag. A topic that is missing or has another type stops the run, as does a topic whose `format` is not `h264` or `h265` (`vp9` and `av1` are not decoded) or changes mid-topic.         |
| `--as <topic>`              | Name of the image topic to create. Only with a single source topic. Must not name a topic that survives the rewrite (it may reuse the source's own name when the source is replaced). Long-form only. Default: the source name without a trailing `/video` (undoing `video encode`'s default), else `<source>/image`. |
| `-o`, `--output <path>`     | Output bag path. When omitted, the input bag is rewritten in place (atomic tmp swap). Same rules as [`video encode`](#in-place-vs--o).                                                                                                                                                                                |
| `-w`, `--overwrite`         | Overwrite an existing `-o/--output` path. Has no effect in in-place mode.                                                                                                                                                                                                                                             |
| `--keep-inputs`             | Keep the source video topics in the output next to the image topics. Default: replace them.                                                                                                                                                                                                                           |
| `--format <format>`         | The message written per frame: `jpeg` or `png` (`sensor_msgs/msg/CompressedImage` with that `format`) or `raw` (`sensor_msgs/msg/Image`, `bgr8`, `step = width * 3`). Default: `jpeg`.                                                                                                                                |
| `--quality <n>`             | JPEG quality for `--format jpeg`, `1` (smallest) to `100` (best). Ignored by the other formats. Default: `90`.                                                                                                                                                                                                        |

### Frames, stamps and names

- Each video message is fed to the decoder as one packet. For streams as
  `video encode` writes them (no B-frames, keyframes carrying their parameter
  sets) every packet decodes to one frame right away; a stream that reorders
  frames still decodes, only with a delay, and frames are then matched to
  their messages in order. Either way each image message carries its own
  video message's `timestamp` as `header.stamp` and its `frame_id`, and is
  written at that message's record time.
- The stream's own range tag decides how levels are converted back, so a
  full-range stream from JPEG sources and a limited-range one from raw frames
  both come back at the original levels.
- A message that carries no data is skipped with a warning. Messages that
  produced no frame by the end of the topic (undecodable, or a stream that
  starts before its first keyframe) are counted in a warning.
- A selected topic that carries no messages is declared in the output
  (empty) and reported with a warning.

---

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
