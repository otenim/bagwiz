// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BASE__TERMINAL_INPUT_HPP_
#define BAGWIZ__CORE__BASE__TERMINAL_INPUT_HPP_

#include <termios.h>

#include <optional>
#include <string_view>

namespace bagwiz::core
{

// High-level events the interactive UI cares about. Raw key bytes (single
// chars or escape sequences) are collapsed into these buckets by
// classify_key().
enum class KeyEvent {
  kNext,                 // next message
  kPrev,                 // previous message
  kFirst,                // jump to first
  kLast,                 // jump to last (may force a full scan in the caller)
  kStepForward1s,        // jump to the next message at least one second ahead
  kStepBackward1s,       // jump to the previous message at least one second behind
  kStepForward10s,       // jump ~10 seconds ahead ('>')
  kStepBackward10s,      // jump ~10 seconds behind ('<')
  kScrollUp,             // scroll the current message's rendered body up by one line
  kScrollDown,           // scroll the current message's rendered body down by one line
  kScrollHead,           // jump to the top of the current message's body
  kScrollTail,           // jump to the bottom of the current message's body
  kSaveYaml,             // save current message body as YAML (walk command)
  kToggleArrayExpand,    // toggle full-expansion of long primitive arrays (walk command)
  kTogglePreview,        // toggle in-terminal image preview (walk command)
  kToggleRectify,        // toggle rectify in image preview (walk command)
  kToggleProjectPcd,     // toggle point-cloud projection overlay in image preview (walk command)
  kSelectPcdTopic,       // choose a PointCloud2 topic for the projection overlay
  kCyclePcdProperty,     // cycle point-cloud visualization property (walk command)
  kCyclePcdScheme,       // cycle point-cloud color scheme (walk command)
  kTogglePcdRange,       // toggle point-cloud value range auto/manual (walk command)
  kPcdPointSizeUp,       // increase projected point size (walk command)
  kPcdPointSizeDown,     // decrease projected point size (walk command)
  kPcdAlphaUp,           // increase point overlay opacity (walk command)
  kPcdAlphaDown,         // decrease point overlay opacity (walk command)
  kToggleEditExtrinsic,  // toggle the extrinsic edit mode in image preview (walk command)
  kSelectEditEdge,       // choose the static TF edge the edit mode nudges
  kEditTransXUp,         // nudge the edited edge's translation x up one step
  kEditTransXDown,       // nudge the edited edge's translation x down one step
  kEditTransYUp,         // nudge the edited edge's translation y up one step
  kEditTransYDown,       // nudge the edited edge's translation y down one step
  kEditTransZUp,         // nudge the edited edge's translation z up one step
  kEditTransZDown,       // nudge the edited edge's translation z down one step
  kEditRollUp,           // nudge the edited edge's roll up one step
  kEditRollDown,         // nudge the edited edge's roll down one step
  kEditPitchUp,          // nudge the edited edge's pitch up one step
  kEditPitchDown,        // nudge the edited edge's pitch down one step
  kEditYawUp,            // nudge the edited edge's yaw up one step
  kEditYawDown,          // nudge the edited edge's yaw down one step
  kEditStepUp,           // increase the edit nudge step size
  kEditStepDown,         // decrease the edit nudge step size
  kEditReset,            // reset the edited edge to its original bag value
  kEditDumpYaml,         // export the edited edges as static-TF YAML (prompts)
  kEditApplyToBag,       // overwrite the input bag's static TF with the edits (prompts)
  kPinScene,             // pin/unpin the displayed frame as a preview scene tile
  kHelp,                 // show the key-help overlay (walk command)
  kBack,                 // back out one level (lone ESC): close the help,
                         // leave the edit mode / preview; inert at the top
  kConfirm,              // confirm the current prompt/selection
  kQuit,                 // quit the current view ('q'/'Q', ^C/^D); the '?'
                         // help overlays swallow it (close with ESC first)
  kResize,               // terminal was resized (synthesised by read_key_event
                         // from a SIGWINCH flag set by the signal_handler
                         // module; never produced by classify_key)
  kUnknown,              // unrecognized input; caller should ignore or beep
};

// Pure classifier for an already-captured byte sequence. Exposed so unit
// tests can exercise every key mapping without touching /dev/tty.
//
// Accepted input:
//   * single bytes: Space (next), 'b' (prev), 'g' (first), 'G' (last),
//     '.' (step forward one second), ',' (step backward one second),
//     '>' (step forward ~10 seconds), '<' (step backward ~10 seconds),
//     'k' (scroll up), 'j' (scroll down), 'H' (scroll head), 'T' (scroll
//     tail), 'S' (save as yaml — walk), 'a' (toggle array expand — walk),
//     'i' (toggle image preview — walk), 'u' (toggle rectify — walk),
//     the point-cloud overlay keys 'p' (toggle overlay), 't' (topic
//     picker), 'f'/'c'/'r' (property / scheme / range), '='/'+' and '-'
//     (point size), ']'/'[' (alpha) — all walk,
//     'e'/'E' (extrinsic edit mode toggle / edge picker — walk), the edit
//     nudges 'x'/'X', 'y'/'Y', 'z'/'Z' (translation up/down), 'l'/'L'
//     (roll), 'n'/'N' (pitch), 'w'/'W' (yaw), 'm'/'M' (step size), '0'
//     (reset edge), 'D' (dump edited edges as YAML — walk),
//     'A' (apply the edited edges to the input bag's static TF — walk),
//     'P' (pin/unpin the displayed frame as a preview scene — walk),
//     '?' (show the key-help overlay — walk),
//     Enter/Return (confirm the current prompt or selection),
//     a lone ESC (0x1B) to back out one level (kBack), and 'q'/'Q' plus
//     the control chars ^C / ^D to quit the current view (kQuit)
//   * three-byte ANSI sequences "ESC [ C" (Right -> next), "ESC [ D"
//     (Left -> prev), "ESC [ A" (Up -> scroll up), "ESC [ B" (Down ->
//     scroll down), "ESC [ H" (Home -> scroll head), "ESC [ F" (End ->
//     scroll tail)
// Anything else -> kUnknown.
KeyEvent classify_key(std::string_view bytes);

// RAII guard that puts STDIN_FILENO into a minimal "cbreak" mode (no echo,
// no line buffering, VMIN=1 VTIME=0). On destruction the previous termios
// is restored. Construction is a no-op when stdin is not a TTY, in which
// case active() returns false.
class TerminalRawMode
{
public:
  TerminalRawMode();
  ~TerminalRawMode();

  TerminalRawMode(const TerminalRawMode &) = delete;
  TerminalRawMode & operator=(const TerminalRawMode &) = delete;
  TerminalRawMode(TerminalRawMode &&) = delete;
  TerminalRawMode & operator=(TerminalRawMode &&) = delete;

  bool active() const { return active_; }

  // Temporarily restore canonical stdin so callers can use line-oriented
  // reads (e.g. std::getline). Pair every call with resume_after_line_input()
  // when raw mode should resume; otherwise the destructor still restores
  // the saved (cooked) settings safely.
  void suspend_for_line_input();
  void resume_after_line_input();

private:
  bool active_ = false;
  termios saved_{};
};

// Blocks on stdin until a single KeyEvent can be produced. Handles the ESC
// prefetch needed to distinguish a lone ESC from an arrow-key sequence.
// Returns kQuit when the read is interrupted (e.g. SIGINT) or on EOF.
// Undefined unless stdin is a TTY.
KeyEvent read_key_event();

// Bounded-wait variant of read_key_event(): waits at most `timeout_ms` for
// the first input byte and returns std::nullopt when none arrives in time
// (a pending resize is still surfaced as kResize). Once a byte arrives the
// call behaves exactly like read_key_event(). Lets interactive loops refresh
// on-screen progress while waiting for keys.
// Undefined unless stdin is a TTY.
std::optional<KeyEvent> read_key_event(int timeout_ms);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BASE__TERMINAL_INPUT_HPP_
