// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TUI__RENDERER_HPP_
#define BAGWIZ__CORE__TUI__RENDERER_HPP_

#include <ostream>
#include <string_view>

namespace bagwiz::core::tui
{

// Thin VT100/ECMA-48 ANSI primitives. All functions write into an
// arbitrary ostream so tests can capture the byte stream verbatim;
// production callers pass std::cout.
//
// Escape sequences emitted (parenthesised name = mnemonic in docs):
//   move_cursor(row, col)    -> "\x1B[<row>;<col>H"  (CUP)
//   erase_in_line()          -> "\x1B[2K"            (EL2 - entire line)
//   hide_cursor()            -> "\x1B[?25l"          (DEC private mode 25 off)
//   show_cursor()            -> "\x1B[?25h"          (DEC private mode 25 on)
//   set_autowrap(on)         -> "\x1B[?7h" / "\x1B[?7l"  (DEC private mode 7)
//   enter_alt_screen()       -> "\x1B[?1049h"        (xterm 1049 — alt buffer)
//   leave_alt_screen()       -> "\x1B[?1049l"
//
// `row` and `col` are 1-based. Out-of-range values are not clamped here
// — the caller is responsible for passing values that make sense for
// the active terminal size.

void move_cursor(std::ostream & out, int row, int col);
void erase_in_line(std::ostream & out);
void hide_cursor(std::ostream & out);
void show_cursor(std::ostream & out);
void set_autowrap(std::ostream & out, bool on);
void enter_alt_screen(std::ostream & out);
void leave_alt_screen(std::ostream & out);

// Render one line into the terminal at row `row`, column 1: move the
// cursor, erase the existing line, then write `text` truncated to
// `max_cols` display columns. Does not emit a trailing newline; the
// caller advances rows by issuing another move_cursor(). UTF-8 and
// embedded CSI escape sequences in `text` are preserved by the
// width-aware truncation in width.hpp.
void draw_line(std::ostream & out, int row, std::string_view text, int max_cols);

}  // namespace bagwiz::core::tui

#endif  // BAGWIZ__CORE__TUI__RENDERER_HPP_
