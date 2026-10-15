// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// An 80x24 VT100-style terminal screen, fed with the bytes a serial port
// sends: enough of the VT100/ANSI set for IRIX, its installer and the PROM
// (cursor movement, erasing, scroll regions, insert/delete, attributes, line
// drawing), with a scrollback of the lines that leave the top.
#pragma once

#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "core/types.h"

namespace sg::app {

class Terminal {
public:
    static constexpr int kCols = 80, kRows = 24;
    enum Attr : u8 { kBold = 1, kUnderline = 2, kReverse = 4, kBlink = 8 };
    struct Cell {
        char ch = ' ';  // Latin-1; DEC line drawing comes out as + - |
        u8 attr = 0;
    };
    using Line = std::vector<Cell>;

    explicit Terminal(std::size_t scrollback = 2000);

    void feed(u8 byte);
    void feed(const std::string& text) {
        for (char c : text) feed(static_cast<u8>(c));
    }
    void reset();

    const Cell& at(int row, int col) const { return screen_[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)]; }
    int cursor_row() const { return row_; }
    int cursor_col() const { return col_; }
    bool cursor_visible() const { return cursor_visible_; }
    const std::deque<Line>& scrollback() const { return scrollback_; }
    // Bumped by every change to the screen or scrollback.
    u64 version() const { return version_; }
    // The screen as text, one line per row with trailing spaces removed.
    std::string screen_text() const;

    // What the terminal sends back to the host (cursor position reports).
    void set_reply(std::function<void(const std::string&)> reply) { reply_ = std::move(reply); }

private:
    enum class State { Normal, Escape, Csi, Charset, Hash };
    void put(char c);
    void line_feed();
    void reverse_line_feed();
    void scroll_up(int top, int bottom, int lines);
    void scroll_down(int top, int bottom, int lines);
    void erase(int row, int from, int to);
    void csi(char final);
    int param(std::size_t i, int fallback) const;
    void clamp_cursor();
    Line blank_line() const;

    std::vector<Line> screen_;
    std::deque<Line> scrollback_;
    std::size_t scrollback_limit_;
    int row_ = 0, col_ = 0;
    bool wrap_pending_ = false;
    bool autowrap_ = true;
    bool cursor_visible_ = true;
    bool origin_mode_ = false;
    int top_ = 0, bottom_ = kRows - 1;  // scroll region, inclusive
    u8 attr_ = 0;
    bool g1_graphics_ = false, g0_graphics_ = false, shifted_ = false;
    int saved_row_ = 0, saved_col_ = 0;
    u8 saved_attr_ = 0;

    State state_ = State::Normal;
    std::vector<int> params_;
    bool private_ = false;
    char charset_slot_ = '(';
    u64 version_ = 0;
    std::function<void(const std::string&)> reply_;
};

}  // namespace sg::app
