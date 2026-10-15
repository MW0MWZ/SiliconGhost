// SPDX-License-Identifier: GPL-2.0-or-later
#include "app/terminal.h"

#include <algorithm>

namespace sg::app {

namespace {

// DEC special graphics (ESC ( 0) for the characters the default font lacks.
char line_drawing(char c) {
    switch (c) {
        case 'j': case 'k': case 'l': case 'm': case 'n': case 't': case 'u': case 'v': case 'w': return '+';
        case 'q': case 'o': case 'p': case 'r': case 's': return '-';
        case 'x': return '|';
        case 'a': return '#';
        case '`': return '*';
        case 'f': return '\xB0';  // degree
        case 'g': return '\xB1';  // plus/minus
        case '~': return '\xB7';  // bullet
        case 'y': return '<';
        case 'z': return '>';
        default: return c;
    }
}

}  // namespace

Terminal::Terminal(std::size_t scrollback) : scrollback_limit_(scrollback) { reset(); }

Terminal::Line Terminal::blank_line() const { return Line(kCols); }

void Terminal::reset() {
    screen_.assign(kRows, blank_line());
    row_ = col_ = 0;
    wrap_pending_ = false;
    autowrap_ = cursor_visible_ = true;
    origin_mode_ = false;
    top_ = 0;
    bottom_ = kRows - 1;
    attr_ = 0;
    g0_graphics_ = g1_graphics_ = shifted_ = false;
    saved_row_ = saved_col_ = 0;
    saved_attr_ = 0;
    state_ = State::Normal;
    ++version_;
}

std::string Terminal::screen_text() const {
    std::string out;
    for (const Line& line : screen_) {
        std::string s;
        for (const Cell& c : line) s += c.ch;
        s.erase(s.find_last_not_of(' ') + 1);
        out += s + '\n';
    }
    return out;
}

void Terminal::feed(u8 b) {
    ++version_;
    const char c = static_cast<char>(b);
    // Control characters act in any state (as on a VT100), except that ESC
    // and CAN/SUB restart or cancel a sequence.
    if (b < 0x20 && b != 0x1B) {
        switch (b) {
            case 0x07: return;  // bell
            case 0x08:
                if (col_ > 0) --col_;
                wrap_pending_ = false;
                return;
            case 0x09:
                col_ = std::min(kCols - 1, (col_ / 8 + 1) * 8);
                wrap_pending_ = false;
                return;
            case 0x0A: case 0x0B: case 0x0C: line_feed(); return;
            case 0x0D:
                col_ = 0;
                wrap_pending_ = false;
                return;
            case 0x0E: shifted_ = true; return;
            case 0x0F: shifted_ = false; return;
            case 0x18: case 0x1A: state_ = State::Normal; return;
            default: return;
        }
    }
    switch (state_) {
        case State::Normal:
            if (b == 0x1B) state_ = State::Escape;
            else if (b != 0x7F) put(c);
            return;
        case State::Escape:
            state_ = State::Normal;
            switch (c) {
                case '[':
                    state_ = State::Csi;
                    params_.clear();
                    private_ = false;
                    return;
                case '(': case ')':
                    charset_slot_ = c;
                    state_ = State::Charset;
                    return;
                case '#': state_ = State::Hash; return;
                case '7':
                    saved_row_ = row_, saved_col_ = col_, saved_attr_ = attr_;
                    return;
                case '8':
                    row_ = saved_row_, col_ = saved_col_, attr_ = saved_attr_;
                    wrap_pending_ = false;
                    return;
                case 'D': line_feed(); return;
                case 'E':
                    col_ = 0;
                    line_feed();
                    return;
                case 'M': reverse_line_feed(); return;
                case 'c': reset(); return;
                case 0x1B: state_ = State::Escape; return;
                default: return;  // keypad modes and the rest: ignored
            }
        case State::Charset:
            (charset_slot_ == '(' ? g0_graphics_ : g1_graphics_) = c == '0';
            state_ = State::Normal;
            return;
        case State::Hash: state_ = State::Normal; return;
        case State::Csi:
            if (c >= '0' && c <= '9') {
                if (params_.empty()) params_.push_back(0);
                params_.back() = std::min(9999, params_.back() * 10 + (c - '0'));
            } else if (c == ';') {
                if (params_.empty()) params_.push_back(0);
                params_.push_back(0);
            } else if (c == '?') {
                private_ = true;
            } else if (b == 0x1B) {
                state_ = State::Escape;
            } else if (c >= 0x40 && c <= 0x7E) {
                state_ = State::Normal;
                csi(c);
            }
            return;
    }
}

void Terminal::put(char c) {
    if ((shifted_ ? g1_graphics_ : g0_graphics_) && c >= 0x5F && c <= 0x7E) c = line_drawing(c);
    if (wrap_pending_) {
        col_ = 0;
        line_feed();
    }
    screen_[static_cast<std::size_t>(row_)][static_cast<std::size_t>(col_)] = {c, attr_};
    if (col_ == kCols - 1) wrap_pending_ = autowrap_;
    else ++col_;
}

void Terminal::line_feed() {
    wrap_pending_ = false;
    if (row_ == bottom_) scroll_up(top_, bottom_, 1);
    else if (row_ < kRows - 1) ++row_;
}

void Terminal::reverse_line_feed() {
    wrap_pending_ = false;
    if (row_ == top_) scroll_down(top_, bottom_, 1);
    else if (row_ > 0) --row_;
}

void Terminal::scroll_up(int top, int bottom, int lines) {
    lines = std::min(lines, bottom - top + 1);
    for (int i = 0; i < lines; ++i) {
        // Only lines leaving the top of the whole screen are kept.
        if (top == 0 && bottom == kRows - 1 && scrollback_limit_) {
            scrollback_.push_back(screen_[0]);
            if (scrollback_.size() > scrollback_limit_) scrollback_.pop_front();
        }
        screen_.erase(screen_.begin() + top);
        screen_.insert(screen_.begin() + bottom, blank_line());
    }
}

void Terminal::scroll_down(int top, int bottom, int lines) {
    lines = std::min(lines, bottom - top + 1);
    for (int i = 0; i < lines; ++i) {
        screen_.erase(screen_.begin() + bottom);
        screen_.insert(screen_.begin() + top, blank_line());
    }
}

void Terminal::erase(int row, int from, int to) {
    for (int c = std::max(0, from); c <= std::min(kCols - 1, to); ++c)
        screen_[static_cast<std::size_t>(row)][static_cast<std::size_t>(c)] = {' ', 0};
}

int Terminal::param(std::size_t i, int fallback) const {
    return i < params_.size() && params_[i] != 0 ? params_[i] : fallback;
}

void Terminal::clamp_cursor() {
    row_ = std::clamp(row_, 0, kRows - 1);
    col_ = std::clamp(col_, 0, kCols - 1);
    wrap_pending_ = false;
}

void Terminal::csi(char final) {
    const int n = param(0, 1);
    if (private_) {
        if (final == 'h' || final == 'l') {
            const bool on = final == 'h';
            for (int p : params_) {
                if (p == 25) cursor_visible_ = on;
                else if (p == 7) autowrap_ = on;
                else if (p == 6) {
                    origin_mode_ = on;
                    row_ = on ? top_ : 0;
                    col_ = 0;
                }
            }
        }
        return;
    }
    switch (final) {
        case 'A': row_ = std::max(row_ >= top_ ? top_ : 0, row_ - n); clamp_cursor(); return;
        case 'B': row_ = std::min(row_ <= bottom_ ? bottom_ : kRows - 1, row_ + n); clamp_cursor(); return;
        case 'C': col_ += n; clamp_cursor(); return;
        case 'D': col_ -= n; clamp_cursor(); return;
        case 'E': row_ += n; col_ = 0; clamp_cursor(); return;
        case 'F': row_ -= n; col_ = 0; clamp_cursor(); return;
        case 'G': case '`': col_ = n - 1; clamp_cursor(); return;
        case 'd': row_ = n - 1 + (origin_mode_ ? top_ : 0); clamp_cursor(); return;
        case 'H': case 'f':
            row_ = param(0, 1) - 1 + (origin_mode_ ? top_ : 0);
            col_ = param(1, 1) - 1;
            clamp_cursor();
            return;
        case 'J': {
            const int mode = params_.empty() ? 0 : params_[0];
            if (mode == 0) {
                erase(row_, col_, kCols - 1);
                for (int r = row_ + 1; r < kRows; ++r) erase(r, 0, kCols - 1);
            } else if (mode == 1) {
                erase(row_, 0, col_);
                for (int r = 0; r < row_; ++r) erase(r, 0, kCols - 1);
            } else {
                for (int r = 0; r < kRows; ++r) erase(r, 0, kCols - 1);
            }
            return;
        }
        case 'K': {
            const int mode = params_.empty() ? 0 : params_[0];
            if (mode == 0) erase(row_, col_, kCols - 1);
            else if (mode == 1) erase(row_, 0, col_);
            else erase(row_, 0, kCols - 1);
            return;
        }
        case 'L':
            if (row_ >= top_ && row_ <= bottom_) scroll_down(row_, bottom_, n);
            return;
        case 'M':
            if (row_ >= top_ && row_ <= bottom_) scroll_up(row_, bottom_, n);
            return;
        case 'P': {
            auto& line = screen_[static_cast<std::size_t>(row_)];
            const int k = std::min(n, kCols - col_);
            line.erase(line.begin() + col_, line.begin() + col_ + k);
            line.insert(line.end(), static_cast<std::size_t>(k), Cell{});
            return;
        }
        case '@': {
            auto& line = screen_[static_cast<std::size_t>(row_)];
            const int k = std::min(n, kCols - col_);
            line.insert(line.begin() + col_, static_cast<std::size_t>(k), Cell{});
            line.resize(kCols);
            return;
        }
        case 'X': erase(row_, col_, col_ + n - 1); return;
        case 'm':
            if (params_.empty()) attr_ = 0;
            for (int p : params_) {
                if (p == 0) attr_ = 0;
                else if (p == 1) attr_ |= kBold;
                else if (p == 4) attr_ |= kUnderline;
                else if (p == 5) attr_ |= kBlink;
                else if (p == 7) attr_ |= kReverse;
                else if (p == 22) attr_ &= ~kBold;
                else if (p == 24) attr_ &= ~kUnderline;
                else if (p == 25) attr_ &= ~kBlink;
                else if (p == 27) attr_ &= ~kReverse;
            }
            return;
        case 'r': {
            const int t = param(0, 1) - 1, b = param(1, kRows) - 1;
            if (t < b && b < kRows) {
                top_ = t;
                bottom_ = b;
                row_ = origin_mode_ ? top_ : 0;
                col_ = 0;
                wrap_pending_ = false;
            }
            return;
        }
        case 's':
            saved_row_ = row_, saved_col_ = col_;
            return;
        case 'u':
            row_ = saved_row_, col_ = saved_col_;
            wrap_pending_ = false;
            return;
        case 'n':
            if (params_.size() == 1 && params_[0] == 6 && reply_)
                reply_("\x1B[" + std::to_string(row_ + 1) + ";" + std::to_string(col_ + 1) + "R");
            else if (params_.size() == 1 && params_[0] == 5 && reply_)
                reply_("\x1B[0n");
            return;
        case 'c':
            if (reply_) reply_("\x1B[?1;2c");  // a VT100 with advanced video
            return;
        default: return;
    }
}

}  // namespace sg::app
