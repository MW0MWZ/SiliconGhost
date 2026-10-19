// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// A graphics board in the GIO graphics slot, as the machine and the front
// end see it: the slot's bus window, reset, its interrupt, and the picture
// it puts out.
#pragma once

#include <functional>
#include <vector>

#include "core/types.h"
#include "mem/bus.h"

namespace sg::ip22 {

class GfxBoard : public Device {
public:
    // The device the machine maps over the whole graphics slot (the board,
    // or a wrapper that decodes the slot for it).
    virtual Device& slot_device() { return *this; }
    virtual void reset() = 0;

    // The board's interrupt lines to the system: the line (board-defined)
    // and its level. How each reaches the CPU depends on the machine (Indy).
    void set_irq_callback(std::function<void(unsigned, bool)> cb) { irq_cb_ = std::move(cb); }

    // Bumped by anything that can change the picture: a front end can skip
    // render() while it stays the same.
    virtual u64 display_version() const = 0;
    // Scans out the visible screen as 0xAARRGGBB pixels. Returns false while
    // the board puts out no signal. `argb` should hold the previous call's
    // picture: a board may convert only what changed since.
    virtual bool render(std::vector<u32>& argb, unsigned& width, unsigned& height) = 0;
    // The same picture converted from scratch (screenshots).
    virtual bool render_full(std::vector<u32>& argb, unsigned& width, unsigned& height) const = 0;
    virtual u64 frames() const = 0;
    // Where the system has put the hardware cursor: the top left of its
    // glyph in screen coordinates. False while the cursor is off.
    virtual bool cursor_position(int& x, int& y) const = 0;
    // Pixels the board has drawn (a measure of how busy it is).
    virtual u64 pixels_drawn() const = 0;

protected:
    void set_irq(unsigned line, bool level) {
        if (irq_cb_) irq_cb_(line, level);
    }

private:
    std::function<void(unsigned, bool)> irq_cb_;
};

}  // namespace sg::ip22
