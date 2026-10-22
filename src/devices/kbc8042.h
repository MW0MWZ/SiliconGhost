// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Intel 8042-compatible keyboard/mouse controller with an attached PS/2
// keyboard and mouse that answer the standard command set. Host input is
// injected as scan codes with key_input() and as mouse movement with
// mouse_motion().
#pragma once

#include <array>
#include <deque>
#include <functional>

#include "core/types.h"

namespace sg {

class Kbc8042 {
public:
    using IrqCallback = std::function<void(bool asserted)>;

    Kbc8042();
    void reset();
    // Held in reset (the IOC's reset register): it answers nothing, takes
    // no input and drops what it holds; released, it starts afresh.
    void set_held(bool held);

    u8 read_data();
    u8 read_status() const;
    void write_data(u8 value);
    void write_command(u8 value);

    void key_input(u8 scancode);
    // Scan code set the keyboard was told to use (2 after reset, 3 if selected).
    unsigned scan_set() const { return scan_set_; }
    // Set 3 per-key modes: whether a key sends a break code, and whether it
    // repeats (typematic) while held. All keys do both until told otherwise.
    bool sends_break(u8 set3_code) const { return key_modes_[set3_code] & kModeBreak; }
    bool repeats(u8 set3_code) const { return key_modes_[set3_code] & kModeTypematic; }
    void mouse_input(u8 byte);
    // Relative movement (dy > 0 is up, as PS/2 reports it), the buttons held
    // (bit 0 left, 1 right, 2 middle, 3 and 4 the fourth and fifth) and wheel
    // steps (dz > 0 towards the user, as an IntelliMouse reports it). Sent
    // once the guest has enabled reporting, as standard 3-byte packets, or
    // with the wheel (and the fourth and fifth buttons) in a fourth byte
    // once the guest has switched the mouse to IntelliMouse (ID 3) or
    // IntelliMouse Explorer (ID 4) mode; large movements are split.
    void mouse_motion(int dx, int dy, unsigned buttons, int dz = 0);
    // The mouse's ID: 0 standard, 3 wheel, 4 wheel and five buttons.
    u8 mouse_id() const { return mouse_id_; }
    bool mouse_reporting() const { return mouse_reporting_; }
    void set_irq_callback(IrqCallback cb) { irq_cb_ = std::move(cb); }
    bool irq() const { return irq_; }

private:
    static constexpr u8 kModeTypematic = 1, kModeBreak = 2;
    enum class Pending { None, CommandByte, ToAux, ToOutputKbd, ToOutputAux, OutputPort };

    void push(u8 byte, bool aux);
    void push_keyboard(u8 byte);
    void keyboard_command(u8 value);
    void mouse_command(u8 value);
    void update_irq();

    std::deque<std::pair<u8, bool>> out_;  // (byte, from aux device)
    u8 command_byte_ = 0x47;
    bool held_ = false;
    bool system_flag_ = false;
    bool last_was_command_ = false;
    Pending pending_ = Pending::None;
    u8 kbd_arg_command_ = 0;    // keyboard command awaiting its argument byte
    unsigned scan_set_ = 2;
    std::array<u8, 256> key_modes_{};
    int listing_mode_ = -1;  // after FB/FC/FD: the mode the keys that follow get
    bool translate_break_ = false;  // translating: a break prefix was seen
    u8 mouse_arg_command_ = 0;  // mouse command awaiting its argument byte
    bool mouse_reporting_ = false;  // stream mode data reporting enabled (F4)
    unsigned mouse_buttons_ = 0;    // as last reported
    u8 mouse_id_ = 0;               // see mouse_id()
    u8 mouse_rate_ = 100;           // samples per second, as last set
    std::array<u8, 3> mouse_rates_{};  // the last three rates set, newest last: the mode switches
    bool irq_ = false;
    IrqCallback irq_cb_;
};

}  // namespace sg
