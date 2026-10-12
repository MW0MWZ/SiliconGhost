// SPDX-License-Identifier: GPL-2.0-or-later
// Intel 8042-compatible keyboard/mouse controller with an attached PS/2
// keyboard and mouse that answer the standard command set. Host input is
// injected as scan codes with key_input()/mouse_input().
#pragma once

#include <deque>
#include <functional>

#include "core/types.h"

namespace sg {

class Kbc8042 {
public:
    using IrqCallback = std::function<void(bool asserted)>;

    Kbc8042();
    void reset();

    u8 read_data();
    u8 read_status() const;
    void write_data(u8 value);
    void write_command(u8 value);

    void key_input(u8 scancode);
    void mouse_input(u8 byte);
    void set_irq_callback(IrqCallback cb) { irq_cb_ = std::move(cb); }
    bool irq() const { return irq_; }

private:
    enum class Pending { None, CommandByte, ToAux, ToOutputKbd, ToOutputAux };

    void push(u8 byte, bool aux);
    void keyboard_command(u8 value);
    void mouse_command(u8 value);
    void update_irq();

    std::deque<std::pair<u8, bool>> out_;  // (byte, from aux device)
    u8 command_byte_ = 0x47;
    bool system_flag_ = false;
    bool last_was_command_ = false;
    Pending pending_ = Pending::None;
    u8 kbd_arg_command_ = 0;    // keyboard command awaiting its argument byte
    u8 mouse_arg_command_ = 0;  // mouse command awaiting its argument byte
    bool irq_ = false;
    IrqCallback irq_cb_;
};

}  // namespace sg
