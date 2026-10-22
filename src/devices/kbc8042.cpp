// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "devices/kbc8042.h"

#include "core/log.h"

namespace sg {

namespace {
constexpr u8 kAck = 0xFA;
constexpr u8 kSelfTestPassed = 0xAA;

// Command byte bits
constexpr u8 kCmdKbdInt = 0x01;
constexpr u8 kCmdAuxInt = 0x02;
constexpr u8 kCmdSystem = 0x04;
constexpr u8 kCmdKbdDisable = 0x10;
constexpr u8 kCmdAuxDisable = 0x20;
constexpr u8 kCmdTranslate = 0x40;

// The controller's translation of the keyboard's bytes to scan code set 1
// (set 2 codes in, as most systems keep the keyboard in set 2). Bytes from
// 0x80 up pass through, apart from 0x83 (F7).
constexpr u8 kTranslate[128] = {
    0xff, 0x43, 0x41, 0x3f, 0x3d, 0x3b, 0x3c, 0x58, 0x64, 0x44, 0x42, 0x40, 0x3e, 0x0f, 0x29, 0x59,
    0x65, 0x38, 0x2a, 0x70, 0x1d, 0x10, 0x02, 0x5a, 0x66, 0x71, 0x2c, 0x1f, 0x1e, 0x11, 0x03, 0x5b,
    0x67, 0x2e, 0x2d, 0x20, 0x12, 0x05, 0x04, 0x5c, 0x68, 0x39, 0x2f, 0x21, 0x14, 0x13, 0x06, 0x5d,
    0x69, 0x31, 0x30, 0x23, 0x22, 0x15, 0x07, 0x5e, 0x6a, 0x72, 0x32, 0x24, 0x16, 0x08, 0x09, 0x5f,
    0x6b, 0x33, 0x25, 0x17, 0x18, 0x0b, 0x0a, 0x60, 0x6c, 0x34, 0x35, 0x26, 0x27, 0x19, 0x0c, 0x61,
    0x6d, 0x73, 0x28, 0x74, 0x1a, 0x0d, 0x62, 0x6e, 0x3a, 0x36, 0x1c, 0x1b, 0x75, 0x2b, 0x63, 0x76,
    0x55, 0x56, 0x77, 0x78, 0x79, 0x7a, 0x0e, 0x7b, 0x7c, 0x4f, 0x7d, 0x4b, 0x47, 0x7e, 0x7f, 0x6f,
    0x52, 0x53, 0x50, 0x4c, 0x4d, 0x48, 0x01, 0x45, 0x57, 0x4e, 0x51, 0x4a, 0x37, 0x49, 0x46, 0x54,
};

// Status bits
constexpr u8 kStatOutputFull = 0x01;
constexpr u8 kStatSystem = 0x04;
constexpr u8 kStatCommand = 0x08;
constexpr u8 kStatNotInhibited = 0x10;
constexpr u8 kStatAuxData = 0x20;
}  // namespace

Kbc8042::Kbc8042() { reset(); }

void Kbc8042::reset() {
    out_.clear();
    command_byte_ = 0x47;
    system_flag_ = false;
    last_was_command_ = false;
    pending_ = Pending::None;
    kbd_arg_command_ = mouse_arg_command_ = 0;
    scan_set_ = 2;
    key_modes_.fill(kModeTypematic | kModeBreak);
    listing_mode_ = -1;
    translate_break_ = false;
    mouse_reporting_ = false;
    mouse_buttons_ = 0;
    mouse_id_ = 0;
    mouse_rate_ = 100;
    mouse_rates_ = {};
    update_irq();
}

void Kbc8042::set_held(bool held) {
    if (held == held_) return;
    held_ = held;
    reset();
}

u8 Kbc8042::read_status() const {
    if (held_) return 0;
    u8 s = kStatNotInhibited;
    if (!out_.empty()) {
        s |= kStatOutputFull;
        if (out_.front().second) s |= kStatAuxData;
    }
    if (system_flag_ || (command_byte_ & kCmdSystem)) s |= kStatSystem;
    if (last_was_command_) s |= kStatCommand;
    return s;
}

u8 Kbc8042::read_data() {
    if (held_ || out_.empty()) return 0;
    const u8 b = out_.front().first;
    out_.pop_front();
    update_irq();
    return b;
}

void Kbc8042::push(u8 byte, bool aux) {
    out_.emplace_back(byte, aux);
    update_irq();
}

// A byte from the keyboard, translated if the command byte asks: a set 2
// break prefix (F0) is dropped and sets bit 7 of the code after it.
void Kbc8042::push_keyboard(u8 byte) {
    if (command_byte_ & kCmdTranslate) {
        if (byte == 0xF0) {
            translate_break_ = true;
            return;
        }
        byte = byte < 0x80 ? kTranslate[byte] : byte == 0x83 ? 0x41 : byte;
        if (translate_break_) byte |= 0x80;
        translate_break_ = false;
    }
    push(byte, false);
}

void Kbc8042::update_irq() {
    bool pending = false;
    if (!out_.empty()) {
        pending = out_.front().second ? (command_byte_ & kCmdAuxInt) : (command_byte_ & kCmdKbdInt);
    }
    if (pending != irq_) {
        irq_ = pending;
        if (irq_cb_) irq_cb_(irq_);
    }
}

void Kbc8042::write_command(u8 value) {
    if (held_) return;
    last_was_command_ = true;
    switch (value) {
        case 0x20: push(command_byte_, false); break;          // read command byte
        case 0x60: pending_ = Pending::CommandByte; break;      // write command byte
        case 0xA7: command_byte_ |= kCmdAuxDisable; break;     // disable aux
        case 0xA8: command_byte_ &= ~kCmdAuxDisable; break;    // enable aux
        case 0xA9: push(0x00, false); break;                   // aux interface test
        case 0xAA:                                             // controller self-test
            system_flag_ = true;
            push(0x55, false);
            break;
        case 0xAB: push(0x00, false); break;                   // keyboard interface test
        case 0xAD: command_byte_ |= kCmdKbdDisable; break;     // disable keyboard
        case 0xAE: command_byte_ &= ~kCmdKbdDisable; break;    // enable keyboard
        case 0xC0: push(0xFF, false); break;                   // read input port
        case 0xD0: push(0x01, false); break;                   // read output port
        case 0xD1: pending_ = Pending::OutputPort; break;      // write output port: the byte that follows is taken
        case 0xD2: pending_ = Pending::ToOutputKbd; break;
        case 0xD3: pending_ = Pending::ToOutputAux; break;
        case 0xD4: pending_ = Pending::ToAux; break;
        default:
            if (value >= 0xF0) {
                if (!(value & 1)) SG_LOG(Info, "kbc", "system reset pulse requested");
            } else {
                SG_LOG(Debug, "kbc", "unhandled controller command %02x", value);
            }
            break;
    }
    update_irq();
}

void Kbc8042::write_data(u8 value) {
    if (held_) return;
    last_was_command_ = false;
    const Pending p = pending_;
    pending_ = Pending::None;
    switch (p) {
        case Pending::CommandByte:
            command_byte_ = value;
            update_irq();
            return;
        case Pending::ToAux: mouse_command(value); return;
        case Pending::OutputPort: return;  // A20 / reset lines: nothing to do
        case Pending::ToOutputKbd: push(value, false); return;
        case Pending::ToOutputAux: push(value, true); return;
        case Pending::None: keyboard_command(value); return;
    }
}

void Kbc8042::keyboard_command(u8 value) {
    // FB/FC/FD take a list of set 3 key codes, ended by the next command.
    if (listing_mode_ >= 0) {
        if (value < 0xED) {
            key_modes_[value] = static_cast<u8>(listing_mode_);
            push_keyboard(kAck);
            return;
        }
        listing_mode_ = -1;
    }
    if (kbd_arg_command_) {  // argument of a previous command
        const u8 command = kbd_arg_command_;
        kbd_arg_command_ = 0;
        push_keyboard(kAck);
        if (command == 0xF0) {
            if (value == 0) push_keyboard(static_cast<u8>(scan_set_));  // query
            else if (value <= 3) scan_set_ = value;
        }
        return;
    }
    switch (value) {
        case 0xFF:  // reset
            scan_set_ = 2;
            key_modes_.fill(kModeTypematic | kModeBreak);
            push_keyboard(kAck);
            push_keyboard(kSelfTestPassed);
            break;
        case 0xF6:  // set defaults
            key_modes_.fill(kModeTypematic | kModeBreak);
            push_keyboard(kAck);
            break;
        case 0xF7: case 0xF8: case 0xF9: case 0xFA: {  // all keys: typematic / make-break / make only / both
            static constexpr u8 kAll[] = {kModeTypematic, kModeBreak, 0, kModeTypematic | kModeBreak};
            key_modes_.fill(kAll[value - 0xF7]);
            push_keyboard(kAck);
            break;
        }
        case 0xFB: case 0xFC: case 0xFD: {  // listed keys: typematic / make-break / make only
            static constexpr u8 kListed[] = {kModeTypematic, kModeBreak, 0};
            listing_mode_ = kListed[value - 0xFB];
            push_keyboard(kAck);
            break;
        }
        case 0xF2:  // identify
            push_keyboard(kAck);
            push_keyboard(0xAB);
            push_keyboard(0x83);
            break;
        case 0xEE: push_keyboard(0xEE); break;  // echo
        case 0xED:                             // set LEDs
        case 0xF0:                             // select scan code set
        case 0xF3:                             // typematic rate
            kbd_arg_command_ = value;
            push_keyboard(kAck);
            break;
        default: push_keyboard(kAck); break;
    }
}

void Kbc8042::mouse_command(u8 value) {
    if (mouse_arg_command_) {
        const u8 command = mouse_arg_command_;
        mouse_arg_command_ = 0;
        push(kAck, true);
        if (command == 0xF3) {
            // Sample rates 200, 100, 80 in a row switch on the wheel (ID 3);
            // then 200, 200, 80 the fourth and fifth buttons too (ID 4).
            mouse_rate_ = value;
            mouse_rates_ = {mouse_rates_[1], mouse_rates_[2], value};
            if (mouse_rates_ == std::array<u8, 3>{200, 100, 80} && mouse_id_ < 3) mouse_id_ = 3;
            else if (mouse_rates_ == std::array<u8, 3>{200, 200, 80} && mouse_id_ == 3) mouse_id_ = 4;
        }
        return;
    }
    switch (value) {
        case 0xFF:  // reset: back to a standard mouse
            mouse_reporting_ = false;
            mouse_id_ = 0;
            mouse_rate_ = 100;
            mouse_rates_ = {};
            push(kAck, true);
            push(kSelfTestPassed, true);
            push(0x00, true);
            break;
        case 0xF2:  // identify
            push(kAck, true);
            push(mouse_id_, true);
            break;
        case 0xF4:  // enable data reporting
            mouse_reporting_ = true;
            push(kAck, true);
            break;
        case 0xF5:  // disable data reporting
            mouse_reporting_ = false;
            push(kAck, true);
            break;
        case 0xF6:  // set defaults
            mouse_reporting_ = false;
            mouse_rate_ = 100;
            push(kAck, true);
            break;
        case 0xE9:  // status request: stream mode, scaling 1:1, 4 counts/mm, 100 samples/s
            push(kAck, true);
            push(static_cast<u8>((mouse_reporting_ ? 0x20 : 0x00) | (mouse_buttons_ & 1 ? 4 : 0) |
                                 (mouse_buttons_ & 4 ? 2 : 0) | (mouse_buttons_ & 2 ? 1 : 0)),
                 true);
            push(0x02, true);
            push(mouse_rate_, true);
            break;
        case 0xE8:  // set resolution
        case 0xF3:  // set sample rate
            mouse_arg_command_ = value;
            push(kAck, true);
            break;
        default: push(kAck, true); break;
    }
}

void Kbc8042::key_input(u8 scancode) {
    if (!held_ && !(command_byte_ & kCmdKbdDisable)) push_keyboard(scancode);
}

void Kbc8042::mouse_input(u8 byte) {
    if (!held_ && !(command_byte_ & kCmdAuxDisable)) push(byte, true);
}

void Kbc8042::mouse_motion(int dx, int dy, unsigned buttons, int dz) {
    if (held_ || !mouse_reporting_ || (command_byte_ & kCmdAuxDisable)) return;
    buttons &= mouse_id_ == 4 ? 31 : 7;
    if (mouse_id_ < 3) dz = 0;
    do {
        // Movement is a 9-bit two's complement count per packet; the wheel
        // a 4-bit one (-8 to 7) here, as in Explorer mode.
        const int px = dx < -256 ? -256 : dx > 255 ? 255 : dx;
        const int py = dy < -256 ? -256 : dy > 255 ? 255 : dy;
        const int pz = dz < -8 ? -8 : dz > 7 ? 7 : dz;
        dx -= px;
        dy -= py;
        dz -= pz;
        const u8 first = static_cast<u8>(0x08 | (buttons & 7) | (px < 0 ? 0x10 : 0) | (py < 0 ? 0x20 : 0));
        push(first, true);
        push(static_cast<u8>(px & 0xFF), true);
        push(static_cast<u8>(py & 0xFF), true);
        if (mouse_id_ == 3) push(static_cast<u8>(pz & 0xFF), true);
        else if (mouse_id_ == 4) push(static_cast<u8>((pz & 0x0F) | (buttons & 8 ? 0x10 : 0) | (buttons & 16 ? 0x20 : 0)), true);
    } while (dx != 0 || dy != 0 || dz != 0);
    mouse_buttons_ = buttons;
}

}  // namespace sg
