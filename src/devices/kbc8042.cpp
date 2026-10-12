// SPDX-License-Identifier: GPL-2.0-or-later
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
    update_irq();
}

u8 Kbc8042::read_status() const {
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
    if (out_.empty()) return 0;
    const u8 b = out_.front().first;
    out_.pop_front();
    update_irq();
    return b;
}

void Kbc8042::push(u8 byte, bool aux) {
    out_.emplace_back(byte, aux);
    update_irq();
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
        case 0xD1: break;                                      // write output port (byte follows, ignored)
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
    last_was_command_ = false;
    const Pending p = pending_;
    pending_ = Pending::None;
    switch (p) {
        case Pending::CommandByte:
            command_byte_ = value;
            update_irq();
            return;
        case Pending::ToAux: mouse_command(value); return;
        case Pending::ToOutputKbd: push(value, false); return;
        case Pending::ToOutputAux: push(value, true); return;
        case Pending::None: keyboard_command(value); return;
    }
}

void Kbc8042::keyboard_command(u8 value) {
    if (kbd_arg_command_) {  // argument of a previous command
        const u8 command = kbd_arg_command_;
        kbd_arg_command_ = 0;
        push(kAck, false);
        if (command == 0xF0) {
            if (value == 0) push(static_cast<u8>(scan_set_), false);  // query
            else if (value <= 3) scan_set_ = value;
        }
        return;
    }
    switch (value) {
        case 0xFF:  // reset
            scan_set_ = 2;
            push(kAck, false);
            push(kSelfTestPassed, false);
            break;
        case 0xF2:  // identify
            push(kAck, false);
            push(0xAB, false);
            push(0x83, false);
            break;
        case 0xEE: push(0xEE, false); break;  // echo
        case 0xED:                             // set LEDs
        case 0xF0:                             // select scan code set
        case 0xF3:                             // typematic rate
        case 0xFB:
        case 0xFC:
        case 0xFD:
            kbd_arg_command_ = value;
            push(kAck, false);
            break;
        default: push(kAck, false); break;
    }
}

void Kbc8042::mouse_command(u8 value) {
    if (mouse_arg_command_) {
        mouse_arg_command_ = 0;
        push(kAck, true);
        return;
    }
    switch (value) {
        case 0xFF:  // reset
            push(kAck, true);
            push(kSelfTestPassed, true);
            push(0x00, true);
            break;
        case 0xF2:  // identify: standard PS/2 mouse
            push(kAck, true);
            push(0x00, true);
            break;
        case 0xE9:  // status request
            push(kAck, true);
            push(0x00, true);
            push(0x02, true);
            push(0x64, true);
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
    if (!(command_byte_ & kCmdKbdDisable)) push(scancode, false);
}

void Kbc8042::mouse_input(u8 byte) {
    if (!(command_byte_ & kCmdAuxDisable)) push(byte, true);
}

}  // namespace sg
