// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "devices/eeprom93cxx.h"

#include "core/log.h"

namespace sg {

Eeprom93Cxx::Eeprom93Cxx(unsigned words, unsigned addr_bits) : words_(words, 0xFFFF), addr_bits_(addr_bits) {}

void Eeprom93Cxx::set_lines(bool cs, bool clk, bool di) {
    if (!cs) {
        cs_ = clk_ = false;
        state_ = State::Idle;
        nbits_ = 0;
        shift_ = 0;
        do_ = true;
        return;
    }
    if (!cs_) {  // CS rising: start a new command, DO shows ready
        state_ = State::Idle;
        nbits_ = 0;
        shift_ = 0;
        do_ = true;
    }
    cs_ = true;
    const bool rising = clk && !clk_;
    clk_ = clk;
    if (rising) clock_in(di);
}

void Eeprom93Cxx::clock_in(bool bit) {
    switch (state_) {
        case State::Idle:
            if (bit) {  // start bit
                state_ = State::Command;
                shift_ = 0;
                nbits_ = 0;
            }
            break;
        case State::Command:
            shift_ = (shift_ << 1) | (bit ? 1 : 0);
            if (++nbits_ == 2 + addr_bits_) {
                opcode_ = shift_ >> addr_bits_;
                address_ = shift_ & ((1u << addr_bits_) - 1);
                execute();
            }
            break;
        case State::ReadOut:
            // Sequential read: after 16 bits, roll into the next word.
            if (out_bits_ == 0) {
                address_ = (address_ + 1) % words_.size();
                out_ = words_[address_];
                out_bits_ = 16;
            }
            do_ = (out_ >> --out_bits_) & 1;
            break;
        case State::WriteData:
            shift_ = (shift_ << 1) | (bit ? 1 : 0);
            if (++nbits_ == 16) {
                const u16 data = static_cast<u16>(shift_);
                if (write_enabled_) {
                    if (opcode_ == 1) words_[address_ % words_.size()] = data;
                    else for (auto& w : words_) w = data;  // WRAL
                }
                state_ = State::Idle;
                do_ = true;  // ready
            }
            break;
    }
}

void Eeprom93Cxx::execute() {
    const unsigned word = address_ % words_.size();
    switch (opcode_) {
        case 2:  // READ: a dummy 0 is presented first, then data MSB first
            out_ = words_[word];
            out_bits_ = 16;
            address_ = word;
            do_ = false;
            state_ = State::ReadOut;
            return;
        case 1:  // WRITE
            shift_ = 0;
            nbits_ = 0;
            state_ = State::WriteData;
            return;
        case 3:  // ERASE
            if (write_enabled_) words_[word] = 0xFFFF;
            state_ = State::Idle;
            return;
        default: {  // extended: top two address bits select the operation
            const unsigned sub = (address_ >> (addr_bits_ - 2)) & 3;
            switch (sub) {
                case 3: write_enabled_ = true; break;   // EWEN
                case 0: write_enabled_ = false; break;  // EWDS
                case 2:                                  // ERAL
                    if (write_enabled_)
                        for (auto& w : words_) w = 0xFFFF;
                    break;
                case 1:  // WRAL
                    shift_ = 0;
                    nbits_ = 0;
                    state_ = State::WriteData;
                    return;
            }
            state_ = State::Idle;
            return;
        }
    }
}

}  // namespace sg
