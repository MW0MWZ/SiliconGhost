// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Microwire serial EEPROM (93CS56, x16 organisation) as used for the IP22's
// NVRAM. Driven bit-by-bit through set_lines(); DO is read back with
// data_out(). Writes complete instantly (DO reads "ready"). With PRE high the
// instructions address the protect register: memory words at or above its
// address cannot be written. There are no erase instructions (WRITE needs
// none), and the PE pin is taken as tied high.
#pragma once

#include <vector>

#include "core/types.h"

namespace sg {

class Eeprom93Cxx {
public:
    // 93C56 x16: 128 words, 8 address bits (the top bit is a don't-care).
    explicit Eeprom93Cxx(unsigned words = 128, unsigned addr_bits = 8);

    void set_lines(bool cs, bool clk, bool di, bool pre = false);
    bool data_out() const { return do_; }

    std::vector<u16>& words() { return words_; }
    const std::vector<u16>& words() const { return words_; }
    // Writes that changed the contents, so the host knows when to save them.
    u64 changes() const { return changes_; }

private:
    enum class State { Idle, Command, ReadOut, WriteData };

    void clock_in(bool bit);
    void execute();
    void execute_protect();
    bool writable(unsigned word) const { return word < protect_; }

    std::vector<u16> words_;
    unsigned addr_bits_;
    bool cs_ = false, clk_ = false, do_ = true;
    bool write_enabled_ = false;
    bool pre_ = false;
    // The protect register: the first protected address; 0xFF (cleared)
    // protects nothing. PREN enables the next protect instruction only;
    // PRDS locks the register for good.
    unsigned protect_ = 0xFF;
    bool protect_enabled_ = false, protect_locked_ = false;
    int out_width_ = 16;  // data bits per read: 16, or 8 for PRREAD
    State state_ = State::Idle;
    u64 changes_ = 0;
    u32 shift_ = 0;
    unsigned nbits_ = 0;
    unsigned opcode_ = 0, address_ = 0;
    u32 out_ = 0;
    int out_bits_ = 0;
};

}  // namespace sg
