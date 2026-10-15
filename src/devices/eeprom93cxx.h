// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Microwire serial EEPROM (93C46/56/66 family, x16 organisation), as used for
// the IP22's 93CS56. Driven bit-by-bit through set_lines(); DO is read back
// with data_out(). Writes complete instantly (DO reads "ready").
#pragma once

#include <vector>

#include "core/types.h"

namespace sg {

class Eeprom93Cxx {
public:
    // 93C56 x16: 128 words, 8 address bits (the top bit is a don't-care).
    explicit Eeprom93Cxx(unsigned words = 128, unsigned addr_bits = 8);

    void set_lines(bool cs, bool clk, bool di);
    bool data_out() const { return do_; }

    std::vector<u16>& words() { return words_; }

private:
    enum class State { Idle, Command, ReadOut, WriteData };

    void clock_in(bool bit);
    void execute();

    std::vector<u16> words_;
    unsigned addr_bits_;
    bool cs_ = false, clk_ = false, do_ = true;
    bool write_enabled_ = false;
    State state_ = State::Idle;
    u32 shift_ = 0;
    unsigned nbits_ = 0;
    unsigned opcode_ = 0, address_ = 0;
    u32 out_ = 0;
    int out_bits_ = 0;
};

}  // namespace sg
