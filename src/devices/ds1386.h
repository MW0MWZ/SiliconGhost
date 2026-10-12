// SPDX-License-Identifier: GPL-2.0-or-later
// Dallas DS1386 watchdog timekeeper: BCD real-time clock + battery-backed SRAM.
// Register interface is by byte index (0x00..size-1); time registers are 0x0-0xD,
// the rest is NVRAM. Guest time = host UTC + an offset, so setting the clock in
// the guest never touches the host.
#pragma once

#include <ctime>
#include <functional>
#include <vector>

#include "core/types.h"

namespace sg {

class Ds1386 {
public:
    static constexpr unsigned kRegCommand = 0x0B;
    static constexpr unsigned kNumClockRegs = 0x0E;

    explicit Ds1386(unsigned size_bytes = 8192);

    u8 read(unsigned index);
    void write(unsigned index, u8 value);

    std::vector<u8>& memory() { return mem_; }  // whole array, for NVRAM persistence
    // Host clock source (seconds since epoch, UTC); overridable for tests.
    void set_time_source(std::function<std::time_t()> source) { time_source_ = std::move(source); }

private:
    void snapshot();  // latch the current time into the clock registers
    void commit();    // turn the clock registers back into an offset

    std::vector<u8> mem_;
    std::function<std::time_t()> time_source_;
    s64 offset_ = 0;            // guest time - host time, seconds
    bool frozen_ = false;       // TE = 0: registers hold a snapshot
};

}  // namespace sg
