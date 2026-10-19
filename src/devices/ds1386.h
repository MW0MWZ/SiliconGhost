// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Dallas DS1386 watchdog timekeeper: BCD real-time clock + battery-backed SRAM.
// Register interface is by byte index (0x00..size-1); time registers are 0x0-0xD,
// the rest is NVRAM. The clock runs from a time source (normally host wall
// time at power-on plus elapsed guest time); setting it in the guest only
// changes an offset and never touches the host. EOSC (months register bit 7)
// set stops the oscillator: the time stands still until it is cleared.
//
// The time of day alarm (registers 3, 5, 7, with their mask bits) and the
// watchdog (registers C and D, a countdown in hundredths that repeats until
// they are accessed; zero disables it) set the command register's TDF and
// WAF, which accessing their registers clears; in pulse mode (PU/LVL) a
// flag reads 1 only for the 3 ms the pulse lasts. The flags are worked out
// from the clock's time when read. The interrupt pins are not modelled
// (nothing on the IP22 is known to take them).
#pragma once

#include <cstdint>
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
    const std::vector<u8>& memory() const { return mem_; }
    // Counts writes that changed the battery-backed RAM (not the clock), so
    // the host can tell when its copy needs saving.
    u64 nvram_changes() const { return nvram_changes_; }
    // After restoring memory(): the clock carries on from the time source
    // (a real battery-backed clock kept running while the machine was off),
    // replacing whatever clock registers were saved.
    void power_on();
    // Seconds since the Unix epoch (UTC), fractional; must not go backwards.
    void set_time_source(std::function<double()> source);
    // The year register holds years since `base` (0: the calendar year's
    // last two digits). IRIX on the IP22 counts from 1940.
    void set_year_base(int base) {
        year_base_ = base;
        loaded_centis_ = kNotLoaded;
    }

private:
    int year_base_ = 0;
    double now() const { return stopped_ ? stopped_at_ : source_() + offset_; }
    void set_now(double t);
    void update_oscillator();  // after the months register changed
    u8 alarm_flags();          // TDF and WAF as they stand now
    void alarm_registers_touched(unsigned index);
    bool alarm_matches(s64 minute) const;
    void load_registers();  // clock registers <- current time
    bool registers_valid(s64& seconds) const;
    void commit();          // current time <- clock registers, if they hold a valid time

    std::vector<u8> mem_;
    std::function<double()> source_;
    double offset_ = 0;   // guest time - source time, seconds
    double alarm_from_ = 0;     // the alarm looks for matches after this time
    double tod_alarm_at_ = -1;  // when the time of day alarm last went off (-1: not since cleared)
    double watchdog_from_ = 0;  // when the watchdog last started counting
    bool stopped_ = false;  // EOSC: the oscillator is off
    double stopped_at_ = 0; // the time it stopped at
    static constexpr s64 kNotLoaded = INT64_MIN;
    s64 loaded_centis_ = kNotLoaded;  // time (1/100 s) the clock registers were last loaded with
    bool frozen_ = false; // TE = 0: registers hold a snapshot
    bool valid_ = true;   // registers describe a real date/time
    u64 nvram_changes_ = 0;
};

}  // namespace sg
