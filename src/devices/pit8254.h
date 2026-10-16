// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Intel 8254 programmable interval timer.
//
// Counters are evaluated lazily from the scheduler clock (CPU cycles) rather
// than ticked; only output transitions are scheduled as events. Register
// interface: reg 0..2 = counter data, reg 3 = control word.
#pragma once

#include <array>
#include <functional>
#include <memory>

#include "core/scheduler.h"
#include "core/types.h"

namespace sg {

class Pit8254 {
public:
    // Called on every OUT pin change.
    using OutputCallback = std::function<void(unsigned counter, bool level)>;

    Pit8254(Scheduler& scheduler, double cpu_hz, std::array<double, 3> clock_hz);

    void reset();
    u8 read(unsigned reg);
    void write(unsigned reg, u8 value);
    void set_gate(unsigned counter, bool level);
    bool output(unsigned counter) const { return counters_[counter].out; }
    // Changes a counter's input clock (e.g. when it is fed by another counter).
    void set_clock(unsigned counter, double hz);
    // Frequency of a counter's OUT pin when it runs periodically (modes 2/3), else 0.
    double output_frequency(unsigned counter) const;
    void set_output_callback(OutputCallback cb) { on_output_ = std::move(cb); }
    // Called when software latches a count to read it (as it does when it
    // times something against a counter).
    void set_latch_callback(std::function<void()> cb) { on_latch_ = std::move(cb); }

private:
    struct Counter {
        u8 mode = 0;      // 0..5
        u8 rw = 3;        // 1 = LSB, 2 = MSB, 3 = LSB then MSB
        bool bcd = false;
        bool gate = true;
        bool out = false;
        bool null_count = true;   // a new count has been written but not loaded
        bool running = false;
        bool armed = false;       // mode 0/1/4/5: terminal count not yet reached
        u32 reload = 0x10000;     // effective count (0 is 65536 binary / 10000 BCD)
        double start_tick = 0;    // tick at which the current count was loaded
        // Mode 2/3: a count written while counting, taken from switch_tick
        // as if loaded at next_start (negative: none).
        u32 next_reload = 0;
        double switch_tick = -1, next_start = 0;
        double tick_base = 0;     // ticks accumulated before cycle_base at earlier clock rates
        u64 cycle_base = 0;
        // Register access state
        bool write_msb_next = false;
        bool read_msb_next = false;
        u8 write_lsb = 0;
        bool latched = false;
        u16 latch = 0;
        bool status_latched = false;
        u8 status = 0;
        std::unique_ptr<Timer> timer;
    };

    double now_ticks(unsigned c) const;
    void take_pending(unsigned c, double now);
    u16 current_count(unsigned c) const;
    void load_count(unsigned c, u16 raw);
    void set_out(unsigned c, bool level);
    void schedule_next(unsigned c);
    void on_timer(unsigned c);
    u8 status_byte(unsigned c) const;

    Scheduler& scheduler_;
    double cpu_hz_;
    std::array<double, 3> clock_hz_;
    std::array<Counter, 3> counters_;
    OutputCallback on_output_;
    std::function<void()> on_latch_;
};

}  // namespace sg
