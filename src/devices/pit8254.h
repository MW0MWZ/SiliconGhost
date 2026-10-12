// SPDX-License-Identifier: GPL-2.0-or-later
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
    void set_output_callback(OutputCallback cb) { on_output_ = std::move(cb); }

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
};

}  // namespace sg
