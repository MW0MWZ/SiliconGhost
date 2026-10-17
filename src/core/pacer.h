// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Ties guest time (the CPU cycle counter, which drives every emulated clock)
// to the host's clock.
//
//   Cycle     Each instruction costs one cycle, so guest time is a pure function
//             of the instructions executed and runs are reproducible. When the
//             guest gets ahead of the wall clock (idle skipping) it waits; when
//             the host is too slow, guest time falls behind.
//   Realtime  Guest time follows the wall clock. The CPU executes as many
//             instructions as the host manages and the time charged per
//             instruction adapts, so a fast host behaves like a faster CPU and a
//             slow one like a slower CPU, while a guest second stays a second.
//             While the guest times the CPU (calibrating its delay loops at
//             boot) the CPU runs at a set speed (set_timed_speed): IRIX 5.3
//             fails to boot when it measures a CPU much faster than real
//             (seen: 350 million instructions a second boots, 360 does not).
//             Delay loops are charged at that speed from then on (see
//             delay_cycles_per_instruction), so a calibrated delay never
//             runs short; everything else runs as fast as the host allows,
//             or up to an optional cap (set_max_speed), beyond which the
//             guest is simply ahead of the wall clock and waits, as in Cycle
//             mode. When the host falls behind (a busy stretch, a stall)
//             the guest catches up by charging more per instruction; only
//             beyond 10 s (the host was suspended) is the lost time given up.
//   Max       No waiting and one cycle per instruction: as fast as possible,
//             with guest time running ahead of the wall clock. Except while
//             a key is held (set_key_held): the guest repeats a held key by
//             its own clock, so guest time is then held to the wall clock.
#pragma once

#include <algorithm>
#include <functional>

#include "core/types.h"

namespace sg {

class Pacer {
public:
    enum class Mode { Cycle, Realtime, Max };

    // Host clock: monotonic nanoseconds and a sleep. Defaults to the system's.
    struct HostClock {
        std::function<u64()> now_ns;
        std::function<void(u64 ns)> sleep_ns;
    };

    Pacer(Mode mode, double cycles_per_second, HostClock clock = {});

    // Call after each slice of emulation with the CPU's counters. Waits if the
    // guest is ahead of the wall clock and returns the cycles per instruction
    // to charge from now on. `timing_cpu`: the guest is measuring the CPU's
    // speed (Realtime mode then holds it at the cap).
    double update(u64 guest_cycles, u64 instructions, bool timing_cpu = false);

    Mode mode() const { return mode_; }
    // Realtime mode, as multiples of the real CPU (one cycle per
    // instruction): the CPU's speed while the guest times it (default 2), and
    // the most it may be sped up otherwise (0, the default: no cap).
    void set_timed_speed(double factor) { timed_speed_ = factor; }
    void set_max_speed(double factor) { max_speed_ = factor; }
    // A key is held down on the guest's keyboard (see Max).
    void set_key_held(bool held) {
        if (held && !key_held_) key_rebase_ = true;
        key_held_ = held;
    }
    double cycles_per_instruction() const { return cpi_; }
    // The least charge for an instruction of a delay loop (see
    // R4000::set_delay_cycles_per_instruction): in Realtime mode the charge
    // while the guest times the CPU; 0 (none) otherwise.
    double delay_cycles_per_instruction() const { return mode_ == Mode::Realtime ? timed_cpi() : 0; }

    // Limits on the time charged per instruction in Realtime mode.
    static constexpr double kMinCpi = 1.0 / 64, kMaxCpi = 16.0;

private:
    void rebase(u64 now_ns, u64 guest_cycles);

    Mode mode_;
    double hz_;
    HostClock clock_;
    bool started_ = false;
    u64 epoch_ns_ = 0, epoch_cycles_ = 0;
    u64 last_ns_ = 0, last_instructions_ = 0;  // end of the previous update, after any wait
    double rate_ = 0;                          // measured host speed, instructions per second
    u64 sample_ran_ = 0, sample_busy_ns_ = 0;  // slices pooled until they make a sample
    double min_cpi() const { return max_speed_ > 0 ? std::clamp(1.0 / max_speed_, kMinCpi, kMaxCpi) : kMinCpi; }
    double timed_cpi() const {
        return std::max(timed_speed_ > 0 ? std::min(1.0 / timed_speed_, kMaxCpi) : 1.0, min_cpi());
    }

    double cpi_ = 1.0;
    double timed_speed_ = 2.0, max_speed_ = 0;  // see set_timed_speed(), set_max_speed()
    bool key_held_ = false, key_rebase_ = false;
};

}  // namespace sg
