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
//             The speed-up is capped (set_max_speed): guest software measures
//             the CPU against its timers at boot, and IRIX 5.3 fails to boot
//             on a CPU much faster than real (seen: 350 million instructions
//             a second boots, 360 does not). Beyond the cap the guest is
//             simply ahead of the wall clock and waits, as in Cycle mode.
//             While the guest times the CPU (calibrating its delay loops at
//             boot) the CPU runs at that cap, so it is never faster later
//             and a calibrated delay never runs short. When the host falls
//             behind (a busy stretch, a stall) the guest catches up by
//             charging more per instruction; only beyond 10 s (the host
//             was suspended) is the lost time given up.
//   Max       No waiting and one cycle per instruction: as fast as possible,
//             with guest time running ahead of the wall clock.
#pragma once

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
    // Realtime mode: the most the guest CPU may be sped up, as a multiple of
    // the real CPU (one cycle per instruction). Default 2.
    void set_max_speed(double factor) { min_cpi_ = factor > 0 ? 1.0 / factor : kMaxCpi; }
    double cycles_per_instruction() const { return cpi_; }

    // Limits on the time charged per instruction in Realtime mode.
    static constexpr double kMaxCpi = 16.0;

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
    double cpi_ = 1.0;
    double min_cpi_ = 0.5;  // see set_max_speed()
};

}  // namespace sg
