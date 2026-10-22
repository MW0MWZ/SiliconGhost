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
//             boot) the CPU runs at the real CPU's speed, and delay loops
//             run at it throughout (see delay_cycles_per_instruction): the
//             guest calibrates its delays by timing one, and at the real
//             speed it times enough of it for the timer's resolution, so its
//             delays come out their real length and never short.
//             Everything else runs as fast as the host allows. When the
//             host falls behind (a busy stretch, a stall)
//             the guest catches up by charging more per instruction; only
//             beyond 10 s (the host was suspended) is the lost time given up.
//   Max       As Realtime, with disk and Ethernet at the host's speed rather than their
//             real rates (see Indy::set_fast_io). The real speed while the
//             guest times the CPU, and the charge for delay loops, stay: they
//             keep the guest's delays their real length (NetBSD's SCSI driver
//             times out without them) and cost no speed elsewhere.
//   Free      No waiting and one cycle per instruction: guest time runs
//             ahead of the wall clock. For runs of a set length and
//             benchmarks, where the wall clock does not matter.
#pragma once

#include <algorithm>
#include <functional>

#include "core/types.h"

namespace sg {

class Pacer {
public:
    enum class Mode { Cycle, Realtime, Max, Free };

    // Host clock: monotonic nanoseconds and a sleep. Defaults to the system's.
    struct HostClock {
        std::function<u64()> now_ns;
        std::function<void(u64 ns)> sleep_ns;
    };

    Pacer(Mode mode, double cycles_per_second, HostClock clock = {});

    // Call after each slice of emulation with the CPU's counters. Waits if the
    // guest is ahead of the wall clock and returns the cycles per instruction
    // to charge from now on. `timing_cpu`: the guest is measuring the CPU's
    // speed (Realtime mode then holds it at the real CPU's). `idle_cycles`:
    // the CPU's count of cycles skipped while idle.
    double update(u64 guest_cycles, u64 instructions, bool timing_cpu = false, u64 idle_cycles = 0);

    Mode mode() const { return mode_; }
    // Switches mode while running (a turbo switch: Realtime or Cycle to Max
    // and back). Guest time carries on from where it is, without catching
    // up or waiting for any difference the old mode left.
    void set_mode(Mode mode);
    double cycles_per_instruction() const { return cpi_; }
    // How many times as fast as the real CPU the CPU runs when busy, by the
    // host's speed measured over a long stretch of busy running (0 until
    // measured; 1 in Cycle and Free modes). Steadier than the figure that
    // paces the CPU, which follows each change of work.
    double cpu_speed() const;
    // Seconds of guest time behind the wall clock (never set in Free mode,
    // where guest time runs ahead by design).
    double lag() const { return lag_; }
    // The least charge for an instruction of a delay loop (see
    // R4000::set_delay_cycles_per_instruction): in Realtime and Max the real
    // CPU's, one cycle; 0 (none) otherwise. IRIX times
    // 1024 instructions of such a loop against a 1 MHz timer: at twice the
    // real speed that is two or three ticks, and the delays it calibrates
    // from them come out up to four times too long.
    double delay_cycles_per_instruction() const {
        return mode_ == Mode::Realtime || mode_ == Mode::Max ? 1.0 : 0;
    }

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
    u64 last_cycles_ = 0, last_idle_ = 0;
    double rate_ = 0;                          // measured host speed, instructions per second
    double speed_rate_ = 0, speed_weight_ = 0;  // see cpu_speed(): the mean, and instructions behind it
    double lag_ = 0;
    u64 sample_ran_ = 0, sample_busy_ns_ = 0;  // slices pooled until they make a sample

    double cpi_ = 1.0;
};

}  // namespace sg
