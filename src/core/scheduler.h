// SPDX-License-Identifier: GPL-2.0-or-later
// Event scheduler. The CPU cycle counter is the master clock; devices own
// Timers that fire callbacks at absolute cycle counts. The machine run loop
// executes the CPU up to the next due event, then dispatches events, so no
// device is polled per instruction (and the JIT has a single exit condition).
#pragma once

#include <functional>
#include <vector>

#include "core/types.h"

namespace sg {

class Scheduler;

class Timer {
public:
    using Callback = std::function<void()>;

    Timer(Scheduler& scheduler, Callback callback) : scheduler_(scheduler), callback_(std::move(callback)) {}
    ~Timer();
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;

    void schedule_at(u64 cycle);
    void schedule_in(u64 cycles);
    void cancel();
    bool active() const { return active_; }
    u64 when() const { return when_; }

private:
    friend class Scheduler;
    Scheduler& scheduler_;
    Callback callback_;
    u64 when_ = 0;
    bool active_ = false;
};

class Scheduler {
public:
    static constexpr u64 kNever = ~u64{0};

    // Source of "now": normally the CPU's cycle counter.
    void set_clock(std::function<u64()> now) { now_ = std::move(now); }
    u64 now() const { return now_ ? now_() : 0; }

    // Cycle of the earliest pending event, or kNever.
    u64 next_event() const { return queue_.empty() ? kNever : queue_.front()->when_; }

    // Fire every event due at or before `now`, in time order. Callbacks may
    // (re)schedule timers, including themselves.
    void run_due(u64 now);

private:
    friend class Timer;
    void insert(Timer* t);
    void remove(Timer* t);

    std::function<u64()> now_;
    std::vector<Timer*> queue_;  // sorted by when_; device count is small, so insertion sort is fine
};

}  // namespace sg
