// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "core/pacer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace sg {

namespace {

// Beyond this much lag the guest stops trying to catch up (the host was
// suspended, or is simply too slow for Cycle mode) and time is re-based.
constexpr double kMaxLagRealtime = 10.0;  // seconds
constexpr double kMaxLagCycle = 1.0;
// Waits shorter than this are not worth a sleep.
constexpr u64 kMinSleepNs = 500000;
// Slices with fewer instructions than this say little about the host's speed.
constexpr u64 kMinSample = 20000;
// cpu_speed() is the host's speed over about this many of the latest
// instructions run with the CPU busy throughout, each stretch weighing as
// many instructions as it ran: a steady figure that the odd short burst
// (interrupts, code just woken up) barely moves.
constexpr double kSpeedWindow = 200e6;

u64 steady_ns() {
    return static_cast<u64>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

}  // namespace

Pacer::Pacer(Mode mode, double cycles_per_second, HostClock clock)
    : mode_(mode), hz_(cycles_per_second), clock_(std::move(clock)) {
    if (!clock_.now_ns) clock_.now_ns = steady_ns;
    if (!clock_.sleep_ns) clock_.sleep_ns = [](u64 ns) { std::this_thread::sleep_for(std::chrono::nanoseconds(ns)); };
}

void Pacer::rebase(u64 now_ns, u64 guest_cycles) {
    epoch_ns_ = now_ns;
    epoch_cycles_ = guest_cycles;
}

double Pacer::update(u64 guest_cycles, u64 instructions, bool timing_cpu, u64 idle_cycles) {
    u64 now = clock_.now_ns();
    if (!started_) {
        started_ = true;
        rebase(now, guest_cycles);
        last_ns_ = now;
        last_instructions_ = instructions;
        last_cycles_ = guest_cycles;
        last_idle_ = idle_cycles;
        return cpi_;
    }
    if (mode_ == Mode::Free) return cpi_ = 1.0;  // (lag_ stays 0: guest time is meant to run ahead)
    if (guest_cycles < epoch_cycles_) rebase(now, guest_cycles);  // clock replaced (not expected)

    // Host speed while busy: instructions per second of host time spent
    // running (waits excluded). Short slices are pooled until they are long
    // enough to measure; otherwise a high charge per instruction (few
    // instructions per slice) could never be measured down again. Slices
    // mostly idle say nothing: their few instructions are outweighed by the
    // devices' share of the host's time, and would have the CPU, when it
    // wakes, start out as slow as the host appeared.
    const u64 slice_idle = idle_cycles - last_idle_, slice_ran = instructions - last_instructions_;
    if (slice_idle * 2 <= guest_cycles - last_cycles_) {
        sample_ran_ += slice_ran;
        sample_busy_ns_ += now - last_ns_;
    }
    if (slice_idle == 0 && slice_ran > 0 && now > last_ns_) {
        const double ran = static_cast<double>(slice_ran);
        const double rate = ran * 1e9 / static_cast<double>(now - last_ns_);
        const double keep = std::exp(-ran / kSpeedWindow);
        speed_weight_ = speed_weight_ * keep + ran;
        speed_rate_ += (rate - speed_rate_) * ran / speed_weight_;  // instruction-weighted mean
    }
    const bool follows = mode_ != Mode::Cycle;  // guest time follows the wall clock
    if (follows && sample_ran_ >= kMinSample && sample_busy_ns_ > 0) {
        const double sample = static_cast<double>(sample_ran_) * 1e9 / static_cast<double>(sample_busy_ns_);
        rate_ = rate_ > 0 ? 0.8 * rate_ + 0.2 * sample : sample;
        sample_ran_ = sample_busy_ns_ = 0;
    }

    const double wall = static_cast<double>(now - epoch_ns_) * 1e-9;
    const double guest = static_cast<double>(guest_cycles - epoch_cycles_) / hz_;
    const double lag = wall - guest;  // > 0: guest behind
    lag_ = std::max(lag, 0.0);
    if (lag > (follows ? kMaxLagRealtime : kMaxLagCycle)) {
        rebase(now, guest_cycles);
    } else if (lag < 0) {
        const u64 wait = static_cast<u64>(-lag * 1e9);
        if (wait >= kMinSleepNs) {
            clock_.sleep_ns(wait);
            now = clock_.now_ns();
        }
    }

    if (follows && timing_cpu) {
        cpi_ = timed_cpi();
    } else if (follows && rate_ > 0) {
        // Charge each instruction so that the host's speed covers one guest
        // second per wall second, plus a little extra while catching up.
        double cpi = hz_ / rate_;
        if (lag > 0) cpi *= 1.0 + std::min(lag / 0.1, 1.0);
        cpi = std::clamp(cpi, min_cpi(), kMaxCpi);
        // Small corrections are not worth making: a stable charge keeps
        // iteration lengths comparable for idle detection.
        if (std::abs(cpi - cpi_) > 0.02 * cpi_) cpi_ = cpi;
    } else {
        cpi_ = 1.0;
    }
    last_ns_ = now;
    last_instructions_ = instructions;
    last_cycles_ = guest_cycles;
    last_idle_ = idle_cycles;
    return cpi_;
}

void Pacer::set_mode(Mode mode) {
    if (mode == mode_) return;
    mode_ = mode;
    if (started_) rebase(clock_.now_ns(), last_cycles_);
    lag_ = 0;
}

double Pacer::cpu_speed() const {
    if (mode_ == Mode::Free || mode_ == Mode::Cycle) return 1.0;
    if (speed_rate_ <= 0) return 0;
    return 1.0 / std::clamp(hz_ / speed_rate_, min_cpi(), kMaxCpi);
}

}  // namespace sg
