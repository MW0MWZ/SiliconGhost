// SPDX-License-Identifier: GPL-2.0-or-later
#include "devices/pit8254.h"

#include <algorithm>
#include <cmath>

#include "core/log.h"

namespace sg {

Pit8254::Pit8254(Scheduler& scheduler, double cpu_hz, std::array<double, 3> clock_hz)
    : scheduler_(scheduler), cpu_hz_(cpu_hz), clock_hz_(clock_hz) {
    for (unsigned c = 0; c < 3; ++c) {
        counters_[c].timer = std::make_unique<Timer>(scheduler_, [this, c] { on_timer(c); });
    }
    reset();
}

void Pit8254::reset() {
    for (auto& k : counters_) {
        k.timer->cancel();
        auto timer = std::move(k.timer);
        k = Counter{};
        k.timer = std::move(timer);
    }
}

double Pit8254::now_ticks(unsigned c) const {
    return std::floor(static_cast<double>(scheduler_.now()) * clock_hz_[c] / cpu_hz_);
}

void Pit8254::set_out(unsigned c, bool level) {
    Counter& k = counters_[c];
    if (k.out == level) return;
    k.out = level;
    if (on_output_) on_output_(c, level);
}

u16 Pit8254::current_count(unsigned c) const {
    const Counter& k = counters_[c];
    if (!k.running) return static_cast<u16>(k.reload);
    const double e = now_ticks(c) - k.start_tick;
    if (e < 0) return static_cast<u16>(k.reload);
    const u64 el = static_cast<u64>(e);
    const u64 n = k.reload;
    switch (k.mode) {
        case 2: return static_cast<u16>(n - el % n);
        case 3: {
            const u64 half = (n + 1) / 2;
            return static_cast<u16>(std::min<u64>(n, n - 2 * (el % half)) & ~u64{1});
        }
        default: return static_cast<u16>((n - el) & 0xFFFF);  // modes 0/1/4/5 keep wrapping
    }
}

void Pit8254::load_count(unsigned c, u16 raw) {
    Counter& k = counters_[c];
    k.reload = raw ? raw : (k.bcd ? 10000u : 0x10000u);
    if ((k.mode == 2 || k.mode == 3) && k.reload < 2) k.reload = 2;
    k.null_count = false;
    k.start_tick = now_ticks(c) + 1;  // a written count is loaded on the next clock
    switch (k.mode) {
        case 0:
            k.armed = true;
            k.running = true;
            set_out(c, false);
            break;
        case 1:
        case 5:
            k.running = false;  // wait for a gate trigger
            set_out(c, true);
            break;
        case 2:
        case 3:
            k.running = k.gate;
            set_out(c, true);
            break;
        case 4:
            k.armed = true;
            k.running = k.gate;
            set_out(c, true);
            break;
    }
    if (k.bcd) SG_LOG(Warn, "pit", "BCD counting not modelled (counter %u)", c);
    schedule_next(c);
}

void Pit8254::schedule_next(unsigned c) {
    Counter& k = counters_[c];
    if (!k.running) {
        k.timer->cancel();
        return;
    }
    const double now = now_ticks(c);
    const double e = std::max(0.0, now - k.start_tick);
    const u64 n = k.reload;
    const u64 el = static_cast<u64>(e);
    double edge = -1;  // absolute tick of the next OUT transition
    switch (k.mode) {
        case 0:
        case 1:
            if (k.armed) edge = k.start_tick + static_cast<double>(n);
            break;
        case 2: {
            const u64 p = el % n, base = el - p;
            edge = k.start_tick + static_cast<double>(p < n - 1 ? base + n - 1 : base + n);
            break;
        }
        case 3: {
            const u64 p = el % n, base = el - p, high = (n + 1) / 2;
            edge = k.start_tick + static_cast<double>(p < high ? base + high : base + n);
            break;
        }
        case 4:
        case 5:
            if (k.armed) edge = k.start_tick + static_cast<double>(el < n ? n : n + 1);
            break;
    }
    if (edge < 0) {
        k.timer->cancel();
        return;
    }
    u64 cycle = static_cast<u64>(std::ceil(edge * cpu_hz_ / clock_hz_[c]));
    if (cycle <= scheduler_.now()) cycle = scheduler_.now() + 1;
    k.timer->schedule_at(cycle);
}

void Pit8254::on_timer(unsigned c) {
    Counter& k = counters_[c];
    const double e = now_ticks(c) - k.start_tick;
    const u64 n = k.reload;
    if (e >= 0) {
        const u64 el = static_cast<u64>(e);
        switch (k.mode) {
            case 0:
            case 1:
                if (k.armed && el >= n) {
                    k.armed = false;
                    set_out(c, true);
                }
                break;
            case 2: set_out(c, el % n != n - 1); break;
            case 3: set_out(c, el % n < (n + 1) / 2); break;
            case 4:
            case 5:
                if (k.armed && el == n) {
                    set_out(c, false);
                } else if (k.armed && el > n) {
                    k.armed = false;
                    set_out(c, true);
                }
                break;
        }
    }
    schedule_next(c);
}

void Pit8254::set_gate(unsigned c, bool level) {
    Counter& k = counters_[c];
    const bool rising = level && !k.gate;
    k.gate = level;
    if (k.null_count) return;
    switch (k.mode) {
        case 1:
        case 5:
            if (rising) {
                k.start_tick = now_ticks(c) + 1;
                k.armed = true;
                k.running = true;
                if (k.mode == 1) set_out(c, false);
            }
            break;
        case 2:
        case 3:
            if (rising) {
                k.start_tick = now_ticks(c) + 1;
                k.running = true;
            } else if (!level) {
                k.running = false;
                set_out(c, true);
            }
            break;
        case 4: k.running = level; break;
        default: break;
    }
    schedule_next(c);
}

u8 Pit8254::status_byte(unsigned c) const {
    const Counter& k = counters_[c];
    return static_cast<u8>((k.out ? 0x80 : 0) | (k.null_count ? 0x40 : 0) | (k.rw << 4) | (k.mode << 1) |
                           (k.bcd ? 1 : 0));
}

u8 Pit8254::read(unsigned reg) {
    if (reg >= 3) return 0xFF;  // control word is write-only
    Counter& k = counters_[reg];
    if (k.status_latched) {
        k.status_latched = false;
        return k.status;
    }
    const u16 v = k.latched ? k.latch : current_count(reg);
    u8 result;
    switch (k.rw) {
        case 1:
            result = static_cast<u8>(v);
            k.latched = false;
            break;
        case 2:
            result = static_cast<u8>(v >> 8);
            k.latched = false;
            break;
        default:
            result = k.read_msb_next ? static_cast<u8>(v >> 8) : static_cast<u8>(v);
            if (k.read_msb_next) k.latched = false;
            k.read_msb_next = !k.read_msb_next;
            break;
    }
    return result;
}

void Pit8254::write(unsigned reg, u8 value) {
    if (reg == 3) {
        const unsigned sc = value >> 6;
        if (sc == 3) {  // read-back command
            for (unsigned c = 0; c < 3; ++c) {
                if (!(value & (2u << c))) continue;
                Counter& k = counters_[c];
                if (!(value & 0x20) && !k.latched) {
                    k.latch = current_count(c);
                    k.latched = true;
                    k.read_msb_next = false;
                }
                if (!(value & 0x10) && !k.status_latched) {
                    k.status = status_byte(c);
                    k.status_latched = true;
                }
            }
            return;
        }
        Counter& k = counters_[sc];
        const unsigned rw = (value >> 4) & 3;
        if (rw == 0) {  // counter latch command
            if (!k.latched) {
                k.latch = current_count(sc);
                k.latched = true;
                k.read_msb_next = false;
            }
            return;
        }
        k.rw = static_cast<u8>(rw);
        k.mode = static_cast<u8>((value >> 1) & 7);
        if (k.mode > 5) k.mode -= 4;  // 6/7 alias 2/3
        k.bcd = value & 1;
        k.null_count = true;
        k.running = false;
        k.armed = false;
        k.latched = false;
        k.write_msb_next = false;
        k.read_msb_next = false;
        k.timer->cancel();
        set_out(sc, k.mode != 0);
        return;
    }
    if (reg > 3) return;
    Counter& k = counters_[reg];
    switch (k.rw) {
        case 1: load_count(reg, value); break;
        case 2: load_count(reg, static_cast<u16>(value << 8)); break;
        default:
            if (!k.write_msb_next) {
                k.write_lsb = value;
                k.write_msb_next = true;
                if (k.mode == 0) {  // writing the first byte stops a mode 0 count
                    k.running = false;
                    k.timer->cancel();
                }
            } else {
                k.write_msb_next = false;
                load_count(reg, static_cast<u16>(k.write_lsb | (value << 8)));
            }
            break;
    }
}

}  // namespace sg
