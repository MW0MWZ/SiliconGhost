// SPDX-License-Identifier: GPL-2.0-or-later
#include "devices/ds1386.h"

namespace sg {

namespace {

u8 to_bcd(unsigned v) { return static_cast<u8>(((v / 10) << 4) | (v % 10)); }
unsigned from_bcd(u8 v) { return (v >> 4) * 10 + (v & 0x0F); }

// Days since 1970-01-01 for a proleptic Gregorian date (H. Hinnant's algorithm).
s64 days_from_civil(s64 y, unsigned m, unsigned d) {
    y -= m <= 2;
    const s64 era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<s64>(doe) - 719468;
}

void civil_from_days(s64 z, s64& y, unsigned& m, unsigned& d) {
    z += 719468;
    const s64 era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    y = static_cast<s64>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp < 10 ? mp + 3 : mp - 9;
    y += m <= 2;
}

}  // namespace

Ds1386::Ds1386(unsigned size_bytes) : mem_(size_bytes, 0) {
    time_source_ = [] { return std::time(nullptr); };
    mem_[kRegCommand] = 0x80;  // TE set: clock registers track time
}

void Ds1386::snapshot() {
    const s64 t = static_cast<s64>(time_source_()) + offset_;
    s64 days = t / 86400, secs = t % 86400;
    if (secs < 0) {
        secs += 86400;
        --days;
    }
    s64 year;
    unsigned month, day;
    civil_from_days(days, year, month, day);
    const unsigned hour = static_cast<unsigned>(secs / 3600), minute = static_cast<unsigned>(secs / 60 % 60);
    const unsigned second = static_cast<unsigned>(secs % 60);
    const unsigned dow = static_cast<unsigned>(((days % 7) + 7 + 4) % 7) + 1;  // 1970-01-01 was a Thursday; 1 = Sunday

    mem_[0x0] = 0;
    mem_[0x1] = to_bcd(second);
    mem_[0x2] = to_bcd(minute);
    // Preserve the 12/24 mode bit; 24-hour format is what IRIX/PROM use.
    if (mem_[0x4] & 0x40) {
        const unsigned h12 = hour % 12 == 0 ? 12 : hour % 12;
        mem_[0x4] = static_cast<u8>(0x40 | (hour >= 12 ? 0x20 : 0) | to_bcd(h12));
    } else {
        mem_[0x4] = to_bcd(hour);
    }
    mem_[0x6] = static_cast<u8>(dow);
    mem_[0x8] = to_bcd(day);
    mem_[0x9] = static_cast<u8>((mem_[0x9] & 0xC0) | to_bcd(month));
    mem_[0xA] = to_bcd(static_cast<unsigned>(year % 100));
}

void Ds1386::commit() {
    unsigned hour;
    if (mem_[0x4] & 0x40) {
        hour = from_bcd(mem_[0x4] & 0x1F) % 12 + ((mem_[0x4] & 0x20) ? 12 : 0);
    } else {
        hour = from_bcd(mem_[0x4] & 0x3F);
    }
    // Two-digit year: 70..99 -> 19xx, 00..69 -> 20xx (IRIX's convention).
    const unsigned yy = from_bcd(mem_[0xA]);
    const s64 year = yy >= 70 ? 1900 + yy : 2000 + yy;
    const s64 days = days_from_civil(year, from_bcd(mem_[0x9] & 0x1F), from_bcd(mem_[0x8] & 0x3F));
    const s64 guest = days * 86400 + hour * 3600 + from_bcd(mem_[0x2] & 0x7F) * 60 + from_bcd(mem_[0x1] & 0x7F);
    offset_ = guest - static_cast<s64>(time_source_());
}

u8 Ds1386::read(unsigned index) {
    if (index >= mem_.size()) return 0xFF;
    if (index < kNumClockRegs && index != kRegCommand && !frozen_) snapshot();
    return mem_[index];
}

void Ds1386::write(unsigned index, u8 value) {
    if (index >= mem_.size()) return;
    if (index == kRegCommand) {
        const bool te = value & 0x80;
        if (!te && !frozen_) {
            snapshot();
            frozen_ = true;
        } else if (te && frozen_) {
            commit();
            frozen_ = false;
        }
        mem_[index] = value;
        return;
    }
    if (index < kNumClockRegs && !frozen_) {
        // Writes with TE=1 still land; apply them immediately.
        snapshot();
        mem_[index] = value;
        commit();
        return;
    }
    mem_[index] = value;
}

}  // namespace sg
