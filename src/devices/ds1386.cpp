// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "devices/ds1386.h"

#include <ctime>

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

bool valid_bcd(u8 v, unsigned max) { return (v & 0x0F) <= 9 && (v >> 4) <= 9 && from_bcd(v) <= max; }

}  // namespace

Ds1386::Ds1386(unsigned size_bytes) : mem_(size_bytes, 0) {
    source_ = [] { return static_cast<double>(std::time(nullptr)); };
    mem_[kRegCommand] = 0x80;  // TE set: clock registers track time
    load_registers();
}

void Ds1386::set_time_source(std::function<double()> source) {
    loaded_centis_ = kNotLoaded;
    const double t = now();
    source_ = std::move(source);
    offset_ = t - source_();
}

void Ds1386::power_on() {
    loaded_centis_ = kNotLoaded;
    offset_ = 0;
    frozen_ = false;
    valid_ = true;
    mem_[kRegCommand] |= 0x80;  // TE
    load_registers();
}

void Ds1386::load_registers() {
    const double t = now();
    // Software polls the clock constantly; only re-derive the registers when
    // the time they show (to the hundredth) has moved on.
    const s64 centis = static_cast<s64>(t * 100);
    if (centis == loaded_centis_) return;
    loaded_centis_ = centis;
    s64 days = static_cast<s64>(t) / 86400, secs = static_cast<s64>(t) % 86400;
    if (secs < 0) {
        secs += 86400;
        --days;
    }
    s64 year;
    unsigned month, day;
    civil_from_days(days, year, month, day);
    const unsigned hour = static_cast<unsigned>(secs / 3600), minute = static_cast<unsigned>(secs / 60 % 60);
    const unsigned second = static_cast<unsigned>(secs % 60);
    const unsigned hundredths = static_cast<unsigned>((t - static_cast<double>(static_cast<s64>(t))) * 100) % 100;
    const unsigned dow = static_cast<unsigned>(((days % 7) + 7 + 4) % 7) + 1;  // 1970-01-01 was a Thursday; 1 = Sunday

    mem_[0x0] = to_bcd(hundredths);
    mem_[0x1] = to_bcd(second);
    mem_[0x2] = to_bcd(minute);
    if (mem_[0x4] & 0x40) {  // 12-hour mode
        const unsigned h12 = hour % 12 == 0 ? 12 : hour % 12;
        mem_[0x4] = static_cast<u8>(0x40 | (hour >= 12 ? 0x20 : 0) | to_bcd(h12));
    } else {
        mem_[0x4] = to_bcd(hour);
    }
    mem_[0x6] = static_cast<u8>(dow);
    mem_[0x8] = to_bcd(day);
    mem_[0x9] = static_cast<u8>((mem_[0x9] & 0xC0) | to_bcd(month));
    mem_[0xA] = to_bcd(static_cast<unsigned>(((year - year_base_) % 100 + 100) % 100));
}

bool Ds1386::registers_valid(s64& seconds) const {
    const u8 sec = mem_[0x1] & 0x7F, min = mem_[0x2] & 0x7F, date = mem_[0x8] & 0x3F, mon = mem_[0x9] & 0x1F;
    const u8 yr = mem_[0xA];
    if (!valid_bcd(sec, 59) || !valid_bcd(min, 59) || !valid_bcd(date, 31) || !valid_bcd(mon, 12) ||
        !valid_bcd(yr, 99) || from_bcd(date) == 0 || from_bcd(mon) == 0)
        return false;
    unsigned hour;
    if (mem_[0x4] & 0x40) {
        const u8 h = mem_[0x4] & 0x1F;
        if (!valid_bcd(h, 12) || from_bcd(h) == 0) return false;
        hour = from_bcd(h) % 12 + ((mem_[0x4] & 0x20) ? 12 : 0);
    } else {
        const u8 h = mem_[0x4] & 0x3F;
        if (!valid_bcd(h, 23)) return false;
        hour = from_bcd(h);
    }
    // Two-digit year; with the calendar base, 70..99 -> 19xx, 00..69 -> 20xx.
    const unsigned yy = from_bcd(yr);
    const s64 year = year_base_ ? year_base_ + yy : (yy >= 70 ? 1900 + yy : 2000 + yy);
    const s64 days = days_from_civil(year, from_bcd(mon), from_bcd(date));
    seconds = days * 86400 + hour * 3600 + from_bcd(min) * 60 + from_bcd(sec);
    return true;
}

void Ds1386::commit() {
    loaded_centis_ = kNotLoaded;
    s64 t;
    valid_ = registers_valid(t);
    if (valid_) offset_ = static_cast<double>(t) + from_bcd(mem_[0x0] & 0xFF) / 100.0 - source_();
}

u8 Ds1386::read(unsigned index) {
    index &= static_cast<unsigned>(mem_.size() - 1);  // smaller parts (the DS1286) repeat
    if (index < kNumClockRegs && index != kRegCommand && !frozen_ && valid_) load_registers();
    return mem_[index];
}

void Ds1386::write(unsigned index, u8 value) {
    index &= static_cast<unsigned>(mem_.size() - 1);
    if (index < kNumClockRegs) loaded_centis_ = kNotLoaded;  // registers no longer show the loaded time
    if (index == kRegCommand) {
        const bool te = value & 0x80;
        if (!te && !frozen_) {
            if (valid_) load_registers();
            frozen_ = true;
        } else if (te && frozen_) {
            frozen_ = false;
            commit();
        }
        mem_[index] = static_cast<u8>((value & 0xFC) | (mem_[index] & 0x03));  // TDF and WAF are read-only
        return;
    }
    if (index < kNumClockRegs) {
        // Writes land verbatim; the clock takes them up once they form a valid time.
        if (!frozen_ && valid_) load_registers();
        mem_[index] = value;
        if (!frozen_) commit();
        return;
    }
    if (mem_[index] != value) ++nvram_changes_;
    mem_[index] = value;
}

}  // namespace sg
