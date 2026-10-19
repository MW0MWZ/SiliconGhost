// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "devices/ds1386.h"

#include <cmath>
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
    alarm_from_ = watchdog_from_ = now();
}

void Ds1386::set_time_source(std::function<double()> source) {
    loaded_centis_ = kNotLoaded;
    const double t = now();
    source_ = std::move(source);
    set_now(t);
    alarm_from_ = watchdog_from_ = t;
}

void Ds1386::set_now(double t) {
    if (stopped_) stopped_at_ = t;
    else offset_ = t - source_();
}

void Ds1386::update_oscillator() {
    const bool stop = mem_[0x9] & 0x80;
    if (stop == stopped_) return;
    const double t = now();
    stopped_ = stop;
    set_now(t);
}

// The minute `minute` (minutes since the epoch, in the clock's time) against
// the alarm registers: minutes, hours and day of the week each compared
// unless their mask bit (bit 7) is set; all masked, it goes off every minute.
bool Ds1386::alarm_matches(s64 minute) const {
    const s64 day = minute >= 0 ? minute / 1440 : (minute - 1439) / 1440;
    const unsigned m = static_cast<unsigned>(minute - day * 1440);
    const unsigned hour = m / 60, min = m % 60;
    const unsigned dow = static_cast<unsigned>(((day % 7) + 7 + 4) % 7) + 1;  // 1 = Sunday
    if (!(mem_[0x3] & 0x80) && from_bcd(mem_[0x3] & 0x7F) != min) return false;
    if (!(mem_[0x5] & 0x80)) {
        unsigned h;
        if (mem_[0x5] & 0x40) h = from_bcd(mem_[0x5] & 0x1F) % 12 + ((mem_[0x5] & 0x20) ? 12 : 0);
        else h = from_bcd(mem_[0x5] & 0x3F);
        if (h != hour) return false;
    }
    if (!(mem_[0x7] & 0x80) && (mem_[0x7] & 0x07) != dow) return false;
    return true;
}

u8 Ds1386::alarm_flags() {
    const double t = now();
    const bool pulse = mem_[kRegCommand] & 0x10;
    constexpr double kPulse = 0.003;
    u8 flags = 0;
    // Time of day: checked each second; the first matching minute after the
    // alarm was last cleared sets TDF (a week holds every pattern).
    if (tod_alarm_at_ < 0 && t > alarm_from_) {
        const s64 first = static_cast<s64>(std::floor(alarm_from_ / 60)), last = static_cast<s64>(std::floor(t / 60));
        for (s64 m = first; m <= last && m <= first + 7 * 1440; ++m) {
            if (!alarm_matches(m)) continue;
            // Every minute (all masked) it goes off as the seconds roll over to 00.
            const bool every_minute = (mem_[0x3] & mem_[0x5] & mem_[0x7] & 0x80) != 0;
            const double at = std::max(alarm_from_, static_cast<double>(m * 60));
            if (every_minute && at != static_cast<double>(m * 60)) continue;
            if (at <= t) {
                tod_alarm_at_ = at;
                break;
            }
        }
        if (tod_alarm_at_ < 0) alarm_from_ = t;  // nothing yet: look on from here next time
    }
    if (tod_alarm_at_ >= 0 && (!pulse || t - tod_alarm_at_ < kPulse)) flags |= 0x01;
    // Watchdog: the period counts down from the last access, over and over.
    const double period = from_bcd(mem_[0xD]) + from_bcd(mem_[0xC]) / 100.0;
    if (period > 0 && t - watchdog_from_ >= period) {
        const double since = std::fmod(t - watchdog_from_, period);
        if (!pulse || since < kPulse) flags |= 0x02;
    }
    return flags;
}

void Ds1386::power_on() {
    loaded_centis_ = kNotLoaded;
    stopped_ = false;
    offset_ = 0;
    frozen_ = false;
    valid_ = true;
    mem_[kRegCommand] |= 0x80;  // TE
    update_oscillator();  // a clock saved with its oscillator off stays stopped
    load_registers();
    alarm_from_ = watchdog_from_ = now();  // the alarm and watchdog count from power-on
    tod_alarm_at_ = -1;
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
    if (valid_) set_now(static_cast<double>(t) + from_bcd(mem_[0x0] & 0xFF) / 100.0);
}

u8 Ds1386::read(unsigned index) {
    index &= static_cast<unsigned>(mem_.size() - 1);  // smaller parts (the DS1286) repeat
    if (index == kRegCommand) return static_cast<u8>((mem_[index] & 0xFC) | alarm_flags());
    if (index < kNumClockRegs && !frozen_ && valid_) load_registers();
    const u8 v = mem_[index];
    alarm_registers_touched(index);
    return v;
}

// Reading or writing the time of day alarm registers clears TDF; the
// watchdog's, WAF, and starts its count again.
void Ds1386::alarm_registers_touched(unsigned index) {
    if (index == 0x3 || index == 0x5 || index == 0x7) {
        tod_alarm_at_ = -1;
        alarm_from_ = now();
    } else if (index == 0xC || index == 0xD) {
        watchdog_from_ = now();
    }
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
        if (index == 0x9) update_oscillator();
        if (!frozen_) commit();
        alarm_registers_touched(index);
        return;
    }
    if (mem_[index] != value) ++nvram_changes_;
    mem_[index] = value;
}

}  // namespace sg
