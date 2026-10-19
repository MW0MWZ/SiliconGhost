// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "devices/z85230.h"

#include <algorithm>

#include "core/log.h"

namespace sg {

namespace {
constexpr unsigned kRxFifoDepth = 8;
constexpr unsigned kTxFifoDepth = 4;

// RR0
constexpr u8 kRr0RxAvail = 0x01, kRr0ZeroCount = 0x02, kRr0TxEmpty = 0x04, kRr0Dcd = 0x08, kRr0Cts = 0x20,
             kRr0TxUnderrun = 0x40;
// RR1
constexpr u8 kRr1AllSent = 0x01, kRr1Residue = 0x06, kRr1Parity = 0x10, kRr1Overrun = 0x20, kRr1Framing = 0x40;
// WR1
constexpr u8 kWr1ExtIe = 0x01, kWr1TxIe = 0x02, kWr1ParitySpecial = 0x04;
// WR5, WR14, WR15, WR7'
constexpr u8 kWr5TxEnable = 0x08;
constexpr u8 kWr14BrgEnable = 0x01, kWr14AutoEcho = 0x08, kWr14LocalLoopback = 0x10;
constexpr u8 kWr15Wr7Prime = 0x01, kWr15ZeroCountIe = 0x02;
constexpr u8 kWr7pRxHalfFull = 0x08, kWr7pTxFifoEmpty = 0x20, kWr7pExtendedRead = 0x40;
// WR9
constexpr u8 kWr9Mie = 0x08, kWr9StatusHigh = 0x10, kWr9SoftIntack = 0x20;
}  // namespace

Z85230::Z85230(Scheduler& scheduler, u64 char_cycles, double pclk_cycles)
    : scheduler_(scheduler), char_cycles_(char_cycles), pclk_cycles_(pclk_cycles) {
    for (unsigned ch = 0; ch < 2; ++ch) {
        chan_[ch].tx_timer = std::make_unique<Timer>(scheduler_, [this, ch] { tx_done(ch); });
        chan_[ch].zc_timer = std::make_unique<Timer>(scheduler_, [this, ch] {
            // The baud rate generator reached zero: its latch closes the
            // external/status latches (if they are open and it is enabled).
            Channel& c = chan_[ch];
            if (c.ext_latched || !(c.wr[15] & kWr15ZeroCountIe)) return;
            c.ext_latched = true;
            c.ext_latched_rr0 = static_cast<u8>(rr0_live(ch) | kRr0ZeroCount);
            if (c.wr[1] & kWr1ExtIe) c.ext_ip = true;
            update_irq();
        });
    }
    reset();
}

// The reset values are the SCC/ESCC user manual's table; registers it marks
// unchanged (the time constants among them) keep their values.
void Z85230::reset() {
    wr9_ = 0;  // MIE, DLC, NV, VIS, status high and software INTACK clear
    for (unsigned ch = 0; ch < 2; ++ch) channel_reset(ch, true);
}

void Z85230::channel_reset(unsigned ch, bool hardware) {
    Channel& c = chan_[ch];
    c.tx_timer->cancel();
    c.zc_timer->cancel();
    c.wr[0] = 0;
    c.wr[1] &= 0x24;
    c.wr[3] &= 0xFE;  // receiver off
    c.wr[4] |= 0x04;
    c.wr[5] &= 0x61;  // transmitter off, DTR and RTS negated
    c.wr7p = 0x20;
    // WR14: local loopback, auto echo and the DTR/request function clear (and
    // with a hardware reset the generator's enable and source); D7-D5 are
    // DPLL commands, not state.
    if (hardware) {
        c.wr[10] = 0;
        c.wr[11] = 0x08;
        c.wr[14] = 0;
    } else {
        c.wr[10] &= 0x60;
        c.wr[14] &= 0x03;
    }
    c.wr[15] = 0xF8;
    c.pointer = 0;
    c.rx.clear();
    c.rx_locked = false;
    c.rx_first_armed = true;
    c.rr1_latched = 0;
    c.tx.clear();
    c.tx_shifting = false;
    c.tx_ip = false;
    c.tx_underrun = true;
    c.ext_latched = false;
    c.ext_ip = false;
    c.ius.fill(false);
    update_irq();
}

u8 Z85230::read_control(unsigned ch) {
    Channel& c = chan_[ch];
    const unsigned reg = c.pointer;
    c.pointer = 0;
    return read_register(ch, reg);
}

void Z85230::write_control(unsigned ch, u8 value) {
    Channel& c = chan_[ch];
    const unsigned reg = c.pointer;
    c.pointer = 0;
    write_register(ch, reg, value);
}

u8 Z85230::read_data(unsigned ch) { return read_register(ch, 8); }

void Z85230::write_data(unsigned ch, u8 value) { write_register(ch, 8, value); }

// --- status -------------------------------------------------------------------

// RR0 as the pins and state show it now. DCD and CTS are asserted; SYNC is
// not driven and no break arrives; the zero count is momentary.
u8 Z85230::rr0_live(unsigned ch) const {
    const Channel& c = chan_[ch];
    return static_cast<u8>((c.rx.empty() ? 0 : kRr0RxAvail) | (c.tx.size() < kTxFifoDepth ? kRr0TxEmpty : 0) |
                           kRr0Dcd | kRr0Cts | (c.tx_underrun ? kRr0TxUnderrun : 0));
}

// With the external/status latches closed, the bits enabled in WR15 read
// as they were when the latches closed (the zero count always reads live).
u8 Z85230::rr0(unsigned ch) const {
    const Channel& c = chan_[ch];
    const u8 live = rr0_live(ch);
    if (!c.ext_latched) return live;
    const u8 mask = c.wr[15] & 0xF8;
    return static_cast<u8>((live & ~mask) | (c.ext_latched_rr0 & mask));
}

// The exit character's error bits, the latched ones, the residue code (011
// in asynchronous mode) and All Sent.
u8 Z85230::rr1(unsigned ch) const {
    const Channel& c = chan_[ch];
    const u8 exit = c.rx.empty() ? 0 : c.rx.front().status;
    const bool all_sent = !c.tx_shifting && c.tx.empty();
    return static_cast<u8>(kRr1Residue | (all_sent ? kRr1AllSent : 0) | exit | c.rr1_latched);
}

// An enabled external/status condition changed: the latches close on the
// new state and the IP is set (with the master external/status enable),
// unless they are already closed (the change is looked at again when they
// open). The underrun latch interrupts only on being set.
void Z85230::ext_change(unsigned ch, u8 old_rr0) {
    Channel& c = chan_[ch];
    const u8 now = rr0_live(ch);
    u8 changed = static_cast<u8>((old_rr0 ^ now) & c.wr[15] & 0xF8);
    if (!(now & kRr0TxUnderrun)) changed &= static_cast<u8>(~kRr0TxUnderrun);
    if (!changed || c.ext_latched) return;
    c.ext_latched = true;
    c.ext_latched_rr0 = now;
    if (c.wr[1] & kWr1ExtIe) c.ext_ip = true;
    update_irq();
}

// While the zero count can interrupt (the generator running, its enable
// set, the latches open), the next zero is due a time constant + 2 PCLKs on.
// (/RTxC, the other source, is taken as running at PCLK too.)
void Z85230::schedule_zero_count(unsigned ch) {
    Channel& c = chan_[ch];
    if (pclk_cycles_ <= 0 || c.ext_latched || !(c.wr[14] & kWr14BrgEnable) || !(c.wr[15] & kWr15ZeroCountIe)) {
        c.zc_timer->cancel();
        return;
    }
    if (c.zc_timer->active()) return;
    const unsigned tc = c.wr[12] | (c.wr[13] << 8);
    c.zc_timer->schedule_in(static_cast<u64>((tc + 2) * pclk_cycles_) + 1);
}

// --- receive ------------------------------------------------------------------

bool Z85230::rx_special(unsigned ch) const {
    const Channel& c = chan_[ch];
    if (c.rx_locked) return true;
    auto special = [&](const RxEntry& e) {
        return (e.status & (kRr1Overrun | kRr1Framing)) || ((e.status & kRr1Parity) && (c.wr[1] & kWr1ParitySpecial));
    };
    const unsigned mode = (c.wr[1] >> 3) & 3;
    if (mode != 2 || c.rx.empty()) return false;
    // All characters mode: the exit character's condition, or (four-byte
    // mode) one in the four nearest the exit, interrupts at once.
    const std::size_t look = (c.wr7p & kWr7pRxHalfFull) ? std::min<std::size_t>(4, c.rx.size()) : 1;
    for (std::size_t i = 0; i < look; ++i)
        if (special(c.rx[i])) return true;
    return false;
}

bool Z85230::rx_ip(unsigned ch) const {
    const Channel& c = chan_[ch];
    switch ((c.wr[1] >> 3) & 3) {
        case 1: return c.rx_locked || (c.rx_first_armed && !c.rx.empty());
        case 2:
            if (rx_special(ch)) return true;
            return (c.wr7p & kWr7pRxHalfFull) ? c.rx.size() >= 4 : !c.rx.empty();
        case 3: return c.rx_locked;
        default: return false;
    }
}

// A full FIFO takes the new character over its newest one, which is
// flagged as overrun.
void Z85230::rx_push(unsigned ch, u8 byte) {
    Channel& c = chan_[ch];
    if (!(c.wr[3] & 0x01)) return;  // receiver disabled
    const u8 old = rr0_live(ch);
    if (c.rx.size() >= kRxFifoDepth) c.rx.back() = RxEntry{byte, kRr1Overrun};
    else c.rx.push_back(RxEntry{byte, 0});
    ext_change(ch, old);
    update_irq();
}

// Reading pops the exit character and latches its overrun and parity bits.
// In the first-character and special-only modes a character with a special
// condition stays at the exit, locking the FIFO, until Error Reset.
u8 Z85230::rx_read(unsigned ch) {
    Channel& c = chan_[ch];
    if (c.rx.empty()) return 0;
    const RxEntry e = c.rx.front();
    if (c.rx_locked) return e.byte;
    const u8 old = rr0_live(ch);
    c.rr1_latched |= e.status & (kRr1Overrun | kRr1Parity);
    const unsigned mode = (c.wr[1] >> 3) & 3;
    const bool special =
        (e.status & (kRr1Overrun | kRr1Framing)) || ((e.status & kRr1Parity) && (c.wr[1] & kWr1ParitySpecial));
    if (special && (mode == 1 || mode == 3)) c.rx_locked = true;
    else c.rx.pop_front();
    if (mode == 1) c.rx_first_armed = false;
    ext_change(ch, old);
    update_irq();
    return e.byte;
}

void Z85230::receive(unsigned ch, u8 byte) {
    Channel& c = chan_[ch];
    if ((c.wr[14] & kWr14AutoEcho) && tx_sink_) tx_sink_(ch, byte);  // auto echo: RxD goes straight back out on TxD
    if (c.wr[14] & kWr14LocalLoopback) return;  // the receiver hears the transmitter instead
    rx_push(ch, byte);
}

// --- transmit -----------------------------------------------------------------

// The shift register takes the FIFO's exit character while the transmitter
// is enabled. The transmit IP is set as the FIFO empties (WR7' D5) or as
// its entry location frees, unless Reset Tx Int Pending has held it off.
void Z85230::tx_start(unsigned ch) {
    Channel& c = chan_[ch];
    if (c.tx_shifting || c.tx.empty() || !(c.wr[5] & kWr5TxEnable)) return;
    c.tx_byte = c.tx.front();
    c.tx.pop_front();
    c.tx_shifting = true;
    c.tx_timer->schedule_in(char_cycles_);
    const bool level = (c.wr7p & kWr7pTxFifoEmpty) ? c.tx.empty() : c.tx.size() < kTxFifoDepth;
    if ((c.wr[1] & kWr1TxIe) && level && !c.tx_ip_held) c.tx_ip = true;
    update_irq();
}

void Z85230::tx_done(unsigned ch) {
    Channel& c = chan_[ch];
    c.tx_shifting = false;
    tx_output(ch, c.tx_byte);
    const u8 old = rr0_live(ch);
    tx_start(ch);
    if (!c.tx_shifting) c.tx_underrun = true;  // nothing more to send
    ext_change(ch, old);
    update_irq();
}

// Auto echo keeps transmitted data off the line; local loopback hands it to
// the receiver as well.
void Z85230::tx_output(unsigned ch, u8 byte) {
    Channel& c = chan_[ch];
    if (c.wr[14] & kWr14AutoEcho) return;
    if (tx_sink_) tx_sink_(ch, byte);
    if (c.wr[14] & kWr14LocalLoopback) rx_push(ch, byte);
}

// --- interrupts ---------------------------------------------------------------

bool Z85230::ip(unsigned ch, Source s) const {
    const Channel& c = chan_[ch];
    switch (s) {
        case kRx: return rx_ip(ch);
        case kTx: return c.tx_ip && (c.wr[1] & kWr1TxIe);
        default: return c.ext_ip && (c.wr[1] & kWr1ExtIe);
    }
}

bool Z85230::next_interrupt(unsigned& ch, Source& s) const {
    for (unsigned n = 0; n < 6; ++n) {
        const unsigned c = n / 3;
        const Source src = static_cast<Source>(n % 3);
        if (chan_[c].ius[src]) return false;  // under service: it and everything below wait
        if (ip(c, src)) {
            ch = c;
            s = src;
            return true;
        }
    }
    return false;
}

u8 Z85230::rr3() const {
    return static_cast<u8>((ip(0, kRx) ? 0x20 : 0) | (ip(0, kTx) ? 0x10 : 0) | (ip(0, kExt) ? 0x08 : 0) |
                           (ip(1, kRx) ? 0x04 : 0) | (ip(1, kTx) ? 0x02 : 0) | (ip(1, kExt) ? 0x01 : 0));
}

// The vector status code of the highest priority pending interrupt (011,
// channel B special receive, if none): channel B transmit 0, external/status
// 1, receive 2, special receive 3; channel A the same plus 4.
u8 Z85230::vector_status() const {
    unsigned ch;
    Source s;
    if (!next_interrupt(ch, s)) return 3;
    const unsigned base = ch == 0 ? 4 : 0;
    switch (s) {
        case kTx: return static_cast<u8>(base);
        case kExt: return static_cast<u8>(base + 1);
        default: return static_cast<u8>(base + (rx_special(ch) ? 3 : 2));
    }
}

u8 Z85230::modified_vector() const {
    const unsigned code = vector_status();
    if (wr9_ & kWr9StatusHigh) {
        // Status high: bits 4..6 carry the code, in reversed bit order.
        const unsigned rev = ((code & 1) << 2) | (code & 2) | ((code >> 2) & 1);
        return static_cast<u8>((wr2_ & 0x8F) | (rev << 4));
    }
    return static_cast<u8>((wr2_ & 0xF1) | (code << 1));
}

// A software interrupt acknowledge (a read of RR2 with WR9 D5 set): the
// highest priority pending source goes under service.
void Z85230::acknowledge() {
    unsigned ch;
    Source s;
    if (next_interrupt(ch, s)) chan_[ch].ius[s] = true;
    update_irq();
}

void Z85230::update_irq() {
    unsigned ch;
    Source s;
    const bool pending = (wr9_ & kWr9Mie) && next_interrupt(ch, s);
    if (pending != irq_) {
        irq_ = pending;
        if (irq_cb_) irq_cb_(irq_);
    }
}

// --- registers ----------------------------------------------------------------

u8 Z85230::read_register(unsigned ch, unsigned reg) {
    const Channel& c = chan_[ch];
    const bool ext = c.wr7p & kWr7pExtendedRead;
    switch (reg) {
        case 0: return rr0(ch);
        case 4: return ext ? c.wr[4] : rr0(ch);
        case 1: return rr1(ch);
        case 5: return ext ? c.wr[5] : rr1(ch);
        case 2:
        case 6: {
            const u8 v = ch == 0 ? wr2_ : modified_vector();
            if (wr9_ & kWr9SoftIntack) acknowledge();
            return v;
        }
        case 3:
        case 7: return ch == 0 ? rr3() : 0;
        case 8: return rx_read(ch);
        case 9: return ext ? c.wr[3] : c.wr[13];
        case 10: return 0;  // asynchronous: not on loop, no clocks missing
        case 11: return ext ? c.wr[10] : static_cast<u8>(c.wr[15] & 0xFA);
        case 12: return c.wr[12];
        case 13: return c.wr[13];
        case 14: return ext ? c.wr7p : 0;
        case 15: return c.wr[15] & 0xFA;
        default: return 0;
    }
}

void Z85230::command(unsigned ch, u8 value) {
    Channel& c = chan_[ch];
    c.pointer = value & 7;
    switch ((value >> 3) & 7) {
        case 1: c.pointer |= 8; break;  // point high
        case 2: {                       // reset external/status interrupts: the latches open
            c.ext_ip = false;
            const u8 held = c.ext_latched_rr0;
            c.ext_latched = false;
            // A change that persisted while they were closed interrupts again.
            ext_change(ch, held);
            schedule_zero_count(ch);
            break;
        }
        case 4: c.rx_first_armed = true; break;  // enable interrupt on next receive character
        case 5:                                  // reset Tx interrupt pending
            c.tx_ip = false;
            c.tx_ip_held = true;
            break;
        case 6:  // error reset: the latched errors clear and a locked FIFO exit is released
            c.rr1_latched = 0;
            if (c.rx_locked) {
                c.rx_locked = false;
                if (!c.rx.empty()) c.rx.pop_front();
            }
            break;
        case 7:  // reset highest IUS
            for (unsigned n = 0; n < 6; ++n) {
                bool& ius = chan_[n / 3].ius[n % 3];
                if (ius) {
                    ius = false;
                    break;
                }
            }
            break;
        default: break;
    }
    if ((value >> 6) == 3 && (c.wr[5] & kWr5TxEnable)) c.tx_underrun = false;  // reset Tx underrun/EOM latch
    update_irq();
}

void Z85230::write_register(unsigned ch, unsigned reg, u8 value) {
    Channel& c = chan_[ch];
    switch (reg) {
        case 0: command(ch, value); return;
        case 2: wr2_ = value; return;
        case 7:
            if (c.wr[15] & kWr15Wr7Prime) c.wr7p = value & 0x7F;
            else c.wr[7] = value;
            return;
        case 8: {  // transmit: into the FIFO's entry (over its last character if full)
            const u8 old = rr0_live(ch);
            if (c.tx.size() >= kTxFifoDepth) c.tx.back() = value;
            else c.tx.push_back(value);
            c.tx_ip = false;
            c.tx_ip_held = false;
            tx_start(ch);
            // Entry-location mode: the IP returns as soon as there is room again.
            if (!(c.wr7p & kWr7pTxFifoEmpty) && (c.wr[1] & kWr1TxIe) && c.tx.size() < kTxFifoDepth) c.tx_ip = true;
            ext_change(ch, old);
            update_irq();
            return;
        }
        case 9:
            switch (value >> 6) {
                case 1: channel_reset(1, false); break;
                case 2: channel_reset(0, false); break;
                case 3: reset(); break;
                default: break;
            }
            wr9_ = value & 0x3F;
            update_irq();
            return;
        default: {
            const u8 before = c.wr[reg];
            c.wr[reg] = value;
            if (reg == 1) {
                const unsigned mode = (value >> 3) & 3;
                if (mode == 1 && ((before >> 3) & 3) != 1) c.rx_first_armed = true;
                if (mode == 0) c.rx_locked = false;  // receive interrupts off: no special condition pending
                if (!(value & kWr1ExtIe)) c.ext_ip = false;
            }
            if (reg == 5 && (value & kWr5TxEnable)) tx_start(ch);
            if (reg == 1 || reg == 12 || reg == 13 || reg == 14 || reg == 15) schedule_zero_count(ch);
            update_irq();
            return;
        }
    }
}

}  // namespace sg
