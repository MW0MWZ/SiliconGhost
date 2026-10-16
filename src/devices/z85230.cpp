// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "devices/z85230.h"

#include "core/log.h"

namespace sg {

namespace {
constexpr unsigned kRxFifoDepth = 8;  // Z85230 has an 8-byte receive FIFO

// RR0 bits
constexpr u8 kRr0RxAvail = 0x01;
constexpr u8 kRr0TxEmpty = 0x04;
constexpr u8 kRr0Dcd = 0x08;
constexpr u8 kRr0Cts = 0x20;
constexpr u8 kRr0TxUnderrun = 0x40;
// WR1 bits
constexpr u8 kWr1TxIe = 0x02;
// WR9 bits
constexpr u8 kWr9Mie = 0x08;
constexpr u8 kWr9StatusHigh = 0x10;
}  // namespace

Z85230::Z85230(Scheduler& scheduler, u64 char_cycles) : scheduler_(scheduler), char_cycles_(char_cycles) {
    for (unsigned ch = 0; ch < 2; ++ch) {
        chan_[ch].tx_timer = std::make_unique<Timer>(scheduler_, [this, ch] { tx_done(ch); });
    }
    reset();
}

// Resets change only some write register bits (the SCC user manual's reset
// table); the time constants (WR12, WR13) and the rest keep their values.
void Z85230::reset() {
    wr9_ = 0;  // hardware reset clears MIE, VIS, NV
    wr2_ = 0;
    for (unsigned ch = 0; ch < 2; ++ch) {
        channel_reset(ch);
        Channel& c = chan_[ch];
        c.wr[10] = 0;
        c.wr[11] = 0x08;
        c.wr[14] = static_cast<u8>((c.wr[14] & 0xC0) | 0x30);
    }
}

void Z85230::channel_reset(unsigned ch) {
    Channel& c = chan_[ch];
    c.tx_timer->cancel();
    c.wr[0] = 0;
    c.wr[1] &= 0x24;
    c.wr[3] &= 0xFE;  // receiver off
    c.wr[4] |= 0x04;
    c.wr[5] &= 0x61;  // transmitter off, DTR and RTS negated
    c.wr[10] &= 0x61;
    c.wr[14] = static_cast<u8>((c.wr[14] & 0xC3) | 0x20);
    c.wr[15] = 0xF8;
    c.pointer = 0;
    c.rx.clear();
    c.rx_overrun = false;
    c.tx_empty = true;
    c.tx_busy = false;
    c.rx_ip = c.tx_ip = c.ext_ip = false;
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
    if (c.pointer != 0) {
        const unsigned reg = c.pointer;
        c.pointer = 0;
        write_register(ch, reg, value);
        return;
    }
    write_register(ch, 0, value);
}

u8 Z85230::read_data(unsigned ch) { return read_register(ch, 8); }

void Z85230::write_data(unsigned ch, u8 value) { write_register(ch, 8, value); }

u8 Z85230::read_register(unsigned ch, unsigned reg) {
    const Channel& c = chan_[ch];
    switch (reg) {
        case 0:
        case 4:  // RR4 images RR0 when WR7' extended read is off
            return static_cast<u8>((c.rx.empty() ? 0 : kRr0RxAvail) | (c.tx_empty ? kRr0TxEmpty : 0) | kRr0Dcd |
                                   kRr0Cts | (c.tx_busy ? 0 : kRr0TxUnderrun));
        case 1:
        case 5:
            return static_cast<u8>((c.tx_busy ? 0 : 0x01) | 0x06 | (c.rx_overrun ? 0x20 : 0));  // All Sent, residue
        case 2:
        case 6: return ch == 0 ? wr2_ : modified_vector();
        case 3:
        case 7: return ch == 0 ? rr3() : 0;
        case 8: {
            Channel& m = chan_[ch];
            if (m.rx.empty()) return 0;
            const u8 b = m.rx.front();
            m.rx.pop_front();
            m.rx_ip = !m.rx.empty() && (m.wr[1] & 0x18) != 0;
            update_irq();
            return b;
        }
        case 10:
        case 14: return 0;
        case 12: return c.wr[12];
        case 13:
        case 9: return c.wr[13];
        case 15:
        case 11: return c.wr[15] & 0xFA;
        default: return 0;
    }
}

void Z85230::write_register(unsigned ch, unsigned reg, u8 value) {
    Channel& c = chan_[ch];
    switch (reg) {
        case 0: {
            c.pointer = value & 7;
            switch ((value >> 3) & 7) {
                case 1: c.pointer |= 8; break;  // point high
                case 2: c.ext_ip = false; break;  // reset ext/status interrupts
                case 5: c.tx_ip = false; break;   // reset Tx interrupt pending
                case 6: c.rx_overrun = false; break;  // error reset
                default: break;
            }
            update_irq();
            return;
        }
        case 2: wr2_ = value; return;
        case 8:  // transmit
            c.tx_empty = false;
            c.tx_ip = false;
            c.tx_byte = value;
            if (!c.tx_busy) {
                c.tx_busy = true;
                c.tx_timer->schedule_in(char_cycles_);
            }
            update_irq();
            return;
        case 9:
            switch (value >> 6) {
                case 1: channel_reset(1); break;
                case 2: channel_reset(0); break;
                case 3: reset(); break;
                default: break;
            }
            wr9_ = value & 0x3F;
            update_irq();
            return;
        default:
            c.wr[reg] = value;
            if (reg == 1) {
                // Enabling Tx interrupts with an empty buffer does not by itself
                // raise one; it arrives when the next character completes.
                if (!(value & 0x18)) c.rx_ip = false;
                else c.rx_ip = !c.rx.empty();
                update_irq();
            }
            return;
    }
}

void Z85230::tx_done(unsigned ch) {
    Channel& c = chan_[ch];
    c.tx_busy = false;
    if (c.wr[5] & 0x08) {  // Tx enable
        if (tx_sink_) tx_sink_(ch, c.tx_byte);
    } else {
        SG_LOG(Debug, "scc", "channel %c: byte %02x dropped, transmitter disabled", 'A' + ch, c.tx_byte);
    }
    c.tx_empty = true;
    if (c.wr[1] & kWr1TxIe) c.tx_ip = true;
    update_irq();
}

void Z85230::receive(unsigned ch, u8 byte) {
    Channel& c = chan_[ch];
    if (!(c.wr[3] & 0x01)) return;  // receiver disabled
    if (c.rx.size() >= kRxFifoDepth) {
        c.rx_overrun = true;
        return;
    }
    c.rx.push_back(byte);
    if (c.wr[1] & 0x18) c.rx_ip = true;
    update_irq();
}

u8 Z85230::rr3() const {
    return static_cast<u8>((chan_[0].rx_ip ? 0x20 : 0) | (chan_[0].tx_ip ? 0x10 : 0) | (chan_[0].ext_ip ? 0x08 : 0) |
                           (chan_[1].rx_ip ? 0x04 : 0) | (chan_[1].tx_ip ? 0x02 : 0) | (chan_[1].ext_ip ? 0x01 : 0));
}

u8 Z85230::modified_vector() const {
    // Highest priority pending source, per the Z85230 vector status codes.
    unsigned code = 3;  // no interrupt pending
    if (chan_[0].rx_ip) code = 6;
    else if (chan_[0].tx_ip) code = 4;
    else if (chan_[0].ext_ip) code = 5;
    else if (chan_[1].rx_ip) code = 2;
    else if (chan_[1].tx_ip) code = 0;
    else if (chan_[1].ext_ip) code = 1;
    if (wr9_ & kWr9StatusHigh) {
        // Status high: bits 4..6 carry the code, in reversed bit order.
        const unsigned rev = ((code & 1) << 2) | (code & 2) | ((code >> 2) & 1);
        return static_cast<u8>((wr2_ & 0x8F) | (rev << 4));
    }
    return static_cast<u8>((wr2_ & 0xF1) | (code << 1));
}

void Z85230::update_irq() {
    const bool pending = (wr9_ & kWr9Mie) && rr3() != 0;
    if (pending != irq_) {
        irq_ = pending;
        if (irq_cb_) irq_cb_(irq_);
    }
}

}  // namespace sg
