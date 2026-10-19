// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "devices/seeq8003.h"

#include <algorithm>

namespace sg {

namespace {
constexpr u8 kRxModeMask = 0xC0, kRxAny = 0x40, kRxBroadcast = 0x80, kRxMulticast = 0xC0;
constexpr u8 kTxBankMask = 0x60;  // 80C03 register bank for registers 0-5
constexpr u8 kBank1 = 0x20, kBank2 = 0x40;
// Transmit control (bank 2, register 3) and the flags (register 5)
constexpr u8 kCtlSqe = 0x04, kCtlNoCarrier = 0x20;
constexpr u8 kFlagSqe = 0x01, kFlagNoCarrier = 0x02;
}  // namespace

void Seeq8003::reset() {
    rx_command_ = tx_command_ = 0;
    rx_status_ = tx_status_ = kStatusOld;  // no new status yet
    control_ = config_ = flags_ = 0;
    update_irq();
}

u8 Seeq8003::read(unsigned reg) {
    switch (reg & 7) {
        case 6: {
            const u8 v = rx_status_;
            rx_status_ |= kStatusOld;
            update_irq();
            return v;
        }
        case 7: {
            const u8 v = tx_status_;
            tx_status_ |= kStatusOld;
            update_irq();
            return v;
        }
        case 5: return flags_;
        default: return 0;  // collision counters (no collisions), test register
    }
}

void Seeq8003::write(unsigned reg, u8 value) {
    reg &= 7;
    switch (reg) {
        case 6:
            rx_command_ = value;
            update_irq();
            return;
        case 7:
            tx_command_ = value;
            update_irq();
            return;
        default:
            switch (tx_command_ & kTxBankMask) {
                case 0: station_[reg] = value; return;
                case kBank1: multicast_filter_[reg] = value; return;
                case kBank2:
                    if (reg < 2) multicast_filter_[6 + reg] = value;
                    else if (reg == 3) {
                        control_ = value;
                        if (!(value & kCtlSqe)) flags_ &= static_cast<u8>(~kFlagSqe);
                        if (!(value & kCtlNoCarrier)) flags_ &= static_cast<u8>(~kFlagNoCarrier);
                    } else if (reg == 4) {
                        config_ = value;
                    }
                    return;
                default: return;
            }
    }
}

bool Seeq8003::accepts(const u8* frame, std::size_t length) const {
    if (length < 6) return false;
    const u8 mode = rx_command_ & kRxModeMask;
    if (mode == 0) return false;
    if (mode == kRxAny) return true;
    // With the group address option the low 4 bits of the address are not compared.
    const unsigned whole = (config_ & kCfgGroupAddress) ? 5 : 6;
    if (std::equal(station_.begin(), station_.begin() + whole, frame) &&
        (whole == 6 || ((frame[5] ^ station_[5]) & 0xF0) == 0))
        return true;
    if (std::all_of(frame, frame + 6, [](u8 b) { return b == 0xFF; })) return mode == kRxBroadcast || mode == kRxMulticast;
    return (frame[0] & 1) && mode == kRxMulticast;  // the multicast filter is taken as passing everything
}

bool Seeq8003::received(u8 status) {
    rx_status_ = status & ~kStatusOld;
    update_irq();
    return (rx_status_ & rx_command_ & 0x3F) != 0;
}

bool Seeq8003::transmitted(u8 status, bool carrier) {
    if ((status & kTxDone) && (control_ & kCtlSqe)) flags_ |= kFlagSqe;
    if (!carrier && (control_ & kCtlNoCarrier)) flags_ |= kFlagNoCarrier;
    tx_status_ = status & ~kStatusOld;
    update_irq();
    return (tx_status_ & tx_command_ & 0x0F) != 0;
}

// Each command bit enables the interrupt for the status bit in the same place.
void Seeq8003::update_irq() {
    const bool rx = !(rx_status_ & kStatusOld) && (rx_status_ & rx_command_ & 0x3F);
    const bool tx = !(tx_status_ & kStatusOld) && (tx_status_ & tx_command_ & 0x0F);
    const bool level = rx || tx;
    if (level != irq_) {
        irq_ = level;
        if (irq_cb_) irq_cb_(irq_);
    }
}

}  // namespace sg
