// SPDX-License-Identifier: GPL-2.0-or-later
#include "devices/seeq8003.h"

#include <algorithm>

namespace sg {

namespace {
constexpr u8 kRxModeMask = 0xC0, kRxAny = 0x40, kRxBroadcast = 0x80, kRxMulticast = 0xC0;
constexpr u8 kTxBankMask = 0x60;  // 80C03 register bank for registers 0-5
}  // namespace

void Seeq8003::reset() {
    rx_command_ = tx_command_ = 0;
    rx_status_ = tx_status_ = 0;
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
        default: return 0;  // collision counters and flags
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
            if ((tx_command_ & kTxBankMask) == 0) station_[reg] = value;
            return;
    }
}

bool Seeq8003::accepts(const u8* frame, std::size_t length) const {
    if (length < 6) return false;
    const u8 mode = rx_command_ & kRxModeMask;
    if (mode == 0) return false;
    if (mode == kRxAny) return true;
    if (std::equal(station_.begin(), station_.end(), frame)) return true;
    if (std::all_of(frame, frame + 6, [](u8 b) { return b == 0xFF; })) return mode == kRxBroadcast || mode == kRxMulticast;
    return (frame[0] & 1) && mode == kRxMulticast;  // the multicast filter is taken as passing everything
}

bool Seeq8003::received(u8 status) {
    rx_status_ = status & ~kStatusOld;
    update_irq();
    return (rx_status_ & rx_command_ & 0x3F) != 0;
}

bool Seeq8003::transmitted(u8 status) {
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
