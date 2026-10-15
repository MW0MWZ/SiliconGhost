// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Seeq 8003/80C03 Ethernet data link controller: station address, receive
// address filter, and the receive and transmit command and status
// registers. The data moves through the host interface's DMA (HPC3 on the
// Indy), which reports frames and their outcome here.
//
// Registers (byte-wide): writes to 0-5 load the station address (or, on the
// 80C03, multicast filter bytes in register banks 1-2), 6 is the receive
// command and 7 the transmit command; reads of 6 and 7 return the receive
// and transmit status. Reading a status marks it old (bit 7), which drops
// its interrupt. As on the 80C03 the Indy has, reads of 0-3 return its
// collision counters (none on an emulated wire) and 4-5 read as zero.
//
// References: Linux drivers/net/ethernet/seeq/sgiseeq.h.
#pragma once

#include <array>
#include <cstddef>
#include <functional>

#include "core/types.h"

namespace sg {

class Seeq8003 {
public:
    // Receive status
    static constexpr u8 kRxOverflow = 0x01, kRxCrcError = 0x02, kRxDribble = 0x04, kRxShort = 0x08;
    static constexpr u8 kRxEndOfFrame = 0x10, kRxGood = 0x20, kStatusOld = 0x80;
    // Transmit status
    static constexpr u8 kTxUnderflow = 0x01, kTxCollision = 0x02, kTx16Retries = 0x04, kTxDone = 0x08;

    using IrqCallback = std::function<void(bool asserted)>;

    Seeq8003() { reset(); }
    void reset();

    u8 read(unsigned reg);
    void write(unsigned reg, u8 value);

    // Whether a frame with this destination would be taken in.
    bool accepts(const u8* frame, std::size_t length) const;
    // The outcome of a received or transmitted frame (new status); true if
    // the event is one the commands ask to interrupt on.
    bool received(u8 status);
    bool transmitted(u8 status);

    u8 rx_status() const { return rx_status_; }
    u8 tx_status() const { return tx_status_; }
    const std::array<u8, 6>& station_address() const { return station_; }
    bool irq() const { return irq_; }
    void set_irq_callback(IrqCallback cb) { irq_cb_ = std::move(cb); }

private:
    void update_irq();

    std::array<u8, 6> station_{};
    u8 rx_command_ = 0, tx_command_ = 0;
    u8 rx_status_ = 0, tx_status_ = 0;
    bool irq_ = false;
    IrqCallback irq_cb_;
};

}  // namespace sg
