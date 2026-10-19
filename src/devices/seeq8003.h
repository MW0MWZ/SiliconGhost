// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Seeq 8003/80C03 Ethernet data link controller: station address, receive
// address filter, and the receive and transmit command and status
// registers. The data moves through the host interface's DMA (HPC3 on the
// Indy), which reports frames and their outcome here.
//
// Registers (byte-wide): writes to 0-5 load the station address, 6 is the
// receive command and 7 the transmit command; reads of 6 and 7 return the
// receive and transmit status. Reading a status marks it old (bit 7), which
// drops its interrupt. The 80C03 the Indy has adds register banks, chosen by
// transmit command bits 6:5: bank 1 takes multicast filter bytes 0-5, bank 2
// bytes 6-7 in registers 0-1, the transmit control register in 3 and the
// configuration register in 4. Reads of 0-3 return its collision counters
// (none on an emulated wire), 4 reads as zero and 5 returns its flags: SQE,
// set after a frame is sent, and no carrier, set when one is sent with no
// cable; each is cleared by writing 0 to its control bit and kept only
// while that bit is 1.
//
// The multicast filter's bytes are kept but not applied: its hash is not
// known (no 80C03 data sheet), and with multicast frames received the guest
// keeps or drops each group itself.
//
// References: Linux drivers/net/ethernet/seeq/sgiseeq.h; NetBSD
// sys/dev/ic/seeq8003reg.h.
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
    // `carrier`: a cable was there to send on.
    bool transmitted(u8 status, bool carrier = true);
    // Configuration register: frames this station sent are not taken back
    // (in loopback).
    bool receives_own() const { return !(config_ & kCfgNoOwn); }

    u8 rx_status() const { return rx_status_; }
    u8 tx_status() const { return tx_status_; }
    const std::array<u8, 6>& station_address() const { return station_; }
    bool irq() const { return irq_; }
    void set_irq_callback(IrqCallback cb) { irq_cb_ = std::move(cb); }

private:
    void update_irq();

    static constexpr u8 kCfgGroupAddress = 0x01, kCfgNoOwn = 0x08;
    std::array<u8, 6> station_{};
    std::array<u8, 8> multicast_filter_{};
    u8 control_ = 0, config_ = 0, flags_ = 0;
    u8 rx_command_ = 0, tx_command_ = 0;
    u8 rx_status_ = 0, tx_status_ = 0;
    bool irq_ = false;
    IrqCallback irq_cb_;
};

}  // namespace sg
