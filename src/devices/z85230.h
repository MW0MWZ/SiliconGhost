// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Zilog 85C30 serial controller with the Z85230 (ESCC) enhancements the
// IOC2's 85CX30 has: a 4-byte transmit FIFO, an 8-byte receive FIFO, WR7'
// and extended register reads. Asynchronous mode only.
//
// Interrupts follow the SCC's structure: per source (receive, transmit,
// external/status, in each channel) an enable, a pending bit (RR3) and an
// under-service bit, set by a software acknowledge (WR9 D5 and a read of
// RR2) and cleared by Reset Highest IUS; a source under service masks
// itself and every lower one (channel A before B; receive, transmit,
// external/status within a channel). Receive interrupts take the WR1 mode
// (first character, all characters, special conditions only) and WR7' D3
// (four bytes); a special condition (overrun, or a parity error when WR1
// D2 makes it one) locks the FIFO's exit in the first-character and
// special-only modes until Error Reset. Transmit interrupts come when the
// FIFO empties (WR7' D5 set) or has room (clear). External/status bits
// enabled in WR15 latch on a change, interrupting with WR1 D0, until Reset
// External/Status Interrupts; the sources here are the transmit underrun
// latch and the baud rate generator's zero count. DCD and CTS are always
// asserted, nothing drives SYNC, and no break or framing or parity error
// arrives from the host. WR14 local loopback and auto echo are honoured.
//
// Interface: per-channel control and data ports (channel 0 = A, 1 = B).
// Transmitted bytes go to a sink callback (host console, file, pty...);
// received bytes are pushed in with receive().
//
// References: Zilog SCC/ESCC User Manual (UM0109); Z85C30 product
// specification; SGI IOC2 specification (85CX30).
#pragma once

#include <array>
#include <deque>
#include <functional>
#include <memory>

#include "core/scheduler.h"
#include "core/types.h"

namespace sg {

class Z85230 {
public:
    using TxSink = std::function<void(unsigned channel, u8 byte)>;
    using IrqCallback = std::function<void(bool asserted)>;

    // `char_cycles`: CPU cycles taken to shift out one character.
    // `pclk_cycles`: CPU cycles per PCLK (the baud rate generator's clock).
    Z85230(Scheduler& scheduler, u64 char_cycles, double pclk_cycles = 0);

    void reset();
    u8 read_control(unsigned ch);
    // Reading the control port is side-effect free only for RR0 (pointer 0).
    bool control_read_is_pure(unsigned ch) const { return chan_[ch].pointer == 0; }
    void write_control(unsigned ch, u8 value);
    u8 read_data(unsigned ch);
    void write_data(unsigned ch, u8 value);

    void receive(unsigned ch, u8 byte);
    bool rx_ready(unsigned ch) const { return !chan_[ch].rx.empty(); }
    bool rx_enabled(unsigned ch) const { return (chan_[ch].wr[3] & 0x01) != 0; }
    void set_tx_sink(TxSink sink) { tx_sink_ = std::move(sink); }
    void set_irq_callback(IrqCallback cb) { irq_cb_ = std::move(cb); }
    bool irq() const { return irq_; }

private:
    struct RxEntry {
        u8 byte;
        u8 status;  // RR1 error bits for this character
    };
    struct Channel {
        std::array<u8, 16> wr{};
        u8 wr7p = 0x20;  // WR7'
        unsigned pointer = 0;
        std::deque<RxEntry> rx;
        bool rx_locked = false;      // a special condition holds the FIFO's exit
        bool rx_first_armed = true;  // first-character mode: the next character interrupts
        u8 rr1_latched = 0;          // overrun and parity, until Error Reset
        std::deque<u8> tx;           // the transmit FIFO
        bool tx_shifting = false;    // the shift register holds a character
        u8 tx_byte = 0;
        bool tx_ip = false;
        bool tx_ip_held = false;  // Reset Tx Int Pending: no transmit IP until the next character
        bool tx_underrun = true;  // RR0 D6: the Tx underrun/EOM latch
        // External/status latches: closed (holding `latched`) or open.
        bool ext_latched = false;
        u8 ext_latched_rr0 = 0;
        bool ext_ip = false;
        std::array<bool, 3> ius{};  // receive, transmit, external/status under service
        std::unique_ptr<Timer> tx_timer;
        std::unique_ptr<Timer> zc_timer;  // the baud rate generator's next zero count
    };
    enum Source : unsigned { kRx, kTx, kExt };

    void channel_reset(unsigned ch, bool hardware);
    u8 read_register(unsigned ch, unsigned reg);
    void write_register(unsigned ch, unsigned reg, u8 value);
    void command(unsigned ch, u8 value);
    void tx_start(unsigned ch);
    void tx_done(unsigned ch);
    void tx_output(unsigned ch, u8 byte);
    void rx_push(unsigned ch, u8 byte);
    u8 rx_read(unsigned ch);
    u8 rr0_live(unsigned ch) const;
    u8 rr0(unsigned ch) const;
    u8 rr1(unsigned ch) const;
    void ext_change(unsigned ch, u8 old_rr0);
    void schedule_zero_count(unsigned ch);
    bool rx_special(unsigned ch) const;
    bool rx_ip(unsigned ch) const;
    bool ip(unsigned ch, Source s) const;
    // The highest priority pending source not masked by one under service
    // (channel, source), or false if none.
    bool next_interrupt(unsigned& ch, Source& s) const;
    u8 rr3() const;
    u8 vector_status() const;
    u8 modified_vector() const;
    void acknowledge();
    void update_irq();

    Scheduler& scheduler_;
    u64 char_cycles_;
    double pclk_cycles_;
    std::array<Channel, 2> chan_;
    u8 wr9_ = 0;  // shared master interrupt control
    u8 wr2_ = 0;  // shared interrupt vector
    bool irq_ = false;
    TxSink tx_sink_;
    IrqCallback irq_cb_;
};

}  // namespace sg
