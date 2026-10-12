// SPDX-License-Identifier: GPL-2.0-or-later
// Zilog Z85230 ESCC (dual serial controller), asynchronous mode only.
// Interface: per-channel control and data ports (channel 0 = A, 1 = B).
// Transmitted bytes go to a sink callback (host console, file, pty...);
// received bytes are pushed in with receive().
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
    Z85230(Scheduler& scheduler, u64 char_cycles);

    void reset();
    u8 read_control(unsigned ch);
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
    struct Channel {
        std::array<u8, 16> wr{};
        unsigned pointer = 0;
        std::deque<u8> rx;
        bool rx_overrun = false;
        bool tx_empty = true;
        bool tx_busy = false;
        u8 tx_byte = 0;
        bool rx_ip = false, tx_ip = false, ext_ip = false;
        std::unique_ptr<Timer> tx_timer;
    };

    void channel_reset(unsigned ch);
    u8 read_register(unsigned ch, unsigned reg);
    void write_register(unsigned ch, unsigned reg, u8 value);
    void tx_done(unsigned ch);
    void update_irq();
    u8 rr3() const;
    u8 modified_vector() const;

    Scheduler& scheduler_;
    u64 char_cycles_;
    std::array<Channel, 2> chan_;
    u8 wr9_ = 0;  // shared master interrupt control
    u8 wr2_ = 0;  // shared interrupt vector
    bool irq_ = false;
    TxSink tx_sink_;
    IrqCallback irq_cb_;
};

}  // namespace sg
