// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Western Digital WD33C93A/B SCSI bus interface controller (initiator role).
//
// Host interface: an address register (SASR) selects one of 32 internal
// registers, which is then accessed through the data port (SCMD); reading
// SASR returns the auxiliary status. Targets attach through ScsiTarget; with
// no target at an ID, selection times out exactly as on a real bus.
// Select-and-Transfer runs the whole command, data and status sequence (and
// pauses, connected, if the transfer count runs out mid-data); Select plus
// Transfer Info walks the bus phases one at a time, including the message
// phases. Data moves through the host DMA channel, or the data register in
// polled mode; command, status and message bytes always use the register.
#pragma once

#include <array>
#include <deque>
#include <functional>
#include <memory>
#include <vector>

#include "core/scheduler.h"
#include "core/types.h"
#include "devices/scsi_disk.h"

namespace sg {

class Wd33c93 {
public:
    using IrqCallback = std::function<void(bool asserted)>;
    // DMA hooks: move up to `length` bytes to or from host memory; return the
    // number actually moved.
    using DmaToMemory = std::function<u32(const u8* data, u32 length)>;
    using DmaFromMemory = std::function<u32(u8* data, u32 length)>;

    // `cycles_per_ms`: CPU cycles per millisecond, for command timing.
    Wd33c93(Scheduler& scheduler, double cycles_per_ms);

    // Data phases take next to no time, as on a much faster bus; commands
    // still take their usual time to act on (drivers count on that).
    void set_fast_transfers(bool fast) { bytes_per_ms_ = fast ? kFastBytesPerMs : kBytesPerMs; }
    // A selection nobody answers times out within a millisecond, however
    // long the period programmed: software only learns that nobody is there,
    // and scanning the bus for absent targets (three tries each) takes the
    // PROM seconds at the full period.
    void set_quick_selection_timeout(bool quick) { quick_selection_timeout_ = quick; }

    void reset();
    // The master reset line was held and is now released: the chip comes out
    // of reset and interrupts with the reset status, as after a RESET command.
    void reset_released();
    // The host's DMA channel has started: a transfer waiting for it goes on.
    void dma_kick();
    u8 read_aux_status() const { return aux_status_; }
    void write_address(u8 value) { address_ = value & 0x1F; }
    u8 read_data();
    void write_data(u8 value);

    void set_irq_callback(IrqCallback cb) { irq_cb_ = std::move(cb); }
    void set_dma(DmaToMemory to_memory, DmaFromMemory from_memory) {
        dma_to_memory_ = std::move(to_memory);
        dma_from_memory_ = std::move(from_memory);
    }
    void attach(unsigned id, ScsiTarget* target) { targets_[id & 7] = target; }
    // The SCSI bus RST line was asserted: every target resets.
    void bus_reset();
    bool irq() const { return (aux_status_ & kAuxInt) != 0; }

private:
    static constexpr u8 kAuxInt = 0x80, kAuxLci = 0x40, kAuxBusy = 0x20, kAuxCip = 0x10, kAuxDbr = 0x01;

    enum Reg : unsigned {
        kOwnId = 0x00, kControl = 0x01, kTimeout = 0x02, kTargetLun = 0x0F, kCommandPhase = 0x10,
        kSyncTransfer = 0x11, kCountHigh = 0x12, kCountMid = 0x13, kCountLow = 0x14, kDestinationId = 0x15,
        kSourceId = 0x16, kScsiStatus = 0x17, kCommand = 0x18, kData = 0x19, kQueueTag = 0x1A, kAuxStatus = 0x1F,
    };

    void command(u8 cmd);
    void complete(u8 status);  // finish the current command with an interrupt
    void finish_in(u8 status, double ms);
    u32 transfer_count() const;
    void set_transfer_count(u32 count);
    bool dma_mode() const { return (regs_[kControl] & 0xE0) != 0; }
    void advance_address();

    // Bus and target side
    void select(ScsiTarget& target, bool atn);
    void release_bus();
    void issue_command(const u8* cdb, unsigned length);
    void end_data();
    bool receive_messages(const std::vector<u8>& m);
    void selection_timeout();

    void select_and_transfer();
    void sat_complete(u32 moved);
    u32 dma_data();
    void start_polled_data(bool sat);
    void polled_data_done();

    void transfer_info();
    void transfer_data();
    void sat_data();
    bool dma_starved(u32 moved) const;
    u32 pio_count() const;
    void pio_consumed();
    void end_pio_out();
    void pio_in_taken();
    void negate_ack();
    void next_phase(u8 phase);
    void data_phase_done(double ms);

    // Approximate bus throughput for completion timing: 5 MB/s (or 1 GB/s
    // with fast transfers).
    static constexpr double kBytesPerMs = 5000.0, kFastBytesPerMs = 1e6;

    Scheduler& scheduler_;
    double cycles_per_ms_;
    double bytes_per_ms_ = kBytesPerMs;
    bool quick_selection_timeout_ = false;
    std::array<u8, 32> regs_{};
    u8 address_ = 0;
    u8 aux_status_ = 0;
    u8 pending_status_ = 0;
    u8 queued_status_ = 0;  // interrupt status delivered after the current one is read
    std::unique_ptr<Timer> timer_;
    IrqCallback irq_cb_;
    DmaToMemory dma_to_memory_;
    DmaFromMemory dma_from_memory_;
    std::array<ScsiTarget*, 8> targets_{};

    // Connection state
    ScsiTarget* target_ = nullptr;  // connected target, if any
    u8 bus_phase_ = 0;              // phase the target is requesting
    u8 cdb_[12] = {};
    u8 target_status_ = 0;
    u8 lun_ = 0;                    // logical unit addressed (Target LUN register or IDENTIFY)
    bool command_done_ = false;     // status sent; COMMAND COMPLETE follows
    bool single_byte_ = false;      // the current command moves one byte, leaving the count alone
    enum class DmaWait { None, TransferInfo, SelectAndTransfer } dma_wait_ = DmaWait::None;
    // ATN raised (Assert ATN): the target takes MESSAGE OUT at its next
    // phase change, then goes on to the phase it would have requested.
    bool atn_ = false;
    u8 resume_phase_ = 0;
    std::vector<u8> buffer_;        // data phase bytes
    std::size_t buffer_pos_ = 0;
    std::size_t polled_end_ = 0;    // end of the current polled data chunk
    bool polled_sat_ = false;       // polled chunk belongs to Select-and-Transfer
    bool data_out_ = false;
    std::vector<u8> pio_;           // command / status / message bytes of a Transfer Info
    std::size_t pio_pos_ = 0;
    std::deque<u8> msg_in_;         // message bytes the target has to send
};

}  // namespace sg
