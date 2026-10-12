// SPDX-License-Identifier: GPL-2.0-or-later
// Western Digital WD33C93A/B SCSI bus interface controller (initiator role).
//
// Host interface: an address register (SASR) selects one of 32 internal
// registers, which is then accessed through the data port (SCMD); reading
// SASR returns the auxiliary status. Targets attach through ScsiTarget; with
// no target at an ID, selection times out exactly as on a real bus.
#pragma once

#include <array>
#include <functional>
#include <memory>

#include "core/scheduler.h"
#include "core/types.h"

namespace sg {

class Wd33c93 {
public:
    using IrqCallback = std::function<void(bool asserted)>;

    // `cycles_per_ms`: CPU cycles per millisecond, for command timing.
    Wd33c93(Scheduler& scheduler, double cycles_per_ms);

    void reset();
    u8 read_aux_status() const { return aux_status_; }
    void write_address(u8 value) { address_ = value & 0x1F; }
    u8 read_data();
    void write_data(u8 value);

    void set_irq_callback(IrqCallback cb) { irq_cb_ = std::move(cb); }
    bool irq() const { return (aux_status_ & kAuxInt) != 0; }

private:
    static constexpr u8 kAuxInt = 0x80, kAuxLci = 0x40, kAuxBusy = 0x20, kAuxCip = 0x10;

    enum Reg : unsigned {
        kOwnId = 0x00, kControl = 0x01, kTimeout = 0x02, kTargetLun = 0x0F, kCommandPhase = 0x10,
        kSyncTransfer = 0x11, kCountHigh = 0x12, kCountMid = 0x13, kCountLow = 0x14, kDestinationId = 0x15,
        kSourceId = 0x16, kScsiStatus = 0x17, kCommand = 0x18, kData = 0x19, kQueueTag = 0x1A, kAuxStatus = 0x1F,
    };

    void command(u8 cmd);
    void complete(u8 status);  // finish the current command with an interrupt
    void advance_address();

    Scheduler& scheduler_;
    double cycles_per_ms_;
    std::array<u8, 32> regs_{};
    u8 address_ = 0;
    u8 aux_status_ = 0;
    u8 pending_status_ = 0;
    std::unique_ptr<Timer> timer_;
    IrqCallback irq_cb_;
};

}  // namespace sg
