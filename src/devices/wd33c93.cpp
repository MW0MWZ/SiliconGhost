// SPDX-License-Identifier: GPL-2.0-or-later
// Reference: Western Digital WD33C93A/B data sheet.
#include "devices/wd33c93.h"

#include "core/log.h"

namespace sg {

namespace {

// Commands
constexpr u8 kCmdReset = 0x00;
constexpr u8 kCmdAbort = 0x01;
constexpr u8 kCmdAssertAtn = 0x02;
constexpr u8 kCmdNegateAck = 0x03;
constexpr u8 kCmdDisconnect = 0x04;
constexpr u8 kCmdSelectAtn = 0x06;
constexpr u8 kCmdSelect = 0x07;
constexpr u8 kCmdSelectAtnTransfer = 0x08;
constexpr u8 kCmdSelectTransfer = 0x09;

// SCSI status codes
constexpr u8 kStatusReset = 0x00;
constexpr u8 kStatusResetAdvanced = 0x01;
constexpr u8 kStatusSelectionTimeout = 0x42;

constexpr u8 kOwnIdAdvancedFeatures = 0x08;

}  // namespace

Wd33c93::Wd33c93(Scheduler& scheduler, double cycles_per_ms)
    : scheduler_(scheduler), cycles_per_ms_(cycles_per_ms) {
    timer_ = std::make_unique<Timer>(scheduler_, [this] { complete(pending_status_); });
    reset();
}

void Wd33c93::reset() {
    timer_->cancel();
    regs_.fill(0);
    address_ = 0;
    aux_status_ = 0;
    if (irq_cb_) irq_cb_(false);
}

// The address register steps after each access, except for the auxiliary
// status, command and data registers.
void Wd33c93::advance_address() {
    if (address_ != kAuxStatus && address_ != kCommand && address_ != kData) address_ = (address_ + 1) & 0x1F;
}

u8 Wd33c93::read_data() {
    const unsigned reg = address_;
    u8 v;
    switch (reg) {
        case kScsiStatus:
            v = regs_[kScsiStatus];
            if (aux_status_ & kAuxInt) {  // reading the status acknowledges the interrupt
                aux_status_ &= static_cast<u8>(~kAuxInt);
                if (irq_cb_) irq_cb_(false);
            }
            break;
        case kAuxStatus: v = aux_status_; break;
        case kData: v = 0; break;  // no information transfer in progress
        default: v = regs_[reg]; break;
    }
    advance_address();
    return v;
}

void Wd33c93::write_data(u8 value) {
    const unsigned reg = address_;
    switch (reg) {
        case kCommand:
            regs_[kCommand] = value;
            command(value);
            break;
        case kScsiStatus:
        case kAuxStatus: break;  // read-only
        default: regs_[reg] = value; break;
    }
    advance_address();
}

void Wd33c93::command(u8 cmd) {
    if (aux_status_ & (kAuxBusy | kAuxCip)) {
        aux_status_ |= kAuxLci;  // a command is still executing: ignored
        return;
    }
    aux_status_ &= static_cast<u8>(~kAuxLci);
    SG_LOG(Trace, "wd33c93", "command %02x", cmd);
    switch (cmd & 0x7F) {
        case kCmdReset: {
            const u8 own_id = regs_[kOwnId];
            const u8 timeout = regs_[kTimeout];
            regs_.fill(0);
            regs_[kOwnId] = own_id;
            regs_[kTimeout] = timeout;
            pending_status_ = (own_id & kOwnIdAdvancedFeatures) ? kStatusResetAdvanced : kStatusReset;
            aux_status_ |= kAuxCip;
            timer_->schedule_in(static_cast<u64>(cycles_per_ms_ * 0.01) + 1);
            return;
        }
        case kCmdSelectAtn:
        case kCmdSelect:
        case kCmdSelectAtnTransfer:
        case kCmdSelectTransfer: {
            // No targets are attached yet: selection times out after the
            // programmed period (timeout register x 8192 input clocks; 20 MHz).
            const unsigned period = regs_[kTimeout] ? regs_[kTimeout] : 1;
            const double ms = period * 8192.0 / 20000.0;
            SG_LOG(Debug, "wd33c93", "select target %u: no device", regs_[kDestinationId] & 7);
            pending_status_ = kStatusSelectionTimeout;
            aux_status_ |= kAuxBusy;
            timer_->schedule_in(static_cast<u64>(cycles_per_ms_ * ms) + 1);
            return;
        }
        case kCmdAbort:
        case kCmdDisconnect:  // releases the bus; with no connection there is nothing to report
        case kCmdAssertAtn:
        case kCmdNegateAck: return;  // no bus phase to act on
        default:
            SG_LOG(Debug, "wd33c93", "command %02x ignored (no bus activity)", cmd);
            aux_status_ |= kAuxLci;
            return;
    }
}

void Wd33c93::complete(u8 status) {
    regs_[kScsiStatus] = status;
    aux_status_ = static_cast<u8>((aux_status_ & ~(kAuxBusy | kAuxCip)) | kAuxInt);
    if (irq_cb_) irq_cb_(true);
}

}  // namespace sg
