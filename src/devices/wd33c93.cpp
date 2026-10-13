// SPDX-License-Identifier: GPL-2.0-or-later
// Reference: Western Digital WD33C93A/B data sheet.
#include "devices/wd33c93.h"

#include <algorithm>

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
constexpr u8 kCmdTransferInfo = 0x20;

// Interrupt status codes; where noted the low three bits carry a bus phase.
constexpr u8 kStatusReset = 0x00;
constexpr u8 kStatusResetAdvanced = 0x01;
constexpr u8 kStatusSelectDone = 0x11;
constexpr u8 kStatusSelectTransferDone = 0x16;
constexpr u8 kStatusTransferDone = 0x18;     // Transfer Info done | phase now requested
constexpr u8 kStatusMessageInPaused = 0x20;  // message byte received, ACK still asserted
constexpr u8 kStatusSelectionTimeout = 0x42;
constexpr u8 kStatusUnexpectedPhase = 0x48;  // | phase requested
constexpr u8 kStatusDisconnect = 0x85;
constexpr u8 kStatusServiceRequired = 0x88;  // | phase requested

// SCSI bus phases (MSG, C/D, I/O)
constexpr u8 kBusDataOut = 0, kBusDataIn = 1, kBusCommand = 2, kBusStatus = 3, kBusMessageOut = 6,
             kBusMessageIn = 7;

// Command Phase register values during Select-and-Transfer
constexpr u8 kPhaseCdbSent = 0x30;        // + number of CDB bytes sent
constexpr u8 kPhaseTransferCount = 0x46;  // transfer count exhausted in the data phase
constexpr u8 kPhaseCommandComplete = 0x60;

constexpr u8 kMsgCommandComplete = 0x00;
constexpr u8 kMsgMessageReject = 0x07;

// Approximate bus throughput for completion timing: 5 MB/s.
constexpr double kBytesPerMs = 5000.0;

constexpr u8 kOwnIdAdvancedFeatures = 0x08;

unsigned cdb_length(u8 opcode) {
    switch (opcode >> 5) {
        case 0: return 6;
        case 1:
        case 2: return 10;
        case 5: return 12;
        default: return 6;
    }
}

bool is_data_phase(u8 phase) { return phase == kBusDataOut || phase == kBusDataIn; }

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
    queued_status_ = 0;
    release_bus();
    if (irq_cb_) irq_cb_(false);
}

void Wd33c93::release_bus() {
    target_ = nullptr;
    bus_phase_ = 0;
    buffer_.clear();
    buffer_pos_ = polled_end_ = 0;
    data_out_ = false;
    command_done_ = false;
    pio_.clear();
    pio_pos_ = 0;
    msg_in_.clear();
}

u32 Wd33c93::transfer_count() const {
    return (u32{regs_[kCountHigh]} << 16) | (u32{regs_[kCountMid]} << 8) | regs_[kCountLow];
}

void Wd33c93::set_transfer_count(u32 count) {
    regs_[kCountHigh] = static_cast<u8>(count >> 16);
    regs_[kCountMid] = static_cast<u8>(count >> 8);
    regs_[kCountLow] = static_cast<u8>(count);
}

void Wd33c93::finish_in(u8 status, double ms) {
    pending_status_ = status;
    aux_status_ |= kAuxBusy;
    timer_->schedule_in(static_cast<u64>(cycles_per_ms_ * ms) + 1);
}

// --- target side -------------------------------------------------------------

void Wd33c93::select(ScsiTarget& target, bool atn) {
    release_bus();
    target_ = &target;
    bus_phase_ = atn ? kBusMessageOut : kBusCommand;
}

// The target has the whole CDB: it executes the command and asks for the
// data phase, if any, or goes straight to status.
void Wd33c93::issue_command(const u8* cdb, unsigned length) {
    std::fill(std::begin(cdb_), std::end(cdb_), u8{0});
    std::copy(cdb, cdb + std::min<unsigned>(length, sizeof cdb_), cdb_);
    ScsiTarget::Result r = target_->command(cdb_, length);
    SG_LOG(Debug, "wd33c93", "cdb %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x: count %u, in %zu, out %u, status %02x",
           cdb_[0], cdb_[1], cdb_[2], cdb_[3], cdb_[4], cdb_[5], cdb_[6], cdb_[7], cdb_[8], cdb_[9], transfer_count(),
           r.data_in.size(), r.data_out_length, r.status);
    target_status_ = r.status;
    data_out_ = r.data_out_length != 0;
    if (data_out_) buffer_.assign(r.data_out_length, 0);
    else buffer_ = std::move(r.data_in);
    buffer_pos_ = 0;
    bus_phase_ = buffer_.empty() ? kBusStatus : (data_out_ ? kBusDataOut : kBusDataIn);
}

// All data moved: a data-out command commits its data, then status follows.
void Wd33c93::end_data() {
    if (data_out_) target_status_ = target_->finish_data_out(cdb_, buffer_);
    data_out_ = false;
    buffer_.clear();
    buffer_pos_ = 0;
    bus_phase_ = kBusStatus;
}

// Interprets message-out bytes; replies are queued for MESSAGE IN. Returns
// false if the target released the bus (ABORT, BUS DEVICE RESET).
bool Wd33c93::receive_messages(const std::vector<u8>& m) {
    for (std::size_t i = 0; i < m.size();) {
        const u8 b = m[i];
        if (b & 0x80) {  // IDENTIFY
            ++i;
        } else if (b == 0x01 && i + 1 < m.size()) {  // extended message
            const std::size_t len = m[i + 1];
            if (len == 3 && i + 4 < m.size() && m[i + 2] == 0x01) {
                // SDTR: agree to asynchronous transfers (offset 0)
                for (u8 r : {u8{0x01}, u8{0x03}, u8{0x01}, m[i + 3], u8{0x00}}) msg_in_.push_back(r);
            } else {
                msg_in_.push_back(kMsgMessageReject);
            }
            i += 2 + len;
        } else if (b == 0x06 || b == 0x0C) {  // ABORT / BUS DEVICE RESET
            return false;
        } else {
            if (b != 0x08) msg_in_.push_back(kMsgMessageReject);  // all but NO OPERATION
            ++i;
        }
    }
    return true;
}

// --- Select-and-Transfer -------------------------------------------------------

// Runs the command from wherever the connection stands up to command
// complete. It pauses, still connected, if the transfer count runs out while
// the target is still in a data phase; issuing it again resumes.
void Wd33c93::select_and_transfer() {
    if (bus_phase_ == kBusMessageOut || bus_phase_ == kBusCommand) {
        const unsigned len = cdb_length(regs_[0x03]);  // IDENTIFY (if any) goes first
        issue_command(&regs_[0x03], len);
        regs_[kCommandPhase] = static_cast<u8>(kPhaseCdbSent + len);
    }
    u32 moved = 0;
    if (is_data_phase(bus_phase_)) {
        if (!dma_mode()) {  // polled: the host drains or fills the data register
            start_polled_data(true);
            return;
        }
        moved = dma_data();
        if (buffer_pos_ < buffer_.size()) {
            regs_[kCommandPhase] = kPhaseTransferCount;
            finish_in(static_cast<u8>(kStatusUnexpectedPhase | bus_phase_), 0.05 + moved / kBytesPerMs);
            return;
        }
        end_data();
    }
    sat_complete(moved);
}

// STATUS and COMMAND COMPLETE, after which the target releases the bus.
void Wd33c93::sat_complete(u32 moved) {
    regs_[kTargetLun] = target_status_;
    regs_[kCommandPhase] = kPhaseCommandComplete;
    release_bus();
    finish_in(kStatusSelectTransferDone, 0.05 + moved / kBytesPerMs);
}

u32 Wd33c93::dma_data() {
    const u32 n = static_cast<u32>(std::min<std::size_t>(transfer_count(), buffer_.size() - buffer_pos_));
    u32 moved = 0;
    if (data_out_) moved = dma_from_memory_ ? dma_from_memory_(buffer_.data() + buffer_pos_, n) : 0;
    else moved = dma_to_memory_ ? dma_to_memory_(buffer_.data() + buffer_pos_, n) : 0;
    buffer_pos_ += moved;
    set_transfer_count(transfer_count() - moved);
    return moved;
}

void Wd33c93::start_polled_data(bool sat) {
    polled_sat_ = sat;
    polled_end_ = buffer_pos_ + std::min<std::size_t>(transfer_count(), buffer_.size() - buffer_pos_);
    aux_status_ |= kAuxBusy;
    if (polled_end_ == buffer_pos_) {
        polled_data_done();
        return;
    }
    aux_status_ |= kAuxDbr;
}

void Wd33c93::polled_data_done() {
    aux_status_ &= static_cast<u8>(~kAuxDbr);
    const bool more = buffer_pos_ < buffer_.size();
    if (!more) end_data();
    if (!polled_sat_) {
        finish_in(static_cast<u8>(kStatusTransferDone | bus_phase_), 0.05);
    } else if (more) {
        regs_[kCommandPhase] = kPhaseTransferCount;
        finish_in(static_cast<u8>(kStatusUnexpectedPhase | bus_phase_), 0.05);
    } else {
        sat_complete(0);
    }
}

// --- Transfer Info -------------------------------------------------------------

// Moves the bytes of the current bus phase, up to the transfer count, and
// stops when the target requests another phase. Only the data phases use
// DMA; everything else goes through the data register.
void Wd33c93::transfer_info() {
    SG_LOG(Debug, "wd33c93", "transfer info: bus phase %u, count %u", bus_phase_, transfer_count());
    pio_.clear();
    pio_pos_ = 0;
    aux_status_ |= kAuxBusy;
    switch (bus_phase_) {
        case kBusDataOut:
        case kBusDataIn:
            if (!dma_mode()) {
                start_polled_data(false);
            } else {
                const u32 moved = dma_data();
                if (buffer_pos_ == buffer_.size()) end_data();
                finish_in(static_cast<u8>(kStatusTransferDone | bus_phase_), 0.05 + moved / kBytesPerMs);
            }
            return;
        case kBusMessageOut:
        case kBusCommand:
            pio_.resize(transfer_count());
            if (pio_.empty()) end_pio_out();
            else aux_status_ |= kAuxDbr;
            return;
        case kBusStatus: pio_.push_back(target_status_); break;
        case kBusMessageIn: pio_.assign(msg_in_.begin(), msg_in_.end()); break;
    }
    // Inbound bytes are offered one at a time through the data register.
    if (pio_.empty() || transfer_count() == 0) finish_in(static_cast<u8>(kStatusServiceRequired | bus_phase_), 0.01);
    else aux_status_ |= kAuxDbr;
}

void Wd33c93::end_pio_out() {
    aux_status_ &= static_cast<u8>(~kAuxDbr);
    if (bus_phase_ == kBusMessageOut) {
        for (u8 b : pio_) SG_LOG(Debug, "wd33c93", "message out %02x", b);
        if (!receive_messages(pio_)) {
            release_bus();
            finish_in(kStatusDisconnect, 0.01);
            return;
        }
        bus_phase_ = msg_in_.empty() ? kBusCommand : kBusMessageIn;
    } else {  // COMMAND
        issue_command(pio_.data(), static_cast<unsigned>(pio_.size()));
    }
    pio_.clear();
    finish_in(static_cast<u8>(kStatusTransferDone | bus_phase_), 0.01);
}

// The host took an inbound byte from the data register.
void Wd33c93::pio_in_taken() {
    set_transfer_count(transfer_count() - 1);
    if (bus_phase_ == kBusStatus) {
        regs_[kTargetLun] = target_status_;
        msg_in_.assign(1, kMsgCommandComplete);
        command_done_ = true;
        bus_phase_ = kBusMessageIn;
        aux_status_ &= static_cast<u8>(~kAuxDbr);
        finish_in(static_cast<u8>(kStatusTransferDone | bus_phase_), 0.01);
        return;
    }
    // MESSAGE IN: ACK is held on the byte that exhausts the count (or ends
    // the message) so the host can inspect or reject it.
    msg_in_.pop_front();
    ++pio_pos_;
    if (transfer_count() && !msg_in_.empty()) return;
    aux_status_ &= static_cast<u8>(~kAuxDbr);
    finish_in(kStatusMessageInPaused, 0.01);
}

void Wd33c93::negate_ack() {
    if (!target_ || bus_phase_ != kBusMessageIn) return;
    if (!msg_in_.empty()) {
        finish_in(static_cast<u8>(kStatusServiceRequired | kBusMessageIn), 0.01);
    } else if (command_done_) {  // after COMMAND COMPLETE the target releases the bus
        release_bus();
        finish_in(kStatusDisconnect, 0.01);
    } else {
        bus_phase_ = kBusCommand;
        finish_in(static_cast<u8>(kStatusServiceRequired | kBusCommand), 0.01);
    }
}

// Nothing answers: selection times out after the programmed period
// (timeout register x 8192 input clocks; 20 MHz).
void Wd33c93::selection_timeout() {
    const unsigned period = regs_[kTimeout] ? regs_[kTimeout] : 1;
    SG_LOG(Debug, "wd33c93", "select target %u: no device", regs_[kDestinationId] & 7);
    finish_in(kStatusSelectionTimeout, period * 8192.0 / 20000.0);
}

// --- host interface ------------------------------------------------------------

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
                if (queued_status_) {  // a second condition was already pending
                    regs_[kScsiStatus] = queued_status_;
                    queued_status_ = 0;
                    aux_status_ |= kAuxInt;
                    if (irq_cb_) irq_cb_(true);
                }
            }
            break;
        case kAuxStatus: v = aux_status_; break;
        case kData:
            v = 0;
            if (!(aux_status_ & kAuxDbr)) break;
            if (is_data_phase(bus_phase_)) {
                if (!data_out_ && buffer_pos_ < polled_end_) {
                    v = buffer_[buffer_pos_++];
                    set_transfer_count(transfer_count() - 1);
                    if (buffer_pos_ == polled_end_) polled_data_done();
                }
            } else if ((bus_phase_ == kBusStatus || bus_phase_ == kBusMessageIn) && pio_pos_ < pio_.size()) {
                v = pio_[pio_pos_];
                pio_in_taken();
            }
            break;
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
        case kData:
            regs_[kData] = value;
            if (!(aux_status_ & kAuxDbr)) break;
            if (is_data_phase(bus_phase_)) {
                if (data_out_ && buffer_pos_ < polled_end_) {
                    buffer_[buffer_pos_++] = value;
                    set_transfer_count(transfer_count() - 1);
                    if (buffer_pos_ == polled_end_) polled_data_done();
                }
            } else if ((bus_phase_ == kBusMessageOut || bus_phase_ == kBusCommand) && pio_pos_ < pio_.size()) {
                pio_[pio_pos_++] = value;
                set_transfer_count(transfer_count() - 1);
                if (pio_pos_ == pio_.size()) end_pio_out();
            }
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
    SG_LOG(Debug, "wd33c93", "command %02x (dest %u, phase %02x)", cmd, regs_[kDestinationId] & 7, regs_[kCommandPhase]);
    ScsiTarget* const dest = targets_[regs_[kDestinationId] & 7];
    switch (cmd & 0x7F) {
        case kCmdReset: {
            const u8 own_id = regs_[kOwnId];
            const u8 timeout = regs_[kTimeout];
            regs_.fill(0);
            regs_[kOwnId] = own_id;
            regs_[kTimeout] = timeout;
            release_bus();
            aux_status_ |= kAuxCip;
            finish_in((own_id & kOwnIdAdvancedFeatures) ? kStatusResetAdvanced : kStatusReset, 0.01);
            return;
        }
        case kCmdSelectAtnTransfer:
        case kCmdSelectTransfer:
            if (!target_) {
                if (!dest) {
                    selection_timeout();
                    return;
                }
                select(*dest, false);
            }
            select_and_transfer();
            return;
        case kCmdSelectAtn:
        case kCmdSelect:
            if (!dest) {
                selection_timeout();
                return;
            }
            // Selected; the target then requests MESSAGE OUT (ATN raised) or
            // COMMAND, which the chip reports as a second interrupt.
            select(*dest, (cmd & 0x7F) == kCmdSelectAtn);
            queued_status_ = static_cast<u8>(kStatusServiceRequired | bus_phase_);
            finish_in(kStatusSelectDone, 0.01);
            return;
        case kCmdTransferInfo:
            if (target_) transfer_info();
            else aux_status_ |= kAuxLci;
            return;
        case kCmdNegateAck: negate_ack(); return;
        case kCmdAbort:
        case kCmdDisconnect: release_bus(); return;  // drops off the bus; no interrupt
        case kCmdAssertAtn: return;
        default:
            SG_LOG(Debug, "wd33c93", "command %02x ignored", cmd);
            aux_status_ |= kAuxLci;
            return;
    }
}

void Wd33c93::complete(u8 status) {
    regs_[kScsiStatus] = status;
    SG_LOG(Debug, "wd33c93", "interrupt %02x (phase %02x, count %u)", status, regs_[kCommandPhase], transfer_count());
    aux_status_ = static_cast<u8>((aux_status_ & ~(kAuxBusy | kAuxCip)) | kAuxInt);
    if (irq_cb_) irq_cb_(true);
}

}  // namespace sg
