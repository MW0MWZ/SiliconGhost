// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
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
constexpr u8 kStatusSelectAborted = 0x22;
constexpr u8 kStatusTransferAborted = 0x28;  // | phase requested
constexpr u8 kStatusInvalidCommand = 0x40;
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

// From the end of the command phase to STATUS: a target takes a while to
// act on a command, and drivers count on not seeing the status interrupt
// at once (OpenBSD's frees the request it is still looking at).
constexpr double kCommandMs = 0.2;

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

void Wd33c93::reset_released() {
    reset();
    complete(kStatusReset);
}

void Wd33c93::release_bus() {
    target_ = nullptr;
    bus_phase_ = 0;
    atn_ = false;
    resume_phase_ = 0;
    buffer_.clear();
    buffer_pos_ = polled_end_ = 0;
    dma_wait_ = DmaWait::None;
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
    lun_ = regs_[kTargetLun] & 7;
    bus_phase_ = atn ? kBusMessageOut : kBusCommand;
    resume_phase_ = kBusCommand;
}

// The target moves on to `phase`, by way of MESSAGE OUT if ATN is raised.
void Wd33c93::next_phase(u8 phase) {
    if (atn_ && phase != kBusMessageOut) {
        resume_phase_ = phase;
        phase = kBusMessageOut;
    }
    bus_phase_ = phase;
}

// The target has the whole CDB: it executes the command and asks for the
// data phase, if any, or goes straight to status. The LUN addressed goes to
// the target in the CDB's LUN field (it serves LUN 0 only).
void Wd33c93::issue_command(const u8* cdb, unsigned length) {
    std::fill(std::begin(cdb_), std::end(cdb_), u8{0});
    std::copy(cdb, cdb + std::min<unsigned>(length, sizeof cdb_), cdb_);
    if (lun_) cdb_[1] = static_cast<u8>((cdb_[1] & 0x1F) | (lun_ << 5));
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
            lun_ = b & 7;
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
            if (b == 0x0C && target_) target_->bus_reset();
            return false;
        } else {
            // NO OPERATION, and MESSAGE REJECT (of the target's last
            // message, which it then does without), need no reply.
            if (b != 0x08 && b != kMsgMessageReject) msg_in_.push_back(kMsgMessageReject);
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
    sat_data();
}

// The data phase of Select-and-Transfer, then the end of the command. With
// DMA, the transfer waits for the host's DMA channel if it is not running
// yet (see dma_kick()).
void Wd33c93::sat_data() {
    u32 moved = 0;
    if (is_data_phase(bus_phase_)) {
        if (!dma_mode()) {  // polled: the host drains or fills the data register
            start_polled_data(true);
            return;
        }
        if (dma_starved(moved = dma_data())) {
            dma_wait_ = DmaWait::SelectAndTransfer;
            aux_status_ |= kAuxBusy;
            return;
        }
        if (buffer_pos_ < buffer_.size()) {
            regs_[kCommandPhase] = kPhaseTransferCount;
            finish_in(static_cast<u8>(kStatusUnexpectedPhase | bus_phase_), 0.05 + moved / bytes_per_ms_);
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
    finish_in(kStatusSelectTransferDone, kCommandMs + moved / bytes_per_ms_);
}

// TRANSFER INFO in a data phase, by DMA (waiting for the host's channel if
// it is not running yet).
void Wd33c93::transfer_data() {
    const u32 moved = dma_data();
    if (dma_starved(moved)) {
        dma_wait_ = DmaWait::TransferInfo;
        return;
    }
    data_phase_done(0.05 + moved / bytes_per_ms_);
}

// A Transfer Info in a data phase has moved what it could. If the target's
// data ran out first it went on to STATUS with count left over: an
// unexpected phase.
void Wd33c93::data_phase_done(double ms) {
    if (buffer_pos_ < buffer_.size()) {
        finish_in(static_cast<u8>(kStatusTransferDone | bus_phase_), ms);
        return;
    }
    end_data();
    next_phase(kBusStatus);
    const bool early = transfer_count() > 0;
    finish_in(static_cast<u8>((early ? kStatusUnexpectedPhase : kStatusTransferDone) | bus_phase_), ms);
}

// Data is due but the host's DMA moved none: the chip keeps requesting.
bool Wd33c93::dma_starved(u32 moved) const {
    return moved == 0 && transfer_count() > 0 && buffer_pos_ < buffer_.size();
}

void Wd33c93::dma_kick() {
    const DmaWait w = dma_wait_;
    dma_wait_ = DmaWait::None;
    if (w == DmaWait::TransferInfo) transfer_data();
    else if (w == DmaWait::SelectAndTransfer) sat_data();
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
    if (!polled_sat_) {
        data_phase_done(0.05);
        return;
    }
    if (!more) end_data();
    if (more) {
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
            if (!dma_mode()) start_polled_data(false);
            else transfer_data();
            return;
        case kBusMessageOut:
        case kBusCommand:
            pio_.resize(pio_count());
            if (pio_.empty()) end_pio_out();
            else aux_status_ |= kAuxDbr;
            return;
        case kBusStatus: pio_.push_back(target_status_); break;
        case kBusMessageIn: pio_.assign(msg_in_.begin(), msg_in_.end()); break;
    }
    // Inbound bytes are offered one at a time through the data register.
    if (pio_.empty() || pio_count() == 0) finish_in(static_cast<u8>(kStatusServiceRequired | bus_phase_), 0.01);
    else aux_status_ |= kAuxDbr;
}

// Bytes a non-data transfer moves: one for a single-byte transfer (SBT),
// otherwise the transfer count, which each byte then decrements.
u32 Wd33c93::pio_count() const { return single_byte_ ? 1 : transfer_count(); }

void Wd33c93::pio_consumed() {
    if (!single_byte_) set_transfer_count(transfer_count() - 1);
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
        atn_ = false;
        if (!msg_in_.empty()) {
            bus_phase_ = kBusMessageIn;
        } else if (resume_phase_ == kBusMessageIn) {  // the message rejected was the target's last
            release_bus();
            pio_.clear();
            finish_in(kStatusDisconnect, 0.01);
            return;
        } else {
            bus_phase_ = resume_phase_;
        }
        pio_.clear();
        finish_in(static_cast<u8>(kStatusTransferDone | bus_phase_), 0.01);
        return;
    }
    issue_command(pio_.data(), static_cast<unsigned>(pio_.size()));  // COMMAND
    pio_.clear();
    finish_in(static_cast<u8>(kStatusTransferDone | bus_phase_), kCommandMs);
}

// The host took an inbound byte from the data register.
void Wd33c93::pio_in_taken() {
    pio_consumed();
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
    if (!single_byte_ && transfer_count() && !msg_in_.empty()) return;
    aux_status_ &= static_cast<u8>(~kAuxDbr);
    finish_in(kStatusMessageInPaused, 0.01);
}

void Wd33c93::negate_ack() {
    if (!target_ || bus_phase_ != kBusMessageIn) return;
    if (atn_) {  // the host rejects (or answers) the message: MESSAGE OUT first
        // Message in comes before the command (a reply to negotiation) or
        // after status (COMMAND COMPLETE, after which the target leaves the
        // bus); the rest of a rejected message is not sent.
        resume_phase_ = command_done_ ? kBusMessageIn : kBusCommand;
        msg_in_.clear();
        bus_phase_ = kBusMessageOut;
        finish_in(static_cast<u8>(kStatusServiceRequired | kBusMessageOut), 0.01);
    } else if (!msg_in_.empty()) {
        finish_in(static_cast<u8>(kStatusServiceRequired | kBusMessageIn), 0.01);
    } else if (command_done_) {  // after COMMAND COMPLETE the target releases the bus
        release_bus();
        finish_in(kStatusDisconnect, 0.01);
    } else {
        bus_phase_ = kBusCommand;
        finish_in(static_cast<u8>(kStatusServiceRequired | kBusCommand), 0.01);
    }
}

// Nothing answers: selection times out after the programmed period, value x
// 80 / input clock (MHz) milliseconds, the clock given by the Own ID
// register's frequency select (20 MHz on the Indy); 0 never times out. At
// most a millisecond with quick selection timeouts.
void Wd33c93::selection_timeout() {
    SG_LOG(Debug, "wd33c93", "select target %u: no device", regs_[kDestinationId] & 7);
    regs_[kCommandPhase] = 0;
    if (!regs_[kTimeout]) {
        aux_status_ |= kAuxBusy;  // selecting for ever
        return;
    }
    static constexpr double kClockMhz[4] = {10.0, 15.0, 20.0, 20.0};
    const double ms = regs_[kTimeout] * 80.0 / kClockMhz[regs_[kOwnId] >> 6];
    finish_in(kStatusSelectionTimeout, quick_selection_timeout_ ? std::min(ms, 1.0) : ms);
}

void Wd33c93::bus_reset() {
    for (ScsiTarget* t : targets_)
        if (t) t->bus_reset();
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
    SG_LOG(Trace, "wd33c93", "read %02x = %02x", reg, v);
    advance_address();
    return v;
}

void Wd33c93::write_data(u8 value) {
    const unsigned reg = address_;
    SG_LOG(Trace, "wd33c93", "write %02x = %02x", reg, value);
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
                pio_consumed();
                if (pio_pos_ == pio_.size()) end_pio_out();
            }
            break;
        case kScsiStatus:
        case kAuxStatus: break;  // read-only
        default: regs_[reg] = value; break;
    }
    advance_address();
}

// Level I commands (reset, abort, ATN, ACK, disconnect) are taken while a
// Level II command is still running (busy, but not yet interpreting the
// last command). Anything written while an interrupt is pending is ignored.
void Wd33c93::command(u8 cmd) {
    const u8 code = cmd & 0x7F;
    const bool level1 = code <= kCmdDisconnect;
    if ((aux_status_ & kAuxCip) || ((aux_status_ & kAuxBusy) && !level1) || ((aux_status_ & kAuxInt) && code != kCmdReset)) {
        aux_status_ |= kAuxLci;
        return;
    }
    aux_status_ &= static_cast<u8>(~kAuxLci);
    single_byte_ = (cmd & 0x80) != 0;  // SBT
    SG_LOG(Debug, "wd33c93", "command %02x (dest %u, phase %02x)", cmd, regs_[kDestinationId] & 7, regs_[kCommandPhase]);
    ScsiTarget* const dest = targets_[regs_[kDestinationId] & 7];
    switch (code) {
        case kCmdReset: {
            // Registers 0x01-0x16 clear; the Own ID register keeps its value.
            timer_->cancel();
            const u8 own_id = regs_[kOwnId];
            regs_.fill(0);
            regs_[kOwnId] = own_id;
            release_bus();
            queued_status_ = 0;
            aux_status_ = kAuxCip;
            if (irq_cb_) irq_cb_(false);
            finish_in((own_id & kOwnIdAdvancedFeatures) ? kStatusResetAdvanced : kStatusReset, 0.01);
            return;
        }
        case kCmdAbort: {
            // A selection being attempted is given up (and the bus left
            // free); a transfer stops, the count holding what was not moved,
            // and the target stays connected.
            const bool busy = aux_status_ & kAuxBusy;
            timer_->cancel();
            dma_wait_ = DmaWait::None;
            aux_status_ &= static_cast<u8>(~(kAuxBusy | kAuxDbr));
            queued_status_ = 0;
            if (target_ && (busy || bus_phase_ != kBusCommand)) {
                pio_.clear();
                finish_in(static_cast<u8>(kStatusTransferAborted | bus_phase_), 0.01);
            } else {
                release_bus();
                finish_in(kStatusSelectAborted, 0.01);
            }
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
        case kCmdDisconnect:  // drops off the bus; no interrupt
            timer_->cancel();
            aux_status_ &= static_cast<u8>(~(kAuxBusy | kAuxDbr));
            release_bus();
            return;
        case kCmdAssertAtn:  // no interrupt; the target sees ATN at its next phase change
            if (target_) atn_ = true;
            return;
        default:
            if (level1) return;
            // Target-role and other commands the initiator cannot run here.
            SG_LOG(Debug, "wd33c93", "command %02x invalid", cmd);
            finish_in(kStatusInvalidCommand, 0.01);
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
