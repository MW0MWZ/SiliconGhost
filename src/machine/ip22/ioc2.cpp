// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// References: Linux arch/mips/include/asm/sgi/ioc.h, ip22-int.c, ip22-time.c;
// NetBSD sys/arch/sgimips/dev/int.c, int2reg.h.
#include "machine/ip22/ioc2.h"

#include "core/log.h"

namespace sg::ip22 {

namespace {

constexpr u8 kSysIdIndy = 0x26;  // Guinness, board revision 3, chip revision 1
constexpr double kPitClock2 = 1.0e6;  // counter 2 input; its output clocks counters 0 and 1

// Front panel bits
constexpr u8 kPanelPowerOn = 0x01;
constexpr u8 kPanelPowerInt = 0x02;
constexpr u8 kPanelVolDownInt = 0x10;
constexpr u8 kPanelVolDownHold = 0x20;  // 0 while the button is held
constexpr u8 kPanelVolUpInt = 0x40;
constexpr u8 kPanelVolUpHold = 0x80;
constexpr u8 kPanelIntMask = kPanelPowerInt | kPanelVolDownInt | kPanelVolUpInt;
// The interrupt and hold bits are active low: a button press clears its
// interrupt bit (pending until the driver writes a 1 to it) and its hold bit
// (until it is released). IRIX writes 0xF3 to acknowledge everything.
constexpr u8 kPanelReset = 0xF3;  // power on, nothing pending, buttons released

enum Index : unsigned {
    kSccBCtrl = 0x0C, kSccBData = 0x0D, kSccACtrl = 0x0E, kSccAData = 0x0F,
    kKbdData = 0x10, kKbdCmd = 0x11, kGcSel = 0x12, kGenCtrl = 0x13, kPanel = 0x14,
    kSysId = 0x16, kRead = 0x18, kDmaSel = 0x1A, kReset = 0x1C, kWrite = 0x1E,
    kL0Status = 0x20, kL0Mask = 0x21, kL1Status = 0x22, kL1Mask = 0x23, kMapStatus = 0x24,
    kMapMask0 = 0x25, kMapMask1 = 0x26, kMapPol = 0x27, kTimerClear = 0x28, kErrStat = 0x29,
    kPit0 = 0x2C, kPitControl = 0x2F,
};

}  // namespace

Ioc2::Ioc2(Scheduler& scheduler, double cpu_hz, u64 scc_char_cycles)
    : scc_(scheduler, scc_char_cycles), pit_(scheduler, cpu_hz, {0.0, 0.0, kPitClock2}) {
    scc_.set_irq_callback([this](bool a) { set_mappable(kMapSerial, a); });
    kbc_.set_irq_callback([this](bool a) { set_mappable(kMapKeyboard, a); });
    pit_.set_output_callback([this](unsigned c, bool level) {
        // Counters 0 and 1 interrupt on their output's rising edge; the
        // request is held until cleared through the timer clear register.
        if (c < 2 && level && !timer_int_[c] && !programming_pit_) {
            timer_int_[c] = true;
            update_outputs();
        }
    });
    reset();
}

void Ioc2::reset() {
    scc_.reset();
    kbc_.reset();
    pit_.reset();
    pit_.set_clock(0, 0);
    pit_.set_clock(1, 0);
    pport_.fill(0);
    panel_ = kPanelReset;
    dmasel_ = reset_ = write_ = 0;
    local0_mask_ = local1_mask_ = map_mask0_ = map_mask1_ = map_pol_ = 0;
    err_stat_ = 0;
    timer_int_.fill(false);
    update_outputs();
}

void Ioc2::set_bit(u8& reg, unsigned bit, bool level) {
    const u8 nv = level ? static_cast<u8>(reg | (1u << bit)) : static_cast<u8>(reg & ~(1u << bit));
    if (nv == reg) return;
    reg = nv;
    update_outputs();
}

void Ioc2::panel_button(PanelButton button, bool pressed) {
    u8 irq = kPanelPowerInt, hold = 0;
    if (button == PanelButton::VolumeDown) irq = kPanelVolDownInt, hold = kPanelVolDownHold;
    if (button == PanelButton::VolumeUp) irq = kPanelVolUpInt, hold = kPanelVolUpHold;
    SG_LOG(Debug, "ioc", "panel button %d %s (local1 mask %02x)", static_cast<int>(button), pressed ? "down" : "up",
           local1_mask_);
    if (pressed) panel_ = static_cast<u8>(panel_ & ~(irq | hold));
    else panel_ |= hold;
    update_outputs();
}

u8 Ioc2::local0_status() const {
    u8 s = local0_raw_ & 0x7F;
    if (map_raw_ & map_mask0_) s |= 1u << kL0Mappable0;
    return s;
}

u8 Ioc2::local1_status() const {
    u8 s = local1_raw_ & ~(1u << kL1Mappable1) & ~(1u << kL1Panel);
    if (map_raw_ & map_mask1_) s |= 1u << kL1Mappable1;
    if (~panel_ & kPanelIntMask) s |= 1u << kL1Panel;
    return s;
}

void Ioc2::update_outputs() {
    const std::array<bool, 4> lines = {
        (local0_status() & local0_mask_) != 0,
        (local1_status() & local1_mask_) != 0,
        timer_int_[0],
        timer_int_[1],
    };
    for (unsigned i = 0; i < 4; ++i) {
        if (lines[i] != lines_[i]) {
            lines_[i] = lines[i];
            if (cpu_line_) cpu_line_(i, lines[i]);
        }
    }
}

// Counters 0 and 1 are clocked by counter 2's output.
void Ioc2::update_cascade() {
    const double f = pit_.output_frequency(2);
    pit_.set_clock(0, f);
    pit_.set_clock(1, f);
}

u8 Ioc2::read(unsigned index) {
    if (index < pport_.size()) return pport_[index];
    switch (index) {
        case kSccBCtrl: return scc_.read_control(1);
        case kSccBData: return scc_.read_data(1);
        case kSccACtrl: return scc_.read_control(0);
        case kSccAData: return scc_.read_data(0);
        case kKbdData: return kbc_.read_data();
        case kKbdCmd: return kbc_.read_status();
        case kGcSel:
        case kGenCtrl:
        case kRead: return 0;
        case kPanel: return panel_;
        case kSysId: return kSysIdIndy;
        case kDmaSel: return dmasel_;
        case kReset: return reset_;
        case kWrite: return write_;
        case kL0Status: return local0_status();
        case kL0Mask: return local0_mask_;
        case kL1Status: return local1_status();
        case kL1Mask: return local1_mask_;
        case kMapStatus: return map_raw_;
        case kMapMask0: return map_mask0_;
        case kMapMask1: return map_mask1_;
        case kMapPol: return map_pol_;
        case kErrStat: return err_stat_;
        case kPit0:
        case kPit0 + 1:
        case kPit0 + 2: return pit_.read(index - kPit0);
        default: SG_LOG(Debug, "ioc", "read of unknown register %02x", index * 4); return 0;
    }
}

// Status and configuration registers are plain state; data ports (which
// consume input) and the 8254 counters (which move with time) are not.
bool Ioc2::read_is_pure(unsigned index) const {
    if (index < pport_.size()) return true;
    switch (index) {
        case kSccBCtrl: return scc_.control_read_is_pure(1);
        case kSccACtrl: return scc_.control_read_is_pure(0);
        case kKbdCmd:
        case kGcSel:
        case kGenCtrl:
        case kRead:
        case kPanel:
        case kSysId:
        case kDmaSel:
        case kReset:
        case kWrite:
        case kL0Status:
        case kL0Mask:
        case kL1Status:
        case kL1Mask:
        case kMapStatus:
        case kMapMask0:
        case kMapMask1:
        case kMapPol:
        case kErrStat: return true;
        default: return false;
    }
}

void Ioc2::write(unsigned index, u8 value) {
    if (index < pport_.size()) {
        pport_[index] = value;
        return;
    }
    switch (index) {
        case kSccBCtrl: scc_.write_control(1, value); return;
        case kSccBData: scc_.write_data(1, value); return;
        case kSccACtrl: scc_.write_control(0, value); return;
        case kSccAData: scc_.write_data(0, value); return;
        case kKbdData: kbc_.write_data(value); return;
        case kKbdCmd: kbc_.write_command(value); return;
        case kGcSel:
        case kGenCtrl: return;
        case kPanel:
            SG_LOG(Debug, "ioc", "panel write %02x (was %02x, local1 mask %02x)", value, panel_, local1_mask_);
            // Writing 1 to an interrupt bit acknowledges it; 0 leaves it.
            panel_ = static_cast<u8>(((panel_ | (value & kPanelIntMask)) & ~kPanelPowerOn) | (value & kPanelPowerOn));
            if (!(value & kPanelPowerOn)) {
                SG_LOG(Info, "ioc", "power off requested");
                if (power_off_) power_off_();
            }
            update_outputs();
            return;
        case kDmaSel: dmasel_ = value; return;
        case kReset:
            if ((reset_ & 0x02) && !(value & 0x02)) kbc_.reset();  // keyboard/mouse held in reset
            reset_ = value;
            return;
        case kWrite: write_ = value; return;
        case kL0Mask: local0_mask_ = value; break;
        case kL1Mask: local1_mask_ = value; break;
        case kMapMask0: map_mask0_ = value; break;
        case kMapMask1: map_mask1_ = value; break;
        case kMapPol: map_pol_ = value; return;
        case kTimerClear:
            if (value & 1) timer_int_[0] = false;
            if (value & 2) timer_int_[1] = false;
            break;
        case kPit0:
        case kPit0 + 1:
        case kPit0 + 2:
        case kPitControl:
            programming_pit_ = true;
            pit_.write(index - kPit0, value);
            update_cascade();
            programming_pit_ = false;
            return;
        default: SG_LOG(Debug, "ioc", "write to unknown register %02x = %02x", index * 4, value); return;
    }
    update_outputs();
}

}  // namespace sg::ip22
