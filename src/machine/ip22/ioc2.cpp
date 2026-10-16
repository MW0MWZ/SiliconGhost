// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// References: Linux arch/mips/include/asm/sgi/ioc.h, ip22-int.c, ip22-time.c;
// NetBSD sys/arch/sgimips/dev/int.c, int2reg.h.
#include "machine/ip22/ioc2.h"

#include "core/log.h"

namespace sg::ip22 {

namespace {

constexpr u8 kSysIdIndy = 0x26;       // Guinness, board revision 3, chip revision 1
constexpr u8 kSysIdIndigo2 = 0x11;    // Full House, board revision 8, chip revision 0
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

// The INT2's registers (Full House, PBUS channel 4) by index, and the INT3
// index of the same register.
constexpr unsigned kInt2PortConfig = 7, kInt2Pit0 = 12;
constexpr unsigned int2_to_int3(unsigned i) {
    return i <= 6 ? kL0Status + i : i == 8 ? static_cast<unsigned>(kTimerClear) : ~0u;
}
// PORT_CONFIG: the retrace latch clear lines are active low.
constexpr u8 kPconClrSgRetrace = 1u << 3, kPconClrS0Retrace = 1u << 4;

}  // namespace

Ioc2::Ioc2(Scheduler& scheduler, double cpu_hz, u64 scc_char_cycles, bool full_house)
    : scc_(scheduler, scc_char_cycles), pit_(scheduler, cpu_hz, {0.0, 0.0, kPitClock2}), full_house_(full_house) {
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
    dmasel_ = reset_ = write_ = gc_select_ = gen_ctrl_ = 0;
    local0_mask_ = local1_mask_ = map_mask0_ = map_mask1_ = map_pol_ = 0;
    err_stat_ = 0;
    ext_io_ = 0xFFFF;
    port_config_ = 0;
    gfx_irq_.fill(false);
    timer_int_.fill(false);
    update_outputs();
}

// GIO interrupt 0 (FIFO) and 1 (graphics) go to local 0, 2 (retrace) to
// local 1; EXTIO records the slot. The retrace is latched until the driver
// pulses PORT_CONFIG's clear line.
void Ioc2::set_gfx_slot_irq(unsigned gio_int, bool level) {
    static constexpr u16 kExt[3] = {kExtSgIrq2, kExtSgIrq1, kExtSgRetrace};
    if (gio_int > 2) return;
    if (gio_int == 2 && !level) return;  // the latch holds it
    gfx_irq_[gio_int] = level;
    ext_io_ = static_cast<u16>(level ? (ext_io_ & ~kExt[gio_int]) : (ext_io_ | kExt[gio_int]));
    if (gio_int == 0) set_local0(kL0Fifo, level);
    else if (gio_int == 1) set_local0(kL0Graphics, level);
    else set_local1(kL1Retrace, level);
}

void Ioc2::ext_io_write(u16 value) {
    (void)value;  // the status bits are read-only
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
        case kGcSel: return gc_select_;
        case kGenCtrl: return gen_ctrl_;  // nothing drives the pins set as inputs
        case kRead: return 0x50;  // twisted-pair link (bit 7 low), Ethernet +12 V and SCSI +5 V present
        case kPanel: return panel_;
        case kSysId: return full_house_ ? kSysIdIndigo2 : kSysIdIndy;
        case kDmaSel: return dmasel_;
        case kReset: return reset_;
        case kWrite: return write_;
        default:
            if (index >= kL0Status && index <= kPitControl && !full_house_) return int_read(index);
            SG_LOG(Debug, "ioc", "read of unknown register %02x", index * 4);
            return 0;
    }
}

u8 Ioc2::int_read(unsigned reg) {
    switch (reg) {
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
        case kPit0 + 2: return pit_.read(reg - kPit0);
        default: return 0;
    }
}

u8 Ioc2::int2_read(unsigned index) {
    if (index == kInt2PortConfig) return port_config_;
    if (index >= kInt2Pit0 && index < kInt2Pit0 + 4) return int_read(kPit0 + index - kInt2Pit0);
    const unsigned reg = int2_to_int3(index);
    if (reg == ~0u || reg == kTimerClear) {
        SG_LOG(Debug, "ioc", "INT2 read of unknown register %02x", index * 4);
        return 0;
    }
    return int_read(reg);
}

bool Ioc2::int2_read_is_pure(unsigned index) const {
    return index == kInt2PortConfig || (index <= 6);
}

void Ioc2::int2_write(unsigned index, u8 value) {
    if (index == kInt2PortConfig) {
        // A clear line taken low (and released) clears that slot's retrace latch.
        if (!(value & kPconClrSgRetrace) && gfx_irq_[2]) {
            gfx_irq_[2] = false;
            ext_io_ |= kExtSgRetrace;
            set_local1(kL1Retrace, false);
        }
        port_config_ = value;
        return;
    }
    if (index >= kInt2Pit0 && index < kInt2Pit0 + 4) {
        int_write(kPit0 + index - kInt2Pit0, value);
        return;
    }
    const unsigned reg = int2_to_int3(index);
    if (reg == ~0u || reg == kL0Status || reg == kL1Status || reg == kMapStatus) {
        SG_LOG(Debug, "ioc", "INT2 write to register %02x = %02x", index * 4, value);
        return;
    }
    if (int_write(reg, value)) update_outputs();
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
        case kErrStat: return !full_house_;
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
        case kGcSel: gc_select_ = value; return;
        case kGenCtrl: gen_ctrl_ = value; return;
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
        default:
            if (index >= kL0Status && index <= kPitControl && !full_house_) {
                if (int_write(index, value)) update_outputs();
                return;
            }
            SG_LOG(Debug, "ioc", "write to unknown register %02x = %02x", index * 4, value);
            return;
    }
}

// Returns true if the interrupt outputs need updating.
bool Ioc2::int_write(unsigned reg, u8 value) {
    switch (reg) {
        case kL0Mask: local0_mask_ = value; return true;
        case kL1Mask: local1_mask_ = value; return true;
        case kMapMask0: map_mask0_ = value; return true;
        case kMapMask1: map_mask1_ = value; return true;
        case kMapPol: map_pol_ = value; return false;
        case kTimerClear:
            if (value & 1) timer_int_[0] = false;
            if (value & 2) timer_int_[1] = false;
            return true;
        case kPit0:
        case kPit0 + 1:
        case kPit0 + 2:
        case kPitControl:
            programming_pit_ = true;
            pit_.write(reg - kPit0, value);
            update_cascade();
            programming_pit_ = false;
            return false;
        default: SG_LOG(Debug, "ioc", "write to unknown register %02x = %02x", reg * 4, value); return false;
    }
}

}  // namespace sg::ip22
