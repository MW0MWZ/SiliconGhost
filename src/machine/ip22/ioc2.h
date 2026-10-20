// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// IP22 IOC2: Z85230 serial, 8042 keyboard/mouse, front panel, system ID, and
// the interrupt controller with its 8254 timers. Sits on HPC3 PBUS channel 6
// at 0x1FBD9800; every register is one byte on a 4-byte stride (data at byte
// offset 3 on a big-endian CPU).
//
// On the Indy ("Guinness") the interrupt controller (INT3) and the 8254 are
// part of the IOC's register window. On the Indigo2 ("Full House") they are
// a separate INT2 on PBUS channel 4 at 0x1FBD9000, which also has the GIO
// slot reset and retrace-latch register (PORT_CONFIG); the three GIO slots
// share the IOC's GIO interrupt inputs, and a 16-bit register at 0x1FBD9900
// (EXTIO) tells which slot asked, along with the bus error sources.
#pragma once

#include <array>
#include <functional>

#include "core/scheduler.h"
#include "devices/kbc8042.h"
#include "devices/pit8254.h"
#include "devices/z85230.h"

namespace sg::ip22 {

class Ioc2 {
public:
    // INT3 local interrupt sources (bit numbers).
    enum Local0 : unsigned { kL0Fifo = 0, kL0Scsi0 = 1, kL0Scsi1 = 2, kL0Ethernet = 3, kL0McDma = 4,
                             kL0Parallel = 5, kL0Graphics = 6, kL0Mappable0 = 7 };
    enum Local1 : unsigned { kL1Gp0 = 0, kL1Panel = 1, kL1Gp2 = 2, kL1Mappable1 = 3, kL1HpcDma = 4,
                             kL1AcFail = 5, kL1Video = 6, kL1Retrace = 7 };
    enum Mappable : unsigned { kMapVertical = 0, kMapEisa = 3, kMapKeyboard = 4, kMapSerial = 5,
                               kMapGioExp0 = 6, kMapGioExp1 = 7 };

    // CPU interrupt lines driven by the IOC (Int0..Int3 = IP2..IP5).
    using CpuLineCallback = std::function<void(unsigned line, bool asserted)>;

    Ioc2(Scheduler& scheduler, double cpu_hz, u64 scc_char_cycles, bool full_house = false);
    void reset();
    bool full_house() const { return full_house_; }
    // The system ID register: system type (bit 0, Full House), board revision
    // (4:1) and chip revision (7:5).
    void set_sysid(u8 sysid) { sysid_ = sysid; }

    u8 read(unsigned index);  // index = register byte offset / 4
    bool read_is_pure(unsigned index) const;
    void write(unsigned index, u8 value);
    // Full House: the INT2 window (index = byte offset / 4) and EXTIO.
    static constexpr unsigned kExtIoIndex = 0x40;  // IOC window offset 0x100
    u8 int2_read(unsigned index);
    bool int2_read_is_pure(unsigned index) const;
    void int2_write(unsigned index, u8 value);
    u16 ext_io() const { return ext_io_; }
    void ext_io_write(u16 value);

    // EXTIO bits (active low: 0 = the condition is present).
    enum ExtIo : u16 {
        kExtSgStat0 = 1u << 0, kExtSgStat1 = 1u << 1,
        kExtHpcBusErr = 1u << 4, kExtMcBusErr = 1u << 5, kExtEisaBusErr = 1u << 6, kExtGio33Mhz = 1u << 7,
        kExtSgRetrace = 1u << 8, kExtSgIrq1 = 1u << 9, kExtSgIrq2 = 1u << 10, kExtSgIrq3 = 1u << 11,
        kExtS0Retrace = 1u << 12, kExtS0Irq1 = 1u << 13, kExtS0Irq2 = 1u << 14, kExtS0Irq3 = 1u << 15,
    };
    // A card's GIO interrupt lines (0 FIFO, 1 graphics, 2 retrace) in a
    // slot (0 graphics, 1 and 2 the expansion slots). On the Full House the
    // graphics slot and expansion slot 0 each set their EXTIO bits and share
    // the local inputs (FIFO and graphics in local 0, the retrace latched in
    // local 1 until PORT_CONFIG clears it); expansion slot 1 has no lines
    // there. On the Indy the graphics slot's go to local 0 and 1 and each
    // expansion slot's to its mappable input (GIO EXP0, EXP1).
    void set_gio_slot_irq(unsigned slot, unsigned gio_int, bool level);
    void set_gfx_slot_irq(unsigned gio_int, bool level) { set_gio_slot_irq(0, gio_int, level); }
    // Full House: a graphics board's FIFO drain feedback lines (mappable
    // inputs 6 and 7, the Indy's expansion slot interrupts).
    void set_gfx_drain(unsigned line, bool level) {
        if (full_house_ && line < 2) set_mappable(kMapGioExp0 + line, level);
    }

    void set_local0(unsigned bit, bool level) { set_bit(local0_raw_, bit, level); }
    void set_local1(unsigned bit, bool level) { set_bit(local1_raw_, bit, level); }
    void set_mappable(unsigned bit, bool level) { set_bit(map_raw_, bit, level); }

    void set_cpu_line_callback(CpuLineCallback cb) { cpu_line_ = std::move(cb); }
    void set_power_off_callback(std::function<void()> cb) { power_off_ = std::move(cb); }

    // The Indy's front-panel buttons. Each press raises the panel interrupt;
    // the volume buttons also read as held until released.
    enum class PanelButton { Power, VolumeDown, VolumeUp };
    void panel_button(PanelButton button, bool pressed);

    Z85230& scc() { return scc_; }
    Kbc8042& kbc() { return kbc_; }
    // The MC's bus error line: recorded in the error status register (bit 1),
    // on the Full House in EXTIO.
    void set_mc_bus_error(bool level) {
        err_stat_ = static_cast<u8>(level ? (err_stat_ | 0x02) : (err_stat_ & ~0x02));
        ext_io_ = static_cast<u16>(level ? (ext_io_ & ~kExtMcBusErr) : (ext_io_ | kExtMcBusErr));
    }
    Pit8254& pit() { return pit_; }

private:
    void set_bit(u8& reg, unsigned bit, bool level);
    void update_cascade();
    void update_outputs();
    u8 local0_status() const;
    u8 local1_status() const;

    u8 int_read(unsigned reg);  // INT2/INT3 register by its INT3 index (0x20...)
    bool int_write(unsigned reg, u8 value);

    Z85230 scc_;
    Kbc8042 kbc_;
    Pit8254 pit_;
    bool full_house_;
    u8 sysid_;
    u16 ext_io_ = 0xFFFF;
    u8 port_config_ = 0;
    std::array<std::array<bool, 3>, 3> slot_irq_{};  // [slot][GIO interrupt]; the retraces latched
    void update_slot_irqs();

    // IOC registers
    std::array<u8, 10> pport_{};
    u8 panel_ = 0, dmasel_ = 0, reset_ = 0, write_ = 0;
    u8 gc_select_ = 0, gen_ctrl_ = 0;  // general control pins: directions and levels

    // INT3
    u8 local0_raw_ = 0, local1_raw_ = 0, map_raw_ = 0;
    u8 local0_mask_ = 0, local1_mask_ = 0, map_mask0_ = 0, map_mask1_ = 0, map_pol_ = 0;
    u8 err_stat_ = 0;
    std::array<bool, 2> timer_int_{};
    bool programming_pit_ = false;  // output edges caused by register writes do not interrupt
    std::array<bool, 4> lines_{};

    CpuLineCallback cpu_line_;
    std::function<void()> power_off_;
};

}  // namespace sg::ip22
