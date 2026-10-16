// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// IP22 IOC2 (Indy "Guinness"): Z85230 serial, 8042 keyboard/mouse, front
// panel, system ID, and the INT3 interrupt controller with its 8254 timers.
// Sits on HPC3 PBUS channel 6 at 0x1FBD9800; every register is one byte on
// a 4-byte stride (data at byte offset 3 on a big-endian CPU).
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

    Ioc2(Scheduler& scheduler, double cpu_hz, u64 scc_char_cycles);
    void reset();

    u8 read(unsigned index);  // index = register byte offset / 4
    bool read_is_pure(unsigned index) const;
    void write(unsigned index, u8 value);

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
    // The MC's bus error line: recorded in the error status register (bit 1).
    void set_mc_bus_error(bool level) { err_stat_ = static_cast<u8>(level ? (err_stat_ | 0x02) : (err_stat_ & ~0x02)); }
    Pit8254& pit() { return pit_; }

private:
    void set_bit(u8& reg, unsigned bit, bool level);
    void update_cascade();
    void update_outputs();
    u8 local0_status() const;
    u8 local1_status() const;

    Z85230 scc_;
    Kbc8042 kbc_;
    Pit8254 pit_;

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
