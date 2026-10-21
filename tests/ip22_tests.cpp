// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// IP22 chipset tests: MC, HPC3, IOC2/INT3 accessed at their physical addresses.
#include <algorithm>
#include <cmath>
#include <array>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "core/endian.h"
#include "cpu/r4000/r4000.h"
#include "devices/scsi_disk.h"
#include "machine/ip22/indy.h"
#include "mips_asm.h"
#include "net/net_backend.h"
#include "test.h"

using namespace sg;
using namespace sg::ip22;

namespace {

constexpr u32 kMcSysId = 0x1FA0001C;
constexpr u32 kMcMemCfg0 = 0x1FA000C4;
constexpr u32 kMcCpuErrorAddr = 0x1FA000E4;
constexpr u32 kMcCpuErrorStat = 0x1FA000EC;
constexpr u32 kMcRpssDivider = 0x1FA0002C;
constexpr u32 kMcRpssCounter = 0x1FA01004;
constexpr u32 kMcSemaphore0 = 0x1FA10004;
constexpr u32 kIoc = 0x1FBD9800;
constexpr u32 kHpc3Eeprom = 0x1FBB0008;
constexpr u32 kNvram = 0x1FBE0100;

u32 ioc_addr(unsigned reg_offset) { return kIoc + reg_offset + 3; }

// A machine whose PROM is an idle loop, so the CPU runs harmlessly.
struct IndyRig {
    Indy m;
    std::string serial;
    explicit IndyRig(unsigned ram_mb = 64, Board board = Board::Indy) : m([ram_mb, board] {
        IndyConfig c;
        c.board = board;
        c.ram_mb = ram_mb;
        if (board == Board::Indigo2R10000) {
            c.cpu_core = mips::Core::R10000;
            c.prid = 0x0926;
            c.cpu_config = mips::R4000::kR10000Config;
        }
        return c;
    }()) {
        std::vector<u8> prom(8);
        store_be<u32>(&prom[0], test::as::BEQ(0, 0, -1));
        store_be<u32>(&prom[4], test::as::NOP);
        m.load_prom_image(prom.data(), prom.size());
        m.set_serial_output([this](unsigned ch, u8 b) {
            if (ch == 1) serial.push_back(static_cast<char>(b));
        });
        m.reset();
    }
    u32 r32(u32 a) {
        u32 v = 0;
        CHECK(m.bus().read<u32>(a, v));
        return v;
    }
    void w32(u32 a, u32 v) { m.bus().write<u32>(a, v); }
    u8 r8(u32 a) {
        u8 v = 0;
        CHECK(m.bus().read<u8>(a, v));
        return v;
    }
    void w8(u32 a, u8 v) { m.bus().write<u8>(a, v); }
    bool ip(unsigned n) { return (m.cpu().cp0_read(mips::cop0::Cause) >> (8 + n)) & 1; }
};

}  // namespace

TEST(ip22_mc_identity_and_semaphores) {
    IndyRig t;
    CHECK_EQ(t.r32(kMcSysId), 0x03u);
    CHECK_EQ(t.r32(kMcSemaphore0), 0u);  // acquire
    CHECK_EQ(t.r32(kMcSemaphore0), 1u);  // already held
    t.w32(kMcSemaphore0, 0);
    CHECK_EQ(t.r32(kMcSemaphore0), 0u);
}

TEST(ip22_mc_memory_banks) {
    IndyRig t;  // 64 MiB fitted as one bank
    u32 v;
    CHECK(!t.m.bus().read<u32>(0x08000000, v));  // nothing decoded at reset
    // Bank 0: valid, 64 MiB (MSIZE 0x0F), base 0x08000000 (0x20 << 22).
    t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);
    t.w32(0x08000010, 0xDEADBEEF);
    CHECK_EQ(t.r32(0x08000010), 0xDEADBEEFu);
    CHECK_EQ(t.r32(0x00000010), 0xDEADBEEFu);  // low 512 KiB alias
    CHECK(!t.m.bus().read<u32>(0x0C000000, v));  // beyond the bank
    // Configure 128 MiB over 64 MiB of SIMMs: the upper half mirrors the lower.
    t.w32(kMcMemCfg0, (0x2000u | 0x1F00u | 0x20u) << 16);
    CHECK_EQ(t.r32(0x0C000010), 0xDEADBEEFu);
    // Two sub-banks configured over single-sided SIMMs: the second half is undriven.
    t.w32(kMcMemCfg0, (0x4000u | 0x2000u | 0x1F00u | 0x20u) << 16);
    t.w32(0x0C000010, 0x12345678);
    CHECK_EQ(t.r32(0x0C000010), 0u);
    CHECK_EQ(t.r32(0x08000010), 0xDEADBEEFu);
    // Bank 1 has no SIMMs: configuring it decodes the range without bus errors.
    t.w32(kMcCpuErrorStat, 0);
    t.w32(kMcMemCfg0, ((0x2000u | 0x1F00u | 0x20u) << 16) | 0x2000u | 0x1F00u | 0x80u);
    CHECK(t.m.bus().write<u32>(0x20000000, 0x11111111));
    CHECK_EQ(t.r32(0x20000000), 0u);
    CHECK_EQ(t.r32(kMcCpuErrorStat), 0u);
    // Invalidate: memory disappears.
    t.w32(kMcMemCfg0, 0);
    CHECK(!t.m.bus().read<u32>(0x08000010, v));
}

TEST(ip22_mc_bus_errors) {
    IndyRig t;
    u32 v;
    CHECK(!t.m.bus().read<u32>(0x18000004, v));  // reserved space: synchronous error
    CHECK_EQ(t.r32(kMcCpuErrorAddr), 0x18000004u);
    CHECK(t.r32(kMcCpuErrorStat) & 0x400);
    CHECK(!t.ip(6));
    t.w32(kMcCpuErrorStat, 0);
    CHECK_EQ(t.r32(kMcCpuErrorStat), 0u);
    t.w32(0x18000000, 1);  // posted write: interrupt on IP6
    CHECK(t.ip(6));
    t.w32(kMcCpuErrorStat, 0);
    CHECK(!t.ip(6));
    // Empty GIO expansion slot: a timeout is recorded only with ABORT_EN.
    CHECK(!t.m.bus().read<u32>(0x1F400000, v));
    CHECK_EQ(t.r32(0x1FA000FC), 0xF0u);  // GIO_ERROR_STAT: the byte lanes only
    t.w32(0x1FA000FC, 0);
    CHECK_EQ(t.r32(0x1FA0000C), 0xCu);  // CPUCTRL1 reset value
    t.w32(0x1FA0000C, 0x1C);            // ABORT_EN
    CHECK(!t.m.bus().read<u32>(0x1FB00000, v));
    CHECK_EQ(t.r32(0x1FA000F4), 0x1FB00000u);
    CHECK_EQ(t.r32(0x1FA000FC), 0x4F0u);  // TIME
}

// The big-endian CPU must use the odd word: the even one is a memory bus
// error (CPU_ERROR_STAT ADDR), synchronous for reads, by interrupt for writes.
TEST(ip22_mc_wrong_endian_register_word) {
    IndyRig t;
    u32 v;
    CHECK(!t.m.bus().read<u32>(0x1FA00000, v));
    CHECK(t.r32(kMcCpuErrorStat) & 0x400);
    CHECK_EQ(t.r32(kMcCpuErrorAddr), 0x1FA00000u);
    t.w32(kMcCpuErrorStat, 0);
    t.w32(0x1FA00040, 5);  // CTRLD's even word: not written
    CHECK(t.ip(6));
    CHECK_EQ(t.r32(0x1FA00044), 0x0C30u);
    t.w32(kMcCpuErrorStat, 0);
}

// REF_CTR counts CTRLD down at the 50 MHz system clock; DOGC counts the
// refresh bursts and clears on any MC register write.
TEST(ip22_mc_refresh_counter_and_watchdog_count) {
    IndyRig t;  // 150 MHz CPU: 3 cycles a system clock
    t.w32(0x1FA00044, 999);  // a burst every 1000 system clocks (20 us)
    t.m.run(150000);         // 1 ms: 50 bursts
    const u32 ref = t.r32(0x1FA0004C);
    CHECK(ref <= 999);
    const u32 dog = t.r32(0x1FA00014);
    CHECK(dog >= 49 && dog <= 51);
    t.w32(0x1FA0002C, 0x0104);  // any register write (here RPSS_DIVIDER) clears it
    CHECK_EQ(t.r32(0x1FA00014), 0u);
    t.m.run(3000);  // 20 us
    CHECK(t.r32(0x1FA00014) <= 1u);
}

// With CPUCTRL0.DOG set, the watchdog rolling over (2^20 bursts) resets the
// machine as power cycling does.
TEST(ip22_mc_watchdog_resets_the_machine) {
    IndyRig t;
    t.w32(0x1FA00044, 0);  // a burst every system clock: 2^20 in 21 ms
    t.w32(0x1FA00004, 0x00100012 | 0x100);  // DOG
    t.m.run(1500000);  // 10 ms: not yet
    CHECK_EQ(t.r32(0x1FA00044), 0u);
    t.m.run(3000000);  // past 21 ms: reset (CTRLD back to its reset value)
    CHECK_EQ(t.r32(0x1FA00044), 0x0C30u);
    CHECK_EQ(t.r32(0x1FA00004) & 0x100, 0u);
}

// A write to the boot PROM interrupts (GIO_ERROR_STAT PROM) unless
// CPUCTRL0.PROM_WR_EN is set; the PROM itself is unchanged.
TEST(ip22_mc_prom_write_bus_error) {
    IndyRig t;
    const u32 before = t.r32(0x1FC00000);
    t.w32(0x1FC00000, 0x12345678);
    CHECK_EQ(t.r32(0x1FC00000), before);
    CHECK(t.ip(6));
    CHECK_EQ(t.r32(0x1FA000FC), 0x800u);
    CHECK_EQ(t.r32(0x1FA000F4), 0x1FC00000u);
    t.w32(0x1FA000FC, 0);
    CHECK(!t.ip(6));
    t.w32(0x1FA00004, 0x00100012 | 0x8000);  // PROM_WR_EN
    t.w32(0x1FC00000, 0);
    CHECK(!t.ip(6));
}

// CPUCTRL0.WRST: a warm reset of the processor only (Status SR set,
// ErrorEPC the restart PC); WR_ST: its full reset sequence. The MC keeps
// its registers either way.
TEST(ip22_mc_processor_resets) {
    IndyRig t;
    t.w32(0x1FA00044, 0x1234);
    t.m.run(1000);
    t.w32(0x1FA00004, 0x00100012 | (1u << 19));
    t.m.run(1000);
    CHECK(t.m.cpu().cp0_read(mips::cop0::Status) & (1u << 20));  // SR: a soft reset
    CHECK_EQ(t.m.cpu().cp0_read(mips::cop0::ErrorEPC) & 0xFFFFFFFFu, 0xBFC00000u);  // the PROM's loop
    CHECK_EQ(t.r32(0x1FA00044), 0x1234u);
    CHECK_EQ(t.r32(0x1FA00004) & (1u << 19), 0u);
    t.w32(0x1FA00004, 0x00100012 | (1u << 16));
    t.m.run(1000);
    CHECK_EQ(t.m.cpu().cp0_read(mips::cop0::Status) & (1u << 20), 0u);  // a full reset clears SR
    CHECK_EQ(t.r32(0x1FA00044), 0x1234u);
}

// The IOC decodes 64 registers, repeated through its PBUS window; reset
// register bit 1 low (as at power-on) holds the keyboard controller in reset.
TEST(ip22_ioc_register_repeat_and_keyboard_reset) {
    IndyRig t;
    t.m.reset();
    CHECK_EQ(t.r8(0x1FBD9858 + 3), t.r8(0x1FBD9958 + 3));  // SYSID at +0x100
    CHECK_EQ(t.r8(0x1FBD9844 + 3), 0u);  // keyboard status: held, nothing answers
    t.w8(0x1FBD9870 + 3, 0x13);         // released, as the PROM does first
    CHECK(t.r8(0x1FBD9844 + 3) != 0u);
    t.w8(0x1FBD9970 + 3, 0x11);  // through the repeat: held again
    CHECK_EQ(t.r8(0x1FBD9844 + 3), 0u);
}

TEST(ip22_rpss_counts_100ns) {
    IndyRig t;
    t.w32(kMcRpssDivider, 0x0104);  // +1 every 5 system clocks at 50 MHz
    const u32 before = t.r32(kMcRpssCounter);
    t.m.run(150000);  // 1 ms at 150 MHz
    const u32 after = t.r32(kMcRpssCounter);
    CHECK(after - before >= 9999 && after - before <= 10001);
}

TEST(ip22_ioc_identity_and_panel) {
    IndyRig t;
    CHECK_EQ(t.r8(ioc_addr(0x58)), 0x26);  // sysid: Indy
    CHECK_EQ(t.r8(ioc_addr(0x50)), 0xF3);  // panel: power on, nothing pending, buttons released
}

TEST(ip22_panel_buttons_are_active_low) {
    IndyRig t;
    t.m.ioc().panel_button(ip22::Ioc2::PanelButton::Power, true);
    CHECK_EQ(t.r8(ioc_addr(0x50)), 0xF1);  // power interrupt pending
    t.w8(ioc_addr(0x50), 0x03);            // acknowledged, as Linux does
    CHECK_EQ(t.r8(ioc_addr(0x50)), 0xF3);
    t.m.ioc().panel_button(ip22::Ioc2::PanelButton::VolumeUp, true);
    CHECK_EQ(t.r8(ioc_addr(0x50)), 0x33);  // volume-up interrupt pending, button held
    t.w8(ioc_addr(0x50), 0x41);
    CHECK_EQ(t.r8(ioc_addr(0x50)), 0x73);  // acknowledged, still held
    t.m.ioc().panel_button(ip22::Ioc2::PanelButton::VolumeUp, false);
    CHECK_EQ(t.r8(ioc_addr(0x50)), 0xF3);
}

TEST(ip22_pit_counts_microseconds) {
    IndyRig t;
    // Linux-style calibration: counter 2, LSB+MSB, mode 2, count 10255.
    t.w8(ioc_addr(0xBC), 0xB4);
    t.w8(ioc_addr(0xB8), 10255 & 0xFF);
    t.w8(ioc_addr(0xB8), 10255 >> 8);
    t.m.run(150000);  // 1 ms
    t.w8(ioc_addr(0xBC), 0x80);  // latch counter 2
    const unsigned lo = t.r8(ioc_addr(0xB8)), hi = t.r8(ioc_addr(0xB8));
    const unsigned count = (hi << 8) | lo;
    CHECK(count >= 10255 - 1001 && count <= 10255 - 999);
}

TEST(ip22_pit_timer0_interrupt_via_cascade) {
    IndyRig t;
    // Counter 2: 1 MHz / 100 = 10 kHz into counter 0; counter 0: /10 -> 1 kHz.
    t.w8(ioc_addr(0xBC), 0xB4);
    t.w8(ioc_addr(0xB8), 100);
    t.w8(ioc_addr(0xB8), 0);
    t.w8(ioc_addr(0xBC), 0x34);
    t.w8(ioc_addr(0xB0), 10);
    t.w8(ioc_addr(0xB0), 0);
    t.m.run(100000);  // < 1 ms: no interrupt yet
    CHECK(!t.ip(4));
    t.m.run(200000);
    CHECK(t.ip(4));
    t.w8(ioc_addr(0xA0), 1);  // timer clear
    CHECK(!t.ip(4));
}

TEST(ip22_int3_mappable_serial_interrupt) {
    IndyRig t;
    // Unmask MAP serial (bit 5) into LOCAL0 bit 7, and LOCAL0 bit 7.
    t.w8(ioc_addr(0x94), 0x20);
    t.w8(ioc_addr(0x84), 0x80);
    // SCC channel B: master interrupt enable, Rx interrupts on all characters, Rx enable.
    const u32 ctrl_b = ioc_addr(0x30);
    t.w8(ctrl_b, 9);
    t.w8(ctrl_b, 0x08);
    t.w8(ctrl_b, 1);
    t.w8(ctrl_b, 0x10);
    t.w8(ctrl_b, 3);
    t.w8(ctrl_b, 0xC1);
    CHECK(!t.ip(2));
    t.m.serial_input(1, 'k');
    t.m.run(200000);  // input is delivered on a 1 ms guest tick
    CHECK(t.r8(ioc_addr(0x90)) & 0x20);  // MAP status: serial
    CHECK(t.r8(ioc_addr(0x80)) & 0x80);  // LOCAL0: mappable 0
    CHECK(t.ip(2));
    CHECK_EQ(t.r8(ioc_addr(0x34)), 'k');
    CHECK(!t.ip(2));
}

TEST(ip22_serial_console_output) {
    IndyRig t;
    const u32 ctrl_b = ioc_addr(0x30), data_b = ioc_addr(0x34);
    t.w8(ctrl_b, 5);
    t.w8(ctrl_b, 0x68);  // Tx enable
    for (char c : std::string("ok")) {
        t.w8(data_b, static_cast<u8>(c));
        while (!(t.r8(ctrl_b) & 0x04)) t.m.run(500);
    }
    for (int i = 0; i < 1000; ++i) {  // until RR1 All Sent
        t.w8(ctrl_b, 1);
        if (t.r8(ctrl_b) & 0x01) break;
        t.m.run(500);
    }
    CHECK(t.serial == "ok");
}

TEST(ip22_nvram_and_rtc) {
    IndyRig t;
    t.w8(kNvram + 4 * 250 + 3, 0x08);  // first byte of the Ethernet address
    CHECK_EQ(t.m.rtc().memory()[0x40 + 250], 0x08);
    CHECK_EQ(t.r32(kNvram + 4 * 250) & 0xFF, 0x08u);  // word access: data in the low byte
    const u8 month = t.r8(0x1FBE0000 + 4 * 9 + 3) & 0x1F;
    CHECK(month >= 0x01 && month <= 0x12);
    // The 8 KiB repeats through the 32K-word window.
    CHECK_EQ(t.r32(0x1FBE0000 + 0x18000 + 4 * (0x40 + 250)) & 0xFF, 0x08u);
    t.w8(0x1FBE0000 + 0x10000 + 4 * (0x40 + 251) + 3, 0x5A);
    CHECK_EQ(t.m.rtc().memory()[0x40 + 251], 0x5A);
}

TEST(ip22_nvram_default_ethernet_address) {
    IndyRig t;
    t.m.reset();
    const u8* nv = &t.m.rtc().memory()[0x40 + 250];
    CHECK_EQ(nv[0], 0x08);
    CHECK_EQ(nv[1], 0x00);
    CHECK_EQ(nv[2], 0x69);
    t.m.rtc().memory()[0x40 + 255] = 0x77;  // an address already in NVRAM is kept
    t.m.reset();
    CHECK_EQ(nv[5], 0x77);
}

namespace {

// The PROM's environment checksum, worked independently of the machine's
// own: the 256 bytes as 16-bit words, each XORed in (less the checksum byte
// itself) and rotated, from 0xA5.
u8 env_checksum(const u8* env) {
    u8 sum = 0xA5;
    for (unsigned i = 0; i < 256; i += 2) {
        sum ^= env[i + 1];
        if (i) sum ^= env[i];
        sum = static_cast<u8>(sum << 1 | sum >> 7);
    }
    return sum;
}

// A valid environment as the PROM leaves it: layout revision 8, console "g".
void set_up_env(u8* env) {
    std::fill_n(env, 250, u8{0});
    env[1] = 8;
    env[2] = 'g';
    env[0] = env_checksum(env);
}

}  // namespace

TEST(ip22_nvram_console_setting) {
    for (const Board board : {Board::Indy, Board::Indigo2}) {
        IndyConfig c;
        c.board = board;
        c.console = 'd';
        c.mac = std::array<u8, 6>{0x08, 0x00, 0x69, 0x11, 0x22, 0x33};
        Indy m{c};
        std::vector<u8> image = m.nvram_image();
        u8* env = board == Board::Indigo2 ? image.data() : image.data() + 0x40;
        set_up_env(env);
        m.load_nvram_image(image);
        m.reset();
        image = m.nvram_image();
        env = board == Board::Indigo2 ? image.data() : image.data() + 0x40;
        CHECK_EQ(env[2], 'd');
        CHECK_EQ(env[3], 0);
        CHECK_EQ(env[253], 0x11);  // the address is set too,
        CHECK_EQ(env[0], env_checksum(env));  // and the checksum still holds
    }
    // A blank environment is left for the PROM to set up with its defaults.
    IndyConfig c;
    c.console = 'd';
    Indy m{c};
    m.reset();
    const u8* env = &m.rtc().memory()[0x40];
    CHECK_EQ(env[2], 0);
    CHECK(env[0] != env_checksum(env));
}

TEST(ip22_second_scsi_port_unconnected) {
    IndyRig t;
    const u32 scsi1 = 0x1FBC8000;
    t.w8(scsi1 + 3, 0x00);  // SASR = own ID
    t.w8(scsi1 + 7, 0x07);
    t.w8(scsi1 + 3, 0x00);
    CHECK_EQ(t.r8(scsi1 + 7), 0x00);  // nothing answers: no controller
    CHECK_EQ(t.r8(scsi1 + 3), 0x00);
}

TEST(ip22_hpc3_eeprom_read) {
    IndyRig t;
    t.m.reset();
    // Read word 0 with the Linux bit-bang sequence: CS, then 11 command bits.
    auto put = [&](bool cs, bool clk, bool di) {
        t.w32(kHpc3Eeprom, (cs ? 2u : 0u) | (clk ? 4u : 0u) | (di ? 8u : 0u));
    };
    put(true, false, false);
    const u32 cmd = 0xC000;  // start bit, READ, address 0
    for (int i = 15; i >= 5; --i) {
        const bool b = (cmd >> i) & 1;
        put(true, false, b);
        put(true, true, b);
    }
    u32 v = 0;
    for (int i = 0; i < 16; ++i) {
        put(true, false, false);
        put(true, true, false);
        v = (v << 1) | ((t.r32(kHpc3Eeprom) >> 4) & 1);
    }
    put(false, false, false);
    CHECK_EQ(v, 0xFFFFu);  // erased part
}

TEST(ip22_events_scheduled_by_running_code_fire_on_time) {
    // A PROM that sends one character: the transmit completion event is
    // scheduled while the CPU runs and must not wait for later events.
    using namespace test::as;
    Indy m{IndyConfig{}};
    std::string out;
    m.set_serial_output([&](unsigned, u8 b) { out.push_back(static_cast<char>(b)); });
    const u32 prog[] = {LUI(t0, 0xBFBD), ORI(t0, t0, 0x9833), ADDIU(t1, zero, 5), I(0x28, t0, t1, 0),
                        ADDIU(t1, zero, 0x68), I(0x28, t0, t1, 0), ADDIU(t1, zero, 'X'), I(0x28, t0, t1, 4),
                        BEQ(zero, zero, -1), NOP};
    std::vector<u8> prom(sizeof(prog));
    for (std::size_t i = 0; i < std::size(prog); ++i) store_be<u32>(&prom[i * 4], prog[i]);
    m.load_prom_image(prom.data(), prom.size());
    m.reset();
    m.run(20000);
    CHECK(out == "X");
}

namespace {
constexpr u32 kHal2 = 0x1FBD8000;     // HPC3 PBUS PIO channel 0
constexpr u32 kPbus0 = 0x1FB80000;    // PBUS DMA channel 0: bptr, dptr; control at +0x1000
constexpr u32 kPbusCfg0 = 0x1FBDC000;  // PBUS DMA channel 0 configuration
constexpr u32 kHpcIntStat = 0x1FBB0000;

void hal2_write(IndyRig& t, u32 reg, u32 value) {
    t.w32(kHal2 + 0x40, value & 0xFFFF);
    t.w32(kHal2 + 0x50, value >> 16);
    t.w32(kHal2 + 0x30, reg);
}
u32 hal2_read(IndyRig& t, u32 reg) {
    t.w32(kHal2 + 0x30, reg | 0x80);
    const u32 lo = t.r32(kHal2 + 0x40) & 0xFFFF;  // a 16-bit device: both halves of the word
    t.w32(kHal2 + 0x30, reg | 0x81);
    return lo | ((t.r32(kHal2 + 0x40) & 0xFFFF) << 16);
}
}  // namespace

// PBUS device registers fill the word: an 8-bit register's byte in every
// lane (the IOC's SYSID), a 16-bit one's halfword in both halves (HAL2).
TEST(ip22_pbus_registers_are_replicated) {
    IndyRig t;
    CHECK_EQ(t.r32(0x1FBD9858), 0x26262626u);
    CHECK_EQ(t.r8(0x1FBD9858), 0x26);
    CHECK_EQ(t.r8(0x1FBD9858 + 1), 0x26);
    CHECK_EQ(t.r8(0x1FBD9858 + 3), 0x26);
    t.w32(kHal2 + 0x10, 0x18);
    const u32 rev = t.r32(kHal2 + 0x20);  // HAL2 revision
    CHECK_EQ(rev >> 16, rev & 0xFFFF);
}

TEST(ip22_hal2_indirect_registers_read_back) {
    IndyRig t;
    t.w32(kHal2 + 0x10, 0x18);  // out of reset
    hal2_write(t, 0x1408, 0x000103CC);
    CHECK_EQ(hal2_read(t, 0x1408), 0x000103CCu);
    CHECK_EQ(t.r32(kHal2 + 0x10) & 1, 0u);  // indirect access never busy
}

// The DAC, clocked at 8 kHz, plays a two-descriptor chain on PBUS channel 0:
// each descriptor takes as long as its frames last, raises the DMA
// interrupt, and the chain stops at its end.
TEST(ip22_hal2_dac_dma_runs_at_the_sample_rate) {
    IndyRig t;
    t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);  // 64 MiB at 0x08000000
    std::vector<s16> played;
    t.m.hpc3().set_audio_sink([&](const s16* s, std::size_t frames, unsigned voices, double rate) {
        CHECK_EQ(voices, 2u);
        CHECK(rate == 8000.0);
        played.insert(played.end(), s, s + frames * voices);
    });
    // 100 stereo frames per descriptor, a 32-bit word (24-bit sample) per
    // voice: the channel's 16-bit device bus mode (DS16), as the PROM sets it
    t.w32(kPbusCfg0, 0x08248844);
    constexpr u32 kDesc = 0x08100000, kBuf = 0x08101000, kBytes = 800;
    for (u32 i = 0; i < 2 * kBytes / 4; ++i) t.w32(kBuf + 4 * i, (static_cast<u32>(static_cast<s32>(i * 3 - 200)) << 8) & 0xFFFFFF00u);
    t.w32(kDesc + 0, kBuf);
    t.w32(kDesc + 4, (1u << 29) | kBytes);  // XIE
    t.w32(kDesc + 8, kDesc + 16);
    t.w32(kDesc + 16, kBuf + kBytes);
    t.w32(kDesc + 20, (1u << 31) | (1u << 29) | kBytes);  // EOX, XIE
    t.w32(kDesc + 24, 0);

    t.w32(kHal2 + 0x10, 0x18);
    hal2_write(t, 0x2104, 0);           // generator 1 from 48 kHz...
    hal2_write(t, 0x2108, 0xFFFA0001);  // ...times 1/6
    hal2_write(t, 0x1404, 0x208);       // DAC: PBUS channel 0, clock 1, stereo
    hal2_write(t, 0x9104, 0x08);        // codec transmit DMA port on
    t.w32(kHal2 + 0x800, 0xFF);         // full volume
    t.w32(kHal2 + 0x804, 0xFF);
    t.w32(kPbus0 + 4, kDesc);
    t.w32(kPbus0 + 0x1000, 0x30);       // LD | ACT
    CHECK_EQ(t.r32(kPbus0 + 0x1000), 2u);  // active

    const u64 per_desc = static_cast<u64>(t.m.cycles_per_second() * 100 / 8000);
    t.m.run(per_desc * 9 / 10);
    CHECK(played.empty());
    CHECK_EQ(t.r32(kHpcIntStat) & 1, 0u);
    t.m.run(per_desc * 2 / 10);
    CHECK_EQ(played.size(), 200u);
    CHECK_EQ(t.r32(kHpcIntStat) & 1, 1u);
    CHECK_EQ(t.r32(kPbus0 + 0x1000), 3u);  // interrupt and active; reading clears the interrupt
    CHECK_EQ(t.r32(kPbus0 + 0x1000), 2u);
    t.m.run(per_desc);
    CHECK_EQ(played.size(), 400u);
    CHECK_EQ(t.r32(kPbus0 + 0x1000), 1u);  // done
    bool same = true;
    for (u32 i = 0; i < played.size(); ++i) same &= played[i] == static_cast<s16>(i * 3 - 200);
    CHECK(same);
}

// Without DS16 (the configuration the NetBSD and Linux drivers use), the
// samples are packed 16-bit values.
TEST(ip22_hal2_dac_dma_packed_16_bit_samples) {
    IndyRig t;
    t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);
    std::vector<s16> played;
    t.m.hpc3().set_audio_sink([&](const s16* s, std::size_t frames, unsigned voices, double rate) {
        CHECK_EQ(voices, 2u);
        CHECK(rate == 48000.0);
        played.insert(played.end(), s, s + frames * voices);
    });
    constexpr u32 kDesc = 0x08100000, kBuf = 0x08101000, kBytes = 400;  // 100 stereo frames
    for (u32 i = 0; i < kBytes / 4; ++i)
        t.w32(kBuf + 4 * i, (static_cast<u32>(static_cast<u16>(i * 7 - 300)) << 16) | static_cast<u16>(-static_cast<s32>(i)));
    t.w32(kDesc + 0, kBuf);
    t.w32(kDesc + 4, (1u << 31) | kBytes);  // EOX
    t.w32(kDesc + 8, 0);

    t.w32(kPbusCfg0, 0x08208844);
    t.w32(kHal2 + 0x10, 0x18);
    hal2_write(t, 0x2104, 0);           // generator 1 from 48 kHz...
    hal2_write(t, 0x2108, 0xFFFF0004);  // ...times 4/4, as NetBSD writes it
    hal2_write(t, 0x1404, 0x208);       // DAC: PBUS channel 0, clock 1, stereo
    hal2_write(t, 0x9104, 0x08);
    t.w32(kHal2 + 0x800, 0xFF);  // full volume
    t.w32(kHal2 + 0x804, 0xFF);
    t.w32(kPbus0 + 4, kDesc);
    t.w32(kPbus0 + 0x1000, 0x30);

    const u64 duration = static_cast<u64>(t.m.cycles_per_second() * 100 / 48000);
    t.m.run(duration * 4 / 10);
    CHECK(played.empty());
    // Software that follows the buffer pointer refills what lies behind it:
    // the frames below it have been played by then.
    const u32 at = t.r32(kPbus0) - kBuf;
    CHECK(at >= 36 * 4 && at <= 44 * 4);
    CHECK_EQ(played.size(), at / 4 * 2);
    t.m.run(duration * 7 / 10);
    CHECK_EQ(played.size(), 200u);
    bool same = true;
    for (u32 i = 0; i < played.size() / 2; ++i)
        same &= played[2 * i] == static_cast<s16>(i * 7 - 300) && played[2 * i + 1] == static_cast<s16>(-static_cast<s32>(i));
    CHECK(same);
}

// The ADC records what the host supplies, in the layout of its PBUS
// channel: packed 16-bit samples, or 24-bit values in 32-bit words (DS16).
TEST(ip22_hal2_adc_records_from_the_audio_source) {
    for (const bool ds16 : {false, true}) {
        IndyRig t;
        t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);
        unsigned calls = 0;
        std::size_t next = 0;  // the source's running sample count
        t.m.hpc3().set_audio_source([&](s16* s, std::size_t frames, unsigned voices, double rate) {
            CHECK_EQ(voices, 2u);
            CHECK(rate == 48000.0);
            for (std::size_t i = 0; i < frames * voices; ++i, ++next) s[i] = static_cast<s16>(100 * next - 1000);
            ++calls;
        });
        const u32 size = ds16 ? 4 : 2;
        constexpr u32 kDesc = 0x08100000, kBuf = 0x08101000;
        const u32 bytes = 50 * 2 * size;  // 50 stereo frames
        t.w32(kDesc + 0, kBuf);
        t.w32(kDesc + 4, (1u << 31) | bytes);  // EOX
        t.w32(kDesc + 8, 0);
        t.w32(kPbusCfg0 + 0x200, ds16 ? 0x08248844 : 0x08208844);  // channel 1
        t.w32(kHal2 + 0x10, 0x18);
        hal2_write(t, 0x2104, 0);
        hal2_write(t, 0x2108, 0xFFFF0004);  // 48 kHz
        hal2_write(t, 0x1504, 0x209);       // ADC: PBUS channel 1, clock 1, stereo
        hal2_write(t, 0x9104, 0x10);        // codec receive DMA port on
        t.w32(kPbus0 + 0x2000 + 4, kDesc);
        t.w32(kPbus0 + 0x2000 + 0x1000, 0x34);  // LD | ACT | receive
        // Halfway through, what lies below the buffer pointer has arrived.
        t.m.run(static_cast<u64>(t.m.cycles_per_second() * 25 / 48000));
        const u32 at = t.r32(kPbus0 + 0x2000) - kBuf;
        CHECK(at >= 20 * 2 * size && at < bytes);
        CHECK_EQ(calls, 1u);
        CHECK_EQ(t.r32(kBuf), ds16 ? static_cast<u32>(-1000 * 256) : (static_cast<u32>(static_cast<u16>(-1000)) << 16) | static_cast<u16>(-900));
        t.m.run(static_cast<u64>(t.m.cycles_per_second() * 35 / 48000));
        CHECK_EQ(calls, 2u);
        bool same = true;
        for (u32 i = 0; i < 100; ++i) {
            const s16 want = static_cast<s16>(100 * i - 1000);
            if (ds16) same &= t.r32(kBuf + 4 * i) == static_cast<u32>(static_cast<s32>(want) * 256);
            else same &= static_cast<s16>(t.r32(kBuf + 4 * (i / 2)) >> (i % 2 ? 0 : 16)) == want;
        }
        CHECK(same);
    }
}

// The rate generators' 16-bit increment and modulus count 0 as 65536: IRIX
// sets 44.1 kHz as 44.1 kHz * 0/0, Linux as 44.1 kHz * 4/4.
TEST(ip22_hal2_rate_fields_of_zero_mean_65536) {
    for (const u32 c2 : {0xFFFF0000u, 0xFFFF0004u}) {
        IndyRig t;
        t.w32(kHal2 + 0x10, 0x18);
        hal2_write(t, 0x2204, 1);    // generator 2: the 44.1 kHz master
        hal2_write(t, 0x2208, c2);   // inc - mod - 1 = -1, inc = 0 (or 4)
        hal2_write(t, 0x1404, 0x110);  // DAC: PBUS channel 0, clock 2, mono
        hal2_write(t, 0x9104, 0x08);   // codec transmit DMA port on
        const auto port = t.m.hpc3().hal2().port(Hal2::kDac);
        CHECK(port.enabled);
        CHECK(port.rate == 44100.0);
    }
}

// ADC_C2 amplifies each side in 1.5 dB steps, clipping at full scale; a mic
// side starts 9 dB down.
TEST(ip22_hal2_adc_input_gain) {
    for (const bool ds16 : {false, true}) {
        IndyRig t;
        t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);
        t.m.hpc3().set_audio_source([&](s16* s, std::size_t frames, unsigned voices, double) {
            for (std::size_t i = 0; i < frames * voices; ++i) s[i] = i % 2 ? 3000 : 1000;
        });
        const u32 size = ds16 ? 4 : 2;
        constexpr u32 kDesc = 0x08100000, kBuf = 0x08101000;
        const u32 bytes = 8 * 2 * size;  // 8 stereo frames
        t.w32(kDesc + 0, kBuf);
        t.w32(kDesc + 4, (1u << 31) | bytes);
        t.w32(kDesc + 8, 0);
        t.w32(kPbusCfg0 + 0x200, ds16 ? 0x08248844 : 0x08208844);
        t.w32(kHal2 + 0x10, 0x18);
        hal2_write(t, 0x2104, 0);
        hal2_write(t, 0x2108, 0xFFFF0004);  // 48 kHz
        hal2_write(t, 0x1504, 0x209);       // ADC: PBUS channel 1, clock 1, stereo
        hal2_write(t, 0x1508, 0x24F);       // ADC_C2: left +6 dB on mic (-9 dB trim), right +22.5 dB on line
        hal2_write(t, 0x9104, 0x10);
        t.w32(kPbus0 + 0x2000 + 4, kDesc);
        t.w32(kPbus0 + 0x2000 + 0x1000, 0x34);  // LD | ACT | receive
        t.m.run(static_cast<u64>(t.m.cycles_per_second() * 20 / 48000));
        if (ds16) {
            CHECK_EQ(t.r32(kBuf), static_cast<u32>(std::lround(1000 * 256 * std::pow(10.0, -3.0 / 20))));
            CHECK_EQ(t.r32(kBuf + 4), 0x7FFFFFu);  // clipped
        } else {
            CHECK_EQ(t.r32(kBuf), (static_cast<u32>(std::lround(1000 * std::pow(10.0, -3.0 / 20))) << 16) | 0x7FFFu);
        }
    }
}

// DAC_C2 attenuates each side in 1.5 dB steps, or mutes both; the volume
// registers (8-bit levels, 0 silent) scale the result.
TEST(ip22_hal2_dac_attenuation_mute_and_volume) {
    for (const bool mute : {false, true}) {
        IndyRig t;
        t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);
        std::vector<s16> played;
        t.m.hpc3().set_audio_sink([&](const s16* s, std::size_t frames, unsigned voices, double) {
            played.insert(played.end(), s, s + frames * voices);
        });
        constexpr u32 kDesc = 0x08100000, kBuf = 0x08101000;
        for (u32 i = 0; i < 4; ++i) t.w32(kBuf + 4 * i, 0x40004000);  // 16384 on both sides
        t.w32(kDesc + 0, kBuf);
        t.w32(kDesc + 4, (1u << 31) | 16);
        t.w32(kDesc + 8, 0);
        t.w32(kPbusCfg0, 0x08208844);
        t.w32(kHal2 + 0x10, 0x18);
        hal2_write(t, 0x2104, 0);
        hal2_write(t, 0x2108, 0xFFFF0004);
        hal2_write(t, 0x1404, 0x208);
        hal2_write(t, 0x1408, (4u << 23) | (mute ? 0x400u : 0));  // left -6 dB, right 0 dB
        hal2_write(t, 0x9104, 0x08);
        t.w32(kHal2 + 0x800, 0x80);  // right at about half volume
        t.w32(kHal2 + 0x804, 0xFF);
        t.w32(kPbus0 + 4, kDesc);
        t.w32(kPbus0 + 0x1000, 0x30);
        t.m.run(static_cast<u64>(t.m.cycles_per_second() * 10 / 48000));
        CHECK_EQ(played.size(), 8u);
        if (played.size() == 8) {
            CHECK_EQ(played[0], mute ? 0 : 8211);  // 16384 x 10^(-6/20)
            CHECK_EQ(played[1], mute ? 0 : 8224);  // 16384 x 128/255
        }
    }
}

// A chain of empty descriptors that loops forever must not hang the emulator.
TEST(ip22_pbus_dma_follows_an_endless_empty_chain_without_hanging) {
    IndyRig t;
    t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);
    constexpr u32 kDesc = 0x08100000;
    t.w32(kDesc + 0, 0);
    t.w32(kDesc + 4, 0);  // no data, no end of chain...
    t.w32(kDesc + 8, kDesc);  // ...and pointing to itself
    t.w32(kHal2 + 0x10, 0x18);
    hal2_write(t, 0x2104, 0);
    hal2_write(t, 0x2108, 0xFFFF0001);  // 48 kHz
    hal2_write(t, 0x1404, 0x208);
    hal2_write(t, 0x9104, 0x08);
    t.w32(kPbus0 + 4, kDesc);
    t.w32(kPbus0 + 0x1000, 0x30);
    t.m.run(1000000);
    CHECK_EQ(t.r32(kPbus0 + 0x1000), 2u);  // still going
}

namespace {
constexpr u32 kEnetRxNdbp = 0x1FB94004, kEnetRxCtrl = 0x1FB95004, kEnetReset = 0x1FB95014;
constexpr u32 kEnetTxNdbp = 0x1FB96004, kEnetTxCtrl = 0x1FB97004, kSeeq = 0x1FBD4000;
constexpr u32 kOwn = 1u << 14, kTxDone = 1u << 15, kEop = 1u << 30, kEnd = 1u << 31, kXie = 1u << 29;

// Records what the guest sends; frames for the guest are queued by hand.
class LoopBackend final : public net::NetBackend {
public:
    void send(const u8* frame, std::size_t length) override { sent.emplace_back(frame, frame + length); }
    void inject(const std::vector<u8>& frame) { deliver(frame.data(), frame.size()); }
    std::vector<std::vector<u8>> sent;
};

struct EnetRig : IndyRig {
    LoopBackend net;
    EnetRig() {
        w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);
        m.set_network(&net);
        w32(kEnetReset, 3);  // reset the channels and clear the interrupt
        w32(kEnetReset, 0);
        const u8 mac[6] = {0x08, 0x00, 0x69, 0x0c, 0x5e, 0x01};
        w32(kSeeq + 28, 0);  // transmit command: register bank 0
        for (unsigned i = 0; i < 6; ++i) w32(kSeeq + 4 * i, mac[i]);
        w32(kSeeq + 28, 0x08);  // interrupt when a frame has been sent
        w32(kSeeq + 24, 0xA0);  // receive own address and broadcasts, interrupt on good frames
    }
    bool enet_irq() { return (r8(ioc_addr(0x80)) >> Ioc2::kL0Ethernet) & 1; }  // LOCAL0 status
};

std::vector<u8> test_frame(const std::array<u8, 6>& dst, std::size_t length) {
    std::vector<u8> f(length);
    for (std::size_t i = 0; i < length; ++i) f[i] = static_cast<u8>(i * 7 + 1);
    std::copy(dst.begin(), dst.end(), f.begin());
    return f;
}
}  // namespace

TEST(ip22_ethernet_transmits_a_frame_across_descriptors) {
    EnetRig t;
    constexpr u32 kDesc = 0x08200000, kBuf = 0x08201000;
    const std::vector<u8> frame = test_frame({0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, 100);
    for (std::size_t i = 0; i < frame.size(); ++i) t.m.bus().write<u8>(static_cast<u32>(kBuf + i), frame[i]);
    t.w32(kDesc + 0, kBuf);  // the frame in two pieces
    t.w32(kDesc + 4, 40);
    t.w32(kDesc + 8, kDesc + 16);
    t.w32(kDesc + 16, kBuf + 40);
    t.w32(kDesc + 20, kEnd | kEop | 60);
    t.w32(kDesc + 24, 0);
    t.w32(kEnetTxNdbp, kDesc);
    t.w32(kEnetTxCtrl, 0x200);  // active
    t.m.run(200000);
    CHECK_EQ(t.net.sent.size(), 1u);
    if (!t.net.sent.empty()) CHECK(t.net.sent[0] == frame);
    CHECK_EQ(t.r32(kDesc + 4), kTxDone);  // TXD in the packet's first descriptor, its count cleared
    CHECK_EQ(t.r32(kDesc + 20), kEnd | kEop | 60);  // the others untouched
    CHECK_EQ(t.r32(kEnetTxCtrl), 0x08u);  // sent, no longer active
    CHECK(t.enet_irq());
    t.w32(kEnetReset, 2);  // clear the interrupt
    CHECK(!t.enet_irq());
}

TEST(ip22_ethernet_receives_into_owned_descriptors_only) {
    EnetRig t;
    constexpr u32 kDesc = 0x08300000, kBuf0 = 0x08301000, kBuf1 = 0x08302000, kSize = 1586;
    t.w32(kDesc + 0, kBuf0);
    t.w32(kDesc + 4, kOwn | kXie | kSize);
    t.w32(kDesc + 8, kDesc + 16);
    t.w32(kDesc + 16, kBuf1);
    t.w32(kDesc + 20, kEnd | kOwn | kXie | kSize);  // end of the ring
    t.w32(kDesc + 24, kDesc);
    t.w32(kEnetRxNdbp, kDesc);
    t.w32(kEnetRxCtrl, 0x200);

    const std::array<u8, 6> us = {0x08, 0x00, 0x69, 0x0c, 0x5e, 0x01};
    t.net.inject(test_frame({0x08, 0x00, 0x69, 0x00, 0x00, 0x02}, 80));  // for someone else: ignored
    const std::vector<u8> mine = test_frame(us, 200);
    t.net.inject(mine);
    t.net.inject(test_frame({0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, 40));  // broadcast, padded to 60
    t.net.inject(test_frame(us, 100));  // no buffer left: lost
    t.m.run(static_cast<u64>(t.m.cycles_per_second() / 50));  // 20 ms

    // First buffer: two bytes of padding, the frame, the status byte; OWN
    // cleared and the unused count left behind.
    const u32 c0 = t.r32(kDesc + 4);
    CHECK(!(c0 & kOwn));
    CHECK_EQ(c0 & 0x3FFF, kSize - (2 + 200 + 1));
    bool same = true;
    for (std::size_t i = 0; i < mine.size(); ++i) {
        u8 b = 0;
        t.m.bus().read<u8>(static_cast<u32>(kBuf0 + 2 + i), b);
        same &= b == mine[i];
    }
    CHECK(same);
    u8 status = 0;
    t.m.bus().read<u8>(kBuf0 + 2 + 200, status);
    CHECK_EQ(status, 0x30u);  // good frame, end of frame
    CHECK_EQ(t.r32(kDesc + 20) & 0x3FFF, kSize - (2 + 60 + 1));
    CHECK(!(t.r32(kEnetRxCtrl) & 0x200));  // stopped at the end of the ring...
    CHECK_EQ(t.r32(0x1FB98000), kDesc + 16);  // ...at this descriptor, as the driver reads back to restart
    CHECK(t.enet_irq());
    // Restarted on a buffer the driver has not handed back: nothing is written.
    u32 before = t.r32(kBuf0 + 4);
    t.w32(kEnetRxNdbp, kDesc);
    t.w32(kEnetRxCtrl, 0x200);
    t.net.inject(test_frame(us, 120));
    t.m.run(static_cast<u64>(t.m.cycles_per_second() / 100));
    CHECK_EQ(t.r32(kBuf0 + 4), before);
    CHECK_EQ(t.r32(kDesc + 4), c0);
}

// One buffer per received packet: a packet larger than its buffer fills it,
// sets RBO (cleared by reading the control register) and stops the channel.
TEST(ip22_ethernet_receive_buffer_overflow) {
    EnetRig t;
    constexpr u32 kDesc = 0x08300000, kBuf0 = 0x08301000, kBuf1 = 0x08302000;
    t.w32(kDesc + 0, kBuf0);
    t.w32(kDesc + 4, kOwn | kXie | kEop | 64);  // too small for the frame
    t.w32(kDesc + 8, kDesc + 16);
    t.w32(kDesc + 16, kBuf1);
    t.w32(kDesc + 20, kEnd | kOwn | kXie | kEop | 1586);
    t.w32(kDesc + 24, kDesc);
    t.w32(kEnetRxNdbp, kDesc);
    t.w32(kEnetRxCtrl, 0x200);
    const std::array<u8, 6> us = {0x08, 0x00, 0x69, 0x0c, 0x5e, 0x01};
    const std::vector<u8> big = test_frame(us, 200);
    t.net.inject(big);
    t.m.run(static_cast<u64>(t.m.cycles_per_second() / 100));
    CHECK(t.enet_irq());
    const u32 ctrl = t.r32(kEnetRxCtrl);
    CHECK(ctrl & 0x800);     // RBO
    CHECK(!(ctrl & 0x200));  // stopped
    CHECK(!(t.r32(kEnetRxCtrl) & 0x800));  // read: cleared
    CHECK_EQ(t.r32(kDesc + 4) & 0x3FFF, 0u);  // the buffer filled
    u8 b = 0;
    t.m.bus().read<u8>(kBuf0 + 2, b);
    CHECK_EQ(b, big[0]);
    CHECK_EQ(t.r32(kDesc + 20) & 0x3FFF, 1586u);  // the next buffer untouched
}

// With the channel's endian bit set, byte n of each doubleword travels in
// its low-order bits: in the big-endian memory, the frame's bytes sit at
// their addresses XOR 7, both ways.
TEST(ip22_ethernet_little_endian_transfers) {
    EnetRig t;
    constexpr u32 kDesc = 0x08200000, kBuf = 0x08201000, kRxDesc = 0x08300000, kRxBuf = 0x08301000;
    const std::vector<u8> frame = test_frame({0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, 64);
    for (std::size_t i = 0; i < frame.size(); ++i) t.m.bus().write<u8>(static_cast<u32>(kBuf + i) ^ 7, frame[i]);
    t.w32(kDesc + 0, kBuf);
    t.w32(kDesc + 4, kEnd | kEop | 64);
    t.w32(kDesc + 8, 0);
    t.w32(kEnetTxNdbp, kDesc);
    t.w32(kEnetTxCtrl, 0x300);  // active, little-endian
    t.m.run(200000);
    CHECK_EQ(t.net.sent.size(), 1u);
    if (!t.net.sent.empty()) CHECK(t.net.sent[0] == frame);

    t.w32(kRxDesc + 0, kRxBuf);
    t.w32(kRxDesc + 4, kEnd | kOwn | 1586);
    t.w32(kRxDesc + 8, 0);
    t.w32(kEnetRxNdbp, kRxDesc);
    t.w32(kEnetRxCtrl, 0x300);
    const std::vector<u8> mine = test_frame({0x08, 0x00, 0x69, 0x0c, 0x5e, 0x01}, 100);
    t.net.inject(mine);
    t.m.run(static_cast<u64>(t.m.cycles_per_second() / 100));
    bool same = true;
    for (std::size_t i = 0; i < mine.size(); ++i) {
        u8 b = 0;
        t.m.bus().read<u8>(static_cast<u32>(kRxBuf + 2 + i) ^ 7, b);
        same &= b == mine[i];
    }
    CHECK(same);
    u8 status = 0;
    t.m.bus().read<u8>((kRxBuf + 2 + 100) ^ 7, status);
    CHECK_EQ(status, 0x30u);
}

// A host that answers the moment a frame is handed to it (the NAT replying
// to a ping to its own address) is heard only after the request has left
// the wire and a turnaround: IRIX loses replies that land sooner.
TEST(ip22_ethernet_replies_wait_for_the_wire_and_a_turnaround) {
    class AnsweringBackend final : public net::NetBackend {
    public:
        void send(const u8*, std::size_t) override { deliver(reply.data(), reply.size()); }
        std::vector<u8> reply;
    };
    for (const bool fast : {false, true}) {
        EnetRig t;
        AnsweringBackend net;
        net.reply = test_frame({0x08, 0x00, 0x69, 0x0c, 0x5e, 0x01}, 98);
        t.m.set_network(&net);
        t.m.set_fast_io(fast);
        constexpr u32 kRx = 0x08300000, kTx = 0x08310000, kBuf = 0x08301000, kTxBuf = 0x08311000, kSize = 1586;
        t.w32(kRx + 0, kBuf);
        t.w32(kRx + 4, kEnd | kOwn | kXie | kSize);
        t.w32(kRx + 8, kRx);
        t.w32(kEnetRxNdbp, kRx);
        t.w32(kEnetRxCtrl, 0x200);
        t.w32(kTx + 0, kTxBuf);
        t.w32(kTx + 4, kEnd | kEop | 98);
        t.w32(kTx + 8, 0);
        t.w32(kEnetTxNdbp, kTx);
        t.w32(kEnetTxCtrl, 0x200);
        const double hz = t.m.cycles_per_second();
        t.m.run(static_cast<u64>(hz * 100e-6));
        CHECK(t.r32(kTx + 4) & kTxDone);         // sent...
        CHECK(t.r32(kRx + 4) & kOwn);            // ...and nothing back yet
        t.m.run(static_cast<u64>(hz * 1e-3));
        CHECK(!(t.r32(kRx + 4) & kOwn));         // the reply, within a millisecond
        CHECK_EQ(t.r32(kRx + 4) & 0x3FFF, kSize - (2 + 98 + 1));
    }
}

// With fast I/O a frame waits for a buffer instead of being lost, and frames
// follow each other far faster than on a 10 Mbit/s wire.
TEST(ip22_ethernet_fast_io_holds_frames_until_a_buffer_is_ready) {
    EnetRig t;
    t.m.set_fast_io(true);
    constexpr u32 kDesc = 0x08300000, kBuf0 = 0x08301000, kBuf1 = 0x08302000, kSize = 1586;
    t.w32(kDesc + 0, kBuf0);
    t.w32(kDesc + 4, kEnd | kOwn | kXie | kSize);  // a single buffer
    t.w32(kDesc + 8, kDesc);
    t.w32(kEnetRxNdbp, kDesc);
    t.w32(kEnetRxCtrl, 0x200);
    const std::array<u8, 6> us = {0x08, 0x00, 0x69, 0x0c, 0x5e, 0x01};
    const std::vector<u8> first = test_frame(us, 1500), second = test_frame(us, 1400);
    t.net.inject(first);
    t.net.inject(second);
    t.m.run(static_cast<u64>(t.m.cycles_per_second() / 1000));  // 1 ms: 10 Mbit/s would carry one frame
    CHECK_EQ(t.r32(kDesc + 4) & 0x3FFF, kSize - (2 + 1500 + 1));
    CHECK(!(t.r32(kEnetRxCtrl) & 0x200));  // stopped at the end of the ring
    t.w32(kDesc + 0, kBuf1);  // the driver hands over a new buffer and restarts
    t.w32(kDesc + 4, kEnd | kOwn | kXie | kSize);
    t.w32(kEnetRxNdbp, kDesc);
    t.w32(kEnetRxCtrl, 0x200);
    t.m.run(static_cast<u64>(t.m.cycles_per_second() / 5000));  // 200 us
    CHECK_EQ(t.r32(kDesc + 4) & 0x3FFF, kSize - (2 + 1400 + 1));
    bool same = true;
    for (std::size_t i = 0; i < second.size(); ++i) {
        u8 b = 0;
        t.m.bus().read<u8>(static_cast<u32>(kBuf1 + 2 + i), b);
        same &= b == second[i];
    }
    CHECK(same);
}

// Typed-ahead input is fed one byte at a time as the guest takes it; the
// feeder only runs while something is queued, and starts again for input
// that arrives later.
TEST(ip22_serial_input_is_fed_in_order_and_on_demand) {
    IndyRig t;
    const u32 ctrl_b = ioc_addr(0x30), data_b = ioc_addr(0x34);
    t.w8(ctrl_b, 3);
    t.w8(ctrl_b, 0xC1);  // receiver on, 8 bits
    for (char c : {'a', 'b', 'c'}) t.m.serial_input(1, static_cast<u8>(c));
    std::string got;
    for (int i = 0; i < 3; ++i) {
        t.m.run(400000);  // a little over a millisecond at 150 MHz
        if (t.r8(ctrl_b) & 1) got.push_back(static_cast<char>(t.r8(data_b)));
    }
    CHECK(got == "abc");
    t.m.run(1000000);  // nothing queued
    CHECK(!(t.r8(ctrl_b) & 1));
    t.m.serial_input(1, 'd');
    t.m.run(400000);
    CHECK(t.r8(ctrl_b) & 1);
    CHECK_EQ(t.r8(data_b), 'd');
}

// Software finds the graphics board by probing the base of its slot; the
// REX3 registers sit at +0xF0000 within it.
TEST(indy_newport_answers_across_the_graphics_slot) {
    IndyRig t;
    u32 v = 1;
    CHECK(t.m.bus().read<u32>(0x1F000000, v));
    CHECK_EQ(v, 4u);  // no fault: the access width (see below)
    CHECK(t.m.bus().write<u32>(0x1F0F0148, 0x12345678));  // XSTARTI
    CHECK_EQ(t.r32(0x1F0F0100), 0x02B3C000u);              // XSTART
    CHECK(t.m.bus().read<u32>(0x1F400000 - 4, v));
    CHECK(!t.m.bus().read<u32>(0x1F400000, v));  // empty expansion slot

    IndyConfig cfg;
    cfg.graphics = false;
    Indy bare{cfg};
    CHECK(!bare.bus().read<u32>(0x1F000000, v));
}

// The MC's virtual DMA sends a memory rectangle to one GIO address, the
// device's data port: here REX3's host data register with GO, drawing a
// block of host-supplied pixels as IRIX's graphics library does.
// The DMA TLB's page-table pointer reaches the whole low memory window: with
// 256 MiB, IRIX keeps page tables above 0x10000000.
TEST(ip22_mc_dma_page_table_in_high_memory) {
    IndyRig t(256);
    t.w32(kMcMemCfg0, 0x7F207F40u);  // 128 MiB at 0x08000000 and at 0x10000000
    constexpr u32 kMc = 0x1FA00000, kTable = 0x14000000, kPage = 0x15000000, kRex3 = 0x1F0F0000, kGo = 0x800;
    constexpr u32 kVirt = 0x00400000;  // a 4 MiB DMA segment
    t.w32(kTable + 0, ((kPage >> 12) << 6) | 2);  // PTE for the segment's first page: PFN, valid
    for (u32 i = 0; i < 8; ++i) t.w8(kPage + i, static_cast<u8>(0x40 + i));
    t.w32(kMc + 0x16C, 0x100);                  // DMA_CTL: Xlate
    t.w32(kMc + 0x184, kVirt);                  // DMA_TLB_HI_0
    t.w32(kMc + 0x18C, ((kTable >> 12) << 6) | 2);  // DMA_TLB_LO_0: the page table, valid
    t.w32(kRex3 + 0x000, 0x30000000u | 0x400u | 0x100u | 0x80u | 0x8u | 1u | 0x7000u);  // 8-bit host pixels, 8 a word
    t.w32(kRex3 + 0x220, 0xFF);
    t.w32(kRex3 + 0x1328, 0x1E00);
    t.w32(kRex3 + 0x150, 0);
    t.w32(kRex3 + 0x154, (7u << 16) | 0u);
    t.w32(kRex3 + 0x004, 2u | (1u << 2) | (1u << 6) | (1u << 8) | (1u << 9));
    t.w32(kMc + 0x2004, kVirt);            // DMA_MEMADR (translated)
    t.w32(kMc + 0x2014, (1u << 16) | 8u);  // 1 line of 8 bytes
    t.w32(kMc + 0x201C, 1u << 16);
    t.w32(kMc + 0x2034, 1u << 4);
    t.w32(kMc + 0x202C, kRex3 + kGo + 0x230);
    t.m.run(10000);
    t.w32(kRex3 + 0x000, 0x30000000u | 0x100u | 0x80u | 0x8u | 1u | 0x7000u);  // 8-bit host pixels, packed
    t.w32(kRex3 + 0x150, 0);
    t.w32(kRex3 + 0x154, 3u << 16);
    t.w32(kRex3 + kGo + 0x004, 1u);
    CHECK_EQ(t.r32(kRex3 + 0x230), 0x40414243u);
}

TEST(ip22_mc_dma_feeds_one_gio_address) {
    IndyRig t;
    t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);
    constexpr u32 kRex3 = 0x1F0F0000, kGo = 0x800;
    constexpr u32 kMc = 0x1FA00000, kSrc = 0x08100000;
    for (u32 i = 0; i < 16; ++i) t.w8(kSrc + i, static_cast<u8>(0x10 + i));  // 2 lines of 8 pixels
    // 8-bit pixels from the host, eight per doubleword, into the block (0,0)-(7,1)
    t.w32(kRex3 + 0x000, 0x30000000u | 0x400u | 0x100u | 0x80u | 0x8u | 1u | 0x7000u);  // SRC, RWDOUBLE, HD8, RWPACKED, DD8, CI
    t.w32(kRex3 + 0x220, 0xFF);                                                  // WRMASK
    t.w32(kRex3 + 0x1328, 0x1E00);                                               // CLIPMODE: all CID codes
    t.w32(kRex3 + 0x150, 0);                                                     // XYSTARTI (0,0)
    t.w32(kRex3 + 0x154, (7u << 16) | 1u);                                       // XYENDI (7,1)
    t.w32(kRex3 + 0x004, 2u | (1u << 2) | (1u << 6) | (1u << 8) | (1u << 9));   // DRAW BLOCK COLORHOST STOPX STOPY
    t.w32(kMc + 0x2004, kSrc);              // DMA_MEMADR
    t.w32(kMc + 0x2014, (2u << 16) | 8u);   // DMA_SIZE: 2 lines of 8 bytes
    t.w32(kMc + 0x201C, 1u << 16);          // DMA_STRIDE: zoom 1, no gap
    t.w32(kMc + 0x2034, 1u << 4);           // DMA_MODE: memory to GIO, incrementing
    t.w32(kMc + 0x202C, kRex3 + kGo + 0x230);  // DMA_GIO_ADR, and start
    t.m.run(10000);
    CHECK(t.r32(kMc + 0x204C) & (1u << 3));  // DMA_RUN: done
    // Read both lines back.
    t.w32(kRex3 + 0x000, 0x30000000u | 0x100u | 0x80u | 0x8u | 1u | 0x7000u);  // 8-bit host pixels, packed
    for (u32 y = 0; y < 2; ++y) {
        for (u32 x = 0; x < 8; x += 4) {
            t.w32(kRex3 + 0x150, (x << 16) | y);
            t.w32(kRex3 + 0x154, ((x + 3) << 16) | y);
            t.w32(kRex3 + kGo + 0x004, 1u);  // READ
            const u32 base = 0x10 + y * 8 + x;
            CHECK_EQ(t.r32(kRex3 + 0x230), (base << 24) | ((base + 1) << 16) | ((base + 2) << 8) | (base + 3));
        }
    }
}

TEST(ip22_mc_dma_from_unaligned_memory_feeds_the_aligned_data_port) {
    IndyRig t;
    t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);
    constexpr u32 kRex3 = 0x1F0F0000, kGo = 0x800;
    constexpr u32 kMc = 0x1FA00000, kSrc = 0x08100000;
    // 2 lines of 8 pixels starting 5 bytes into a doubleword; IRIX sets the
    // GIO address's low bits to match.
    for (u32 i = 0; i < 16; ++i) t.w8(kSrc + 5 + i, static_cast<u8>(0x10 + i));
    // 8-bit pixels from the host, eight per doubleword, into the block (0,0)-(7,1)
    t.w32(kRex3 + 0x000, 0x30000000u | 0x400u | 0x100u | 0x80u | 0x8u | 1u | 0x7000u);  // SRC, RWDOUBLE, HD8, RWPACKED, DD8, CI
    t.w32(kRex3 + 0x220, 0xFF);                                                  // WRMASK
    t.w32(kRex3 + 0x1328, 0x1E00);                                               // CLIPMODE: all CID codes
    t.w32(kRex3 + 0x150, 0);                                                     // XYSTARTI (0,0)
    t.w32(kRex3 + 0x154, (7u << 16) | 1u);                                       // XYENDI (7,1)
    t.w32(kRex3 + 0x004, 2u | (1u << 2) | (1u << 6) | (1u << 8) | (1u << 9));   // DRAW BLOCK COLORHOST STOPX STOPY
    t.w32(kMc + 0x2004, kSrc + 5);          // DMA_MEMADR
    t.w32(kMc + 0x2014, (2u << 16) | 8u);   // DMA_SIZE: 2 lines of 8 bytes
    t.w32(kMc + 0x201C, 1u << 16);          // DMA_STRIDE: zoom 1, no gap
    t.w32(kMc + 0x2034, 1u << 4);           // DMA_MODE: memory to GIO, incrementing
    t.w32(kMc + 0x202C, kRex3 + kGo + 0x235);  // DMA_GIO_ADR, and start
    t.m.run(10000);
    CHECK(t.r32(kMc + 0x204C) & (1u << 3));  // DMA_RUN: done
    // Read both lines back.
    t.w32(kRex3 + 0x000, 0x30000000u | 0x100u | 0x80u | 0x8u | 1u | 0x7000u);  // 8-bit host pixels, packed
    for (u32 y = 0; y < 2; ++y) {
        for (u32 x = 0; x < 8; x += 4) {
            t.w32(kRex3 + 0x150, (x << 16) | y);
            t.w32(kRex3 + 0x154, ((x + 3) << 16) | y);
            t.w32(kRex3 + kGo + 0x004, 1u);  // READ
            const u32 base = 0x10 + y * 8 + x;
            CHECK_EQ(t.r32(kRex3 + 0x230), (base << 24) | ((base + 1) << 16) | ((base + 2) << 8) | (base + 3));
        }
    }
}

// Latching an 8254 count (as boot code does to time the CPU) ends run() and
// marks the next 50 ms of guest time as timing the CPU.
TEST(ip22_pit_latch_marks_cpu_timing) {
    IndyRig t;
    CHECK(!t.m.timing_cpu());
    t.w8(ioc_addr(0xBC), 0x80);  // latch counter 2
    CHECK(t.m.timing_cpu());
    t.m.run(static_cast<u64>(t.m.cycles_per_second() / 50));
    CHECK(t.m.timing_cpu());
    t.m.run(static_cast<u64>(t.m.cycles_per_second() / 25));
    CHECK(!t.m.timing_cpu());
}

// Reads in the graphics slot that no Newport register answers return the
// access width, as OpenBSD's probe expects of the pipelined slot.
TEST(ip22_gfx_slot_unclaimed_reads_return_the_access_width) {
    IndyRig t;
    CHECK_EQ(t.r32(0x1F000000), 4u);
    u16 h = 0;
    CHECK(t.m.bus().read<u16>(0x1F000002, h));
    CHECK_EQ(h, 2u);
    CHECK_EQ(t.r8(0x1F000003), 1u);
    CHECK(t.r32(0x1F0F0000) != 4u || t.r32(0x1F0F0004) != 4u);  // REX3 registers answer
}

// With Xlate set, a DMA through a segment no TLB entry maps stops with a
// TLB miss (interrupting whatever the mask) and resumes from where it
// stopped once the entry is loaded. MEMADRD loads the default descriptor.
TEST(ip22_mc_dma_tlb_miss_faults_and_resumes) {
    IndyRig t;
    t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);  // 64 MiB at 0x08000000
    constexpr u32 kMc = 0x1FA00000, kTable = 0x08200000, kPage = 0x08300000;
    constexpr u32 kVirt = 0x00400000;
    t.w32(kTable, ((kPage >> 12) << 6) | 2 | 4);  // PTE: page, valid, dirty
    t.w32(kMc + 0x16C, 0x100);                    // Xlate, completion interrupt masked
    t.w32(kMc + 0x200C, kVirt);                   // MEMADRD: defaults
    CHECK_EQ(t.r32(kMc + 0x2014), 0x0001000Cu);
    CHECK_EQ(t.r32(kMc + 0x2034), 0x28u);
    t.w32(kMc + 0x2014, (2u << 16) | 4u);         // 2 lines of 4 bytes
    t.w32(kMc + 0x201C, 1u << 16);
    t.w32(kMc + 0x2034, (1u << 4) | (1u << 3) | (1u << 1));  // increment, fill memory
    t.w32(kMc + 0x202C, 0xA5A5A5A5u);             // the fill value; starts
    t.m.run(10000);
    CHECK_EQ(t.r32(kMc + 0x164) & 0xF, 0x2u);     // TLB miss
    CHECK_EQ(t.r32(kMc + 0x204C), 0x2u);          // stopped, cause mirrored
    CHECK_EQ(t.r32(kMc + 0x2014) >> 16, 2u);      // nothing done yet
    t.w32(kMc + 0x184, kVirt);
    t.w32(kMc + 0x18C, ((kTable >> 12) << 6) | 2);
    t.w32(kMc + 0x164, 0);
    t.w32(kMc + 0x2044, 1);                       // restart
    t.m.run(10000);
    CHECK_EQ(t.r32(kMc + 0x164) & 0xF, 0x8u);     // complete
    CHECK_EQ(t.r32(kPage), 0xA5A5A5A5u);
    CHECK_EQ(t.r32(kPage + 4), 0xA5A5A5A5u);
    CHECK_EQ(t.r32(kPage + 8), 0u);
}

// With fast I/O the DMA engine fills memory 32 times as fast: 2 MiB, a
// quarter of a million 64-bit words, take that many bus clocks otherwise.
TEST(ip22_mc_dma_fast_io_fills_memory_faster) {
    for (const bool fast : {false, true}) {
        IndyRig t;
        t.m.set_fast_io(fast);
        t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);  // 64 MiB at 0x08000000
        constexpr u32 kMc = 0x1FA00000, kBuf = 0x08200000;
        t.w32(kMc + 0x200C, kBuf);                     // MEMADRD: defaults
        t.w32(kMc + 0x2014, (64u << 16) | 0x8000u);    // 64 lines of 32 KiB
        t.w32(kMc + 0x201C, 1u << 16);
        t.w32(kMc + 0x2034, (1u << 4) | (1u << 3) | (1u << 1));  // increment, fill memory
        t.w32(kMc + 0x202C, 0xA5A5A5A5u);              // the fill value; starts
        const double words = 2.0 * 1024 * 1024 / 8, clocks = words * t.m.cycles_per_second() / IndyConfig{}.sysclk_hz;
        t.m.run(static_cast<u64>(clocks / 32 * 1.1));
        CHECK_EQ((t.r32(kMc + 0x204C) & 0x40) == 0, fast);  // still running at the real rate
        t.m.run(static_cast<u64>(clocks));
        CHECK_EQ(t.r32(kMc + 0x204C) & 0x40, 0u);
        CHECK_EQ(t.r32(kBuf + 2 * 1024 * 1024 - 4), 0xA5A5A5A5u);
    }
}

// Only sizes two banks of 16, 32, 64 or 128 MiB can make are accepted.
TEST(ip22_ram_sizes_two_banks) {
    for (unsigned mb : {16u, 32u, 48u, 64u, 80u, 96u, 128u, 144u, 160u, 192u, 256u}) CHECK(ram_size_fits(mb));
    for (unsigned mb : {0u, 8u, 112u, 176u, 208u, 224u, 240u, 272u}) CHECK(!ram_size_fits(mb));
}

// A static ELF program run without the PROM talks to the host through the
// test device: console bytes, the instruction count, and an exit code.
TEST(ip22_load_elf_runs_a_program_with_the_test_device) {
    using namespace test::as;
    const std::vector<u32> code = {
        LUI(8, 0xBF40),       // t0 = test device (KSEG1)
        LW(9, 0, 8),          // signature
        ORI(10, 0, 'O'),
        SW(10, 4, 8),         // putc
        ORI(10, 0, 'K'),
        SW(10, 4, 8),
        LW(11, 0x18, 8),      // instruction count, low word (latches)
        LW(12, 0x1C, 8),      // high word
        ORI(10, 0, 42),
        SW(10, 0xC, 8),       // exit(42)
        BEQ(0, 0, -1),
        NOP,
    };
    std::vector<u8> elf(0x100 + code.size() * 4, 0);
    const u8 ident[] = {0x7F, 'E', 'L', 'F', 1, 2, 1};
    std::copy(std::begin(ident), std::end(ident), elf.begin());
    store_be<u16>(&elf[16], 2);           // ET_EXEC
    store_be<u16>(&elf[18], 8);           // EM_MIPS
    store_be<u32>(&elf[24], 0x88200000);  // entry
    store_be<u32>(&elf[28], 52);          // program headers
    store_be<u16>(&elf[42], 32);
    store_be<u16>(&elf[44], 1);
    store_be<u32>(&elf[52], 1);  // PT_LOAD
    store_be<u32>(&elf[56], 0x100);
    store_be<u32>(&elf[60], 0x88200000);
    store_be<u32>(&elf[68], static_cast<u32>(code.size() * 4));
    store_be<u32>(&elf[72], 0x00100000);  // memsz: zero filled past the code
    for (std::size_t i = 0; i < code.size(); ++i) store_be<u32>(&elf[0x100 + i * 4], code[i]);
    const std::string path = (std::filesystem::temp_directory_path() / "sg_load_elf_test.elf").string();
    {
        std::ofstream f(path, std::ios::binary);
        f.write(reinterpret_cast<const char*>(elf.data()), static_cast<std::streamsize>(elf.size()));
    }

    IndyConfig c;
    c.test_device = true;
    Indy m(c);
    std::string out;
    m.set_serial_output([&](unsigned, u8 b) { out.push_back(static_cast<char>(b)); });
    m.reset();
    std::string error;
    CHECK(m.load_elf(path, error));
    std::filesystem::remove(path);
    u32 memcfg0 = 0;
    CHECK(m.bus().read<u32>(kMcMemCfg0, memcfg0));
    CHECK_EQ(memcfg0 >> 16, 0x2000u | (15u << 8) | (0x08000000u >> 22));  // 64 MiB at 0x08000000
    m.run(100000);
    CHECK(m.powered_off());
    CHECK_EQ(m.exit_code(), 42);
    CHECK(out == "OK");
    CHECK_EQ(static_cast<u32>(m.cpu().gpr[9]), TestDevice::kSignature);
    CHECK(static_cast<u32>(m.cpu().gpr[11]) >= 6u && m.cpu().gpr[12] == 0);
}

TEST(ip22_load_elf_rejects_other_files) {
    const std::string path = (std::filesystem::temp_directory_path() / "sg_load_elf_bad.elf").string();
    {
        std::ofstream f(path, std::ios::binary);
        f << "not an executable at all, but long enough to have a header......";
    }
    Indy m{IndyConfig{}};
    std::string error;
    CHECK(!m.load_elf(path, error));
    CHECK(!error.empty());
    std::filesystem::remove(path);
}

// ---------------------------------------------------------------------------
// Indigo2 (Full House)
// ---------------------------------------------------------------------------

namespace {
constexpr u32 kInt2 = 0x1FBD9000;
constexpr u32 kExtIo = 0x1FBD9900;
u32 int2_addr(unsigned reg_offset) { return kInt2 + reg_offset + 3; }
struct Indigo2Rig : IndyRig {
    explicit Indigo2Rig(unsigned ram_mb = 64) : IndyRig(ram_mb, Board::Indigo2) {}
};
}  // namespace

// Expansion slot 0's GIO interrupts on the Indigo2: EXTIO's S0 bits and the
// local inputs it shares with the graphics slot, its retrace latched until
// PORT_CONFIG bit 4 is pulsed low; expansion slot 1 has no lines. On the
// Indy each expansion slot is a mappable input.
TEST(gio_expansion_slot_interrupts) {
    Indigo2Rig t;
    t.m.ioc().set_gio_slot_irq(1, 1, true);
    CHECK_EQ(t.r32(kExtIo) & 0xF000, 0xD000u);  // S0 graphics (active low)
    CHECK(t.r8(int2_addr(0x00)) & 0x40);        // local 0 graphics
    t.m.ioc().set_gio_slot_irq(1, 1, false);
    t.m.ioc().set_gio_slot_irq(1, 2, true);
    t.m.ioc().set_gio_slot_irq(1, 2, false);  // latched
    CHECK_EQ(t.r32(kExtIo) & 0xF000, 0xE000u);
    CHECK(t.r8(int2_addr(0x08)) & 0x80);      // local 1 retrace
    t.w8(int2_addr(0x1C), 0xFF & ~0x10);      // clear S0 retrace
    t.w8(int2_addr(0x1C), 0xFF);
    CHECK_EQ(t.r32(kExtIo) & 0xFFFF, 0xFFFFu);
    CHECK(!(t.r8(int2_addr(0x08)) & 0x80));
    t.m.ioc().set_gio_slot_irq(2, 0, true);  // expansion slot 1: nothing
    CHECK_EQ(t.r32(kExtIo) & 0xFFFF, 0xFFFFu);
    CHECK(!(t.r8(int2_addr(0x00)) & 0x01));
    t.m.ioc().set_gfx_drain(0, true);  // drain feedback: mappable 6
    CHECK(t.r8(int2_addr(0x10)) & 0x40);

    IndyRig indy;
    indy.m.ioc().set_gio_slot_irq(2, 0, true);
    CHECK_EQ(indy.r8(ioc_addr(0x90)) & 0xC0, 0x80u);  // MAP status: GIO EXP1
    indy.m.ioc().set_gio_slot_irq(2, 0, false);
    indy.m.ioc().set_gio_slot_irq(1, 2, true);
    CHECK_EQ(indy.r8(ioc_addr(0x90)) & 0xC0, 0x40u);  // GIO EXP0
}

TEST(indigo2_identity_registers) {
    Indigo2Rig t;
    CHECK_EQ(t.r32(kMcSysId), 0x13u);         // MC revision C, EISA present
    CHECK_EQ(t.r8(ioc_addr(0x58)), 0x11);     // IOC: Full House
    CHECK_EQ(t.r32(kExtIo) & 0xFFFF, 0xFFFFu);  // EXTIO: nothing pending (active low)
    // The interrupt controller is the INT2, not part of the IOC.
    t.w8(int2_addr(0x04), 0xA5);  // local 0 mask
    t.w8(int2_addr(0x14), 0x5A);  // map mask 0
    CHECK_EQ(t.r8(int2_addr(0x04)), 0xA5);
    CHECK_EQ(t.r8(int2_addr(0x14)), 0x5A);
    CHECK_EQ(t.r8(ioc_addr(0x84)), 0x00);  // no INT3 in the IOC window
    IndyRig indy;
    CHECK_EQ(indy.r32(kMcSysId), 0x03u);
}

TEST(indigo2_int2_timer_interrupt) {
    Indigo2Rig t;
    // As ip22_pit_timer0_interrupt_via_cascade, through the INT2's 8254.
    t.w8(int2_addr(0x3C), 0xB4);
    t.w8(int2_addr(0x38), 100);
    t.w8(int2_addr(0x38), 0);
    t.w8(int2_addr(0x3C), 0x34);
    t.w8(int2_addr(0x30), 10);
    t.w8(int2_addr(0x30), 0);
    t.m.run(100000);
    CHECK(!t.ip(4));
    t.m.run(200000);
    CHECK(t.ip(4));
    t.w8(int2_addr(0x20), 1);  // timer clear
    CHECK(!t.ip(4));
}

TEST(indigo2_graphics_slot_interrupts_in_extio) {
    Indigo2Rig t;
    t.w8(int2_addr(0x04), 0x41);  // local 0: graphics, FIFO
    t.w8(int2_addr(0x0C), 0x80);  // local 1: retrace
    t.m.ioc().set_gfx_slot_irq(1, true);
    CHECK(t.ip(2));
    CHECK_EQ(t.r8(int2_addr(0x00)) & 0x40, 0x40);
    CHECK_EQ(t.r32(kExtIo) & 0xFFFF, 0xFFFFu & ~Ioc2::kExtSgIrq1);
    t.m.ioc().set_gfx_slot_irq(1, false);
    CHECK(!t.ip(2));
    CHECK_EQ(t.r32(kExtIo) & 0xFFFF, 0xFFFFu);
    // The retrace is latched until PORT_CONFIG's clear line is pulsed.
    t.w8(int2_addr(0x1C), 0xFF);
    t.m.ioc().set_gfx_slot_irq(2, true);
    t.m.ioc().set_gfx_slot_irq(2, false);
    CHECK(t.ip(3));
    CHECK_EQ(t.r32(kExtIo) & Ioc2::kExtSgRetrace, 0u);
    t.w8(int2_addr(0x1C), 0xFF & ~0x08);
    t.w8(int2_addr(0x1C), 0xFF);
    CHECK(!t.ip(3));
    CHECK_EQ(t.r32(kExtIo) & 0xFFFF, 0xFFFFu);
}

// SCSI DMA with the channel's endian bit set: as for Ethernet, the bytes
// sit at their addresses XOR 7.
TEST(ip22_scsi_dma_little_endian) {
    const std::string path = (std::filesystem::temp_directory_path() / "sg_scsi_endian.img").string();
    {
        std::ofstream f(path, std::ios::binary);
        for (unsigned i = 0; i < 512; ++i) f.put(static_cast<char>(i * 3 + 1));
    }
    {
        ScsiDisk disk(ScsiDisk::Kind::Disk, path);
        IndyRig t;
        t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);
        Wd33c93& wd = t.m.hpc3().scsi0();
        wd.attach(2, &disk);
        auto reg = [&](u8 r, u8 v) {
            wd.write_address(r);
            wd.write_data(v);
        };
        constexpr u32 kRegs = 0x1FB90000, kDesc = 0x08001000, kBuf = 0x08002000;
        t.w32(kRegs + 0x1004, 0);  // out of reset
        wd.write_address(0x17);
        wd.read_data();  // its reset interrupt taken
        t.w32(kDesc, kBuf);
        t.w32(kDesc + 4, (1u << 31) | 512);
        t.w32(kDesc + 8, 0);
        t.w32(kRegs + 0x0004, kDesc);
        t.w32(kRegs + 0x1004, 0x12);  // active, little-endian, device -> memory
        reg(0x01, 0x80);              // DMA mode
        const u8 cdb[6] = {0x08, 0, 0, 0, 1, 0};  // READ(6) block 0
        for (unsigned i = 0; i < 6; ++i) reg(static_cast<u8>(0x03 + i), cdb[i]);
        reg(0x12, 0);
        reg(0x13, 2);
        reg(0x14, 0);
        reg(0x15, 2);
        reg(0x18, 0x09);  // SELECT-AND-TRANSFER
        t.m.run(static_cast<u64>(t.m.cycles_per_second() / 100));
        wd.write_address(0x17);
        CHECK_EQ(wd.read_data(), 0x16);
        bool same = true;
        for (u32 i = 0; i < 512; ++i) {
            u8 b = 0;
            t.m.bus().read<u8>((kBuf + i) ^ 7, b);
            same &= b == static_cast<u8>(i * 3 + 1);
        }
        CHECK(same);
    }
    std::filesystem::remove(path);
}

TEST(indigo2_second_scsi_controller) {
    Indigo2Rig t;
    const u32 scsi1 = 0x1FBCC000;
    t.w32(0x1FB93004, 0);  // release channel 1 from reset
    t.w8(scsi1 + 3, 0x00);  // SASR = own ID
    t.w8(scsi1 + 7, 0x07);
    t.w8(scsi1 + 3, 0x00);
    CHECK_EQ(t.r8(scsi1 + 7), 0x07);
    t.w32(0x1FB92004, 0x08001000);  // channel 1 NDBP
    CHECK_EQ(t.r32(0x1FB92004), 0x08001000u);
    CHECK_EQ(t.r32(0x1FB90004), 0u);  // channel 0's is separate
}

TEST(indigo2_ram_sizes_three_banks) {
    for (unsigned mb : {16u, 64u, 256u, 272u, 320u, 384u}) CHECK(ram_size_fits(mb, Board::Indigo2));
    for (unsigned mb : {272u, 320u, 384u}) CHECK(!ram_size_fits(mb, Board::Indy));
    for (unsigned mb : {8u, 368u, 400u, 512u}) CHECK(!ram_size_fits(mb, Board::Indigo2));
    CHECK(ram_size_fits(112, Board::Indigo2));  // 64 + 32 + 16
    Indigo2Rig t(384);
    CHECK_EQ(t.m.mc().banks().size(), 3u);
}

TEST(indigo2_nvram_lives_in_the_serial_eeprom) {
    Indigo2Rig t;
    t.m.reset();
    // The Ethernet address goes to bytes 250-255 of the EEPROM.
    const auto& words = t.m.hpc3().eeprom().words();
    CHECK_EQ(words[125], 0x0800);
    CHECK_EQ(words[126] >> 8, 0x69);
    CHECK_EQ(t.m.nvram_size(), 256u + 8192u);
    std::vector<u8> image = t.m.nvram_image();
    CHECK_EQ(image.size(), t.m.nvram_size());
    CHECK_EQ(image[250], 0x08);
    image[10] = 0x12, image[11] = 0x34;  // EEPROM word 5
    image[256 + 0x50] = 0x99;            // RTC memory
    Indigo2Rig u;
    u.m.load_nvram_image(image);
    CHECK_EQ(u.m.hpc3().eeprom().words()[5], 0x1234);
    CHECK_EQ(u.m.rtc().memory()[0x50], 0x99);
    CHECK(u.m.nvram_image() == image);
}

TEST(indigo2_empty_eisa_io_reads_all_ones) {
    Indigo2Rig t;
    CHECK_EQ(t.r32(0x0009FFC0), 0xFFFFFFFFu);
    CHECK_EQ(t.r8(0x00080C83), 0xFF);
    IndyRig indy;
    u32 v = 0;
    CHECK(!indy.m.bus().read<u32>(0x0009FFC0, v));  // nothing there on the Indy
}

TEST(indigo2_takes_r4000_family_modules_only) {
    IndyConfig c;
    c.board = Board::Indigo2;
    c.cpu_core = mips::Core::R5000;
    bool threw = false;
    try {
        Indy m(c);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

// ---------------------------------------------------------------------------
// R10000 Indigo2 (IP28)
// ---------------------------------------------------------------------------

// The IP28's MC (revision 5) counts bank registers in 16 MiB and decodes a
// 512 MiB high window; ALIAS_BIG takes the low alias from 0x20000000; the
// ECC board's register answers where bank 3 points at 0x60000000.
TEST(ip28_mc_large_banks_alias_and_ecc_board) {
    IndyRig t(256, Board::Indigo2R10000);
    CHECK_EQ(t.r32(kMcSysId) & 0xF, 5u);
    CHECK_EQ(t.r8(ioc_addr(0x58)), 0x1Bu);  // Full House, board revision 13 (below: an IP26 baseboard)
    // Bank 0: 256 MiB at 0x30000000, beyond the IP22's 128 MiB high window.
    t.w32(kMcMemCfg0, (0x2000u | (15u << 8) | (0x30000000u >> 24)) << 16);
    t.w32(0x30000000 + (200u << 20), 0x12345678);
    CHECK_EQ(t.r32(0x30000000 + (200u << 20)), 0x12345678u);
    // Bank 0 at 0x20000000 with ALIAS_BIG: its first 512 KiB also appear at 0.
    t.w32(kMcMemCfg0, (0x2000u | (15u << 8) | (0x20000000u >> 24)) << 16);
    t.w32(0x1FA000D4, t.r32(0x1FA000D4) | (1u << 29));
    t.w32(0x20000100, 0xA5A55A5A);
    CHECK_EQ(t.r32(0x100), 0xA5A55A5Au);
    // The ECC register: bus errors until bank 3 points at it.
    u32 v = 0;
    CHECK(!t.m.bus().read<u32>(0x60000000, v));
    t.w32(0x1FA000CC, 0x2060);
    CHECK(t.m.bus().write<u64>(0x60000000, 0));
}
