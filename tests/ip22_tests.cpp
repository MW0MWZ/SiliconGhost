// SPDX-License-Identifier: GPL-2.0-or-later
// IP22 chipset tests: MC, HPC3, IOC2/INT3 accessed at their physical addresses.
#include <string>
#include <vector>

#include "core/endian.h"
#include "cpu/r4000/r4000.h"
#include "machine/ip22/indy.h"
#include "mips_asm.h"
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
    Indy m{IndyConfig{}};
    std::string serial;
    IndyRig() {
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
    // Empty GIO expansion slot.
    CHECK(!t.m.bus().read<u32>(0x1F400000, v));
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
    CHECK_EQ(t.r8(ioc_addr(0x50)), 0xA1);  // panel: power on, buttons released
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
    CHECK(t.serial == "ok");
}

TEST(ip22_nvram_and_rtc) {
    IndyRig t;
    t.w8(kNvram + 4 * 250 + 3, 0x08);  // first byte of the Ethernet address
    CHECK_EQ(t.m.rtc().memory()[0x40 + 250], 0x08);
    CHECK_EQ(t.r32(kNvram + 4 * 250) & 0xFF, 0x08u);  // word access: data in the low byte
    const u8 month = t.r8(0x1FBE0000 + 4 * 9 + 3) & 0x1F;
    CHECK(month >= 0x01 && month <= 0x12);
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
