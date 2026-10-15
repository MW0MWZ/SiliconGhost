// SPDX-License-Identifier: GPL-2.0-or-later
// IP22 chipset tests: MC, HPC3, IOC2/INT3 accessed at their physical addresses.
#include <cmath>
#include <array>
#include <string>
#include <vector>

#include "core/endian.h"
#include "cpu/r4000/r4000.h"
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
    const u32 lo = t.r32(kHal2 + 0x40);
    t.w32(kHal2 + 0x30, reg | 0x81);
    return lo | (t.r32(kHal2 + 0x40) << 16);
}
}  // namespace

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
        t.w32(kPbus0 + 0x2000 + 0x1000, 0x30);
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

// ADC_C2 amplifies each side in 1.5 dB steps, clipping at full scale.
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
        hal2_write(t, 0x1508, 0x34F);       // ADC_C2: left +6 dB, right +22.5 dB, mic selected
        hal2_write(t, 0x9104, 0x10);
        t.w32(kPbus0 + 0x2000 + 4, kDesc);
        t.w32(kPbus0 + 0x2000 + 0x1000, 0x30);
        t.m.run(static_cast<u64>(t.m.cycles_per_second() * 20 / 48000));
        if (ds16) {
            CHECK_EQ(t.r32(kBuf), static_cast<u32>(std::lround(1000 * 256 * std::pow(10.0, 6.0 / 20))));
            CHECK_EQ(t.r32(kBuf + 4), 0x7FFFFFu);  // clipped
        } else {
            CHECK_EQ(t.r32(kBuf), (static_cast<u32>(std::lround(1000 * std::pow(10.0, 6.0 / 20))) << 16) | 0x7FFFu);
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
    CHECK(t.r32(kDesc + 4) & kTxDone);
    CHECK(t.r32(kDesc + 20) & kTxDone);
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
    CHECK_EQ(v, 0u);
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
TEST(ip22_mc_dma_feeds_one_gio_address) {
    IndyRig t;
    t.w32(kMcMemCfg0, (0x2000u | 0x0F00u | 0x20u) << 16);
    constexpr u32 kRex3 = 0x1F0F0000, kGo = 0x800;
    constexpr u32 kMc = 0x1FA00000, kSrc = 0x08100000;
    for (u32 i = 0; i < 16; ++i) t.w8(kSrc + i, static_cast<u8>(0x10 + i));  // 2 lines of 8 pixels
    // 8-bit pixels from the host, eight per doubleword, into the block (0,0)-(7,1)
    t.w32(kRex3 + 0x000, 0x30000000u | 0x400u | 0x100u | 0x80u | 0x8u | 1u);  // SRC, RWDOUBLE, HD8, RWPACKED, DD8, CI
    t.w32(kRex3 + 0x220, 0xFF);                                                  // WRMASK
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
    t.w32(kRex3 + 0x000, 0x30000000u | 0x100u | 0x8u | 1u);
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
    t.w32(kRex3 + 0x000, 0x30000000u | 0x400u | 0x100u | 0x80u | 0x8u | 1u);  // SRC, RWDOUBLE, HD8, RWPACKED, DD8, CI
    t.w32(kRex3 + 0x220, 0xFF);                                                  // WRMASK
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
    t.w32(kRex3 + 0x000, 0x30000000u | 0x100u | 0x8u | 1u);
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
