// SPDX-License-Identifier: GPL-2.0-or-later
// IP22 chipset tests: MC, HPC3, IOC2/INT3 accessed at their physical addresses.
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
    // 100 stereo frames per descriptor, a 32-bit word (24-bit sample) per voice
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
    CHECK(!(t.r32(kEnetRxCtrl) & 0x200));  // stopped at the end of the ring
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
