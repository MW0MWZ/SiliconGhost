// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// R4600 and R5000 tests: identification, CP0 differences, the MIPS IV
// instructions, WAIT, and the 2-way primary and R5000 secondary caches.
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <vector>

#include "core/endian.h"
#include "cpu/r4000/r4000.h"
#include "mem/bus.h"
#include "mips_asm.h"
#include "test.h"

using namespace sg;
using namespace sg::mips;
using namespace sg::test::as;

namespace {

constexpr u64 kCode = 0xFFFFFFFF80001000ull;
constexpr u64 kGeneralVector = 0xFFFFFFFF80000180ull;

enum : unsigned { S = 16, D = 17 };
constexpr u32 FOP(unsigned fmt, unsigned func, unsigned fd, unsigned fs, unsigned ft = 0) {
    return (0x11u << 26) | (fmt << 21) | (ft << 16) | (fs << 11) | (fd << 6) | func;
}
constexpr u32 CFC1(unsigned rt, unsigned fs) { return (0x11u << 26) | (2u << 21) | (rt << 16) | (fs << 11); }
constexpr u32 CTC1(unsigned rt, unsigned fs) { return (0x11u << 26) | (6u << 21) | (rt << 16) | (fs << 11); }
constexpr u32 CACHE(unsigned op, s32 off, unsigned base) { return I(0x2F, base, op, off); }
// MIPS IV
constexpr u32 MOVZ(unsigned rd, unsigned rs, unsigned rt) { return R(0x0A, rs, rt, rd); }
constexpr u32 MOVN(unsigned rd, unsigned rs, unsigned rt) { return R(0x0B, rs, rt, rd); }
constexpr u32 MOVCI(unsigned rd, unsigned rs, unsigned cc, bool tf) {
    return (rs << 21) | (cc << 18) | (u32{tf} << 16) | (rd << 11) | 0x01;
}
constexpr u32 PREF(unsigned hint, s32 off, unsigned base) { return I(0x33, base, hint, off); }
constexpr u32 COP1X(unsigned func, unsigned fr, unsigned ft, unsigned fs, unsigned fd) {
    return (0x13u << 26) | (fr << 21) | (ft << 16) | (fs << 11) | (fd << 6) | func;
}
constexpr u32 CCOND(unsigned fmt, unsigned cond, unsigned cc, unsigned fs, unsigned ft) {
    return FOP(fmt, 0x30 | cond, cc << 2, fs, ft);
}
constexpr u32 BC1(unsigned cc, bool likely, bool tf, s32 off) {
    return (0x11u << 26) | (8u << 21) | (cc << 18) | (u32{likely} << 17) | (u32{tf} << 16) |
           (static_cast<u32>(off) & 0xFFFF);
}
constexpr u32 WAIT = 0x42000020;
constexpr unsigned kLt = 0xC, kEq = 0x2;  // C.LT, C.EQ

u64 dbits(double d) {
    u64 b;
    std::memcpy(&b, &d, 8);
    return b;
}
u32 sbits(float f) {
    u32 b;
    std::memcpy(&b, &f, 4);
    return b;
}

R4000::Options model(Core core, u32 scache = 0) {
    R4000::Options o;
    o.core = core;
    o.prid = core == Core::R4600 ? 0x2020 : core == Core::R5000 ? 0x2321 : 0x0450;
    o.config = core == Core::R4600 ? R4000::kR4600Config : core == Core::R5000 ? R4000::kR5000Config
                                                                               : R4000::kDefaultConfig;
    o.scache_bytes = scache;
    return o;
}

struct Rig {
    std::vector<u8> ram = std::vector<u8>(8u << 20);
    Bus bus;
    R4000 cpu;
    explicit Rig(const R4000::Options& o, u32 status = sr::CU1 | sr::FR) : cpu(bus, o) {
        bus.map_ram(0, ram.size(), ram.data());
        cpu.cp0_write(cop0::Status, status);
        cpu.set_pc(kCode);
    }
    void load(std::initializer_list<u32> prog, u64 paddr = 0x1000) {
        for (u32 w : prog) {
            store_be<u32>(&ram[paddr], w);
            paddr += 4;
        }
    }
    void steps(int n) {
        for (int i = 0; i < n; ++i) cpu.step();
    }
    unsigned exc_code() const { return (cpu.cp0_read(cop0::Cause) >> 2) & 31; }
    u64 r64(u64 paddr) const { return load_be<u64>(&ram[paddr]); }
};

}  // namespace

TEST(cpu_models_report_their_identity_and_caches) {
    {
        Rig t(model(Core::R4600, 512u << 10));  // an SC module: the processor still reports none
        CHECK_EQ(t.cpu.cp0_read(cop0::PRId), 0x2020ull);
        CHECK_EQ(t.cpu.cp0_read(cop0::Config), u64{R4000::kR4600Config | (1u << 17)});
        t.load({CFC1(t0, 0)});
        t.steps(1);
        CHECK_EQ(t.cpu.gpr[t0], 0x2020ull);  // FPU implementation 0x20
    }
    {
        Rig t(model(Core::R5000));
        const u64 config = t.cpu.cp0_read(cop0::Config);
        CHECK((config >> 17) & 1);          // SC: none
        CHECK_EQ((config >> 20) & 3, 3ull);  // SS: none
        CHECK_EQ((config >> 9) & 7, 3ull);   // 32 KiB primary caches, 32-byte lines
        CHECK_EQ((config >> 6) & 7, 3ull);
        CHECK_EQ((config >> 4) & 3, 3ull);
        t.load({CFC1(t0, 0)});
        t.steps(1);
        CHECK_EQ(t.cpu.gpr[t0], 0x2321ull);
    }
    {
        Rig t(model(Core::R5000, 512u << 10));
        const u64 config = t.cpu.cp0_read(cop0::Config);
        CHECK(!((config >> 17) & 1));
        CHECK_EQ((config >> 20) & 3, 0ull);  // 512 KiB
        CHECK_EQ((config >> 22) & 3, 1ull);  // 32-byte lines
        CHECK(!((config >> 12) & 1));        // SE off after reset
    }
}

TEST(cpu_models_config_and_watch_registers) {
    Rig r5(model(Core::R5000, 512u << 10));
    const u64 before = r5.cpu.cp0_read(cop0::Config);
    r5.cpu.cp0_write(cop0::Config, (1u << 12) | 2u);  // SE, K0 uncached; the rest read-only
    CHECK_EQ(r5.cpu.cp0_read(cop0::Config), (before & ~u64{0x1007}) | 0x1002);
    r5.cpu.cp0_write(cop0::WatchLo, 0x1234);  // no watch registers
    CHECK_EQ(r5.cpu.cp0_read(cop0::WatchLo), 0ull);

    Rig r46(model(Core::R4600));
    r46.cpu.cp0_write(cop0::Config, 0);
    CHECK_EQ(r46.cpu.cp0_read(cop0::Config) & 0x30, 0x30ull);  // line sizes fixed at 32 bytes
    CHECK_EQ(r46.cpu.cp0_read(cop0::Config) & 7, 0ull);

    Rig r44(model(Core::R4000));
    r44.cpu.cp0_write(cop0::WatchLo, 0x1234);
    CHECK_EQ(r44.cpu.cp0_read(cop0::WatchLo), 0x1234ull);
}

TEST(cpu_mips4_instructions_are_reserved_before_the_r5000) {
    for (Core core : {Core::R4000, Core::R4600}) {
        for (u32 w : {MOVN(t0, t1, t2), MOVZ(t0, t1, t2), MOVCI(t0, t1, 0, true), PREF(0, 0, zero),
                      COP1X(0x01, s0, zero, 0, 2)}) {
            Rig t(model(core));
            t.load({w});
            t.steps(1);
            CHECK_EQ(t.cpu.pc, kGeneralVector);
            CHECK_EQ(t.exc_code(), static_cast<unsigned>(w >> 26 == 0x13 ? Exc::CpU : Exc::RI));
        }
    }
}

TEST(cpu_r5000_conditional_moves_and_prefetch) {
    Rig t(model(Core::R5000));
    t.cpu.gpr[t1] = 0x1111;
    t.cpu.gpr[t2] = 0;
    t.cpu.gpr[t3] = 5;
    t.cpu.gpr[a0] = 0xAAAA;
    t.cpu.gpr[a1] = 0xBBBB;
    t.load({MOVZ(a0, t1, t2),             // t2 == 0: moves
            MOVN(a1, t1, t2),             // no move
            MOVN(a2, t3, t3),             // moves
            MOVZ(a3, t3, t3),             // no move
            PREF(4, 0x40, t1),            // a hint: nothing happens, whatever the address
            MOVCI(v0, t1, 3, true),       // FCC3 clear: no move for MOVT
            MOVCI(v1, t1, 3, false)});    // moves for MOVF
    t.cpu.gpr[a2] = 0xCCCC;
    t.cpu.gpr[a3] = 0xDDDD;
    t.steps(7);
    CHECK_EQ(t.cpu.pc, kCode + 28);
    CHECK_EQ(t.cpu.gpr[a0], 0x1111ull);
    CHECK_EQ(t.cpu.gpr[a1], 0xBBBBull);
    CHECK_EQ(t.cpu.gpr[a2], 5ull);
    CHECK_EQ(t.cpu.gpr[a3], 0xDDDDull);
    CHECK_EQ(t.cpu.gpr[v0], 0ull);
    CHECK_EQ(t.cpu.gpr[v1], 0x1111ull);

    Rig u(model(Core::R5000), 0);  // MOVF/MOVT need the FPU
    u.load({MOVCI(v0, t1, 0, false)});
    u.steps(1);
    CHECK_EQ(u.exc_code(), static_cast<unsigned>(Exc::CpU));
}

TEST(cpu_r5000_mips4_needs_xx_in_user_mode) {
    for (bool xx : {false, true}) {
        Rig t(model(Core::R5000));
        // ERET to user mode at 0x2000 (mapped through the TLB below), which runs MOVN.
        t.cpu.cp0_write(cop0::EntryHi, 0x2000);  // VPN2 1: the even page is 0x2000
        t.cpu.cp0_write(cop0::EntryLo0, (0x2000 >> 12) << 6 | 0x16);  // uncached, D, V
        t.cpu.cp0_write(cop0::EntryLo1, 0);
        t.cpu.cp0_write(cop0::Index, 0);
        t.cpu.cp0_write(cop0::EPC, 0x2000);
        t.cpu.cp0_write(cop0::Status, sr::KSU_USER | sr::EXL | sr::CU1 | (xx ? sr::XX : 0));
        t.load({TLBWI, ERET});
        t.load({MOVN(t0, t1, t1)}, 0x2000);
        t.cpu.gpr[t1] = 7;
        t.steps(3);
        if (xx) {
            CHECK_EQ(t.cpu.gpr[t0], 7ull);
            CHECK_EQ(t.cpu.pc, 0x2004ull);
        } else {
            CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::RI));
            CHECK_EQ(t.cpu.cp0_read(cop0::EPC), 0x2000ull);
        }
    }
}

TEST(cpu_r5000_fpu_condition_codes) {
    Rig t(model(Core::R5000));
    t.cpu.fpr[2] = dbits(1.0);
    t.cpu.fpr[4] = dbits(2.0);
    t.cpu.fpr[6] = dbits(6.0);
    t.cpu.fpr[8] = dbits(8.0);
    t.load({CCOND(D, kLt, 5, 2, 4),        // 1 < 2: FCC5
            CCOND(D, kEq, 0, 2, 4),        // 1 == 2: FCC0 clear
            BC1(5, false, true, 2),        // BC1T 5: taken
            ADDIU(t0, zero, 1),            //   delay slot
            ADDIU(t1, zero, 1),            //   skipped
            BC1(0, true, true, 2),         // BC1TL 0: not taken, delay slot nullified
            ADDIU(t2, zero, 1),
            FOP(D, 0x11, 10, 6, (5 << 2) | 1),  // MOVT.D f10, f6, FCC5: moves
            FOP(D, 0x11, 12, 8, (5 << 2) | 0),  // MOVF.D f12, f8, FCC5: no move
            CFC1(t3, 31)});
    t.steps(8);
    CHECK_EQ(t.cpu.gpr[t0], 1ull);
    CHECK_EQ(t.cpu.gpr[t1], 0ull);
    CHECK_EQ(t.cpu.gpr[t2], 0ull);
    CHECK_EQ(t.cpu.fpr[10], dbits(6.0));
    CHECK_EQ(t.cpu.fpr[12], 0ull);
    CHECK_EQ(t.cpu.gpr[t3], u64{1u << 29});  // FCC5 is FCR31 bit 29, FCC0 (bit 23) clear

    // CTC1 writes all eight condition codes on the R5000, only FCC0 before it.
    for (Core core : {Core::R5000, Core::R4000}) {
        Rig c(model(core));
        c.cpu.gpr[t0] = 0xFE800000u;
        c.load({CTC1(t0, 31), CFC1(t1, 31)});
        c.steps(2);
        CHECK_EQ(c.cpu.gpr[t1], core == Core::R5000 ? sext32(0xFE800000u) : 0x00800000ull);
    }
}

TEST(cpu_r5000_fpu_moves_on_gpr_and_reciprocals) {
    Rig t(model(Core::R5000));
    t.cpu.fpr[2] = dbits(4.0);
    t.cpu.fpr[3] = u64{sbits(16.0f)};
    t.cpu.gpr[t0] = 0;
    t.cpu.gpr[t1] = 9;
    t.load({FOP(D, 0x12, 4, 2, t0),  // MOVZ.D f4, f2, t0: moves
            FOP(D, 0x13, 6, 2, t0),  // MOVN.D f6, f2, t0: no move
            FOP(D, 0x13, 8, 2, t1),  // MOVN.D f8, f2, t1: moves
            FOP(D, 0x15, 10, 2),     // RECIP.D f10 = 0.25
            FOP(S, 0x16, 12, 3),     // RSQRT.S f12 = 0.25
            FOP(D, 0x16, 14, 2)});   // RSQRT.D f14 = 0.5
    t.steps(6);
    CHECK_EQ(t.cpu.fpr[4], dbits(4.0));
    CHECK_EQ(t.cpu.fpr[6], 0ull);
    CHECK_EQ(t.cpu.fpr[8], dbits(4.0));
    CHECK_EQ(t.cpu.fpr[10], dbits(0.25));
    CHECK_EQ(t.cpu.fpr[12] & 0xFFFFFFFF, u64{sbits(0.25f)});
    CHECK_EQ(t.cpu.fpr[14], dbits(0.5));
    CHECK_EQ(t.cpu.fcr31 & 0x3F000, 0u);  // exact

    Rig z(model(Core::R5000));  // RECIP of zero: divide by zero, infinity
    z.load({FOP(D, 0x15, 4, 2)});
    z.steps(1);
    CHECK_EQ(z.cpu.fpr[4], 0x7FF0000000000000ull);
    CHECK(z.cpu.fcr31 & (1u << 15));

    Rig r(model(Core::R4600));  // not before the R5000: unimplemented
    r.cpu.fpr[2] = dbits(4.0);
    r.load({FOP(D, 0x15, 4, 2)});
    r.steps(1);
    CHECK_EQ(r.exc_code(), static_cast<unsigned>(Exc::FPE));
    CHECK(r.cpu.fcr31 & (1u << 17));
}

TEST(cpu_r5000_multiply_add_rounds_the_product) {
    // (1 + 2^-30)(1 - 2^-30) = 1 - 2^-60 rounds to 1, so with the product
    // rounded first MADD gives exactly 0 where a fused operation gives -2^-60.
    const double a = 1.0 + std::ldexp(1.0, -30), b = 1.0 - std::ldexp(1.0, -30);
    struct Case {
        unsigned func;
        double c, want;
    };
    for (const Case k : {Case{0x21, -1.0, 0.0},    // MADD.D: a*b + c
                         Case{0x29, 1.0, 0.0},     // MSUB.D: a*b - c
                         Case{0x31, 2.0, -3.0},    // NMADD.D: -(a*b + c)
                         Case{0x39, 2.0, 1.0}}) {  // NMSUB.D: -(a*b - c)
        Rig t(model(Core::R5000));
        t.cpu.fpr[2] = dbits(a);
        t.cpu.fpr[4] = dbits(b);
        t.cpu.fpr[6] = dbits(k.c);
        t.load({COP1X(k.func, 6, 4, 2, 8)});  // fd = f8, fr = f6, fs = f2, ft = f4
        t.steps(1);
        CHECK_EQ(t.cpu.pc, kCode + 4);
        CHECK_EQ(t.cpu.fpr[8], dbits(k.want));
        CHECK(t.cpu.fcr31 & (1u << 12));  // the product was inexact
    }
    Rig s(model(Core::R5000));  // single: 3 * 4 + 0.5
    s.cpu.fpr[2] = sbits(3.0f);
    s.cpu.fpr[4] = sbits(4.0f);
    s.cpu.fpr[6] = sbits(0.5f);
    s.load({COP1X(0x20, 6, 4, 2, 8)});
    s.steps(1);
    CHECK_EQ(s.cpu.fpr[8] & 0xFFFFFFFF, u64{sbits(12.5f)});
    CHECK_EQ(s.cpu.fcr31 & 0x3F000, 0u);

    Rig d(model(Core::R5000));  // a denormal operand: left to software
    d.cpu.fpr[2] = 1;
    d.cpu.fpr[4] = dbits(1.0);
    d.load({COP1X(0x21, 6, 4, 2, 8)});
    d.steps(1);
    CHECK_EQ(d.exc_code(), static_cast<unsigned>(Exc::FPE));
    CHECK(d.cpu.fcr31 & (1u << 17));
}

TEST(cpu_r5000_indexed_fpu_loads_and_stores) {
    Rig t(model(Core::R5000));
    t.cpu.gpr[s0] = 0xFFFFFFFF80010000ull;
    t.cpu.gpr[t0] = 0x18;
    t.cpu.gpr[t1] = 0x24;
    store_be<u64>(&t.ram[0x10018], dbits(2.5));
    store_be<u32>(&t.ram[0x10024], sbits(1.5f));
    t.load({COP1X(0x01, s0, t0, 0, 2),     // LDXC1 f2, t0(s0)
            COP1X(0x00, s0, t1, 0, 4),     // LWXC1 f4, t1(s0)
            COP1X(0x09, s0, t1, 2, 0),     // SDXC1 f2, t1(s0): misaligned for a doubleword
            COP1X(0x0F, s0, t1, 0, 0)});   // PREFX
    t.steps(2);
    CHECK_EQ(t.cpu.fpr[2], dbits(2.5));
    CHECK_EQ(t.cpu.fpr[4] & 0xFFFFFFFF, u64{sbits(1.5f)});
    t.steps(1);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::AdES));
    CHECK_EQ(t.cpu.cp0_read(cop0::BadVAddr), 0xFFFFFFFF80010024ull);

    Rig u(model(Core::R5000));
    u.cpu.gpr[s0] = 0xFFFFFFFF80010000ull;
    u.cpu.gpr[t0] = 0x20;
    u.cpu.fpr[6] = dbits(-7.0);
    u.load({COP1X(0x09, s0, t0, 6, 0), COP1X(0x0F, s0, t0, 0, 0)});
    u.steps(2);
    CHECK_EQ(u.r64(0x10020), dbits(-7.0));
    CHECK_EQ(u.cpu.pc, kCode + 8);

    Rig v(model(Core::R5000), 0);  // without the FPU: coprocessor unusable
    v.load({COP1X(0x01, s0, t0, 0, 2)});
    v.steps(1);
    CHECK_EQ(v.exc_code(), static_cast<unsigned>(Exc::CpU));
}

TEST(cpu_wait_stops_until_an_interrupt_is_requested) {
    for (bool blocks : {false, true}) {
        Rig t(model(Core::R4600), sr::IE | (1u << 15));  // IM7: the timer
        t.cpu.set_block_cache(blocks);
        t.cpu.cp0_write(cop0::Count, 0);
        t.cpu.cp0_write(cop0::Compare, 5000);  // Count runs at half the pipeline clock
        t.load({ADDIU(t0, zero, 1), WAIT, ADDIU(t1, zero, 1)});
        t.load({BEQ(zero, zero, -1), NOP}, 0x180);
        t.cpu.run_until(9000);
        CHECK_EQ(t.cpu.gpr[t0], 1ull);
        CHECK_EQ(t.cpu.gpr[t1], 0ull);  // still waiting
        CHECK_EQ(t.cpu.pc, kCode + 8);
        CHECK(t.cpu.instructions() < 10);  // the time passed without running anything
        t.cpu.run_until(10100);
        CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::Int));
        CHECK_EQ(t.cpu.cp0_read(cop0::EPC), kCode + 8);  // returns past the WAIT
    }
    // An interrupt already requested: WAIT falls through.
    Rig t(model(Core::R5000), 1u << 8);  // IM0, interrupts disabled
    t.cpu.cp0_write(cop0::Cause, 1u << 8);
    t.load({WAIT, ADDIU(t1, zero, 1)});
    t.steps(2);
    CHECK_EQ(t.cpu.gpr[t1], 1ull);
    Rig r(model(Core::R4000));  // no WAIT on the R4000
    r.load({WAIT});
    r.steps(1);
    CHECK_EQ(r.exc_code(), static_cast<unsigned>(Exc::RI));
}

TEST(cpu_two_way_primary_cache_tags) {
    for (Core core : {Core::R4600, Core::R5000}) {
        Rig t(model(core));
        const u64 way_size = core == Core::R4600 ? 0x2000 : 0x4000;
        t.cpu.gpr[t0] = 0xFFFFFFFF80000040ull;             // way 0, set 2
        t.cpu.gpr[t1] = 0xFFFFFFFF80000040ull + way_size;  // the same set, way 1
        t.cpu.gpr[t2] = (0xFFFFFFFF80123040ull & ~(way_size - 1)) | 0x40;  // an address in that set
        // Index Store Tag D: way 1 holds 0x123000's line, way 0 another; Hit
        // Invalidate D finds it in way 1; Index Load Tag reads each way back.
        const u32 tag = static_cast<u32>(((0x123000 & ~(way_size - 1)) >> 12) << 8) | (3u << 6);
        t.cpu.cp0_write(cop0::TagLo, tag);
        t.load({CACHE(0x09, 0, t1), CACHE(0x05, 0, t1), MFC0(a0, cop0::TagLo), CACHE(0x11, 0, t2),
                CACHE(0x05, 0, t1), MFC0(a1, cop0::TagLo), CACHE(0x05, 0, t0), MFC0(a2, cop0::TagLo)});
        t.steps(8);
        CHECK_EQ(t.cpu.gpr[a0], u64{tag});
        CHECK_EQ(t.cpu.gpr[a1], u64{tag & ~0xC0u});  // invalidated
        CHECK_EQ(t.cpu.gpr[a2], 0ull);                // way 0 untouched
    }
}

TEST(cpu_r5000_secondary_cache) {
    Rig t(model(Core::R5000, 512u << 10));
    // The PROM's size probe: lines filled by cached reads at each power of
    // two, then a zero (invalid) tag stored at the base; the size is where
    // the index wraps back to it.
    t.cpu.gpr[t0] = 0xFFFFFFFF88000000ull;
    t.cpu.cp0_write(cop0::TagLo, 0);
    t.load({CACHE(0x0B, 0, t0)});
    t.steps(1);
    u64 size = 0;
    for (u64 pow2 = 128 << 10; pow2 <= (4u << 20); pow2 <<= 1) {
        t.cpu.gpr[t1] = 0xFFFFFFFF88000000ull + pow2;
        t.load({CACHE(0x07, 0, t1)});
        t.cpu.set_pc(kCode);
        t.steps(1);
        if (t.cpu.cp0_read(cop0::TagLo) == 0) {
            size = pow2;
            break;
        }
    }
    CHECK_EQ(size, u64{512u << 10});

    // Index Store Tag takes the tag from the address and the state from
    // TagLo; Page Invalidate clears the page's 128 lines.
    Rig u(model(Core::R5000, 512u << 10));
    u.cpu.gpr[t0] = 0xFFFFFFFF88281020ull;
    u.cpu.gpr[t1] = 0xFFFFFFFF88001000ull;  // the page holding the same index
    u.cpu.cp0_write(cop0::TagLo, 1u << 12);
    u.load({CACHE(0x0B, 0, t0), CACHE(0x07, 0, t0), MFC0(a0, cop0::TagLo), CACHE(0x17, 0, t1),
            CACHE(0x07, 0, t0), MFC0(a1, cop0::TagLo)});
    u.steps(6);
    CHECK_EQ(u.cpu.gpr[a0], u64{((0x8281020u >> 19) << 15) | (1u << 12)});
    CHECK_EQ(u.cpu.gpr[a1], 0ull);
}
