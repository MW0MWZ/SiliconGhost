// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// FPU (COP1) and CACHE instruction tests.
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

enum : unsigned { S = 16, D = 17, W = 20, L = 21 };
constexpr u32 FOP(unsigned fmt, unsigned func, unsigned fd, unsigned fs, unsigned ft = 0) {
    return (0x11u << 26) | (fmt << 21) | (ft << 16) | (fs << 11) | (fd << 6) | func;
}
constexpr u32 CTC1(unsigned rt, unsigned fs) { return (0x11u << 26) | (6u << 21) | (rt << 16) | (fs << 11); }
constexpr u32 CFC1(unsigned rt, unsigned fs) { return (0x11u << 26) | (2u << 21) | (rt << 16) | (fs << 11); }
constexpr u32 MTC1(unsigned rt, unsigned fs) { return (0x11u << 26) | (4u << 21) | (rt << 16) | (fs << 11); }
constexpr u32 BC1T(s32 off) { return (0x11u << 26) | (8u << 21) | (1u << 16) | (static_cast<u32>(off) & 0xFFFF); }
constexpr u32 CACHE(unsigned op, s32 off, unsigned base) { return I(0x2F, base, op, off); }

constexpr u32 kFcrE = 1u << 17, kFcrV = 1u << 16, kFcrZ = 1u << 15, kFcrU = 1u << 13, kFcrI = 1u << 12;

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

struct Rig {
    std::vector<u8> ram = std::vector<u8>(8u << 20);
    Bus bus;
    R4000 cpu{bus};
    Rig(u32 extra_status = sr::FR) {
        bus.map_ram(0, ram.size(), ram.data());
        cpu.cp0_write(cop0::Status, sr::CU1 | extra_status);
        cpu.set_pc(kCode);
    }
    void load(std::initializer_list<u32> prog) {
        u64 a = 0x1000;
        for (u32 w : prog) {
            store_be<u32>(&ram[a], w);
            a += 4;
        }
    }
    void steps(int n) {
        for (int i = 0; i < n; ++i) cpu.step();
    }
    unsigned exc_code() const { return (cpu.cp0_read(cop0::Cause) >> 2) & 31; }
};

}  // namespace

TEST(fpu_basic_double_arithmetic) {
    Rig t;
    t.cpu.fpr[2] = dbits(1.5);
    t.cpu.fpr[4] = dbits(2.25);
    t.load({FOP(D, 0x00, 6, 2, 4), FOP(D, 0x02, 8, 6, 4), FOP(D, 0x04, 10, 4)});
    t.steps(3);
    CHECK_EQ(t.cpu.fpr[6], dbits(3.75));
    CHECK_EQ(t.cpu.fpr[8], dbits(3.75 * 2.25));
    CHECK_EQ(t.cpu.fpr[10], dbits(1.5));
    CHECK_EQ(t.cpu.fcr31, 0u);
}

// Each rounding mode in FCR31 applies to the arithmetic, the inexact flag
// is raised, and the host's own rounding is left as it was.
TEST(fpu_rounding_modes_apply_to_arithmetic) {
    const double tiny = 0x1p-60;
    struct Case {
        u32 rm;
        double a, want;
    };
    for (const Case c : {Case{0, 1.0, 1.0}, Case{1, -1.0, -1.0}, Case{2, 1.0, 0x1.0000000000001p0},
                         Case{3, -1.0, -0x1.0000000000001p0}, Case{2, -1.0, -1.0}, Case{3, 1.0, 1.0}}) {
        Rig t;
        t.cpu.fcr31 = c.rm;
        t.cpu.fpr[2] = dbits(c.a);
        t.cpu.fpr[4] = dbits(c.a > 0 ? tiny : -tiny);
        t.load({FOP(D, 0x00, 6, 2, 4)});  // ADD.D
        t.steps(1);
        CHECK_EQ(t.cpu.fpr[6], dbits(c.want));
        CHECK(t.cpu.fcr31 & kFcrI);
        volatile double x = 1.0, y = tiny;
        CHECK(x + y == 1.0);  // the host still rounds to nearest
    }
}

TEST(fpu_divide_by_zero_untrapped_and_trapped) {
    Rig t;
    t.cpu.fpr[2] = sbits(1.0f);
    t.cpu.fpr[4] = sbits(0.0f);
    t.load({FOP(S, 0x03, 6, 2, 4)});
    t.steps(1);
    CHECK_EQ(t.cpu.fpr[6] & 0xFFFFFFFF, 0x7F800000ull);
    CHECK_EQ(t.cpu.fcr31 & (kFcrZ | (1u << 5)), kFcrZ | (1u << 5));  // cause Z + sticky flag Z

    Rig u;
    u.cpu.fcr31 = 1u << 10;  // enable Z
    u.cpu.fpr[2] = sbits(1.0f);
    u.cpu.fpr[4] = sbits(0.0f);
    u.cpu.fpr[6] = 0x1234;
    u.load({FOP(S, 0x03, 6, 2, 4)});
    u.steps(1);
    CHECK_EQ(u.cpu.pc, kGeneralVector);
    CHECK_EQ(u.exc_code(), static_cast<unsigned>(Exc::FPE));
    CHECK_EQ(u.cpu.fpr[6], 0x1234ull);  // destination untouched
    CHECK_EQ(u.cpu.fcr31 & (1u << 5), 0u);  // flags not updated on a trap
}

TEST(fpu_denormal_operand_is_unimplemented) {
    Rig t;
    t.cpu.fpr[2] = 1;  // smallest double denormal
    t.cpu.fpr[4] = dbits(1.0);
    t.cpu.fpr[6] = 0x55;
    t.load({FOP(D, 0x00, 6, 2, 4)});
    t.steps(1);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::FPE));
    CHECK(t.cpu.fcr31 & kFcrE);
    CHECK_EQ(t.cpu.fpr[6], 0x55ull);
    CHECK_EQ(t.cpu.cp0_read(cop0::EPC), kCode);
}

TEST(fpu_nan_operands) {
    // A quiet NaN (legacy encoding: signal bit clear) traps as unimplemented.
    Rig q;
    q.cpu.fpr[2] = 0x7FF7FFFFFFFFFFFFull;
    q.load({FOP(D, 0x07, 4, 2)});
    q.steps(1);
    CHECK_EQ(q.exc_code(), static_cast<unsigned>(Exc::FPE));
    CHECK(q.cpu.fcr31 & kFcrE);

    // A signaling NaN raises Invalid; untrapped, the default NaN is delivered.
    Rig s;
    s.cpu.fpr[2] = 0x7FF8000000000000ull;
    s.cpu.fpr[4] = dbits(1.0);
    s.load({FOP(D, 0x00, 6, 2, 4)});
    s.steps(1);
    CHECK_EQ(s.cpu.pc, kCode + 4);
    CHECK_EQ(s.cpu.fpr[6], 0x7FF7FFFFFFFFFFFFull);
    CHECK(s.cpu.fcr31 & kFcrV);
}

TEST(fpu_underflow_flush_and_trap) {
    // FS=0: tiny result traps as unimplemented.
    Rig t;
    t.cpu.fpr[2] = sbits(1e-30f);
    t.cpu.fpr[4] = sbits(1e-30f);
    t.load({FOP(S, 0x02, 6, 2, 4)});
    t.steps(1);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::FPE));
    CHECK(t.cpu.fcr31 & kFcrE);

    // FS=1 with U/I disabled: flushed to zero, U and I set.
    Rig u;
    u.cpu.fcr31 = 1u << 24;
    u.cpu.fpr[2] = sbits(1e-30f);
    u.cpu.fpr[4] = sbits(1e-30f);
    u.cpu.fpr[6] = 0xFFFF;
    u.load({FOP(S, 0x02, 6, 2, 4)});
    u.steps(1);
    CHECK_EQ(u.cpu.pc, kCode + 4);
    CHECK_EQ(u.cpu.fpr[6] & 0xFFFFFFFF, 0ull);
    CHECK_EQ(u.cpu.fcr31 & (kFcrU | kFcrI), kFcrU | kFcrI);
}

TEST(fpu_conversions) {
    Rig t;
    t.cpu.fpr[2] = dbits(2.5);
    t.cpu.fpr[4] = dbits(-2.7);
    t.cpu.fpr[12] = 7;  // integer word 7
    t.load({FOP(D, 0x24, 6, 2),    // CVT.W.D 2.5 (round to nearest even) -> 2
            FOP(D, 0x0D, 7, 4),    // TRUNC.W.D -2.7 -> -2
            FOP(D, 0x0E, 8, 4),    // CEIL.W.D -2.7 -> -2
            FOP(D, 0x0F, 9, 4),    // FLOOR.W.D -2.7 -> -3
            FOP(W, 0x21, 10, 12),  // CVT.D.W 7
            FOP(D, 0x20, 11, 2)}); // CVT.S.D 2.5
    t.steps(6);
    CHECK_EQ(t.cpu.fpr[6] & 0xFFFFFFFF, 2ull);
    CHECK_EQ(t.cpu.fpr[7] & 0xFFFFFFFF, 0xFFFFFFFEull);
    CHECK_EQ(t.cpu.fpr[8] & 0xFFFFFFFF, 0xFFFFFFFEull);
    CHECK_EQ(t.cpu.fpr[9] & 0xFFFFFFFF, 0xFFFFFFFDull);
    CHECK_EQ(t.cpu.fpr[10], dbits(7.0));
    CHECK_EQ(t.cpu.fpr[11] & 0xFFFFFFFF, static_cast<u64>(sbits(2.5f)));

    Rig o;  // out of range for a word: unimplemented
    o.cpu.fpr[2] = dbits(3e10);
    o.load({FOP(D, 0x24, 6, 2)});
    o.steps(1);
    CHECK_EQ(o.exc_code(), static_cast<unsigned>(Exc::FPE));
    CHECK(o.cpu.fcr31 & kFcrE);
}

TEST(fpu_compare_and_branch) {
    Rig t;
    t.cpu.fpr[2] = dbits(1.0);
    t.cpu.fpr[4] = dbits(2.0);
    // C.LT.D f2, f4 ; nop ; BC1T +2 ; ADDIU t0 (delay) ; ADDIU t1 (skipped) ; ADDIU t2
    t.load({FOP(D, 0x3C, 0, 2, 4), NOP, BC1T(2), ADDIU(t0, zero, 1), ADDIU(t1, zero, 1), ADDIU(t2, zero, 1)});
    t.steps(5);
    CHECK(t.cpu.fcr31 & (1u << 23));
    CHECK_EQ(t.cpu.gpr[t0], 1ull);
    CHECK_EQ(t.cpu.gpr[t1], 0ull);
    CHECK_EQ(t.cpu.gpr[t2], 1ull);
}

TEST(fpu_control_registers) {
    Rig t;
    t.cpu.gpr[t0] = (1u << 15) | (1u << 10);  // cause Z with enable Z
    t.load({CFC1(t1, 0), CTC1(t0, 31)});
    t.steps(2);
    CHECK_EQ(t.cpu.gpr[t1], 0x500ull);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::FPE));
}

TEST(fpu_fr0_register_pairs) {
    Rig t(0);  // FR = 0
    t.cpu.gpr[t0] = 0x40000000;  // high word of 2.0
    t.cpu.gpr[t1] = 0;
    t.load({MTC1(t1, 0), MTC1(t0, 1), FOP(D, 0x00, 2, 0, 0)});
    t.steps(3);
    CHECK_EQ(t.cpu.fpr[0], dbits(2.0));
    CHECK_EQ(t.cpu.fpr[2], dbits(4.0));
}

TEST(fpu_unusable_without_cu1) {
    Rig t(0);
    t.cpu.cp0_write(cop0::Status, 0);
    t.load({FOP(D, 0x00, 2, 0, 0)});
    t.steps(1);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::CpU));
    CHECK_EQ((t.cpu.cp0_read(cop0::Cause) >> 28) & 3, 1ull);
}

TEST(cache_tag_round_trip) {
    Rig t;
    t.cpu.gpr[t0] = 0xFFFFFFFF80004000ull;
    t.cpu.cp0_write(cop0::TagLo, 0x12345C00);  // stag | SState 7
    t.load({CACHE(0x0B, 0, t0), CACHE(0x07, 0, t0), CACHE(0x09, 0, t0), CACHE(0x05, 0, t0)});
    t.steps(2);
    CHECK_EQ(t.cpu.cp0_read(cop0::TagLo), 0x12345C00ull | (4ull << 7));  // VIndex = VA[14:12]
    t.cpu.cp0_write(cop0::TagLo, 0xABCDEFC0);
    t.steps(2);
    CHECK_EQ(t.cpu.cp0_read(cop0::TagLo), 0xABCDEFC0ull);
}

// SC succeeds on the LLbit alone (a successful SC leaves it set); a CACHE
// operation that invalidates the data line holding the link address clears
// it, one on another line does not.
TEST(cache_op_on_the_link_line_fails_store_conditional) {
    constexpr auto LL = [](unsigned rt, s32 off, unsigned base) { return I(0x30, base, rt, off); };
    constexpr auto SC = [](unsigned rt, s32 off, unsigned base) { return I(0x38, base, rt, off); };
    Rig t;
    t.cpu.gpr[t0] = 0xFFFFFFFF80001000ull;
    t.cpu.gpr[t2] = t.cpu.gpr[t3] = 7;
    t.load({LL(t1, 0, t0), SC(t2, 0, t0), SC(t3, 0, t0)});
    t.steps(3);
    CHECK_EQ(t.cpu.gpr[t2], 1ull);
    CHECK_EQ(t.cpu.gpr[t3], 1ull);
    t.cpu.gpr[t2] = 7;
    t.load({LL(t1, 0, t0), CACHE(0x11, 0x100, t0), SC(t2, 0, t0)});  // Hit Invalidate D, another line
    t.cpu.set_pc(kCode);
    t.steps(3);
    CHECK_EQ(t.cpu.gpr[t2], 1ull);
    t.cpu.gpr[t2] = 7;
    t.load({LL(t1, 0, t0), CACHE(0x11, 4, t0), SC(t2, 0, t0)});  // the link line
    t.cpu.set_pc(kCode);
    t.steps(3);
    CHECK_EQ(t.cpu.gpr[t2], 0ull);
}

TEST(cache_secondary_size_probe) {
    // Store a zero tag at the base, then look for the power of two whose index wraps to it.
    Rig t;
    t.cpu.gpr[t0] = 0xFFFFFFFF80000000ull;
    t.cpu.cp0_write(cop0::TagLo, 0);
    t.load({CACHE(0x0B, 0, t0)});
    t.steps(1);
    u64 size = 0;
    for (u64 pow2 = 128 << 10; pow2 <= (4u << 20); pow2 <<= 1) {
        t.cpu.gpr[t1] = 0xFFFFFFFF80000000ull + pow2;
        t.load({CACHE(0x07, 0, t1)});
        t.cpu.set_pc(kCode);
        t.steps(1);
        if (t.cpu.cp0_read(cop0::TagLo) == 0) {
            size = pow2;
            break;
        }
    }
    CHECK_EQ(size, 1ull << 20);
}

TEST(cp0_config_and_random_after_wired) {
    Rig t;
    t.cpu.cp0_write(cop0::Config, 0);
    CHECK_EQ(t.cpu.cp0_read(cop0::Config) & 0x3F, 0ull);
    CHECK_EQ(t.cpu.cp0_read(cop0::Config) & ~0x3Full, 0x10C08480ull);
    t.steps(5);
    t.cpu.cp0_write(cop0::Wired, 8);
    CHECK_EQ(t.cpu.cp0_read(cop0::Random), 47ull);
}

// CVT.W.D in each rounding mode, ties included, as the R4000 rounds.
TEST(fpu_convert_to_integer_in_every_rounding_mode) {
    struct Case {
        double v;
        s32 want[4];  // RN, RZ, RP, RM
    };
    for (const Case c : {Case{2.5, {2, 2, 3, 2}}, Case{3.5, {4, 3, 4, 3}}, Case{-2.5, {-2, -2, -2, -3}},
                         Case{-2.7, {-3, -2, -2, -3}}, Case{0.49999999999999994, {0, 0, 1, 0}},
                         Case{-0.5, {0, 0, 0, -1}}, Case{1e9 + 0.5, {1000000000, 1000000000, 1000000001, 1000000000}}}) {
        for (u32 rm = 0; rm < 4; ++rm) {
            Rig t;
            t.cpu.fcr31 = rm;
            t.cpu.fpr[2] = dbits(c.v);
            t.load({FOP(D, 0x24, 6, 2)});  // CVT.W.D
            t.steps(1);
            CHECK_EQ(static_cast<s32>(t.cpu.fpr[6] & 0xFFFFFFFF), c.want[rm]);
        }
    }
}
