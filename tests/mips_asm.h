// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Minimal MIPS instruction encoders for writing CPU tests by hand.
#pragma once

#include "core/types.h"

namespace sg::test::as {

enum Reg : unsigned {
    zero, at, v0, v1, a0, a1, a2, a3, t0, t1, t2, t3, t4, t5, t6, t7,
    s0, s1, s2, s3, s4, s5, s6, s7, t8, t9, k0, k1, gp, sp, s8, ra,
};

constexpr u32 R(unsigned funct, unsigned rs, unsigned rt, unsigned rd, unsigned sa = 0) {
    return (rs << 21) | (rt << 16) | (rd << 11) | (sa << 6) | funct;
}
constexpr u32 I(unsigned op, unsigned rs, unsigned rt, s32 imm) {
    return (op << 26) | (rs << 21) | (rt << 16) | (static_cast<u32>(imm) & 0xFFFF);
}

constexpr u32 NOP = 0;
constexpr u32 SLL(unsigned rd, unsigned rt, unsigned sa) { return R(0x00, 0, rt, rd, sa); }
constexpr u32 JR(unsigned rs) { return R(0x08, rs, 0, 0); }
constexpr u32 SYSCALL = 0x0000000C;
constexpr u32 MFHI(unsigned rd) { return R(0x10, 0, 0, rd); }
constexpr u32 MFLO(unsigned rd) { return R(0x12, 0, 0, rd); }
constexpr u32 MULT(unsigned rs, unsigned rt) { return R(0x18, rs, rt, 0); }
constexpr u32 DIV(unsigned rs, unsigned rt) { return R(0x1A, rs, rt, 0); }
constexpr u32 DMULTU(unsigned rs, unsigned rt) { return R(0x1D, rs, rt, 0); }
constexpr u32 DDIV(unsigned rs, unsigned rt) { return R(0x1E, rs, rt, 0); }
constexpr u32 ADD(unsigned rd, unsigned rs, unsigned rt) { return R(0x20, rs, rt, rd); }
constexpr u32 ADDU(unsigned rd, unsigned rs, unsigned rt) { return R(0x21, rs, rt, rd); }
constexpr u32 DADDU(unsigned rd, unsigned rs, unsigned rt) { return R(0x2D, rs, rt, rd); }
constexpr u32 DSLL32(unsigned rd, unsigned rt, unsigned sa) { return R(0x3C, 0, rt, rd, sa); }

constexpr u32 BEQ(unsigned rs, unsigned rt, s32 off) { return I(0x04, rs, rt, off); }
constexpr u32 BNE(unsigned rs, unsigned rt, s32 off) { return I(0x05, rs, rt, off); }
constexpr u32 BEQL(unsigned rs, unsigned rt, s32 off) { return I(0x14, rs, rt, off); }
constexpr u32 ADDIU(unsigned rt, unsigned rs, s32 imm) { return I(0x09, rs, rt, imm); }
constexpr u32 ADDI(unsigned rt, unsigned rs, s32 imm) { return I(0x08, rs, rt, imm); }
constexpr u32 SUBU(unsigned rd, unsigned rs, unsigned rt) { return R(0x23, rs, rt, rd); }
constexpr u32 BGTZ(unsigned rs, s32 off) { return I(0x07, rs, 0, off); }
constexpr u32 BGEZL(unsigned rs, s32 off) { return I(0x01, rs, 0x03, off); }
constexpr u32 ANDI(unsigned rt, unsigned rs, u32 imm) { return I(0x0C, rs, rt, static_cast<s32>(imm)); }
constexpr u32 ORI(unsigned rt, unsigned rs, u32 imm) { return I(0x0D, rs, rt, static_cast<s32>(imm)); }
constexpr u32 LUI(unsigned rt, u32 imm) { return I(0x0F, 0, rt, static_cast<s32>(imm)); }
constexpr u32 LW(unsigned rt, s32 off, unsigned base) { return I(0x23, base, rt, off); }
constexpr u32 LWL(unsigned rt, s32 off, unsigned base) { return I(0x22, base, rt, off); }
constexpr u32 LWR(unsigned rt, s32 off, unsigned base) { return I(0x26, base, rt, off); }
constexpr u32 SW(unsigned rt, s32 off, unsigned base) { return I(0x2B, base, rt, off); }
constexpr u32 SWL(unsigned rt, s32 off, unsigned base) { return I(0x2A, base, rt, off); }
constexpr u32 SWR(unsigned rt, s32 off, unsigned base) { return I(0x2E, base, rt, off); }
constexpr u32 LD(unsigned rt, s32 off, unsigned base) { return I(0x37, base, rt, off); }

constexpr u32 MFC0(unsigned rt, unsigned rd) { return (0x10u << 26) | (0x00u << 21) | (rt << 16) | (rd << 11); }
constexpr u32 MTC0(unsigned rt, unsigned rd) { return (0x10u << 26) | (0x04u << 21) | (rt << 16) | (rd << 11); }
constexpr u32 TLBWI = 0x42000002;
constexpr u32 TLBP = 0x42000008;
constexpr u32 ERET = 0x42000018;

}  // namespace sg::test::as
