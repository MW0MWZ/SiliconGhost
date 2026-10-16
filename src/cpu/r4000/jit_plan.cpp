// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Register planning shared by the recompiler back ends: which guest
// registers a translation keeps in host registers, and in which of the
// eight cache slots; each back end maps the slots to its host registers.
#include <algorithm>

#include "cpu/r4000/r4000.h"

namespace sg::mips {

namespace {

inline unsigned f_rs(u32 i) { return (i >> 21) & 31; }
inline unsigned f_rt(u32 i) { return (i >> 16) & 31; }
inline unsigned f_rd(u32 i) { return (i >> 11) & 31; }

// Guest registers read and written by an instruction the compiler emits
// inline (others go through their handler, which uses the copies in memory).
struct Uses {
    u32 reads = 0;
    u32 writes = 0;
};
Uses inline_uses(u32 w) {
    const unsigned rs = f_rs(w), rt = f_rt(w), rd = f_rd(w);
    auto bit = [](unsigned r) { return r ? 1u << r : 0u; };
    switch (w >> 26) {
        case 0x00:
            switch (w & 63) {
                case 0x00:
                case 0x02:
                case 0x03: return {bit(rt), bit(rd)};
                case 0x08: return {bit(rs), 0};
                case 0x09: return {bit(rs), bit(rd)};
                case 0x10:
                case 0x12: return {0, bit(rd)};
                case 0x11:
                case 0x13: return {bit(rs), 0};
                case 0x18:
                case 0x19:
                case 0x1A:
                case 0x1B:
                case 0x1C:
                case 0x1D:
                case 0x1E:
                case 0x1F: return {bit(rs) | bit(rt), 0};
                case 0x38:
                case 0x3A:
                case 0x3B:
                case 0x3C:
                case 0x3E:
                case 0x3F: return {bit(rt), bit(rd)};
                case 0x30:
                case 0x31:
                case 0x32:
                case 0x33:
                case 0x34:
                case 0x36: return {bit(rs) | bit(rt), 0};
                case 0x14:
                case 0x16:
                case 0x17:
                case 0x20:
                case 0x22:
                case 0x2C:
                case 0x2D:
                case 0x2E:
                case 0x2F:
                case 0x04:
                case 0x06:
                case 0x07:
                case 0x21:
                case 0x23:
                case 0x24:
                case 0x25:
                case 0x26:
                case 0x27:
                case 0x2A:
                case 0x2B: return {bit(rs) | bit(rt), bit(rd)};
                default: return {};
            }
        case 0x01: return (rt < 4 || (rt >= 0x08 && rt <= 0x0E)) ? Uses{bit(rs), 0} : Uses{};
        case 0x03: return {0, bit(31)};
        case 0x04:
        case 0x05:
        case 0x14:
        case 0x15: return {bit(rs) | bit(rt), 0};
        case 0x06:
        case 0x07:
        case 0x16:
        case 0x17: return {bit(rs), 0};
        case 0x08:
        case 0x18:
        case 0x19:
        case 0x09:
        case 0x0A:
        case 0x0B:
        case 0x0C:
        case 0x0D:
        case 0x0E: return {bit(rs), bit(rt)};
        case 0x0F: return {0, bit(rt)};
        case 0x11:  // MFC1, DMFC1, MTC1, DMTC1
            if (rs == 0x00 || rs == 0x01) return {0, bit(rt)};
            if (rs == 0x04 || rs == 0x05) return {bit(rt), 0};
            return {};
        case 0x20:
        case 0x21:
        case 0x23:
        case 0x24:
        case 0x25:
        case 0x27:
        case 0x37: return {bit(rs), bit(rt)};
        case 0x1A:
        case 0x1B:
        case 0x22:
        case 0x26: return {bit(rs) | bit(rt), bit(rt)};  // the merges read rt too
        case 0x28:
        case 0x29:
        case 0x2A:
        case 0x2B:
        case 0x2C:
        case 0x2D:
        case 0x2E:
        case 0x3F: return {bit(rs) | bit(rt), 0};
        case 0x31:  // LWC1, LDC1, SWC1, SDC1: rt is an FPU register
        case 0x35:
        case 0x39:
        case 0x3D: return {bit(rs), 0};
        default: return {};
    }
}

// pc delta (from the branch) of a conditional branch's target.
bool branch_offset(u32 w, s64& delta) {
    const unsigned opc = w >> 26;
    const bool conditional = (opc >= 0x04 && opc <= 0x07) || (opc >= 0x14 && opc <= 0x17) || (opc == 0x01 && f_rt(w) < 4);
    delta = 4 + 4 * static_cast<s64>(static_cast<s16>(w & 0xFFFF));
    return conditional;
}

}  // namespace

R4000::JitPlan R4000::jit_plan(const Block& b) {
    JitPlan p;
    std::fill(std::begin(p.slot), std::end(p.slot), static_cast<s8>(-1));
    unsigned uses[32] = {};
    for (u32 i = 0; i < b.count; ++i) {
        if (i > 0 && is_branch_word(b.ops[i - 1].inst)) {
            if (is_branch_word(b.ops[i].inst)) continue;  // a branch in a delay slot goes to its handler
        }
        const Uses u = inline_uses(b.ops[i].inst);
        for (unsigned r = 1; r < 32; ++r) uses[r] += ((u.reads >> r) & 1) + ((u.writes >> r) & 1);
        p.written |= u.writes;
        s64 delta;
        if (branch_offset(b.ops[i].inst, delta) && !(i > 0 && is_branch_word(b.ops[i - 1].inst)) &&
            4 * static_cast<s64>(i) + delta == 0)
            p.self_loop = true;
    }
    // The registers used at least twice, most used first, each in its home
    // slot (number mod 8) where free.
    unsigned order[31];
    for (unsigned r = 0; r < 31; ++r) order[r] = r + 1;
    std::stable_sort(order, order + 31, [&](unsigned x, unsigned y) { return uses[x] > uses[y]; });
    unsigned taken = 0, n = 0;
    for (unsigned r : order) {
        if (uses[r] < 2 || n == kJitSlots) break;
        unsigned s = r % kJitSlots;
        if ((taken >> s) & 1) {
            s = 0;
            while ((taken >> s) & 1) ++s;
        }
        taken |= 1u << s;
        p.slot[r] = static_cast<s8>(s);
        p.alloc |= 1u << r;
        ++n;
    }
    p.written &= p.alloc;
    return p;
}

void R4000::jit_fp_uses(u32 w, const std::function<void(unsigned, bool, bool)>& use) {
    const unsigned opc = w >> 26, fmt = f_rs(w), ft = f_rt(w), fs = f_rd(w), fd = (w >> 6) & 31, func = w & 63;
    switch (opc) {
        case 0x31: return use(ft, false, true);  // LWC1
        case 0x35: return use(ft, true, true);   // LDC1
        case 0x39: return use(ft, false, false);  // SWC1
        case 0x3D: return use(ft, true, false);   // SDC1
        case 0x11: break;
        default: return;
    }
    switch (fmt) {
        case 0x00:
        case 0x01: return use(fs, fmt == 0x01, false);  // MFC1, DMFC1
        case 0x04:
        case 0x05: return use(fs, fmt == 0x05, true);  // MTC1, DMTC1
        case 0x14:  // CVT.S.W, CVT.D.W
            if (func == 0x20 || func == 0x21) use(fs, false, false), use(fd, func == 0x21, true);
            return;
        case 0x10:
        case 0x11: break;
        default: return;
    }
    const bool dbl = fmt == 0x11;
    if (func <= 0x03) {
        use(fs, dbl, false), use(ft, dbl, false), use(fd, dbl, true);
    } else if (func <= 0x07) {  // SQRT, ABS, MOV, NEG
        use(fs, dbl, false), use(fd, dbl, true);
    } else if ((func == 0x20 && dbl) || (func == 0x21 && !dbl)) {  // CVT.S.D, CVT.D.S
        use(fs, dbl, false), use(fd, !dbl, true);
    } else if (func == 0x0C || func == 0x0D || func == 0x24) {  // ROUND.W, TRUNC.W, CVT.W
        use(fs, dbl, false), use(fd, false, true);
    } else if (func >= 0x30) {  // C.cond
        use(fs, dbl, false), use(ft, dbl, false);
    }
}

}  // namespace sg::mips
