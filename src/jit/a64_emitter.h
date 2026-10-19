// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Minimal AArch64 machine-code emitter for the recompiler: the integer
// instructions it uses, encoded into a byte buffer. Helpers that take any
// immediate or offset fall back to building the value in x16 or x17, which
// are reserved for them (the AAPCS64 intra-procedure-call scratch registers).
//
// References: Arm Architecture Reference Manual for A-profile, A64 base
// instruction encodings.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <vector>

#include "core/types.h"

namespace sg::jit::a64 {

// General registers. 31 is the zero register in most operand positions and
// the stack pointer in a few (address bases, ADD/SUB immediate).
enum Reg : u8 {
    x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15,
    x16, x17, x18, x19, x20, x21, x22, x23, x24, x25, x26, x27, x28, x29, x30,
    xzr = 31, sp = 31,
};
constexpr Reg kScratch0 = x16, kScratch1 = x17;  // the emitter's own

enum class Cond : u8 { Eq = 0, Ne = 1, Hs = 2, Lo = 3, Mi = 4, Pl = 5, Vs = 6, Vc = 7,
                       Hi = 8, Ls = 9, Ge = 10, Lt = 11, Gt = 12, Le = 13, Al = 14 };
constexpr Cond invert(Cond c) { return static_cast<Cond>(static_cast<u8>(c) ^ 1); }

enum class Shift : u8 { Lsl = 0, Lsr = 1, Asr = 2 };

// Bitmask immediate for AND/ORR/EOR/ANDS: true and the N:immr:imms fields if
// `value` (of `width` 32 or 64 bits) is a rotated run of ones repeated.
inline bool encode_logical(u64 value, unsigned width, u32& fields) {
    if (width == 32) value = (value & 0xFFFFFFFFull) | (value << 32);
    if (value == 0 || value == ~u64{0}) return false;
    unsigned size = 64;
    while (size > 2) {  // the smallest repeating element
        const unsigned half = size / 2;
        const u64 mask = (u64{1} << half) - 1;
        if ((value & mask) != ((value >> half) & mask)) break;
        size = half;
    }
    const u64 mask = size == 64 ? ~u64{0} : (u64{1} << size) - 1;
    u64 elt = value & mask;
    // Rotate the element right until its run of ones starts at bit 0.
    unsigned rot = 0;
    auto ror = [&](u64 v) { return ((v >> 1) | (v << (size - 1))) & mask; };
    while ((elt & 1) == 0 || ((elt >> (size - 1)) & 1)) {
        elt = ror(elt);
        if (++rot >= size) return false;
    }
    unsigned ones = 0;
    while ((elt >> ones) & 1) ++ones;
    if ((elt >> ones) != 0) return false;  // more than one run
    const u32 immr = (size - rot) % size;
    const u32 imms = ((~(size - 1) << 1) & 0x3F) | (ones - 1);
    const u32 n = size == 64 ? 1 : 0;
    fields = (n << 22) | (immr << 16) | (imms << 10);
    return true;
}

class Emitter {
public:
    Emitter() { code_.reserve(4096); }
    const std::vector<u8>& code() const { return code_; }
    std::size_t size() const { return code_.size(); }

    // --- labels: a site is the offset of a branch to patch -----------------
    enum class Fix : u8 { B26, B19, B14, Adr };
    struct Site {
        std::size_t at = 0;
        Fix kind = Fix::B26;
    };
    // Points the branch at `site` to the current position (or `target`).
    void bind(const Site& s) { bind_to(s, code_.size()); }
    void bind_to(const Site& s, std::size_t target) {
        const s64 bytes = static_cast<s64>(target) - static_cast<s64>(s.at), delta = bytes / 4;
        u32 w;
        std::memcpy(&w, &code_[s.at], 4);
        switch (s.kind) {
            case Fix::Adr:
                w = (w & 0x9F00001Fu) | ((static_cast<u32>(bytes) & 3) << 29) | ((static_cast<u32>(bytes >> 2) & 0x7FFFFu) << 5);
                break;
            case Fix::B26: w = (w & 0xFC000000u) | (static_cast<u32>(delta) & 0x03FFFFFFu); break;
            case Fix::B19: w = (w & 0xFF00001Fu) | ((static_cast<u32>(delta) & 0x7FFFFu) << 5); break;
            case Fix::B14: w = (w & 0xFFF8001Fu) | ((static_cast<u32>(delta) & 0x3FFFu) << 5); break;
        }
        std::memcpy(&code_[s.at], &w, 4);
    }
    Site b() { return emit_site(0x14000000u, Fix::B26); }
    Site b(Cond c) { return emit_site(0x54000000u | static_cast<u32>(c), Fix::B19); }
    Site cbz(Reg r, bool w64 = true) { return emit_site((w64 ? 0xB4000000u : 0x34000000u) | r, Fix::B19); }
    Site cbnz(Reg r, bool w64 = true) { return emit_site((w64 ? 0xB5000000u : 0x35000000u) | r, Fix::B19); }
    // TBNZ: to the site if bit `bit` of r is set.
    Site cbnz_bit(Reg r, unsigned bit) {
        return emit_site(0x37000000u | ((bit >> 5) << 31) | ((bit & 31) << 19) | r, Fix::B14);
    }
    // TBZ: to the site if bit `bit` of r is clear.
    Site cbz_bit(Reg r, unsigned bit) {
        return emit_site(0x36000000u | ((bit >> 5) << 31) | ((bit & 31) << 19) | r, Fix::B14);
    }
    void b_to(std::size_t target) { bind_to(b(), target); }
    // A branch to `target`, this code being placed at `base` (within 128 MiB).
    void b_abs(const u8* target, const u8* base) {
        emit(0x14000000u | (static_cast<u32>((target - (base + code_.size())) / 4) & 0x03FFFFFFu));
    }
    // ADR: d = the address the site is bound to.
    Site adr(Reg d) { return emit_site(0x10000000u | d, Fix::Adr); }
    void br(Reg r) { emit(0xD61F0000u | (u32{r} << 5)); }
    void blr(Reg r) { emit(0xD63F0000u | (u32{r} << 5)); }
    void ret() { emit(0xD65F03C0u); }

    // --- moves --------------------------------------------------------------
    void mov(Reg d, Reg s, bool w64 = true) {  // ORR d, zr, s (register forms only; not sp)
        emit((w64 ? 0xAA000000u : 0x2A000000u) | (u32{s} << 16) | (31u << 5) | d);
    }
    void mov_from_sp(Reg d) { emit(0x91000000u | (31u << 5) | d); }  // ADD d, sp, #0
    void mov_imm(Reg d, u64 v) {
        // MOVZ, or MOVN when more half-words are all ones than zero, then a
        // MOVK for each half-word the first instruction did not set.
        unsigned zeros = 0, ones = 0;
        for (unsigned hw = 0; hw < 4; ++hw) {
            const u32 part = static_cast<u32>(v >> (16 * hw)) & 0xFFFF;
            zeros += part == 0;
            ones += part == 0xFFFF;
        }
        const bool inverted = ones > zeros;
        const u32 fill = inverted ? 0xFFFF : 0;
        bool first = true;
        for (unsigned hw = 0; hw < 4; ++hw) {
            const u32 part = static_cast<u32>(v >> (16 * hw)) & 0xFFFF;
            if (part == fill) continue;
            if (first) {
                emit((inverted ? 0x92800000u : 0xD2800000u) | (hw << 21) | ((inverted ? ~part & 0xFFFF : part) << 5) | d);
                first = false;
            } else {
                emit(0xF2800000u | (hw << 21) | (part << 5) | d);
            }
        }
        if (first) emit((inverted ? 0x92800000u : 0xD2800000u) | d);  // all zeros or all ones
    }
    // A 64-bit constant: built in place when that takes at most two
    // instructions, otherwise loaded from the literal pool that
    // finish_literals() places after the code (one PC-relative load).
    void load_constant(Reg d, u64 v) {
        unsigned zeros = 0, ones = 0;
        for (unsigned hw = 0; hw < 4; ++hw) {
            const u32 part = static_cast<u32>(v >> (16 * hw)) & 0xFFFF;
            zeros += part == 0;
            ones += part == 0xFFFF;
        }
        if (std::max(zeros, ones) >= 2) return mov_imm(d, v);
        literals_.push_back({emit_site(0x58000000u | d, Fix::B19), v});  // LDR Xd, literal
    }
    // Places the literal pool (8-byte aligned) and points the loads at it.
    // Call once, after the last instruction.
    void finish_literals() {
        if (literals_.empty()) return;
        if (code_.size() % 8) emit(0xD503201Fu);  // NOP: align the pool
        std::vector<std::pair<u64, std::size_t>> pool;  // value, offset
        for (const Literal& l : literals_) {
            std::size_t at = 0;
            bool found = false;
            for (const auto& [value, offset] : pool)
                if (value == l.value) at = offset, found = true;
            if (!found) {
                at = code_.size();
                code_.resize(at + 8);
                std::memcpy(&code_[at], &l.value, 8);  // little-endian, as the host
                pool.emplace_back(l.value, at);
            }
            bind_to(l.site, at);
        }
        literals_.clear();
    }
    void mov_imm32(Reg d, u32 v) {  // W register: upper half cleared
        emit(0x52800000u | ((v & 0xFFFF) << 5) | d);
        if (v >> 16) emit(0x72A00000u | ((v >> 16) << 5) | d);
    }

    // --- arithmetic -----------------------------------------------------------
    // d = n + v / n - v, any v (n may be sp only when v fits).
    void add_imm(Reg d, Reg n, s64 v, bool w64 = true) {
        if (v < 0) return sub_imm(d, n, -v, w64);
        add_sub_imm(false, false, d, n, static_cast<u64>(v), w64);
    }
    void sub_imm(Reg d, Reg n, s64 v, bool w64 = true) {
        if (v < 0) return add_imm(d, n, -v, w64);
        add_sub_imm(true, false, d, n, static_cast<u64>(v), w64);
    }
    void cmp_imm(Reg n, s64 v, bool w64 = true) {  // SUBS zr, n, v
        if (v < 0) return add_sub_imm(false, true, xzr, n, static_cast<u64>(-v), w64);
        add_sub_imm(true, true, xzr, n, static_cast<u64>(v), w64);
    }
    void add(Reg d, Reg n, Reg m, bool w64 = true, Shift sh = Shift::Lsl, unsigned amount = 0) {
        rrr(0x0B000000u, d, n, m, w64, sh, amount);
    }
    void sub(Reg d, Reg n, Reg m, bool w64 = true) { rrr(0x4B000000u, d, n, m, w64); }
    void cmp(Reg n, Reg m, bool w64 = true) { rrr(0x6B000000u, xzr, n, m, w64); }
    void and_(Reg d, Reg n, Reg m, bool w64 = true) { rrr(0x0A000000u, d, n, m, w64); }
    void orr(Reg d, Reg n, Reg m, bool w64 = true) { rrr(0x2A000000u, d, n, m, w64); }
    void eor(Reg d, Reg n, Reg m, bool w64 = true) { rrr(0x4A000000u, d, n, m, w64); }
    void mvn(Reg d, Reg m, bool w64 = true) { rrr(0x2A200000u, d, xzr, m, w64); }  // ORN d, zr, m
    void tst(Reg n, Reg m, bool w64 = true) { rrr(0x6A000000u, xzr, n, m, w64); }
    void and_imm(Reg d, Reg n, u64 v, bool w64 = true) { logical_imm(0x12000000u, 0x0A000000u, d, n, v, w64); }
    void orr_imm(Reg d, Reg n, u64 v, bool w64 = true) { logical_imm(0x32000000u, 0x2A000000u, d, n, v, w64); }
    void eor_imm(Reg d, Reg n, u64 v, bool w64 = true) { logical_imm(0x52000000u, 0x4A000000u, d, n, v, w64); }
    void tst_imm(Reg n, u64 v, bool w64 = true) { logical_imm(0x72000000u, 0x6A000000u, xzr, n, v, w64); }
    void lslv(Reg d, Reg n, Reg m, bool w64 = true) { rrr(0x1AC02000u, d, n, m, w64); }
    void lsrv(Reg d, Reg n, Reg m, bool w64 = true) { rrr(0x1AC02400u, d, n, m, w64); }
    void asrv(Reg d, Reg n, Reg m, bool w64 = true) { rrr(0x1AC02800u, d, n, m, w64); }
    void lsl(Reg d, Reg n, unsigned s, bool w64 = true) {
        const unsigned bits = w64 ? 64 : 32;
        bfm(0x53000000u, d, n, (bits - s) % bits, bits - 1 - s, w64);
    }
    void lsr(Reg d, Reg n, unsigned s, bool w64 = true) { bfm(0x53000000u, d, n, s, w64 ? 63 : 31, w64); }
    void asr(Reg d, Reg n, unsigned s, bool w64 = true) { bfm(0x13000000u, d, n, s, w64 ? 63 : 31, w64); }
    void ubfiz(Reg d, Reg n, unsigned lsb, unsigned width) { bfm(0x53000000u, d, n, (64 - lsb) % 64, width - 1, true); }
    void sxtw(Reg d, Reg n) { bfm(0x13000000u, d, n, 0, 31, true); }
    void sxth(Reg d, Reg n) { bfm(0x13000000u, d, n, 0, 15, true); }
    void sxtb(Reg d, Reg n) { bfm(0x13000000u, d, n, 0, 7, true); }
    void uxth(Reg d, Reg n) { bfm(0x53000000u, d, n, 0, 15, false); }
    void madd(Reg d, Reg n, Reg m, Reg a, bool w64 = true) {
        emit((w64 ? 0x9B000000u : 0x1B000000u) | (u32{m} << 16) | (u32{a} << 10) | (u32{n} << 5) | d);
    }
    void mul(Reg d, Reg n, Reg m, bool w64 = true) { madd(d, n, m, xzr, w64); }
    void smull(Reg d, Reg n, Reg m) { emit(0x9B200000u | (u32{m} << 16) | (31u << 10) | (u32{n} << 5) | d); }
    void umull(Reg d, Reg n, Reg m) { emit(0x9BA00000u | (u32{m} << 16) | (31u << 10) | (u32{n} << 5) | d); }
    void csel(Reg d, Reg n, Reg m, Cond c) {  // d = c ? n : m (64-bit)
        emit(0x9A800000u | (u32{m} << 16) | (static_cast<u32>(c) << 12) | (u32{n} << 5) | d);
    }
    void cset(Reg d, Cond c) {  // CSINC d, zr, zr, !c
        emit(0x9A800400u | (31u << 16) | (static_cast<u32>(invert(c)) << 12) | (31u << 5) | d);
    }
    void rev32(Reg d, Reg n) { emit(0x5AC00800u | (u32{n} << 5) | d); }  // REV Wd
    void rev64(Reg d, Reg n) { emit(0xDAC00C00u | (u32{n} << 5) | d); }  // REV Xd
    // Flag-setting add and subtract (V: signed overflow), any immediate.
    void adds(Reg d, Reg n, Reg m, bool w64 = true) { rrr(0x2B000000u, d, n, m, w64); }
    void subs(Reg d, Reg n, Reg m, bool w64 = true) { rrr(0x6B000000u, d, n, m, w64); }
    void adds_imm(Reg d, Reg n, s64 v, bool w64 = true) {
        if (v < 0) return add_sub_imm(true, true, d, n, static_cast<u64>(-v), w64);
        add_sub_imm(false, true, d, n, static_cast<u64>(v), w64);
    }
    void sdiv(Reg d, Reg n, Reg m, bool w64 = true) { rrr(0x1AC00C00u, d, n, m, w64); }
    void udiv(Reg d, Reg n, Reg m, bool w64 = true) { rrr(0x1AC00800u, d, n, m, w64); }
    void msub(Reg d, Reg n, Reg m, Reg a, bool w64 = true) {  // d = a - n * m
        emit((w64 ? 0x9B008000u : 0x1B008000u) | (u32{m} << 16) | (u32{a} << 10) | (u32{n} << 5) | d);
    }
    void smulh(Reg d, Reg n, Reg m) { emit(0x9B407C00u | (u32{m} << 16) | (u32{n} << 5) | d); }
    void umulh(Reg d, Reg n, Reg m) { emit(0x9BC07C00u | (u32{m} << 16) | (u32{n} << 5) | d); }

    // --- scalar floating point on v registers (dbl: D, else S) -------------
    enum class Fop : u32 { Mul = 0x0800, Div = 0x1800, Add = 0x2800, Sub = 0x3800 };
    void fop(Fop op, bool dbl, unsigned d, unsigned n, unsigned m) {
        emit(0x1E200000u | (dbl ? 0x400000u : 0) | (m << 16) | static_cast<u32>(op) | (n << 5) | d);
    }
    void fsqrt(bool dbl, unsigned d, unsigned n) { emit(0x1E21C000u | (dbl ? 0x400000u : 0) | (n << 5) | d); }
    void fmov(bool dbl, unsigned d, unsigned n) { emit(0x1E204000u | (dbl ? 0x400000u : 0) | (n << 5) | d); }
    // d = a - n * m (FMSUB), d = n * m - a (FNMSUB)
    void fmsub(bool dbl, unsigned d, unsigned n, unsigned m, unsigned a) {
        emit(0x1F008000u | (dbl ? 0x400000u : 0) | (m << 16) | (a << 10) | (n << 5) | d);
    }
    void fnmsub(bool dbl, unsigned d, unsigned n, unsigned m, unsigned a) {
        emit(0x1F208000u | (dbl ? 0x400000u : 0) | (m << 16) | (a << 10) | (n << 5) | d);
    }
    void fcmp(bool dbl, unsigned n, unsigned m) { emit(0x1E202000u | (dbl ? 0x400000u : 0) | (m << 16) | (n << 5)); }
    void fcvt_to_single(unsigned d, unsigned n) { emit(0x1E624000u | (n << 5) | d); }  // FCVT Sd, Dn
    void fcvt_to_double(unsigned d, unsigned n) { emit(0x1E22C000u | (n << 5) | d); }  // FCVT Dd, Sn
    // To a 32- or 64-bit integer, rounding as MIPS RM `rm` (0 nearest even:
    // FCVTNS, 1 toward zero: FCVTZS, 2 up: FCVTPS, 3 down: FCVTMS); saturating.
    void fcvt_to_int(bool dbl, unsigned rm, Reg d, unsigned n, bool x64 = false) {
        static constexpr u32 kRmode[] = {0, 0x180000u, 0x080000u, 0x100000u};
        emit(0x1E200000u | (x64 ? 0x80000000u : 0) | kRmode[rm & 3] | (dbl ? 0x400000u : 0) | (n << 5) | d);
    }
    // From Wn, or Xn.
    void scvtf(bool dbl, unsigned d, Reg n, bool x64 = false) {
        emit(0x1E220000u | (x64 ? 0x80000000u : 0) | (dbl ? 0x400000u : 0) | (u32{n} << 5) | d);
    }
    // MRS and MSR of FPCR and FPSR.
    void mrs_fpcr(Reg t) { emit(0xD53B4400u | t); }
    void msr_fpcr(Reg t) { emit(0xD51B4400u | t); }
    void mrs_fpsr(Reg t) { emit(0xD53B4420u | t); }
    void msr_fpsr(Reg t) { emit(0xD51B4420u | t); }
    void fmov_to_gpr(Reg d, unsigned n, bool dbl) { emit((dbl ? 0x9E660000u : 0x1E260000u) | (n << 5) | d); }
    void fmov_from_gpr(unsigned d, Reg n, bool dbl) { emit((dbl ? 0x9E670000u : 0x1E270000u) | (u32{n} << 5) | d); }
    // LDR/STR of an S (size 4) or D (size 8) register, any offset.
    void fldr(unsigned t, Reg base, s64 off, unsigned size) { fmem(true, t, base, off, size); }
    void fstr(unsigned t, Reg base, s64 off, unsigned size) { fmem(false, t, base, off, size); }
    void rev16(Reg d, Reg n) { emit(0x5AC00400u | (u32{n} << 5) | d); }  // REV16 Wd

    // --- memory ---------------------------------------------------------------
    // Size 1, 2, 4 or 8 bytes; `sign`: sign-extend to 64 bits (loads below 8).
    // Any offset; the base may be sp.
    void ldr(Reg t, Reg base, s64 off, unsigned size = 8, bool sign = false) { mem(true, t, base, off, size, sign); }
    void str(Reg t, Reg base, s64 off, unsigned size = 8) { mem(false, t, base, off, size, false); }
    // [base + index] (byte offset in a register).
    void ldr_index(Reg t, Reg base, Reg index, unsigned size = 8, bool sign = false) {
        emit(ls_reg_op(true, size, sign) | (u32{index} << 16) | (u32{base} << 5) | t);
    }
    void str_index(Reg t, Reg base, Reg index, unsigned size = 8) {
        emit(ls_reg_op(false, size, false) | (u32{index} << 16) | (u32{base} << 5) | t);
    }
    void stp_pre(Reg a, Reg b, Reg base, s32 off) {  // stp a, b, [base, #off]!
        emit(0xA9800000u | ((static_cast<u32>(off / 8) & 0x7F) << 15) | (u32{b} << 10) | (u32{base} << 5) | a);
    }
    void stp(Reg a, Reg b, Reg base, s32 off) {
        emit(0xA9000000u | ((static_cast<u32>(off / 8) & 0x7F) << 15) | (u32{b} << 10) | (u32{base} << 5) | a);
    }
    void ldp(Reg a, Reg b, Reg base, s32 off) {
        emit(0xA9400000u | ((static_cast<u32>(off / 8) & 0x7F) << 15) | (u32{b} << 10) | (u32{base} << 5) | a);
    }
    void ldp_post(Reg a, Reg b, Reg base, s32 off) {  // ldp a, b, [base], #off
        emit(0xA8C00000u | ((static_cast<u32>(off / 8) & 0x7F) << 15) | (u32{b} << 10) | (u32{base} << 5) | a);
    }

    void emit(u32 w) {
        const std::size_t at = code_.size();
        code_.resize(at + 4);
        std::memcpy(&code_[at], &w, 4);  // AArch64 instructions are little-endian, as the host
    }

private:
    Site emit_site(u32 w, Fix kind) {
        Site s{code_.size(), kind};
        emit(w);
        return s;
    }
    void rrr(u32 op, Reg d, Reg n, Reg m, bool w64, Shift sh = Shift::Lsl, unsigned amount = 0) {
        emit(op | (w64 ? 0x80000000u : 0) | (static_cast<u32>(sh) << 22) | (u32{m} << 16) | (amount << 10) |
             (u32{n} << 5) | d);
    }
    void bfm(u32 op, Reg d, Reg n, unsigned immr, unsigned imms, bool w64) {
        emit(op | (w64 ? 0x80400000u : 0) | (immr << 16) | (imms << 10) | (u32{n} << 5) | d);
    }
    void add_sub_imm(bool sub, bool flags, Reg d, Reg n, u64 v, bool w64) {
        const u32 op = (sub ? 0x51000000u : 0x11000000u) | (flags ? 0x20000000u : 0) | (w64 ? 0x80000000u : 0);
        if (v < 4096) return emit(op | (static_cast<u32>(v) << 10) | (u32{n} << 5) | d);
        if ((v & 0xFFF) == 0 && v < (u64{1} << 24))
            return emit(op | (1u << 22) | (static_cast<u32>(v >> 12) << 10) | (u32{n} << 5) | d);
        if (v < (u64{1} << 24) && !flags) {  // two steps
            emit(op | (1u << 22) | (static_cast<u32>(v >> 12) << 10) | (u32{n} << 5) | d);
            return emit(op | (static_cast<u32>(v & 0xFFF) << 10) | (u32{d} << 5) | d);
        }
        mov_imm(kScratch0, v);
        // Register form (n must not be sp here).
        rrr((sub ? 0x4B000000u : 0x0B000000u) | (flags ? 0x20000000u : 0), d, n, kScratch0, w64);
    }
    void logical_imm(u32 imm_op, u32 reg_op, Reg d, Reg n, u64 v, bool w64) {
        u32 fields;
        if (encode_logical(v, w64 ? 64 : 32, fields))
            return emit(imm_op | (w64 ? 0x80000000u : 0) | fields | (u32{n} << 5) | d);
        mov_imm(kScratch0, v);
        rrr(reg_op, d, n, kScratch0, w64);
    }
    static u32 ls_imm_op(bool load, unsigned size, bool sign) {
        switch (size) {
            case 1: return load ? (sign ? 0x39800000u : 0x39400000u) : 0x39000000u;
            case 2: return load ? (sign ? 0x79800000u : 0x79400000u) : 0x79000000u;
            case 4: return load ? (sign ? 0xB9800000u : 0xB9400000u) : 0xB9000000u;
            default: return load ? 0xF9400000u : 0xF9000000u;
        }
    }
    static u32 ls_unscaled_op(bool load, unsigned size, bool sign) {  // LDUR/STUR family
        return ls_imm_op(load, size, sign) & ~0x01000000u;
    }
    static u32 ls_reg_op(bool load, unsigned size, bool sign) {  // [base, index] (LSL #0)
        return (ls_imm_op(load, size, sign) & ~0x01000000u) | 0x00206800u;
    }
    void mem(bool load, Reg t, Reg base, s64 off, unsigned size, bool sign) {
        if (off >= 0 && off % size == 0 && off / size < 4096)
            return emit(ls_imm_op(load, size, sign) | (static_cast<u32>(off / size) << 10) | (u32{base} << 5) | t);
        if (off >= -256 && off < 256)
            return emit(ls_unscaled_op(load, size, sign) | ((static_cast<u32>(off) & 0x1FF) << 12) | (u32{base} << 5) | t);
        mov_imm(kScratch1, static_cast<u64>(off));
        emit(ls_reg_op(load, size, sign) | (u32{kScratch1} << 16) | (u32{base} << 5) | t);
    }

    void fmem(bool load, unsigned t, Reg base, s64 off, unsigned size) {
        const u32 op = (size == 8 ? 0xFD000000u : 0xBD000000u) | (load ? 0x00400000u : 0);
        if (off >= 0 && off % size == 0 && off / size < 4096)
            return emit(op | (static_cast<u32>(off / size) << 10) | (u32{base} << 5) | t);
        mov_imm(kScratch1, static_cast<u64>(off));
        emit(((op & ~0x01000000u) | 0x00206800u) | (u32{kScratch1} << 16) | (u32{base} << 5) | t);
    }

    struct Literal {
        Site site;
        u64 value;
    };
    std::vector<u8> code_;
    std::vector<Literal> literals_;
};

}  // namespace sg::jit::a64
