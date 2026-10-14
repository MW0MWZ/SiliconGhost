// SPDX-License-Identifier: GPL-2.0-or-later
// Minimal x86-64 machine-code emitter: the instructions the dynamic
// recompiler needs, with memory operands of the form [base + disp32].
#pragma once

#include <cstddef>
#include <vector>

#include "core/types.h"

namespace sg::jit::x64 {

enum Reg : unsigned { rax, rcx, rdx, rbx, rsp, rbp, rsi, rdi, r8, r9, r10, r11, r12, r13, r14, r15 };

struct Mem {
    Reg base;
    s32 disp;
};

enum class Alu : unsigned { Add = 0, Or = 1, And = 4, Sub = 5, Xor = 6, Cmp = 7 };
enum class Shift : unsigned { Shl = 4, Shr = 5, Sar = 7 };
enum class Cond : unsigned { B = 0x2, Ae = 0x3, E = 0x4, Ne = 0x5, Be = 0x6, A = 0x7, L = 0xC, Ge = 0xD, Le = 0xE, G = 0xF };

class Emitter {
public:
    const std::vector<u8>& code() const { return buf_; }
    std::size_t size() const { return buf_.size(); }

    void push(Reg r) { rex_opt(false, 0, r), byte(0x50 + (r & 7)); }
    void pop(Reg r) { rex_opt(false, 0, r), byte(0x58 + (r & 7)); }
    void ret() { byte(0xC3); }
    void call(Reg r) { rex_opt(false, 0, r), byte(0xFF), byte(0xC0 | (2 << 3) | (r & 7)); }
    void jmp(Reg r) { rex_opt(false, 0, r), byte(0xFF), byte(0xC0 | (4 << 3) | (r & 7)); }

    // Loads and stores (w: 64-bit operand size)
    void mov(Reg dst, Mem m, bool w = true) { op_mem(w, 0x8B, dst, m); }
    void mov(Mem m, Reg src, bool w = true) { op_mem(w, 0x89, src, m); }
    void mov(Reg dst, Reg src) { op_rr(true, 0x89, src, dst); }
    void mov_imm64(Reg dst, u64 imm) {
        rex(true, 0, dst);
        byte(0xB8 + (dst & 7));
        u64v(imm);
    }
    void mov_imm32(Reg dst, u32 imm) {  // zero-extends
        rex_opt(false, 0, dst);
        byte(0xB8 + (dst & 7));
        u32v(imm);
    }
    // mov qword [m], imm32 (sign-extended)
    void mov_store_imm(Mem m, s32 imm) {
        op_mem(true, 0xC7, Reg(0), m);
        u32v(static_cast<u32>(imm));
    }
    void cmp8_imm(Mem m, u8 imm) {  // cmp byte [m], imm8
        op_mem(false, 0x80, Reg(7), m);
        byte(imm);
    }
    void mov_store_imm8(Mem m, u8 imm) {
        op_mem(false, 0xC6, Reg(0), m);
        byte(imm);
    }
    void movsxd(Reg dst, Reg src) { op_rr(true, 0x63, dst, src); }
    void movsxd(Reg dst, Mem m) { op_mem(true, 0x63, dst, m); }

    // ALU
    void alu(Alu op, Reg dst, Mem m, bool w = true) { op_mem(w, 0x03 + (static_cast<unsigned>(op) << 3), dst, m); }
    void alu(Alu op, Reg dst, Reg src, bool w = true) { op_rr(w, 0x01 + (static_cast<unsigned>(op) << 3), src, dst); }
    void alu_imm(Alu op, Reg dst, s32 imm, bool w = true) {
        rex_opt(w, 0, dst);
        byte(0x81);
        byte(0xC0 | (static_cast<unsigned>(op) << 3) | (dst & 7));
        u32v(static_cast<u32>(imm));
    }
    void alu8(Alu op, Reg dst, Mem m) { op_mem(false, 0x02 + (static_cast<unsigned>(op) << 3), dst, m); }  // r8, [m]
    void shift_imm(Shift op, Reg r, u8 n, bool w) {
        rex_opt(w, 0, r);
        byte(0xC1);
        byte(0xC0 | (static_cast<unsigned>(op) << 3) | (r & 7));
        byte(n);
    }
    void shift_cl(Shift op, Reg r, bool w) {
        rex_opt(w, 0, r);
        byte(0xD3);
        byte(0xC0 | (static_cast<unsigned>(op) << 3) | (r & 7));
    }
    void imul(Reg dst, Reg src) {  // 64-bit, low half
        rex(true, dst, src);
        byte(0x0F), byte(0xAF), byte(0xC0 | ((dst & 7) << 3) | (src & 7));
    }
    void imul_imm(Reg dst, Reg src, s32 imm) {
        rex(true, dst, src);
        byte(0x69), byte(0xC0 | ((dst & 7) << 3) | (src & 7));
        u32v(static_cast<u32>(imm));
    }
    void not_(Reg r) {
        rex(true, 0, r);
        byte(0xF7), byte(0xC0 | (2 << 3) | (r & 7));
    }
    void setcc(Cond c, Reg r) {  // r must be al/cl/dl/bl
        byte(0x0F), byte(0x90 + static_cast<unsigned>(c)), byte(0xC0 | (r & 7));
    }
    void movzx8(Reg dst, Reg src) {  // dst32 <- src8 (al/cl/dl/bl)
        byte(0x0F), byte(0xB6), byte(0xC0 | ((dst & 7) << 3) | (src & 7));
    }
    void movzx8(Reg dst, Mem m) {  // dst32 <- byte [m]
        rex_opt(false, dst, m.base);
        byte(0x0F);
        modrm_mem(0xB6, dst, m);
    }
    void movsx8(Reg dst, Mem m) {  // dst64 <- sign-extended byte [m]
        rex(true, dst, m.base);
        byte(0x0F);
        modrm_mem(0xBE, dst, m);
    }
    void movzx16(Reg dst, Mem m) {  // dst32 <- word [m]
        rex_opt(false, dst, m.base);
        byte(0x0F);
        modrm_mem(0xB7, dst, m);
    }
    void movzx16(Reg dst, Reg src) {  // dst32 <- src16
        rex_opt(false, dst, src);
        byte(0x0F), byte(0xB7), byte(0xC0 | ((dst & 7) << 3) | (src & 7));
    }
    void movsx16(Reg dst, Reg src) {  // dst64 <- sign-extended src16
        rex(true, dst, src);
        byte(0x0F), byte(0xBF), byte(0xC0 | ((dst & 7) << 3) | (src & 7));
    }
    void bswap32(Reg r) {
        rex_opt(false, 0, r);
        byte(0x0F), byte(0xC8 + (r & 7));
    }
    void rol16_8(Reg r) {  // swaps the bytes of r16
        byte(0x66);
        rex_opt(false, 0, r);
        byte(0xC1), byte(0xC0 | (r & 7)), byte(8);
    }
    void test_imm(Reg r, u32 imm) {  // 32-bit
        rex_opt(false, 0, r);
        byte(0xF7), byte(0xC0 | (r & 7));
        u32v(imm);
    }
    void test(Reg a, Reg b) { op_rr(true, 0x85, b, a); }
    // Narrow stores of the low bits of src (src must be rax/rcx/rdx/rbx or r8-r15)
    void mov8(Mem m, Reg src) { op_mem(false, 0x88, src, m); }
    void mov16(Mem m, Reg src) {
        byte(0x66);
        op_mem(false, 0x89, src, m);
    }
    void add_rsp(s32 imm) { alu_imm(Alu::Add, rsp, imm); }
    void sub_rsp(s32 imm) { alu_imm(Alu::Sub, rsp, imm); }

    // Branches: jcc/jmp with a rel32 to be patched; returns the patch site.
    std::size_t jcc(Cond c) {
        byte(0x0F), byte(0x80 + static_cast<unsigned>(c));
        u32v(0);
        return buf_.size();
    }
    std::size_t jmp() {
        byte(0xE9);
        u32v(0);
        return buf_.size();
    }
    // Jumps back to an earlier position.
    void jmp_to(std::size_t target) {
        byte(0xE9);
        u32v(static_cast<u32>(static_cast<s64>(target) - static_cast<s64>(buf_.size() + 4)));
    }
    // Points the jump ending at `site` to the current position.
    void bind(std::size_t site) {
        const u32 rel = static_cast<u32>(buf_.size() - site);
        for (int i = 0; i < 4; ++i) buf_[site - 4 + i] = static_cast<u8>(rel >> (8 * i));
    }

private:
    void byte(unsigned b) { buf_.push_back(static_cast<u8>(b)); }
    void u32v(u32 v) {
        for (int i = 0; i < 4; ++i) byte((v >> (8 * i)) & 0xFF);
    }
    void u64v(u64 v) {
        for (int i = 0; i < 8; ++i) byte((v >> (8 * i)) & 0xFF);
    }
    void rex(bool w, unsigned reg, unsigned rm) { byte(0x40 | (w << 3) | ((reg >> 3) << 2) | (rm >> 3)); }
    void rex_opt(bool w, unsigned reg, unsigned rm) {
        if (w || reg >= 8 || rm >= 8) rex(w, reg, rm);
    }
    // op reg, [base + disp32]
    void op_mem(bool w, unsigned opcode, Reg reg, Mem m) {
        rex_opt(w, reg, m.base);
        modrm_mem(opcode, reg, m);
    }
    void modrm_mem(unsigned opcode, Reg reg, Mem m) {
        byte(opcode);
        byte(0x80 | ((reg & 7) << 3) | (m.base & 7));
        if ((m.base & 7) == rsp) byte(0x24);  // SIB: base only
        u32v(static_cast<u32>(m.disp));
    }
    // op rm, reg (register direct)
    void op_rr(bool w, unsigned opcode, Reg reg, Reg rm) {
        rex_opt(w, reg, rm);
        byte(opcode);
        byte(0xC0 | ((reg & 7) << 3) | (rm & 7));
    }

    std::vector<u8> buf_;
};

}  // namespace sg::jit::x64
