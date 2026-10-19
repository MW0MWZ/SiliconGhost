// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// R4000 dynamic recompiler, AArch64 back end.
//
// The same translation as the x86-64 back end (see jit_x64.cpp): a hot
// decoded block becomes one host function that runs it from its first
// instruction, with register operations, RAM loads and stores and branches
// inline, handler calls for the rest, the guest registers it uses most held
// in host registers, and direct entry into the next block's translation.
#include "cpu/r4000/r4000.h"

#if defined(__aarch64__)

#include <algorithm>
#include <cstddef>
#include <functional>
#include <vector>

#include "jit/a64_emitter.h"
#include "jit/exec_buffer.h"

namespace sg::mips {

namespace {

using namespace sg::jit::a64;
using Site = Emitter::Site;

// Callee-saved registers live across the whole function.
constexpr Reg kCpu = x19;        // R4000*
constexpr Reg kTime = x20;       // the instructions that may still start (see jit_x64.cpp)
constexpr Reg kPc = x21;         // pc at entry (blocks are shared by aliases)
constexpr Reg kTlb = x22;        // this mode's soft-TLB load table (fetch before it, store after it)
constexpr Reg kEntryMode = x23;  // mode_ at entry
// Host registers that hold guest registers (caller-saved: reloaded after
// calls, as on x86-64). x0-x4 are scratch; x16 and x17 belong to the emitter.
constexpr Reg kCacheRegs[] = {x8, x9, x10, x11, x12, x13, x14, x15};
// v registers that hold FPU registers (caller-saved, as above); v0-v4 are scratch.
constexpr unsigned kFpCacheRegs[] = {16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};

inline unsigned f_rs(u32 i) { return (i >> 21) & 31; }
inline unsigned f_rt(u32 i) { return (i >> 16) & 31; }
inline unsigned f_rd(u32 i) { return (i >> 11) & 31; }
inline unsigned f_sa(u32 i) { return (i >> 6) & 31; }

}  // namespace

template <typename T>
static s64 offset_of(const R4000* cpu, const T* member) {
    return reinterpret_cast<const u8*>(member) - reinterpret_cast<const u8*>(cpu);
}

bool R4000::jit_compile_supported() { return true; }

bool R4000::jit_fma_ = true;  // AArch64 always has fused multiply-add
void R4000::set_jit_fma(bool) {}

bool R4000::jit_compile(Block& b) {
    Emitter e;
    auto gpr_off = [&](unsigned r) { return offset_of(this, &gpr[r]); };
    const s64 o_time = offset_of(this, &time_fp_), o_pc = offset_of(this, &pc), o_npc = offset_of(this, &npc_);
    const s64 o_cur_pc = offset_of(this, &cur_pc_), o_in_delay = offset_of(this, &in_delay_);
    const s64 o_next_delay = offset_of(this, &next_delay_), o_io = offset_of(this, &io_);
    const s64 o_invalidated = offset_of(this, &block_invalidated_), o_idle_ready = offset_of(this, &idle_ready_);
    const s64 o_hi = offset_of(this, &hi), o_lo = offset_of(this, &lo), o_mode = offset_of(this, &mode_);
    const s64 o_time0 = offset_of(this, &jit_time0_), o_budget0 = offset_of(this, &jit_budget0_),
              o_weight = offset_of(this, &weight_);
    const s64 o_asid_tag = offset_of(this, &asid_tag_);
    static_assert(sizeof(FastEntry) == 32);
    constexpr s64 kTableBytes = kFastEntries * sizeof(FastEntry);

    // Prologue: frame record, then the callee-saved registers used.
    e.stp_pre(x29, x30, sp, -64);
    e.mov_from_sp(x29);
    e.stp(x19, x20, sp, 16);
    e.stp(x21, x22, sp, 32);
    e.stp(x23, x24, sp, 48);
    e.mov(kCpu, x0);
    e.mov(kPc, x1);
    e.mov(kTime, x2);
    e.ldr(kEntryMode, kCpu, o_mode, 4);
    e.mov_imm(kScratch0, static_cast<u64>(3 * kTableBytes));
    e.madd(kTlb, kEntryMode, kScratch0, kCpu);
    e.add_imm(kTlb, kTlb, offset_of(this, &fast_[0][static_cast<unsigned>(Access::Load)][0]));
    const std::size_t top = e.size();
    // Linked blocks enter here, so the prologue must be the same everywhere.
    if (jit_link_offset_ == 0) jit_link_offset_ = top;
    if (top != jit_link_offset_) return false;
    // The block-end code the translations share, once per buffer of code
    // (see jit_x64.cpp). The enter stub runs the translation of the block at
    // x1 for pc x0 if it fits in the budget x2; the lookup stub finds the
    // block first. Either goes back to x6 when it can't, with x0, x2 and x6
    // as they were.
    if (jit_stub_generation_ != jit_generation_) {
        Emitter s;
        std::vector<Emitter::Site> miss;
        std::size_t enter_at[2], lookup_at[2];
        for (unsigned a = 0; a < 2; ++a) {
            enter_at[a] = s.size();
            s.ldr(x4, x1, offsetof(Block, code));
            miss.push_back(s.cbz(x4));
            s.ldr(x5, x1, offsetof(Block, jit_addr64), 1);
            miss.push_back(a ? s.cbz(x5, false) : s.cbnz(x5, false));
            s.ldr(x5, x1, offsetof(Block, count), 4);
            s.cmp(x2, x5);
            miss.push_back(s.b(Cond::Lt));
            s.mov(kTime, x2);
            s.mov(kPc, x0);
            s.add_imm(x4, x4, static_cast<s64>(top));
            s.br(x4);
        }
        for (unsigned a = 0; a < 2; ++a) {
            lookup_at[a] = s.size();
            s.tst_imm(x0, 3);
            miss.push_back(s.b(Cond::Ne));
            s.lsr(x1, x0, 12);
            s.ubfiz(x3, x1, 5, 10);
            s.ldr(kScratch0, kCpu, o_asid_tag);
            s.orr(x1, x1, kScratch0);
            s.add(x3, x3, kTlb);
            s.sub_imm(x3, x3, kTableBytes);  // the fetch table precedes the load table
            s.ldr(kScratch0, x3, offsetof(FastEntry, tag));
            s.cmp(x1, kScratch0);
            miss.push_back(s.b(Cond::Ne));
            s.ldr(x3, x3, offsetof(FastEntry, host));
            miss.push_back(s.cbz(x3));
            s.and_imm(x1, x0, 0xFFF);
            s.add(x3, x3, x1);  // host address of the instruction
            s.lsr(x1, x3, 2);
            s.and_imm(x1, x1, (1u << kBlockBits) - 1);
            s.mov_imm(kScratch0, sizeof(Block));
            s.load_constant(x4, reinterpret_cast<u64>(blocks_.data()));
            s.madd(x1, x1, kScratch0, x4);
            s.ldr(kScratch0, x1, offsetof(Block, host));
            s.cmp(x3, kScratch0);
            miss.push_back(s.b(Cond::Ne));
            s.b_to(enter_at[a]);
        }
        for (const Emitter::Site& site : miss) s.bind(site);
        s.br(x6);
        s.finish_literals();
        const u8* at = static_cast<const u8*>(jit_buf_->add(s.code().data(), s.size()));
        if (!at) return false;
        for (unsigned a = 0; a < 2; ++a) jit_enter_stub_[a] = at + enter_at[a], jit_lookup_stub_[a] = at + lookup_at[a];
        jit_stub_generation_ = jit_generation_;
    }
    const u8* const base = jit_buf_->next();  // where this translation will go
    const bool addr64 = addr64_;
    b.jit_addr64 = addr64;

    // Guest register cache (as on x86-64).
    // The region (see jit_x64.cpp).
    std::vector<Block> members;
    std::vector<const Block*> rb;
    jit_region(b, members, rb);
    const JitPlan plan = jit_plan(rb);
    Reg host_of[32] = {};
    for (unsigned g = 1; g < 32; ++g)
        if (plan.slot[g] >= 0) host_of[g] = kCacheRegs[plan.slot[g]];
    // FPU register cache (as on x86-64).
    const bool fr = (status_ & sr::FR) != 0;
    const s64 fpr_base = offset_of(this, &fpr[0]);
    auto fp_storage = [&](unsigned n, bool dbl) -> s64 {
        if (fr) return 8 * n;
        return 8 * (n & ~1u) + (dbl ? 0 : 4 * (n & 1));  // little-endian host
    };
    struct FpKey {
        s64 off;
        unsigned width;
        unsigned uses;
        bool written;
    };
    std::vector<FpKey> fp_keys;
    {
        std::vector<FpKey> all;
        for (const Block* rbk : rb)
            for (u32 i = 0; i < rbk->count; ++i) {
                if (i > 0 && is_branch_word(rbk->ops[i - 1].inst) && is_branch_word(rbk->ops[i].inst)) continue;
                jit_fp_uses(rbk->ops[i].inst, [&](unsigned n, bool dbl, bool write_access) {
                    all.push_back({fp_storage(n, dbl), dbl ? 8u : 4u, 1, write_access});
                });
            }
        for (const FpKey& a : all) {
            bool clash = false, seen = false;
            for (const FpKey& o : all)
                if (o.off < a.off + static_cast<s64>(a.width) && a.off < o.off + static_cast<s64>(o.width) &&
                    (o.off != a.off || o.width != a.width))
                    clash = true;
            if (clash) continue;
            for (FpKey& k : fp_keys)
                if (k.off == a.off) {
                    ++k.uses;
                    k.written |= a.written;
                    seen = true;
                }
            if (!seen) fp_keys.push_back(a);
        }
        std::stable_sort(fp_keys.begin(), fp_keys.end(), [](const FpKey& x, const FpKey& y) { return x.uses > y.uses; });
        while (!fp_keys.empty() && fp_keys.back().uses < 2) fp_keys.pop_back();
        if (fp_keys.size() > std::size(kFpCacheRegs)) fp_keys.resize(std::size(kFpCacheRegs));
    }
    auto fp_bit = [](std::size_t k) { return u64{1} << (32 + k); };
    u64 alloc = plan.alloc, written = plan.written;
    for (std::size_t k = 0; k < fp_keys.size(); ++k) {
        alloc |= fp_bit(k);
        if (fp_keys[k].written) written |= fp_bit(k);
    }
    bool region_mode = rb.size() > 1;  // registers held across the region's blocks
    {
        std::vector<u32> next;
        jit_successors(b, next);
        for (u32 off : next) region_mode |= off == b.offset;
    }
    u64 loaded = 0, dirty = 0;
    auto cached = [&](unsigned g) { return ((alloc >> g) & 1) != 0; };
    auto ensure = [&](unsigned g) {
        if (cached(g) && !((loaded >> g) & 1)) {
            e.ldr(host_of[g], kCpu, gpr_off(g));
            loaded |= u64{1} << g;
        }
    };
    // The register holding guest register g: its cache register, or
    // `scratch` loaded from memory. (r0 reads as zero.)
    auto src = [&](unsigned g, Reg scratch) -> Reg {
        if (g == 0) return xzr;
        if (cached(g)) {
            ensure(g);
            return host_of[g];
        }
        e.ldr(scratch, kCpu, gpr_off(g));
        return scratch;
    };
    // As src(), for the operand of an immediate-form instruction, where
    // register 31 means sp rather than zero.
    auto src_imm = [&](unsigned g, Reg scratch) -> Reg {
        if (g != 0) return src(g, scratch);
        e.mov(scratch, xzr);
        return scratch;
    };
    auto read = [&](Reg dst, unsigned g) {
        const Reg r = src(g, dst);
        if (r != dst) e.mov(dst, r);
    };
    auto write = [&](unsigned g, Reg value) {
        if (!cached(g)) return e.str(value, kCpu, gpr_off(g));
        if (host_of[g] != value) e.mov(host_of[g], value);
        loaded |= u64{1} << g;
        dirty |= u64{1} << g;
    };
    auto write_imm = [&](unsigned g, s64 v) {
        if (!cached(g)) {
            e.mov_imm(x0, static_cast<u64>(v));
            return e.str(x0, kCpu, gpr_off(g));
        }
        e.mov_imm(host_of[g], static_cast<u64>(v));
        loaded |= u64{1} << g;
        dirty |= u64{1} << g;
    };
    auto fp_store_key = [&](std::size_t k) { e.fstr(kFpCacheRegs[k], kCpu, fpr_base + fp_keys[k].off, fp_keys[k].width); };
    auto fp_load_key = [&](std::size_t k) { e.fldr(kFpCacheRegs[k], kCpu, fpr_base + fp_keys[k].off, fp_keys[k].width); };
    auto flush = [&](u64 mask) {
        for (unsigned g = 1; g < 32; ++g)
            if ((mask >> g) & 1) e.str(host_of[g], kCpu, gpr_off(g));
        for (std::size_t k = 0; k < fp_keys.size(); ++k)
            if (mask & fp_bit(k)) fp_store_key(k);
    };
    auto restore = [&](u64 mask) {
        for (unsigned g = 1; g < 32; ++g)
            if ((mask >> g) & 1) e.ldr(host_of[g], kCpu, gpr_off(g));
        for (std::size_t k = 0; k < fp_keys.size(); ++k)
            if (mask & fp_bit(k)) fp_load_key(k);
    };
    // FPU register n as a single or a double: its cache slot, or -1 (see jit_x64.cpp).
    auto fp_slot = [&](unsigned n, bool dbl) -> int {
        const s64 off = fp_storage(n, dbl);
        const unsigned width = dbl ? 8 : 4;
        for (std::size_t k = 0; k < fp_keys.size(); ++k) {
            const FpKey& key = fp_keys[k];
            if (key.off == off && key.width == width) {
                if (!(loaded & fp_bit(k))) {
                    fp_load_key(k);
                    loaded |= fp_bit(k);
                }
                return static_cast<int>(k);
            }
            if (key.off < off + static_cast<s64>(width) && off < key.off + static_cast<s64>(key.width)) {
                if (dirty & fp_bit(k)) fp_store_key(k);
                loaded &= ~fp_bit(k);
                dirty &= ~fp_bit(k);
            }
        }
        return -1;
    };
    auto fp_read_gpr = [&](Reg r, unsigned n, bool dbl) {  // the bits, zero-extended
        const int k = fp_slot(n, dbl);
        if (k >= 0) e.fmov_to_gpr(r, kFpCacheRegs[k], dbl);
        else e.ldr(r, kCpu, fpr_base + fp_storage(n, dbl), dbl ? 8 : 4);
    };
    auto fp_write_gpr = [&](unsigned n, bool dbl, Reg r) {
        const int k = fp_slot(n, dbl);
        if (k < 0) return e.str(r, kCpu, fpr_base + fp_storage(n, dbl), dbl ? 8 : 4);
        e.fmov_from_gpr(kFpCacheRegs[k], r, dbl);
        loaded |= fp_bit(k);
        dirty |= fp_bit(k);
    };
    auto fp_src = [&](unsigned n, bool dbl, unsigned scratch) -> unsigned {
        const int k = fp_slot(n, dbl);
        if (k >= 0) return kFpCacheRegs[k];
        e.fldr(scratch, kCpu, fpr_base + fp_storage(n, dbl), dbl ? 8 : 4);
        return scratch;
    };
    auto fp_write_v = [&](unsigned n, bool dbl, unsigned v) {
        const int k = fp_slot(n, dbl);
        if (k < 0) return e.fstr(v, kCpu, fpr_base + fp_storage(n, dbl), dbl ? 8 : 4);
        e.fmov(dbl, kFpCacheRegs[k], v);
        loaded |= fp_bit(k);
        dirty |= fp_bit(k);
    };
    if (region_mode) {
        restore(alloc);
        loaded = alloc;
        dirty = written;
    }
    const std::size_t body = e.size();

    auto set_time = [&](unsigned n) {  // time_fp_ = the block's start time + n instructions
        e.ldr(x0, kCpu, o_budget0);
        e.sub(x0, x0, kTime);  // instructions run before the block
        if (n) e.add_imm(x0, x0, n);
        e.ldr(kScratch0, kCpu, o_weight);
        e.ldr(kScratch1, kCpu, o_time0);
        e.madd(x0, x0, kScratch0, kScratch1);
        e.str(x0, kCpu, o_time);
    };
    auto pc_plus = [&](Reg dst, s64 bytes) {
        if (bytes == 0) return e.mov(dst, kPc);
        e.add_imm(dst, kPc, bytes);
    };
    auto store_byte_imm = [&](s64 off, u32 v) {
        if (v == 0) return e.str(xzr, kCpu, off, 1);
        e.mov_imm32(x0, v);
        e.str(x0, kCpu, off, 1);
    };
    auto call = [&](const void* fn) {
        e.load_constant(kScratch0, reinterpret_cast<u64>(fn));
        e.blr(kScratch0);
    };
    auto call_handler = [&](const Op& op) {
        e.mov(x0, kCpu);
        e.load_constant(x1, reinterpret_cast<u64>(jit_op(b, op)));
        call(reinterpret_cast<const void*>(op.fn));
    };

    // Instructions that can raise an exception, inline when they do not (see
    // jit_x64.cpp).
    const s64 o_store_count = offset_of(this, &store_count_);
    const s64 o_change_epoch = offset_of(this, &change_epoch_);
    const s64 log_host = offset_of(this, &store_log_[0].host);
    const s64 log_size = offset_of(this, &store_log_[0].size);
    const s64 log_value = offset_of(this, &store_log_[0].value);
    auto write_nz = [&](unsigned g, Reg value) {
        if (g) write(g, value);
    };
    auto touch_ = [&] {
        e.ldr(x0, kCpu, o_change_epoch);
        e.add_imm(x0, x0, 1);
        e.str(x0, kCpu, o_change_epoch);
    };
    // 64-bit operations are reserved in user and supervisor mode with 32-bit addressing.
    auto need64 = [&](std::vector<Site>& slow) {
        if (addr64) return;
        e.sub_imm(x0, kEntryMode, kSupervisor, false);
        e.cmp_imm(x0, kUser - kSupervisor, false);
        slow.push_back(e.b(Cond::Ls));
    };
    // MIPS IV instructions (R5000 only): reserved in user and supervisor mode
    // without Status.XX.
    auto need_mips4 = [&](std::vector<Site>& slow) {
        e.sub_imm(x0, kEntryMode, kSupervisor, false);
        e.cmp_imm(x0, kUser - kSupervisor, false);
        const Site kernel = e.b(Cond::Hi);
        e.ldr(x0, kCpu, offset_of(this, &status_), 4);
        slow.push_back(e.cbz_bit(x0, 31));  // XX
        e.bind(kernel);
    };
    // An FPU condition code's FCR31 bit.
    auto fcc_bit_number = [](unsigned cc) { return cc ? 24 + cc : 23u; };
    auto emit_alu = [&](u32 w, std::vector<Site>& slow) -> bool {
        const unsigned opc = w >> 26, rs = f_rs(w), rt = f_rt(w), rd = f_rd(w), sa = f_sa(w);
        const s32 simm = static_cast<s16>(w & 0xFFFF);
        if (opc == 0x08 || opc == 0x18 || opc == 0x19) {  // ADDI, DADDI, DADDIU
            const bool w64 = opc != 0x08;
            if (w64) need64(slow);
            const Reg s = src_imm(rs, x1);
            if (opc == 0x19) {
                e.add_imm(x0, s, simm);
            } else {
                e.adds_imm(x0, s, simm, w64);
                slow.push_back(e.b(Cond::Vs));
            }
            if (!w64) e.sxtw(x0, x0);
            write_nz(rt, x0);
            return true;
        }
        if (opc == 0x01 && rt >= 0x08 && rt <= 0x0E && rt != 0x0D) {  // TGEI, TGEIU, TLTI, TLTIU, TEQI, TNEI
            static constexpr Cond kTrap[] = {Cond::Ge, Cond::Hs, Cond::Lt, Cond::Lo, Cond::Eq, Cond::Eq, Cond::Ne};
            e.cmp_imm(src_imm(rs, x1), simm);
            slow.push_back(e.b(kTrap[rt - 0x08]));  // the trap: the handler raises it
            return true;
        }
        if (opc == 0x33 && mips4_) {  // PREF: a hint
            need_mips4(slow);
            return true;
        }
        if (opc != 0x00) return false;
        const unsigned funct = w & 63;
        if ((funct == 0x0A || funct == 0x0B) && mips4_) {  // MOVZ, MOVN
            need_mips4(slow);
            if (!rd) return true;
            const Reg vt = src_imm(rt, x1), vs = src(rs, x2), vd = src(rd, x3);
            e.cmp_imm(vt, 0);
            e.csel(x0, vs, vd, funct == 0x0A ? Cond::Eq : Cond::Ne);
            write(rd, x0);
            return true;
        }
        switch (funct) {
            case 0x30:  // TGE
            case 0x31:  // TGEU
            case 0x32:  // TLT
            case 0x33:  // TLTU
            case 0x34:  // TEQ
            case 0x36: {  // TNE
                static constexpr Cond kTrap[] = {Cond::Ge, Cond::Hs, Cond::Lt, Cond::Lo, Cond::Eq, Cond::Eq, Cond::Ne};
                e.cmp(src(rs, x0), src(rt, x1));
                slow.push_back(e.b(kTrap[funct - 0x30]));  // the trap: the handler raises it
                return true;
            }
            case 0x14:  // DSLLV
            case 0x16:  // DSRLV
            case 0x17: {  // DSRAV (the count mod 64, as MIPS)
                need64(slow);
                const Reg s = src(rs, x1), t = src(rt, x0);
                if (funct == 0x14) e.lslv(x0, t, s);
                else if (funct == 0x16) e.lsrv(x0, t, s);
                else e.asrv(x0, t, s);
                write_nz(rd, x0);
                return true;
            }
            case 0x38: case 0x3A: case 0x3B:  // DSLL, DSRL, DSRA
            case 0x3C: case 0x3E: case 0x3F: {  // DSLL32, DSRL32, DSRA32
                need64(slow);
                const Reg t = src(rt, x0);
                const unsigned n = sa + (funct >= 0x3C ? 32 : 0);
                if ((funct & 3) == 0) e.lsl(x0, t, n);
                else if ((funct & 3) == 2) e.lsr(x0, t, n);
                else e.asr(x0, t, n);
                write_nz(rd, x0);
                return true;
            }
            case 0x20:  // ADD
            case 0x22:  // SUB
            case 0x2C:  // DADD
            case 0x2D:  // DADDU
            case 0x2E:  // DSUB
            case 0x2F: {  // DSUBU
                const bool w64 = funct >= 0x2C, trap = !(funct & 1);
                if (w64) need64(slow);
                const Reg s = src(rs, x0), t = src(rt, x1);
                if (funct & 2) trap ? e.subs(x0, s, t, w64) : e.sub(x0, s, t, w64);
                else trap ? e.adds(x0, s, t, w64) : e.add(x0, s, t, w64);
                if (trap) slow.push_back(e.b(Cond::Vs));  // overflow: the handler raises it
                if (!w64) e.sxtw(x0, x0);
                write_nz(rd, x0);
                return true;
            }
            case 0x1C:  // DMULT
            case 0x1D: {  // DMULTU
                need64(slow);
                const Reg s = src(rs, x0), t = src(rt, x1);
                e.mul(x2, s, t);
                if (funct == 0x1C) e.smulh(x3, s, t);
                else e.umulh(x3, s, t);
                e.str(x2, kCpu, o_lo);
                e.str(x3, kCpu, o_hi);
                return true;
            }
            case 0x1E:  // DDIV (the one overflow gives MIPS's result here too)
            case 0x1F: {  // DDIVU: a zero divisor goes to the handler
                need64(slow);
                const Reg t = src(rt, x1);
                slow.push_back(e.cbz(t));
                const Reg s = src(rs, x0);
                if (funct == 0x1E) e.sdiv(x2, s, t);
                else e.udiv(x2, s, t);
                e.msub(x3, x2, t, s);
                e.str(x2, kCpu, o_lo);
                e.str(x3, kCpu, o_hi);
                return true;
            }
            default: return false;
        }
    };

    // COP1, inline for the common case (see jit_x64.cpp); results are checked
    // arithmetically, with fused multiply-adds for the rounding errors.
    const s64 o_status = offset_of(this, &status_), o_fcr31 = offset_of(this, &fcr31);
    constexpr u32 kCauseMask = 0x0003F000u | kFcrLazyI, kCauseInexact = 0x1000u, kFlagInexact = 0x4u;  // (see jit_x64.cpp)
    auto fp_usable = [&](std::vector<Site>& slow) {
        e.ldr(x0, kCpu, o_status, 4);
        e.and_imm(x0, x0, sr::CU1 | sr::FR, false);
        e.cmp_imm(x0, sr::CU1 | (fr ? sr::FR : 0), false);
        slow.push_back(e.b(Cond::Ne));
    };
    auto fp_trap_off = [&](std::vector<Site>& slow) {  // the inexact trap off (any rounding)
        e.ldr(x0, kCpu, o_fcr31, 4);
        slow.push_back(e.cbnz_bit(x0, 7));
    };
    // Another way through an operation, placed with its way to the handler
    // (see jit_x64.cpp): from `from`, `body`, then on at `join`.
    using AltBody = std::function<void(std::vector<Site>& slow)>;
    struct AltPath {
        std::vector<Site> from;
        AltBody body;
        std::size_t join = 0;
    };
    std::vector<AltPath> alt_paths;  // of the operation being emitted
    // Rounding other than to nearest (as on x86-64, by the host's flags):
    // FPCR.RMode set from a MIPS rounding mode (RM 0-3: nearest, toward
    // zero, up, down; RMode 0, 3, 1, 2), FCR31.RM's or a fixed one, the
    // flags cleared, the operations, then FPCR back to nearest. Any flag but
    // inexact goes to the handler; the flags stay in x3. x0-x3 are clobbered.
    constexpr int kGuestRounding = -1;
    auto fp_host_rounded = [&](int rm, auto&& op, std::vector<Site>& slow) {
        if (rm == kGuestRounding) {
            e.ldr(x0, kCpu, o_fcr31, 4);
            e.and_imm(x0, x0, 3, false);
            e.lsl(x0, x0, 1, false);
            e.mov_imm32(x1, 0x9C);  // RMode for each RM, two bits apiece
            e.lsrv(x1, x1, x0, false);
            e.and_imm(x1, x1, 3, false);
            e.lsl(x1, x1, 22, false);
        } else {
            static constexpr u32 kRmode[] = {0, 3, 1, 2};
            e.mov_imm32(x1, kRmode[rm & 3] << 22);
        }
        e.mrs_fpcr(x2);
        e.and_imm(x2, x2, ~(u64{3} << 22));
        e.orr(x2, x2, x1);
        e.msr_fpcr(x2);
        e.msr_fpsr(xzr);
        op();
        e.mrs_fpsr(x3);
        e.mrs_fpcr(x2);
        e.and_imm(x2, x2, ~(u64{3} << 22));
        e.msr_fpcr(x2);
        e.tst_imm(x3, 0xF, false);  // invalid, divide by zero, overflow, underflow
        slow.push_back(e.b(Cond::Ne));
    };
    // Rounding to nearest goes on in line; any other mode takes `path`,
    // which rejoins where fp_join() is called. The body sees the machine
    // state at the split, so it must not use the register cache.
    auto fp_if_directed = [&](AltBody path) {
        e.ldr(x6, kCpu, o_fcr31, 4);
        e.tst_imm(x6, 3, false);
        alt_paths.push_back({{e.b(Cond::Ne)}, std::move(path), 0});
    };
    auto fp_join = [&] { alt_paths.back().join = e.size(); };
    // v register v holds zero or a normal number, else slow; its bits in x0.
    auto fp_ordinary = [&](unsigned v, bool dbl, std::vector<Site>& slow) {
        e.fmov_to_gpr(x0, v, dbl);
        e.lsl(x1, x0, 1, dbl);
        const Site zero = e.cbz(x1, dbl);
        e.lsr(x1, x1, dbl ? 53 : 24, dbl);
        e.sub_imm(x1, x1, 1, false);
        e.cmp_imm(x1, dbl ? 0x7FD : 0xFD, false);
        slow.push_back(e.b(Cond::Hi));
        e.bind(zero);
    };
    // FPU register n (zero or normal, else slow) as a v register, its bits in x0.
    auto fp_operand = [&](unsigned n, bool dbl, unsigned scratch, std::vector<Site>& slow) -> unsigned {
        const unsigned v = fp_src(n, dbl, scratch);
        fp_ordinary(v, dbl, slow);
        return v;
    };
    auto fp_commit_x1 = [&] {  // fp_commit() with x1 = 1 for an inexact result
        e.ldr(x2, kCpu, o_fcr31, 4);
        e.and_imm(x2, x2, ~kCauseMask, false);
        e.mov_imm(x3, kCauseInexact | kFlagInexact);
        e.mul(x1, x1, x3);
        e.orr(x2, x2, x1, false);
        e.str(x2, kCpu, o_fcr31, 4);
    };
    auto fp_commit_exact = [&] {
        e.ldr(x2, kCpu, o_fcr31, 4);
        e.and_imm(x2, x2, ~kCauseMask, false);
        e.str(x2, kCpu, o_fcr31, 4);
    };
    auto fp_commit_flags = [&] {  // fp_commit() with the inexact flag in x3 (FPSR.IXC)
        e.lsr(x1, x3, 4, false);
        e.and_imm(x1, x1, 1, false);
        fp_commit_x1();
    };
    // A result (v register v) that is not denormal, else slow; x0, x1 clobbered.
    auto fp_not_denormal = [&](unsigned v, bool dbl, std::vector<Site>& slow) {
        e.fmov_to_gpr(x0, v, dbl);
        e.lsl(x1, x0, 1, dbl);
        const Site zero = e.cbz(x1, dbl);
        e.lsr(x1, x1, dbl ? 53 : 24, dbl);
        slow.push_back(e.cbz(x1, dbl));
        e.bind(zero);
    };
    auto emit_fpu = [&](u32 w, std::vector<Site>& slow) -> bool {
        const unsigned opc = w >> 26;
        if (opc == 0x00 && (w & 63) == 0x01 && mips4_) {  // MOVF, MOVT
            const unsigned rs = f_rs(w), rd = f_rd(w);
            need_mips4(slow);
            e.ldr(x0, kCpu, o_status, 4);
            slow.push_back(e.cbz_bit(x0, 29));  // CU1
            touch_();
            if (!rd) return true;
            const Reg vs = src(rs, x2), vd = src(rd, x3);
            e.ldr(x1, kCpu, o_fcr31, 4);
            e.tst_imm(x1, 1u << fcc_bit_number((w >> 18) & 7), false);
            e.csel(x0, vs, vd, (w >> 16) & 1 ? Cond::Ne : Cond::Eq);
            write(rd, x0);
            return true;
        }
        // COP1X multiply-add: fr in the rs field, the format in the function's low bit.
        const bool madd = opc == 0x13 && (w & 0x06) == 0 && (w & 63) >= 0x20;
        if (opc != 0x11 && !madd) return false;
        if (madd && !mips4_) return false;
        const unsigned fmt = madd ? 0x10 + (w & 1) : f_rs(w), ft = f_rt(w), fs = f_rd(w), fd = f_sa(w), fr_reg = f_rs(w);
        unsigned func = w & 63;
        switch (madd ? 0x10u : fmt) {
            case 0x00:  // MFC1
            case 0x01:  // DMFC1
                if (fmt == 0x01) need64(slow);
                fp_usable(slow);
                fp_read_gpr(x0, fs, fmt == 0x01);
                if (fmt == 0x00) e.sxtw(x0, x0);
                write_nz(ft, x0);
                touch_();
                return true;
            case 0x04:  // MTC1
            case 0x05:  // DMTC1
                if (fmt == 0x05) need64(slow);
                fp_usable(slow);
                fp_write_gpr(fs, fmt == 0x05, src(ft, x2));
                touch_();
                return true;
            case 0x02:  // CFC1
                fp_usable(slow);
                if (fs == 31) {
                    static_assert(kFcrLazyI == 1u << 18);
                    e.ldr(x0, kCpu, o_fcr31, 4);
                    slow.push_back(e.cbnz_bit(x0, 18));  // Cause.I to be worked out first
                    e.sxtw(x0, x0);
                } else {
                    e.mov_imm(x0, fs == 0 ? sext32(fir_) : 0);
                }
                write_nz(ft, x0);
                touch_();
                return true;
            case 0x06:  // CTC1: FCR31 written, unless that raises an exception
                fp_usable(slow);
                if (fs == 31) {
                    e.and_imm(x0, src(ft, x2), mips4_ ? kFcrWriteMask | kFcrCondMips4 : kFcrWriteMask, false);
                    e.lsr(x1, x0, 12, false);
                    e.and_imm(x1, x1, 0x3F, false);  // Cause
                    e.lsr(x3, x0, 7, false);
                    e.and_imm(x3, x3, 0x1F, false);
                    e.orr_imm(x3, x3, 0x20, false);  // the enables, and E (always enabled)
                    e.tst(x1, x3, false);
                    slow.push_back(e.b(Cond::Ne));
                    e.str(x0, kCpu, o_fcr31, 4);
                }
                touch_();
                return true;
            case 0x10:
            case 0x11:
            case 0x14:
            case 0x15:
                break;
            default: return false;
        }
        const bool dbl = fmt == 0x11;
        // x1 = 1 if the rounding error in v register `v` is nonzero.
        auto nonzero = [&](unsigned v, bool d) {
            e.fmov_to_gpr(x1, v, d);
            e.lsl(x1, x1, 1, d);
            e.cmp_imm(x1, 0, d);
            e.cset(x1, Cond::Ne);
        };
        // FCMP of two doubles: x1 = 1 if they differ.
        auto differ = [&](unsigned a, unsigned c) {
            e.fcmp(true, a, c);
            e.cset(x1, Cond::Ne);
        };
        if (fmt == 0x14 || fmt == 0x15) {  // CVT.S.W, CVT.D.W, CVT.S.L, CVT.D.L
            if (func != 0x20 && func != 0x21) return false;
            const bool to_d = func == 0x21, from_l = fmt == 0x15;
            if (from_l) need64(slow);
            fp_usable(slow);
            fp_read_gpr(x4, fs, from_l);
            if (to_d && !from_l) {  // exact
                e.scvtf(true, 0, x4);
                fp_commit_exact();
            } else if (from_l) {  // the host's flags, any rounding
                fp_trap_off(slow);
                fp_host_rounded(kGuestRounding, [&] { e.scvtf(to_d, 0, x4, true); }, slow);
                fp_commit_flags();
            } else {  // inexact if the single differs from the integer
                fp_trap_off(slow);
                fp_if_directed([=, &e](std::vector<Site>& s) {
                    fp_host_rounded(kGuestRounding, [&] { e.scvtf(false, 0, x4); }, s);
                    fp_commit_flags();
                });
                e.scvtf(false, 0, x4);
                e.fcvt_to_double(1, 0);
                e.scvtf(true, 2, x4);
                differ(1, 2);
                fp_commit_x1();
                fp_join();
            }
            fp_write_v(fd, to_d, 0);
            touch_();
            return true;
        }
        const s64 exp_max = dbl ? 0x7FF : 0xFF;
        const s64 min_result = dbl ? 64 : 32, min_dividend = dbl ? 128 : 64;
        // x1 = exponent field of x0 (nonzero x0).
        auto exponent = [&] {
            e.lsl(x1, x0, 1, dbl);
            e.lsr(x1, x1, dbl ? 53 : 24, dbl);
        };
        // A zero result (x0) passes only if one of `zeros` is zero; a nonzero
        // one needs an exponent of at least `min_exp` and below the maximum.
        auto result_check = [&](std::initializer_list<unsigned> zeros, s64 min_exp) {
            e.lsl(x1, x0, 1, dbl);
            const Site zero = e.cbz(x1, dbl);
            e.lsr(x1, x1, dbl ? 53 : 24, dbl);
            e.cmp_imm(x1, min_exp, false);
            slow.push_back(e.b(Cond::Lo));
            e.cmp_imm(x1, exp_max, false);
            slow.push_back(e.b(Cond::Eq));
            const Site ok = e.b();
            e.bind(zero);
            std::vector<Site> exact;
            for (unsigned v : zeros) {
                e.fmov_to_gpr(x1, v, dbl);
                e.lsl(x1, x1, 1, dbl);
                exact.push_back(e.cbz(x1, dbl));
            }
            slow.push_back(e.b());
            for (const Site& site : exact) e.bind(site);
            e.bind(ok);
        };
        if (madd) {  // MADD, MSUB, NMADD, NMSUB: p = fs * ft rounded, then p +- fr rounded
            const bool subtract = (func & 0x08) != 0, negate = (func & 0x10) != 0;
            const Emitter::Fop sum = subtract ? Emitter::Fop::Sub : Emitter::Fop::Add;
            need_mips4(slow);
            fp_usable(slow);
            fp_trap_off(slow);
            const unsigned a = fp_operand(fs, dbl, 4, slow), m = fp_operand(ft, dbl, 3, slow);
            // Other rounding: the product and the sum under it, by the host's
            // flags; fr from its cache register (brought in now) or memory.
            const int kc = fp_slot(fr_reg, dbl);
            const s64 fr_off = fpr_base + fp_storage(fr_reg, dbl);
            fp_if_directed([=, &e](std::vector<Site>& s) {
                unsigned vc = 2;
                if (kc >= 0) vc = kFpCacheRegs[kc];
                else e.fldr(2, kCpu, fr_off, dbl ? 8 : 4);
                fp_ordinary(vc, dbl, s);
                fp_host_rounded(
                    kGuestRounding,
                    [&] {
                        e.fop(Emitter::Fop::Mul, dbl, 0, a, m);
                        e.fmov(dbl, 1, 0);
                        e.fop(sum, dbl, 0, 0, vc);
                    },
                    s);
                fp_not_denormal(1, dbl, s);  // a tiny product: the handler
                fp_not_denormal(0, dbl, s);
                fp_commit_flags();
            });
            e.fop(Emitter::Fop::Mul, dbl, 0, a, m);
            e.fnmsub(dbl, 1, a, m, 0);  // a * m - p
            e.fmov_to_gpr(x0, 0, dbl);
            result_check({a, m}, min_result);  // the product: neither tiny nor overflowing
            nonzero(1, dbl);
            e.mov(x5, x1);  // the product was inexact
            e.fmov(dbl, 5, 0);  // p
            const unsigned c = fp_operand(fr_reg, dbl, 2, slow);
            e.fop(sum, dbl, 0, 5, c);
            e.fop(Emitter::Fop::Sub, dbl, 1, 0, 5);  // TwoSum / TwoDiff with p in v5
            e.fop(subtract ? Emitter::Fop::Add : Emitter::Fop::Sub, dbl, 2, c, 1);
            e.fop(Emitter::Fop::Sub, dbl, 3, 0, 1);
            e.fop(Emitter::Fop::Sub, dbl, 1, 5, 3);
            e.fop(sum, dbl, 1, 1, 2);
            e.fmov_to_gpr(x0, 0, dbl);
            result_check({0}, 1);  // a zero sum is exact
            nonzero(1, dbl);
            e.orr(x1, x1, x5, false);
            fp_commit_x1();
            fp_join();
            if (negate) {
                e.fmov_to_gpr(x0, 0, dbl);
                e.eor_imm(x0, x0, dbl ? u64{1} << 63 : u64{0x80000000u}, dbl);
                fp_write_gpr(fd, dbl, x0);
            } else {
                fp_write_v(fd, dbl, 0);
            }
            touch_();
            return true;
        }
        if (func >= 0x11 && func <= 0x13 && mips4_) {  // MOVF/MOVT.fmt, MOVZ.fmt, MOVN.fmt: Cause untouched
            need_mips4(slow);
            fp_usable(slow);
            touch_();
            // Both registers are brought into the cache before the condition
            // splits the way, so either way leaves the same state.
            const unsigned v = fp_src(fs, dbl, 0);
            fp_slot(fd, dbl);
            Site skip;
            if (func == 0x11) {
                e.ldr(x1, kCpu, o_fcr31, 4);
                const unsigned bit = fcc_bit_number(ft >> 2);
                skip = (ft & 1) ? e.cbz_bit(x1, bit) : e.cbnz_bit(x1, bit);
            } else {
                const Reg vt = src(ft, x1);
                skip = func == 0x12 ? e.cbnz(vt) : e.cbz(vt);
            }
            fp_write_v(fd, dbl, v);
            e.bind(skip);
            return true;
        }
        // Other rounding for the operations below: by the host's flags.
        auto directed = [&](unsigned op, unsigned a, unsigned c) {
            fp_if_directed([=, &e](std::vector<Site>& s) {
                static constexpr Emitter::Fop kOps[] = {Emitter::Fop::Add, Emitter::Fop::Sub, Emitter::Fop::Mul,
                                                        Emitter::Fop::Div};
                fp_host_rounded(
                    kGuestRounding,
                    [&] {
                        if (op == 0x04) e.fsqrt(dbl, 0, a);
                        else e.fop(kOps[op], dbl, 0, a, c);
                    },
                    s);
                fp_not_denormal(0, dbl, s);
                fp_commit_flags();
            });
        };
        const bool recip = func == 0x15 && mips4_;
        if (func <= 0x04 || recip) {  // ADD, SUB, MUL, DIV, SQRT, RECIP (1 / fs, as DIV)
            if (recip) need_mips4(slow);
            fp_usable(slow);
            fp_trap_off(slow);
            unsigned a = 4;
            if (recip) {
                e.mov_imm(x0, dbl ? 0x3FF0000000000000ull : 0x3F800000u);
                e.fmov_from_gpr(4, x0, dbl);
                func = 0x03;
            } else {
                a = fp_operand(fs, dbl, 4, slow);
            }
            unsigned c = a;
            if (func == 0x04) {  // SQRT: a zero or a positive number not too small
                directed(func, a, a);  // (x0 still holds the operand)
                e.lsr(x1, x0, dbl ? 63 : 31, dbl);
                const Site positive = e.cbz(x1, false);
                e.lsl(x1, x0, 1, dbl);
                slow.push_back(e.cbnz(x1, dbl));  // the root of a negative number is invalid
                e.bind(positive);
                e.lsl(x1, x0, 1, dbl);
                const Site zero = e.cbz(x1, dbl);
                exponent();
                e.cmp_imm(x1, min_dividend, false);
                slow.push_back(e.b(Cond::Lo));
                e.bind(zero);
                e.fsqrt(dbl, 0, a);
                e.fmsub(dbl, 1, 0, 0, a);  // a - r * r
            } else {
                c = fp_operand(recip ? fs : ft, dbl, 3, slow);
                directed(func, a, c);  // (x0 still holds c)
                if (func == 0x03) {  // DIV: a nonzero divisor, a dividend zero or not too small
                    e.lsl(x1, x0, 1, dbl);
                    slow.push_back(e.cbz(x1, dbl));
                    e.fmov_to_gpr(x0, a, dbl);
                    e.lsl(x1, x0, 1, dbl);
                    const Site zero = e.cbz(x1, dbl);
                    exponent();
                    e.cmp_imm(x1, min_dividend, false);
                    slow.push_back(e.b(Cond::Lo));
                    e.bind(zero);
                }
                static constexpr Emitter::Fop kOps[] = {Emitter::Fop::Add, Emitter::Fop::Sub, Emitter::Fop::Mul,
                                                        Emitter::Fop::Div};
                e.fop(kOps[func], dbl, 0, a, c);
                if (func == 0x00) {  // TwoSum: (a - (s - (s - a))) + (b - (s - a))
                    e.fop(Emitter::Fop::Sub, dbl, 1, 0, a);
                    e.fop(Emitter::Fop::Sub, dbl, 2, c, 1);
                    e.fop(Emitter::Fop::Sub, dbl, 3, 0, 1);
                    e.fop(Emitter::Fop::Sub, dbl, 1, a, 3);
                    e.fop(Emitter::Fop::Add, dbl, 1, 1, 2);
                } else if (func == 0x01) {  // (a - (s - (s - a))) - (b + (s - a))
                    e.fop(Emitter::Fop::Sub, dbl, 1, 0, a);
                    e.fop(Emitter::Fop::Add, dbl, 2, c, 1);
                    e.fop(Emitter::Fop::Sub, dbl, 3, 0, 1);
                    e.fop(Emitter::Fop::Sub, dbl, 1, a, 3);
                    e.fop(Emitter::Fop::Sub, dbl, 1, 1, 2);
                } else if (func == 0x02) {
                    e.fnmsub(dbl, 1, a, c, 0);  // a * b - p
                } else {
                    e.fmsub(dbl, 1, 0, c, a);  // a - q * b
                }
            }
            e.fmov_to_gpr(x0, 0, dbl);
            if (func <= 0x01) result_check({0}, 1);  // a zero sum is exact
            else if (func == 0x02) result_check({a, c}, min_result);
            else if (func == 0x03) result_check({a}, min_result);
            nonzero(1, dbl);
            fp_commit_x1();
            fp_join();
            fp_write_v(fd, dbl, 0);
            touch_();
            return true;
        }
        switch (func) {
            case 0x06:  // MOV: a plain copy, Cause untouched
                fp_usable(slow);
                fp_write_v(fd, dbl, fp_src(fs, dbl, 0));
                touch_();
                return true;
            case 0x05:  // ABS
            case 0x07: {  // NEG
                fp_usable(slow);
                fp_operand(fs, dbl, 0, slow);
                const u64 sign = dbl ? u64{1} << 63 : u64{0x80000000u};
                if (func == 0x05) e.and_imm(x0, x0, ~sign & (dbl ? ~u64{0} : 0xFFFFFFFFu), dbl);
                else e.eor_imm(x0, x0, sign, dbl);
                fp_commit_exact();
                fp_write_gpr(fd, dbl, x0);
                touch_();
                return true;
            }
            case 0x20:  // CVT.S.D
            case 0x21: {  // CVT.D.S (exact)
                if ((func == 0x20) != dbl) return false;  // CVT.S.S and CVT.D.D are unimplemented
                fp_usable(slow);
                if (dbl) fp_trap_off(slow);
                const unsigned v = fp_operand(fs, dbl, 4, slow);
                if (dbl) {  // a zero, or a normal single, inexact if it differs from the double
                    fp_if_directed([=, &e](std::vector<Site>& s) {
                        fp_host_rounded(kGuestRounding, [&] { e.fcvt_to_single(0, v); }, s);
                        fp_not_denormal(0, false, s);
                        fp_commit_flags();
                    });
                    e.fcvt_to_single(0, v);
                    e.fmov_to_gpr(x0, 0, false);
                    e.lsl(x1, x0, 1, false);
                    const Site zero = e.cbz(x1, false);
                    e.lsr(x1, x1, 24, false);
                    slow.push_back(e.cbz(x1, false));
                    e.cmp_imm(x1, 0xFF, false);
                    slow.push_back(e.b(Cond::Eq));
                    const Site ok = e.b();
                    e.bind(zero);
                    e.fmov_to_gpr(x1, v, true);
                    e.lsl(x1, x1, 1);
                    slow.push_back(e.cbnz(x1));  // underflow
                    e.bind(ok);
                    e.fcvt_to_double(1, 0);
                    differ(1, v);
                    fp_commit_x1();
                    fp_join();
                } else {
                    e.fcvt_to_double(0, v);
                    fp_commit_exact();
                }
                fp_write_v(fd, !dbl, 0);
                touch_();
                return true;
            }
            case 0x08: case 0x09: case 0x0A: case 0x0B:  // ROUND/TRUNC/CEIL/FLOOR.L
            case 0x0C: case 0x0D: case 0x0E: case 0x0F:  // ROUND/TRUNC/CEIL/FLOOR.W
            case 0x24:                                   // CVT.W (FCR31.RM)
            case 0x25: {                                 // CVT.L
                const bool to_long = func == 0x25 || func < 0x0C;
                if (to_long) need64(slow);
                fp_usable(slow);
                fp_trap_off(slow);
                const unsigned v = fp_operand(fs, dbl, 4, slow);
                // The direction in the function's low bits, in FCR31.RM's order.
                if (func >= 0x24) {
                    fp_if_directed([=, &e](std::vector<Site>&) {
                        e.and_imm(x6, x6, 3, false);  // RM, from the split
                        e.cmp_imm(x6, 2, false);
                        const Site up = e.b(Cond::Eq);
                        const Site down = e.b(Cond::Hi);
                        e.fcvt_to_int(dbl, 1, x4, v, to_long);
                        const Site done1 = e.b();
                        e.bind(up);
                        e.fcvt_to_int(dbl, 2, x4, v, to_long);
                        const Site done2 = e.b();
                        e.bind(down);
                        e.fcvt_to_int(dbl, 3, x4, v, to_long);
                        e.bind(done1);
                        e.bind(done2);
                    });
                    e.fcvt_to_int(dbl, 0, x4, v, to_long);
                    fp_join();
                } else {
                    e.fcvt_to_int(dbl, func & 3, x4, v, to_long);
                }
                // Out of range saturates (exactly the most negative or positive
                // integer: the handler sorts it out).
                if (to_long) {
                    e.mov_imm(x1, u64{1} << 63);
                    e.cmp(x4, x1);
                    slow.push_back(e.b(Cond::Eq));
                    e.mvn(x1, x1);
                    e.cmp(x4, x1);
                    slow.push_back(e.b(Cond::Eq));
                } else {
                    e.mov_imm32(x1, 0x80000000u);
                    e.cmp(x4, x1, false);
                    slow.push_back(e.b(Cond::Eq));
                    e.mov_imm32(x1, 0x7FFFFFFFu);
                    e.cmp(x4, x1, false);
                    slow.push_back(e.b(Cond::Eq));
                }
                e.scvtf(true, 1, x4, to_long);  // inexact if the integer differs from the operand
                if (dbl) {
                    differ(1, v);
                } else {
                    e.fcvt_to_double(2, v);
                    differ(1, 2);
                }
                fp_commit_x1();
                fp_write_gpr(fd, to_long, x4);
                touch_();
                return true;
            }
            case 0x16: {  // RSQRT (MIPS IV): the square root rounded, then its reciprocal, by the host's flags
                if (!mips4_) break;
                need_mips4(slow);
                fp_usable(slow);
                fp_trap_off(slow);
                const unsigned a = fp_operand(fs, dbl, 4, slow);
                e.mov_imm(x5, dbl ? 0x3FF0000000000000ull : 0x3F800000u);
                e.fmov_from_gpr(1, x5, dbl);
                fp_host_rounded(
                    kGuestRounding,
                    [&] {
                        e.fsqrt(dbl, 0, a);
                        e.fop(Emitter::Fop::Div, dbl, 0, 1, 0);
                    },
                    slow);
                fp_not_denormal(0, dbl, slow);
                fp_commit_flags();
                fp_write_v(fd, dbl, 0);
                touch_();
                return true;
            }
            default:
                break;
        }
        if (func >= 0x30 && ((fd >> 2) == 0 || mips4_)) {  // C.cond: no NaN operand, so ordered
            const unsigned cond = func & 6;  // less (4), equal (2)
            const unsigned bit = fcc_bit_number(fd >> 2);
            if (fd >> 2) need_mips4(slow);
            fp_usable(slow);
            auto not_nan = [&](unsigned n, unsigned scratch) {
                const unsigned v = fp_src(n, dbl, scratch);
                e.fmov_to_gpr(x1, v, dbl);
                e.lsl(x1, x1, 1, dbl);
                e.mov_imm(x2, dbl ? 0xFFE0000000000000ull : 0xFF000000u);
                e.cmp(x1, x2, dbl);
                slow.push_back(e.b(Cond::Hi));
                return v;
            };
            const unsigned va = not_nan(fs, 0), vb = not_nan(ft, 1);
            if (cond) {
                e.fcmp(dbl, va, vb);
                e.cset(x1, cond == 6 ? Cond::Ls : cond == 4 ? Cond::Mi : Cond::Eq);
                e.lsl(x1, x1, bit, false);
            } else {
                e.mov(x1, xzr);
            }
            e.ldr(x2, kCpu, o_fcr31, 4);
            e.and_imm(x2, x2, ~(kCauseMask | (1u << bit)), false);
            e.orr(x2, x2, x1, false);
            e.str(x2, kCpu, o_fcr31, 4);
            touch_();
            return true;
        }
        return false;
    };

    // Loads and stores of RAM, inline (see jit_x64.cpp).
    auto emit_memory = [&](u32 w, std::vector<Site>& slow) -> bool {
        const unsigned opc = w >> 26, rs = f_rs(w), rt = f_rt(w);
        unsigned size;
        bool is_store = false, merge = false, w64 = false, fp = false;
        // COP1X (R5000): the address is base (rs) + index (rt); the FPU
        // register is fd for loads, fs for stores.
        const bool indexed = opc == 0x13 && mips4_;
        unsigned freg = rt;
        if (indexed) {
            switch (w & 63) {
                case 0x00: size = 4, freg = f_sa(w); break;                    // LWXC1
                case 0x01: size = 8, freg = f_sa(w); break;                    // LDXC1
                case 0x08: size = 4, freg = f_rd(w), is_store = true; break;   // SWXC1
                case 0x09: size = 8, freg = f_rd(w), is_store = true; break;   // SDXC1
                case 0x0F:  // PREFX: a hint
                    need_mips4(slow);
                    fp_usable(slow);
                    touch_();
                    return true;
                default: return false;
            }
            fp = true;
        } else switch (opc) {
            case 0x20:
            case 0x24: size = 1; break;
            case 0x21:
            case 0x25: size = 2; break;
            case 0x23: size = 4; break;
            case 0x27: size = 4, w64 = true; break;  // LWU
            case 0x37: size = 8, w64 = true; break;  // LD
            case 0x28: size = 1, is_store = true; break;
            case 0x29: size = 2, is_store = true; break;
            case 0x2B: size = 4, is_store = true; break;
            case 0x3F: size = 8, is_store = true, w64 = true; break;  // SD
            case 0x22: case 0x26: size = 4, merge = true; break;  // LWL, LWR
            case 0x2A: case 0x2E: size = 4, merge = true, is_store = true; break;  // SWL, SWR
            case 0x1A: case 0x1B: size = 8, merge = true, w64 = true; break;  // LDL, LDR
            case 0x2C: case 0x2D: size = 8, merge = true, is_store = true, w64 = true; break;  // SDL, SDR
            case 0x31: size = 4, fp = true; break;                   // LWC1
            case 0x35: size = 8, fp = true; break;                   // LDC1
            case 0x39: size = 4, fp = true, is_store = true; break;  // SWC1
            case 0x3D: size = 8, fp = true, is_store = true; break;  // SDC1
            default: return false;
        }
        const bool right = opc == 0x26 || opc == 0x2E || opc == 0x1B || opc == 0x2D;
        const bool d = size == 8;
        if (w64) need64(slow);
        if (indexed) need_mips4(slow);
        if (fp) fp_usable(slow);
        ensure(rs);
        if (((is_store || merge) && !fp) || indexed) ensure(rt);
        read(x0, rs);
        if (indexed) e.add(x0, x0, src(rt, x1));
        else if (w & 0xFFFF) e.add_imm(x0, x0, static_cast<s16>(w & 0xFFFF));
        if (!addr64) e.sxtw(x0, x0);
        if (size > 1 && !merge) {
            e.tst_imm(x0, size - 1);
            slow.push_back(e.b(Cond::Ne));
        }
        e.lsr(x1, x0, 12);        // the page number
        e.ubfiz(x2, x1, 5, 10);   // its slot, as a byte offset
        e.ldr(kScratch0, kCpu, o_asid_tag);
        e.orr(x1, x1, kScratch0);  // the tag
        e.add(x2, x2, kTlb);
        if (is_store) e.add_imm(x2, x2, kTableBytes);  // the store table follows
        e.ldr(kScratch0, x2, offsetof(FastEntry, tag));
        e.cmp(x1, kScratch0);
        slow.push_back(e.b(Cond::Ne));
        e.and_imm(x0, x0, 0xFFF);
        if (merge) {  // x3 = the shift, 8 * the byte index (from the other end for the right forms)
            e.and_imm(x3, x0, size - 1);
            if (right) e.eor_imm(x3, x3, size - 1);
            e.lsl(x3, x3, 3);
            e.and_imm(x0, x0, 0xFFF & ~u64{size - 1});
        }
        if (!is_store) {
            e.ldr(x2, x2, offsetof(FastEntry, host));
            slow.push_back(e.cbz(x2));
            if (fp) {
                e.ldr_index(x4, x2, x0, size);
                if (d) e.rev64(x4, x4);
                else e.rev32(x4, x4);
                fp_write_gpr(freg, d, x4);
                touch_();
                return true;
            }
            if (rt == 0) {  // the access still happens, for its exceptions
                e.ldr_index(xzr, x2, x0, size);
                return true;
            }
            if (merge) {
                e.ldr_index(x4, x2, x0, size);
                if (d) e.rev64(x4, x4);
                else e.rev32(x4, x4);
                const Reg t = src(rt, x6);
                if (!right) {  // (m << sh) | (rt & ((1 << sh) - 1))
                    e.lslv(x4, x4, x3, d);
                    e.mov_imm(x5, 1);
                    e.lslv(x5, x5, x3);
                    e.sub_imm(x5, x5, 1);
                    e.and_(x5, x5, t);
                    e.orr(x4, x4, x5, d);
                    if (!d) e.sxtw(x4, x4);
                } else if (d) {  // (m >> sh) | (rt & ~(~0 >> sh))
                    e.lsrv(x4, x4, x3);
                    e.mov_imm(x5, ~u64{0});
                    e.lsrv(x5, x5, x3);
                    e.mvn(x5, x5);
                    e.and_(x5, x5, t);
                    e.orr(x4, x4, x5);
                } else {  // LWR: a full word sign-extends, a part keeps bits 63:32
                    e.lsrv(x4, x4, x3, false);
                    e.mov_imm32(x5, 0xFFFFFFFFu);
                    e.lsrv(x5, x5, x3, false);
                    e.mvn(x5, x5, false);
                    e.and_(x5, x5, t, false);
                    e.orr(x4, x4, x5, false);
                    const Site part = e.cbnz(x3);
                    e.sxtw(x4, x4);
                    const Site done = e.b();
                    e.bind(part);
                    e.lsr(x5, t, 32);
                    e.lsl(x5, x5, 32);
                    e.orr(x4, x4, x5);
                    e.bind(done);
                }
                write(rt, x4);
                return true;
            }
            switch (opc) {
                case 0x20: e.ldr_index(x0, x2, x0, 1, true); break;
                case 0x24: e.ldr_index(x0, x2, x0, 1); break;
                case 0x21:
                case 0x25:
                    e.ldr_index(x0, x2, x0, 2);
                    e.rev16(x0, x0);
                    if (opc == 0x21) e.sxth(x0, x0);
                    break;
                case 0x37:
                    e.ldr_index(x0, x2, x0, 8);
                    e.rev64(x0, x0);
                    break;
                default:
                    e.ldr_index(x0, x2, x0, 4);
                    e.rev32(x0, x0);
                    if (opc == 0x23) e.sxtw(x0, x0);  // LWU leaves it zero-extended
                    break;
            }
            write(rt, x0);
            return true;
        }
        e.ldr(x1, x2, offsetof(FastEntry, host));
        slow.push_back(e.cbz(x1));
        e.ldr(kScratch0, x2, offsetof(FastEntry, code));
        e.ldr(kScratch0, kScratch0, 0, 1);
        slow.push_back(e.cbnz(kScratch0, false));  // translated code lives in this page
        e.add(x1, x1, x0);                         // the host address
        if (merge) {  // merge_store(): (m & keep) | value, in memory byte order
            const Reg t = src(rt, x4);
            // Left forms: value rt >> sh, keep ~(~0 >> sh); right forms:
            // value rt << sh, keep ~(~0 << sh).
            if (right) e.lslv(x5, t, x3, d);
            else e.lsrv(x5, t, x3, d);
            if (d) e.mov_imm(x6, ~u64{0});
            else e.mov_imm32(x6, 0xFFFFFFFFu);
            if (right) e.lslv(x6, x6, x3, d);
            else e.lsrv(x6, x6, x3, d);
            e.mvn(x6, x6, d);
            if (d) e.rev64(x5, x5), e.rev64(x6, x6);
            else e.rev32(x5, x5), e.rev32(x6, x6);
            e.ldr(x7, x1, 0, size);
            e.and_(x7, x7, x6, d);
            e.orr(x7, x7, x5, d);
            e.str(x7, x1, 0, size);
            touch_();
            return true;
        }
        // log_store(), while stores are logged
        e.ldr(x2, kCpu, offset_of(this, &log_stores_), 1);
        const Site unlogged = e.cbz(x2, false);
        e.ldr(x2, kCpu, o_store_count);
        e.add_imm(x0, x2, 1);
        e.str(x0, kCpu, o_store_count);
        e.and_imm(x2, x2, (1u << kStoreLogBits) - 1);
        e.mov_imm(kScratch0, sizeof(StoreRec));
        e.madd(x2, x2, kScratch0, kCpu);
        e.str(x1, x2, log_host);
        e.mov_imm32(x0, size);
        e.str(x0, x2, log_size, 4);
        e.str(xzr, x2, log_value);
        e.bind(unlogged);
        Reg v;
        if (fp) {
            fp_read_gpr(x0, freg, d);
            v = x0;
        } else {
            v = src(rt, x0);
        }
        switch (size) {
            case 1: e.str(v, x1, 0, 1); break;
            case 2:
                e.rev16(x0, v);
                e.str(x0, x1, 0, 2);
                break;
            case 8:
                e.rev64(x0, v);
                e.str(x0, x1, 0, 8);
                break;
            default:
                e.rev32(x0, v);
                e.str(x0, x1, 0, 4);
                break;
        }
        if (fp) touch_();
        return true;
    };

    // Register operations, inline; others of the kind call their handler.
    auto emit_plain = [&](const Op& op) {
        const u32 w = op.inst;
        const unsigned rs = f_rs(w), rt = f_rt(w), rd = f_rd(w), sa = f_sa(w);
        const s32 simm = static_cast<s16>(w & 0xFFFF);
        const u32 zimm = w & 0xFFFF;
        const unsigned opc = w >> 26, funct = w & 63;
        // Results go straight to the destination's cache register where it has one.
        const unsigned dest_g = opc == 0x00 ? rd : rt;
        const Reg d0 = (dest_g && cached(dest_g)) ? host_of[dest_g] : x0;
        auto sext_store = [&](unsigned dest) {  // d0 <- sext32(w(d0)), stored to dest
            e.sxtw(d0, d0);
            write(dest, d0);
        };
        // MTHI, MTLO, MULT(U) and DIV(U) write HI/LO rather than rd.
        const bool is_mult = opc == 0x00 && (funct == 0x11 || funct == 0x13 || (funct >= 0x18 && funct <= 0x1B));
        bool done = true;
        if ((opc == 0x00 ? rd : rt) == 0 && !is_mult) {  // writes r0: decoded as a no-op
        } else if (opc == 0x00) {
            switch (funct) {
                case 0x00:  // SLL
                case 0x02:  // SRL
                case 0x03: {  // SRA
                    const Reg t = src(rt, x0);
                    if (funct == 0x00) e.lsl(d0, t, sa, false);
                    else if (funct == 0x02) e.lsr(d0, t, sa, false);
                    else e.asr(d0, t, sa, false);
                    sext_store(rd);
                    break;
                }
                case 0x04:  // SLLV
                case 0x06:  // SRLV
                case 0x07: {  // SRAV (32-bit register shifts take the count mod 32, as MIPS)
                    const Reg s = src(rs, x1), t = src(rt, x0);
                    if (funct == 0x04) e.lslv(d0, t, s, false);
                    else if (funct == 0x06) e.lsrv(d0, t, s, false);
                    else e.asrv(d0, t, s, false);
                    sext_store(rd);
                    break;
                }
                case 0x10: e.ldr(x0, kCpu, o_hi), write(rd, x0); break;  // MFHI
                case 0x12: e.ldr(x0, kCpu, o_lo), write(rd, x0); break;  // MFLO
                case 0x11: e.str(src(rs, x0), kCpu, o_hi); break;       // MTHI
                case 0x13: e.str(src(rs, x0), kCpu, o_lo); break;       // MTLO
                case 0x1A:  // DIV (the one overflow gives MIPS's result here too)
                case 0x1B: {  // DIVU: a zero divisor through the handler
                    const Reg t = src(rt, x1);
                    const Site rare = e.cbz(t, false);
                    const Reg s = src(rs, x0);
                    if (funct == 0x1A) e.sdiv(x2, s, t, false);
                    else e.udiv(x2, s, t, false);
                    e.msub(x3, x2, t, s, false);
                    e.sxtw(x2, x2);
                    e.sxtw(x3, x3);
                    e.str(x2, kCpu, o_lo);
                    e.str(x3, kCpu, o_hi);
                    const Site to_end = e.b();
                    e.bind(rare);
                    flush(dirty);
                    call_handler(op);
                    restore(loaded);
                    e.bind(to_end);
                    break;
                }
                case 0x18:                                              // MULT
                case 0x19: {                                            // MULTU
                    const Reg s = src(rs, x0), t = src(rt, x1);
                    if (funct == 0x18) e.smull(x0, s, t);
                    else e.umull(x0, s, t);
                    e.sxtw(x1, x0);
                    e.str(x1, kCpu, o_lo);
                    if (funct == 0x18) {
                        e.asr(x0, x0, 32);
                    } else {
                        e.lsr(x0, x0, 32);
                        e.sxtw(x0, x0);
                    }
                    e.str(x0, kCpu, o_hi);
                    break;
                }
                case 0x21:  // ADDU
                case 0x23: {  // SUBU
                    const Reg s = src(rs, x0), t = src(rt, x1);
                    if (funct == 0x21) e.add(d0, s, t, false);
                    else e.sub(d0, s, t, false);
                    sext_store(rd);
                    break;
                }
                case 0x24:  // AND
                case 0x25:  // OR
                case 0x26:  // XOR
                case 0x27: {  // NOR
                    const Reg s = src(rs, x0), t = src(rt, x1);
                    if (funct == 0x24) e.and_(d0, s, t);
                    else if (funct == 0x26) e.eor(d0, s, t);
                    else e.orr(d0, s, t);
                    if (funct == 0x27) e.mvn(d0, d0);
                    write(rd, d0);
                    break;
                }
                case 0x2A:  // SLT
                case 0x2B: {  // SLTU
                    const Reg s = src(rs, x0), t = src(rt, x1);
                    e.cmp(s, t);
                    e.cset(d0, funct == 0x2A ? Cond::Lt : Cond::Lo);
                    write(rd, d0);
                    break;
                }
                default: done = false; break;
            }
        } else {
            switch (opc) {
                case 0x09:  // ADDIU
                    e.add_imm(d0, src_imm(rs, x0), simm, false);
                    sext_store(rt);
                    break;
                case 0x0A:  // SLTI
                case 0x0B:  // SLTIU (the immediate is sign-extended, then compared unsigned)
                    e.cmp_imm(src_imm(rs, x1), simm);
                    e.cset(d0, opc == 0x0A ? Cond::Lt : Cond::Lo);
                    write(rt, d0);
                    break;
                case 0x0C:  // ANDI
                case 0x0D:  // ORI
                case 0x0E: {  // XORI
                    const Reg s = src(rs, x0);
                    if (zimm == 0) {
                        if (opc == 0x0C) e.mov(d0, xzr);
                        else e.mov(d0, s);
                    } else if (opc == 0x0C) {
                        e.and_imm(d0, s, zimm);
                    } else if (opc == 0x0D) {
                        e.orr_imm(d0, s, zimm);
                    } else {
                        e.eor_imm(d0, s, zimm);
                    }
                    write(rt, d0);
                    break;
                }
                case 0x0F: write_imm(rt, static_cast<s32>(zimm << 16)); break;  // LUI
                default: done = false; break;
            }
        }
        if (!done) {
            flush(dirty);
            call_handler(op);
            loaded = dirty = 0;
        }
    };

    const Block* cur = &b;  // the block of the region being emitted
    std::size_t cur_k = 0;
    struct Exit {
        Site site;
        u32 completed;
        u64 flush;
        std::size_t k;  // in rb[k]
    };
    std::vector<Exit> exits;
    auto exit_if = [&](Site s, u32 completed, u64 mask) { exits.push_back({s, completed, mask, cur_k}); };
    // Leave when run_block() would after ops[i] (see jit_x64.cpp).
    auto exit_checks = [&](u32 i) {
        e.ldr(x0, kCpu, o_io, 1);
        e.ldr(x1, kCpu, o_invalidated, 1);
        e.orr(x0, x0, x1, false);
        e.ldr(x1, kCpu, o_idle_ready, 1);
        e.orr(x0, x0, x1, false);
        exit_if(e.cbnz(x0, false), i + 1, 0);
        pc_plus(x0, 4 * i + 4);
        e.ldr(x1, kCpu, o_pc);
        e.cmp(x0, x1);
        exit_if(e.b(Cond::Ne), i + 1, 0);
    };
    // (A branch emitted inline leaves next_delay_ clear; one run by its
    // handler sets it.)
    auto leave_delay_slot = [&](bool by_handler) {
        e.ldr(x0, kCpu, o_npc);
        e.str(x0, kCpu, o_pc);
        e.add_imm(x0, x0, 4);
        e.str(x0, kCpu, o_npc);
        if (by_handler) e.str(xzr, kCpu, o_next_delay, 1);
    };

    u32 page_offset = b.offset;  // of the block being emitted
    const u8* const page_host = b.host - page_offset;
    std::vector<s64> successors;
    auto add_successor = [&](s64 delta) {
        const s64 off = static_cast<s64>(page_offset) + delta;
        if (off >= 0 && off < 0x1000) successors.push_back(delta);
    };

    bool synced = true;

    // Branches and jumps, inline (see jit_x64.cpp).
    // A branch-likely not taken, ending the block before its delay slot
    // (see jit_x64.cpp).
    struct Skip {
        Site site{};
        u32 i = 0;
        u64 loaded = 0, dirty = 0;
        bool pending = false;
    } skip;
    // A conditional branch in a region whose delay slot is in its block
    // (see jit_x64.cpp).
    struct Direct {
        bool active = false;
        Site not_taken{};
        s64 taken = 0;
        bool likely = false;
        u32 i = 0;
        u64 loaded = 0, dirty = 0;
    } direct;
    auto member = [&](s64 delta) {
        for (std::size_t j = 0; j < rb.size(); ++j)
            if (static_cast<s64>(rb[j]->offset) == static_cast<s64>(page_offset) + delta) return j;
        return rb.size();
    };
    auto emit_branch = [&](const Op& op, u32 i) -> bool {
        const u32 w = op.inst;
        const unsigned opc = w >> 26, rs = f_rs(w), rt = f_rt(w);
        const s64 simm = static_cast<s16>(w & 0xFFFF);
        enum Kind { kCompare, kSign, kFpCond, kJump, kJumpReg } kind;
        Cond not_taken = Cond::Eq;
        bool likely = false;
        switch (opc) {
            case 0x04: case 0x14: kind = kCompare, not_taken = Cond::Ne; break;  // BEQ(L)
            case 0x05: case 0x15: kind = kCompare, not_taken = Cond::Eq; break;  // BNE(L)
            case 0x06: case 0x16: kind = kSign, not_taken = Cond::Gt; break;     // BLEZ(L)
            case 0x07: case 0x17: kind = kSign, not_taken = Cond::Le; break;     // BGTZ(L)
            case 0x01:
                switch (rt) {
                    case 0x00: case 0x02: kind = kSign, not_taken = Cond::Ge; break;  // BLTZ(L)
                    case 0x01: case 0x03: kind = kSign, not_taken = Cond::Lt; break;  // BGEZ(L)
                    default: return false;
                }
                likely = rt >= 2;
                break;
            case 0x11:  // BC1F, BC1T, BC1FL, BC1TL
                if (rs != 0x08 || ((rt >> 2) != 0 && !mips4_)) return false;  // condition codes 1-7: R5000
                kind = kFpCond, not_taken = (rt & 1) ? Cond::Eq : Cond::Ne, likely = (rt & 2) != 0;
                break;
            case 0x02: case 0x03: kind = kJump; break;  // J, JAL
            case 0x00:
                if ((w & 63) != 0x08 && (w & 63) != 0x09) return false;
                kind = kJumpReg;  // JR, JALR
                break;
            default: return false;
        }
        if (opc >= 0x14 && opc <= 0x17) likely = true;

        if (kind == kFpCond) {  // the FPU unusable (or MIPS IV for codes 1-7): leave, for the handler
            std::vector<Site> leave;
            if (rt >> 2) need_mips4(leave);
            e.ldr(x0, kCpu, offset_of(this, &status_), 4);
            const Site usable = e.cbnz_bit(x0, 29);
            for (const Site& site : leave) e.bind(site);
            pc_plus(x0, 4 * i);
            e.str(x0, kCpu, o_pc);
            set_time(i);
            exits.push_back({e.b(), i, dirty, cur_k});
            e.bind(usable);
            e.ldr(x0, kCpu, offset_of(this, &change_epoch_));  // touch()
            e.add_imm(x0, x0, 1);
            e.str(x0, kCpu, offset_of(this, &change_epoch_));
        }
        // With its delay slot in the block, only npc_ is needed (see jit_x64.cpp).
        const bool slot_here = i + 2 == cur->count;
        if (!slot_here) {
            pc_plus(x0, 4 * i + 4);
            e.str(x0, kCpu, o_pc);
        }
        Site to_not_taken;
        bool conditional = false;
        if (kind == kFpCond) {
            e.ldr(x1, kCpu, offset_of(this, &fcr31), 4);
            e.tst_imm(x1, 1u << fcc_bit_number(rt >> 2), false);  // the condition
            to_not_taken = e.b(not_taken), conditional = true;
        } else if (kind == kCompare) {
            e.cmp(src(rs, x1), src(rt, x2));
            to_not_taken = e.b(not_taken), conditional = true;
        } else if (kind == kSign) {
            e.cmp_imm(src_imm(rs, x1), 0);
            to_not_taken = e.b(not_taken), conditional = true;
        }

        // Taken: the target in x1.
        const s64 delta = 4 * static_cast<s64>(i) + 4 + 4 * simm;
        if (kind == kCompare || kind == kSign || kind == kFpCond) {
            pc_plus(x1, delta);
            add_successor(delta);
        } else if (kind == kJump) {
            pc_plus(x1, 4 * i + 4);
            e.and_imm(x1, x1, ~u64{0x0FFFFFFF});
            e.orr_imm(x1, x1, (w & 0x03FFFFFFu) << 2);
            add_successor(static_cast<s64>((w << 2) & 0xFFF) - static_cast<s64>(page_offset));  // if in this page
            if (opc == 0x03) {
                pc_plus(x0, 4 * i + 8);
                write(31, x0);
            }
        } else {
            read(x1, rs);
            const unsigned rd = f_rd(w);
            if ((w & 63) == 0x09 && rd != 0) {
                pc_plus(x0, 4 * i + 8);
                write(rd, x0);
            }
            std::vector<u32> seen;  // where it went while interpreted
            jr_targets(*cur, seen);
            for (u32 off : seen) add_successor(static_cast<s64>(off) - static_cast<s64>(page_offset));
        }
        e.str(x1, kCpu, o_npc);
        if (!slot_here) store_byte_imm(o_next_delay, 1);
        const bool maybe_backward = ((kind == kCompare || kind == kSign || kind == kFpCond) && simm < 0) || opc == 0x02;
        if (maybe_backward) {
            const unsigned idx = ((page_offset + 4 * i) >> 2) & ((1u << kLoopTableBits) - 1);
            const s64 o_loop_pc = offset_of(this, &loops_[idx].branch_pc);
            const s64 o_cooldown = offset_of(this, &loops_[idx].cooldown);
            std::vector<Site> done;
            pc_plus(x0, 4 * i);  // cur_pc_
            if (opc == 0x02) {
                e.cmp(x1, x0);
                done.push_back(e.b(Cond::Hi));  // forwards
            }
            e.ldr(x2, kCpu, o_loop_pc);
            e.cmp(x0, x2);
            const Site visit1 = e.b(Cond::Ne);
            e.ldr(x2, kCpu, o_cooldown, 4);
            const Site visit2 = e.cbz(x2, false);
            e.sub_imm(x2, x2, 1, false);
            e.str(x2, kCpu, o_cooldown, 4);
            done.push_back(e.b());
            e.bind(visit1);
            e.bind(visit2);
            e.str(x0, kCpu, o_cur_pc);
            if (slot_here) {  // as run_block() would leave them, should this be an idle loop
                e.add_imm(x0, x0, 4);
                e.str(x0, kCpu, o_pc);
                store_byte_imm(o_next_delay, 1);
            }
            e.mov(x24, x1);  // the target, kept over the time update
            set_time(i + 1);
            flush(dirty);  // the loop bookkeeping compares registers
            e.mov(x0, kCpu);
            e.mov(x1, x24);
            call(reinterpret_cast<const void*>(&R4000::jit_backward_branch));
            e.ldr(x0, kCpu, o_idle_ready, 1);
            exit_if(e.cbnz(x0, false), i + 1, 0);
            if (slot_here) e.str(xzr, kCpu, o_next_delay, 1);
            restore(loaded);
            for (const Site& s : done) e.bind(s);
        }

        if (conditional && slot_here && region_mode &&
            (member(delta) < rb.size() || member(4 * static_cast<s64>(cur->count)) < rb.size())) {
            direct = {true, to_not_taken, delta, likely, i, loaded, dirty};
            return true;
        }
        if (conditional) {
            const Site to_done = e.b();
            e.bind(to_not_taken);
            if (likely && slot_here) {
                // The delay slot is skipped: on to the block's end with the
                // time one instruction short (see the end of the block).
                skip = {e.b(), i, loaded, dirty, true};
            } else if (likely) {  // no delay slot in the block: leave, as run_block() does
                pc_plus(x0, 4 * i + 8);
                e.str(x0, kCpu, o_pc);
                e.add_imm(x0, x0, 4);
                e.str(x0, kCpu, o_npc);
                set_time(i + 1);
                exits.push_back({e.b(), i + 1, dirty, cur_k});
            } else {
                pc_plus(x0, 4 * i + 8);
                e.str(x0, kCpu, o_npc);
                if (!slot_here) store_byte_imm(o_next_delay, 1);
            }
            e.bind(to_done);
        }
        return true;
    };
    // Each block of the region in turn (see jit_x64.cpp).
    std::vector<std::size_t> label(rb.size(), 0);
    std::vector<std::vector<Site>> to_label(rb.size()), to_tail(rb.size());
    for (std::size_t k = 0; k < rb.size(); ++k) {
        cur = rb[k], cur_k = k, page_offset = cur->offset;
        const Block& blk = *cur;
        label[k] = k == 0 ? body : e.size();
        for (const Site& site : to_label[k]) e.bind(site);
        if (k > 0) loaded = alloc, dirty = written;
        successors.clear();
        synced = true;
        bool last_handler = false;
        std::vector<Site> to_common, to_finish_direct;
        bool branch_by_handler = false;  // the last branch ran its handler
        // A non-branch op (see jit_x64.cpp); returns whether its handler ran inline.
        auto emit_op = [&](u32 i, bool delay_slot, bool leave) {
            const Op& op = blk.ops[i];
            const u32 w = op.inst;
            bool plain;
            decode(w, plain);
            if (plain) {
                emit_plain(op);
                if (delay_slot && leave) leave_delay_slot(branch_by_handler);
                synced = delay_slot;
                return false;
            }
            std::vector<Site> slow;
            Site skip_slow;
            const u64 dirty_before = dirty;
            alt_paths.clear();
            const bool fast = emit_alu(w, slow) || emit_fpu(w, slow) || emit_memory(w, slow);
            const u64 loaded_after = loaded;
            if (fast) {
                if (delay_slot && leave) leave_delay_slot(branch_by_handler);
                skip_slow = e.b();
                for (const AltPath& alt : alt_paths) {
                    for (const Site& site : alt.from) e.bind(site);
                    alt.body(slow);
                    e.b_to(alt.join);
                }
                for (const Site& site : slow) e.bind(site);
            }

            flush(fast ? dirty_before : dirty);
            set_time(i + 1);
            pc_plus(x0, 4 * i);
            e.str(x0, kCpu, o_cur_pc);
            if (delay_slot) {
                store_byte_imm(o_in_delay, 1);
                e.str(xzr, kCpu, o_next_delay, 1);
                e.ldr(x0, kCpu, o_npc);
                e.str(x0, kCpu, o_pc);
                e.add_imm(x0, x0, 4);
                e.str(x0, kCpu, o_npc);
            } else {
                e.str(xzr, kCpu, o_in_delay, 1);  // next_delay_ is already clear
                pc_plus(x0, 4 * i + 4);
                e.str(x0, kCpu, o_pc);
                e.add_imm(x0, x0, 4);
                e.str(x0, kCpu, o_npc);
            }
            e.str(xzr, kCpu, o_io, 1);
            call_handler(op);
            e.str(xzr, kCpu, gpr_off(0));
            if (i + 1 < blk.count || fast) exit_checks(i);
            synced = !fast || delay_slot;
            if (fast) {
                restore(loaded_after);
                e.bind(skip_slow);
                return false;
            }
            loaded = dirty = 0;
            return true;
        };
        auto link_direct = [&](s64 delta, u32 executed, bool handler_ran) {
            if (handler_ran) {
                if (executed != blk.count) e.add_imm(kTime, kTime, blk.count - executed);
                to_common.push_back(e.b());
                return;
            }
            auto store_pc = [&] {
                pc_plus(x0, delta);
                e.str(x0, kCpu, o_pc);
                e.add_imm(x0, x0, 4);
                e.str(x0, kCpu, o_npc);
                flush(dirty);
                if (executed != blk.count) e.add_imm(kTime, kTime, blk.count - executed);
            };
            const std::size_t to = member(delta);
            if (to == rb.size()) {
                store_pc();
                to_common.push_back(e.b());
                return;
            }
            e.sub_imm(x2, kTime, executed);  // the next block's budget
            e.cmp_imm(x2, rb[to]->count);
            const Site short_budget = e.b(Cond::Lt);
            restore(alloc & ~loaded);
            e.mov(kTime, x2);
            if (delta) e.add_imm(kPc, kPc, delta);
            if (to <= k) e.b_to(label[to]);
            else to_label[to].push_back(e.b());
            e.bind(short_budget);
            store_pc();
            to_finish_direct.push_back(e.b());
        };
        for (u32 i = 0; i < blk.count; ++i) {
            const Op& op = blk.ops[i];
            const bool delay_slot = i > 0 && is_branch_word(blk.ops[i - 1].inst);
            last_handler = false;
            if (direct.active) {
                direct.active = false;
                const u64 fall = 4 * static_cast<u64>(blk.count);
                last_handler = emit_op(i, true, false);
                link_direct(direct.taken, blk.count, last_handler);
                e.bind(direct.not_taken);
                loaded = direct.loaded, dirty = direct.dirty;
                if (direct.likely) {
                    link_direct(static_cast<s64>(fall), blk.count - 1, false);
                } else {
                    bool plain;
                    decode(op.inst, plain);
                    if (!plain) {  // its handler goes on to npc_
                        pc_plus(x0, static_cast<s64>(fall));
                        e.str(x0, kCpu, o_npc);
                    }
                    last_handler = emit_op(i, true, false);
                    link_direct(static_cast<s64>(fall), blk.count, last_handler);
                }
                for (const Site& site : to_common) e.bind(site);
                loaded = dirty = 0;
                last_handler = true;
                synced = true;
                break;
            }
            if (!delay_slot && is_branch_word(op.inst) && emit_branch(op, i)) {
                synced = true;
                branch_by_handler = false;
                continue;
            }
            // A branch run by its handler sets next_delay_, which its delay
            // slot then clears; one emitted inline leaves it clear.
            if (!delay_slot) branch_by_handler = is_branch_word(op.inst);
            last_handler = emit_op(i, delay_slot, true);
        }
        if (!synced) {
            pc_plus(x0, 4 * blk.count);
            e.str(x0, kCpu, o_pc);
            e.add_imm(x0, x0, 4);
            e.str(x0, kCpu, o_npc);
        }
        if (skip.pending) {
            skip.pending = false;
            const Site join = e.b();
            e.bind(skip.site);
            pc_plus(x0, 4 * skip.i + 8);
            e.str(x0, kCpu, o_pc);
            e.add_imm(x0, x0, 4);
            e.str(x0, kCpu, o_npc);
            e.add_imm(kTime, kTime, 1);
            flush(skip.dirty & ~dirty);
            restore(loaded & ~skip.loaded);
            e.bind(join);
        }
        if (!is_branch_word(blk.ops[blk.count - 1].inst)) add_successor(4 * static_cast<s64>(blk.count));
        std::sort(successors.begin(), successors.end());
        successors.erase(std::unique(successors.begin(), successors.end()), successors.end());

        // Block linking (see jit_x64.cpp). x0: the next pc, x2: its entry time.
        std::vector<Site> to_finish_dirty, to_finish;
        auto flag_checks = [&] {
            e.ldr(x0, kCpu, o_io, 1);
            e.ldr(x1, kCpu, o_invalidated, 1);
            e.orr(x0, x0, x1, false);
            e.ldr(x1, kCpu, o_idle_ready, 1);
            e.orr(x0, x0, x1, false);
            e.ldr(x1, kCpu, o_next_delay, 1);
            e.orr(x0, x0, x1, false);
            to_finish_dirty.push_back(e.cbnz(x0, false));
            e.ldr(x0, kCpu, o_mode, 4);
            e.cmp(x0, kEntryMode, false);
            to_finish_dirty.push_back(e.b(Cond::Ne));
        };
        if (last_handler) flag_checks();
        e.sub_imm(x2, kTime, blk.count);  // the next block's budget
        e.ldr(x0, kCpu, o_pc);
        std::vector<s64> outside;
        for (s64 delta : successors) {
            std::size_t to = rb.size();
            for (std::size_t j = 0; j < rb.size(); ++j)
                if (static_cast<s64>(rb[j]->offset) == static_cast<s64>(page_offset) + delta) to = j;
            if (to == rb.size() || !region_mode) {
                outside.push_back(delta);
                continue;
            }
            pc_plus(x1, delta);
            e.cmp(x0, x1);
            const Site other = e.b(Cond::Ne);
            e.cmp_imm(x2, rb[to]->count);
            to_finish_dirty.push_back(e.b(Cond::Lt));
            restore(alloc & ~loaded);
            e.mov(kTime, x2);
            if (delta) e.add_imm(kPc, kPc, delta);
            if (to <= k) e.b_to(label[to]);
            else to_label[to].push_back(e.b());
            e.bind(other);
        }
        if (!last_handler) {
            flag_checks();
            e.ldr(x0, kCpu, o_pc);
        }
        const u64 dirty_end = dirty;
        flush(dirty_end);
        // On to the shared stubs (above), which come back to to_finish
        // through x6 if they can't enter the next translation.
        to_finish.push_back(e.adr(x6));
        for (s64 delta : outside) {
            pc_plus(x1, delta);
            e.cmp(x0, x1);
            const Site next = e.b(Cond::Ne);
            const u8* const host = page_host + static_cast<u32>(static_cast<s64>(page_offset) + delta);
            e.load_constant(x1, reinterpret_cast<u64>(&blocks_[block_index(host)]));
            e.load_constant(x3, reinterpret_cast<u64>(host));
            e.ldr(kScratch0, x1, offsetof(Block, host));
            e.cmp(x3, kScratch0);
            to_finish.push_back(e.b(Cond::Ne));
            e.b_abs(jit_enter_stub_[addr64], base);
            e.bind(next);
        }
        // A jump through a register to a place in the page with no link:
        // learn it (see jit_jr_miss()).
        const bool ends_jr = blk.count >= 2 && (blk.ops[blk.count - 2].inst >> 26) == 0 &&
                             ((blk.ops[blk.count - 2].inst & 63) == 0x08 || (blk.ops[blk.count - 2].inst & 63) == 0x09);
        if (ends_jr) {
            e.eor(x1, x0, kPc);
            e.lsr(x1, x1, 12);
            const Site other_page = e.cbnz(x1);
            e.mov(x0, kCpu);
            e.load_constant(x1, reinterpret_cast<u64>(&blocks_[block_index(blk.host)]));
            e.load_constant(x2, reinterpret_cast<u64>(&blocks_[block_index(b.host)]));
            call(reinterpret_cast<const void*>(&R4000::jit_jr_miss));
            e.sub_imm(x2, kTime, blk.count);
            e.ldr(x0, kCpu, o_pc);
            to_finish.push_back(e.adr(x6));  // the call may have changed it
            e.bind(other_page);
        }
        // Anywhere else: look up the fetch translation and the block.
        e.b_abs(jit_lookup_stub_[addr64], base);

        // Completed the block.
        for (const Site& site : to_finish_dirty) e.bind(site);
        flush(dirty_end);
        for (const Site& site : to_finish) e.bind(site);
        for (const Site& site : to_finish_direct) e.bind(site);
        set_time(blk.count);
        e.mov_imm32(x0, blk.count);
        to_tail[k].push_back(e.b());
    }
    for (const Exit& x : exits) {
        e.bind(x.site);
        flush(x.flush);
        e.mov_imm32(x0, x.completed);
        to_tail[x.k].push_back(e.b());
    }
    // Tell run_block() which block (its slot, and the block that belongs
    // there) and entry pc the count belongs to.
    std::vector<Site> to_epilogue;
    for (std::size_t k = 0; k < rb.size(); ++k) {
        if (to_tail[k].empty()) continue;
        for (const Site& site : to_tail[k]) e.bind(site);
        e.load_constant(kScratch0, reinterpret_cast<u64>(&blocks_[block_index(rb[k]->host)]));
        e.str(kScratch0, kCpu, offset_of(this, &jit_block_));
        e.load_constant(kScratch0, reinterpret_cast<u64>(rb[k]->host));
        e.str(kScratch0, kCpu, offset_of(this, &jit_block_host_));
        to_epilogue.push_back(e.b());
    }
    for (const Site& site : to_epilogue) e.bind(site);
    e.str(kPc, kCpu, offset_of(this, &jit_entry_pc_));
    e.ldp(x23, x24, sp, 48);
    e.ldp(x21, x22, sp, 32);
    e.ldp(x19, x20, sp, 16);
    e.ldp_post(x29, x30, sp, 64);
    e.ret();
    e.finish_literals();

    const void* fn = jit_buf_->add(e.code().data(), e.size());
    if (fn != base) return false;  // full
    b.code = reinterpret_cast<JitFn>(const_cast<void*>(fn));
    b.jit_region = rb.size() > 1;
    b.region = JitRegion{};
    for (std::size_t k = 1; k < rb.size(); ++k) {
        b.region.offset[b.region.n] = static_cast<u16>(rb[k]->offset);
        b.region.count[b.region.n++] = static_cast<u8>(rb[k]->count);
    }
    b.region.hash = jit_region_hash(page_host, b.region);
    return true;
}

}  // namespace sg::mips

#endif
