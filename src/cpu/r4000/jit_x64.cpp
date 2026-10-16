// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// R4000 dynamic recompiler, x86-64 back end.
//
// A hot decoded block is translated into one host function that runs the
// block from its first instruction, exactly as run_block() would when no
// time check is due and no interrupt check is pending (the caller makes
// sure of both). Register operations, RAM loads and stores, and branches
// are emitted inline; everything else calls its pre-decoded handler with
// the usual bookkeeping and leaves the function on the same conditions that
// end a straight-line run in run_block(). At its end a block may jump
// straight into the translation of the next one. The function returns how
// many instructions it completed in the block it finished in; run_block()
// carries on from there.
//
// The guest registers a block uses most are kept in host registers while it
// runs. They are loaded on first use and written back before anything that
// reads the copies in memory (handlers, the idle-loop bookkeeping) and when
// the function leaves or moves on to another block.
#include "cpu/r4000/r4000.h"

#if defined(__x86_64__) || defined(_M_X64)

#include <algorithm>
#include <cstddef>
#include <vector>

#include "jit/exec_buffer.h"
#include "jit/x64_emitter.h"

namespace sg::mips {

namespace {

using namespace sg::jit::x64;

// Registers live across the whole function (callee-saved). The mode at
// entry is kept at [rsp].
constexpr Reg kCpu = rbx;   // R4000*
constexpr Reg kTime = r12;  // time_fp_ at entry
constexpr Reg kPc = r14;    // pc at entry (blocks are shared by aliases)
constexpr Reg kTlb = rbp;   // this mode's soft-TLB load table (fetch before it, store after it)
// Host registers that hold guest registers; rax, rcx and rdx are scratch.
constexpr Reg kCacheRegs[] = {rsi, rdi, r8, r9, r10, r11, r13, r15};
// xmm registers that hold FPU registers; the ones below are scratch.
// (Windows keeps xmm6-xmm15 for the caller, which leaves too few for exact
// products without FMA: there the host's exception flags are used.)
#if defined(_WIN32)
constexpr unsigned kFpCacheRegs[] = {5};
constexpr bool kDekker = false;
#else
constexpr unsigned kFpCacheRegs[] = {9, 10, 11, 12, 13, 14, 15};
constexpr bool kDekker = true;
#endif

// FMA3, for exact rounding errors of products, quotients and roots.
bool host_has_fma() {
#if defined(__GNUC__) || defined(__clang__)
    __builtin_cpu_init();
    return __builtin_cpu_supports("fma");
#else
    return false;
#endif
}

// Calling convention. Windows (x64): arguments in rcx, rdx, r8; rsi and rdi
// are callee-saved; every call needs 32 bytes of shadow space above the
// return address, which the callee may use. System V: rdi, rsi, rdx.
#if defined(_WIN32)
constexpr Reg kArg0 = rcx, kArg1 = rdx, kArg2 = r8;
constexpr s32 kShadow = 32;
constexpr bool kSaveSiDi = true;
#else
constexpr Reg kArg0 = rdi, kArg1 = rsi, kArg2 = rdx;
constexpr s32 kShadow = 0;
constexpr bool kSaveSiDi = false;
#endif

inline unsigned f_rs(u32 i) { return (i >> 21) & 31; }
inline unsigned f_rt(u32 i) { return (i >> 16) & 31; }
inline unsigned f_rd(u32 i) { return (i >> 11) & 31; }
inline unsigned f_sa(u32 i) { return (i >> 6) & 31; }

}  // namespace

template <typename T>
static s32 offset_of(const R4000* cpu, const T* member) {
    return static_cast<s32>(reinterpret_cast<const u8*>(member) - reinterpret_cast<const u8*>(cpu));
}

bool R4000::jit_compile_supported() { return true; }

bool R4000::jit_fma_ = host_has_fma();
void R4000::set_jit_fma(bool enabled) { jit_fma_ = enabled && host_has_fma(); }

bool R4000::jit_compile(Block& b) {
    Emitter e;
    auto gpr_at = [&](unsigned r) { return Mem{kCpu, offset_of(this, &gpr[r])}; };
    const Mem m_time{kCpu, offset_of(this, &time_fp_)};
    const Mem m_pc{kCpu, offset_of(this, &pc)};
    const Mem m_npc{kCpu, offset_of(this, &npc_)};
    const Mem m_cur_pc{kCpu, offset_of(this, &cur_pc_)};
    const Mem m_in_delay{kCpu, offset_of(this, &in_delay_)};
    const Mem m_next_delay{kCpu, offset_of(this, &next_delay_)};
    const Mem m_io{kCpu, offset_of(this, &io_)};
    const Mem m_invalidated{kCpu, offset_of(this, &block_invalidated_)};
    const Mem m_idle_ready{kCpu, offset_of(this, &idle_ready_)};
    const Mem m_hi{kCpu, offset_of(this, &hi)};
    const Mem m_mode{kCpu, offset_of(this, &mode_)};
    const Mem m_safe{kCpu, offset_of(this, &jit_safe_)};
    const Mem m_lo{kCpu, offset_of(this, &lo)};
    const Mem m_weight{kCpu, offset_of(this, &weight_)};
    const Mem m_asid_tag{kCpu, offset_of(this, &asid_tag_)};
    const Mem m_entry_mode{rsp, kShadow};  // above the callees' shadow space

    // Prologue: the callee-saved registers used, then the entry-mode slot
    // and any shadow space, keeping calls 16-byte aligned (six or eight
    // pushes plus the return address, then 8 or 40 bytes).
    e.push(rbx), e.push(rbp), e.push(r12), e.push(r13), e.push(r14), e.push(r15);
    if (kSaveSiDi) e.push(rsi), e.push(rdi);
    e.sub_rsp(8 + kShadow);
    e.mov(kCpu, kArg0);
    e.mov(kPc, kArg1);
    e.mov(kTime, kArg2);
    e.mov(rax, m_mode, false);
    e.mov(m_entry_mode, rax, false);
    static_assert(sizeof(FastEntry) == 32 && kFastEntries == 1024);
    constexpr s32 kTableBytes = kFastEntries * sizeof(FastEntry);
    e.imul_imm(kTlb, rax, 3 * kTableBytes);
    e.alu(Alu::Add, kTlb, kCpu);
    e.alu_imm(Alu::Add, kTlb, offset_of(this, &fast_[0][static_cast<unsigned>(Access::Load)][0]));
    const std::size_t top = e.size();
    // Linked blocks enter here, so the prologue must be the same everywhere.
    if (jit_link_offset_ == 0) jit_link_offset_ = top;
    if (top != jit_link_offset_) return false;
    // Effective addresses follow the addressing mode the block is compiled
    // for; run_block() only calls it in that mode.
    const bool addr64 = addr64_;
    b.jit_addr64 = addr64;

    // Guest register cache. The registers a block uses at least twice
    // (inline) are given host registers, most used first. `loaded`: the host
    // register holds the value; `dirty`: it may differ from memory.
    // The region: this block and those of its page it can reach directly,
    // translated together with one register plan.
    std::vector<Block> members;
    std::vector<const Block*> rb;
    jit_region(b, members, rb);
    const JitPlan plan = jit_plan(rb);
    Reg host_of[32] = {};
    for (unsigned g = 1; g < 32; ++g)
        if (plan.slot[g] >= 0) host_of[g] = kCacheRegs[plan.slot[g]];
    // FPU register cache: the FPU registers the block's inline operations
    // use at least twice, each always in the same width (storage that is
    // seen both as a single and as a double stays in memory), held in xmm
    // registers. Status.FR as at translation fixes where each register
    // lives (the code checks it before every FPU operation). In the masks
    // below their bits follow the 32 guest registers'.
    const bool fr = (status_ & sr::FR) != 0;
    const s32 fpr_base = offset_of(this, &fpr[0]);
    auto fp_storage = [&](unsigned n, bool dbl) -> s32 {  // byte offset of the register within fpr
        if (fr) return static_cast<s32>(8 * n);
        return static_cast<s32>(8 * (n & ~1u) + (dbl ? 0 : 4 * (n & 1)));  // little-endian host
    };
    struct FpKey {
        s32 off;
        unsigned width;
        unsigned uses;
        bool written;
    };
    std::vector<FpKey> fp_keys;
    {
        std::vector<FpKey> all;  // every inline FPU access
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
                if (o.off < a.off + static_cast<s32>(a.width) && a.off < o.off + static_cast<s32>(o.width) &&
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
    // With more than one block, or a block that branches to itself, the
    // registers are held across the moves between them: every block starts
    // with all of them loaded.
    bool region_mode = rb.size() > 1;
    {
        std::vector<u32> next;
        jit_successors(b, next);
        for (u32 off : next) region_mode |= off == b.offset;
    }
    u64 loaded = 0, dirty = 0;
    auto cached = [&](unsigned g) { return ((alloc >> g) & 1) != 0; };
    auto ensure = [&](unsigned g) {  // load a cached register on first use
        if (cached(g) && !((loaded >> g) & 1)) {
            e.mov(host_of[g], gpr_at(g));
            loaded |= u64{1} << g;
        }
    };
    auto read = [&](Reg dst, unsigned g, bool w64 = true) {  // 32-bit reads zero-extend
        if (!cached(g)) return e.mov(dst, gpr_at(g), w64);
        ensure(g);
        if (w64) e.mov(dst, host_of[g]);
        else e.mov32(dst, host_of[g]);
    };
    auto alu_g = [&](Alu op, Reg dst, unsigned g, bool w64 = true) {
        if (!cached(g)) return e.alu(op, dst, gpr_at(g), w64);
        ensure(g);
        e.alu(op, dst, host_of[g], w64);
    };
    auto movsxd_g = [&](Reg dst, unsigned g) {
        if (!cached(g)) return e.movsxd(dst, gpr_at(g));
        ensure(g);
        e.movsxd(dst, host_of[g]);
    };
    auto write = [&](unsigned g, Reg src) {
        if (!cached(g)) return e.mov(gpr_at(g), src);
        e.mov(host_of[g], src);
        loaded |= u64{1} << g;
        dirty |= u64{1} << g;
    };
    auto write_imm = [&](unsigned g, s32 v) {  // sign-extended
        if (!cached(g)) return e.mov_store_imm(gpr_at(g), v);
        e.mov_imm64(host_of[g], static_cast<u64>(static_cast<s64>(v)));
        loaded |= u64{1} << g;
        dirty |= u64{1} << g;
    };
    auto sext32_to = [&](unsigned g) {  // sext32(eax) to guest register g (not r0)
        if (!cached(g)) {
            e.movsxd(rax, rax);
            return e.mov(gpr_at(g), rax);
        }
        e.movsxd(host_of[g], rax);
        loaded |= u64{1} << g;
        dirty |= u64{1} << g;
    };
    // A 64-bit shift of guest register a into d, in d's cache register where
    // it has one; by cl when `by_cl`.
    auto shift64 = [&](Shift op, unsigned d, unsigned a, bool by_cl, u8 n) {
        Reg h = rax;
        if (cached(d)) {
            h = host_of[d];
            if (a != d) read(h, a);
            else ensure(d);
        } else {
            read(rax, a);
        }
        if (by_cl) e.shift_cl(op, h, true);
        else e.shift_imm(op, h, n, true);
        if (h == rax) return e.mov(gpr_at(d), rax);
        loaded |= u64{1} << d;
        dirty |= u64{1} << d;
    };
    auto fp_store_key = [&](std::size_t k) {
        const Mem m{kCpu, fpr_base + fp_keys[k].off};
        if (fp_keys[k].width == 8) e.movq_store(m, kFpCacheRegs[k]);
        else e.movd_store(m, kFpCacheRegs[k]);
    };
    auto fp_load_key = [&](std::size_t k) {
        const Mem m{kCpu, fpr_base + fp_keys[k].off};
        if (fp_keys[k].width == 8) e.movq_load(kFpCacheRegs[k], m);
        else e.movd_load(kFpCacheRegs[k], m);
    };
    auto flush = [&](u64 mask) {  // write back
        for (unsigned g = 1; g < 32; ++g)
            if ((mask >> g) & 1) e.mov(gpr_at(g), host_of[g]);
        for (std::size_t k = 0; k < fp_keys.size(); ++k)
            if (mask & fp_bit(k)) fp_store_key(k);
    };
    auto restore = [&](u64 mask) {  // reload after a call
        for (unsigned g = 1; g < 32; ++g)
            if ((mask >> g) & 1) e.mov(host_of[g], gpr_at(g));
        for (std::size_t k = 0; k < fp_keys.size(); ++k)
            if (mask & fp_bit(k)) fp_load_key(k);
    };
    // FPU register n as a single or a double: its cache slot, or -1 (then
    // any cached register sharing its storage is written back and dropped,
    // which the planning above should make unnecessary).
    auto fp_slot = [&](unsigned n, bool dbl) -> int {
        const s32 off = fp_storage(n, dbl);
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
            if (key.off < off + static_cast<s32>(width) && off < key.off + static_cast<s32>(key.width)) {
                if (dirty & fp_bit(k)) fp_store_key(k);
                loaded &= ~fp_bit(k);
                dirty &= ~fp_bit(k);
            }
        }
        return -1;
    };
    auto fp_mem = [&](unsigned n, bool dbl) { return Mem{kCpu, fpr_base + fp_storage(n, dbl)}; };
    auto fp_read_gpr = [&](Reg r, unsigned n, bool dbl) {  // the bits, zero-extended
        const int k = fp_slot(n, dbl);
        if (k >= 0) e.movq_from_xmm(r, kFpCacheRegs[k], dbl);
        else e.mov(r, fp_mem(n, dbl), dbl);
    };
    auto fp_write_gpr = [&](unsigned n, bool dbl, Reg r) {
        const int k = fp_slot(n, dbl);
        if (k < 0) return e.mov(fp_mem(n, dbl), r, dbl);
        e.movq_to_xmm(kFpCacheRegs[k], r, dbl);
        loaded |= fp_bit(k);
        dirty |= fp_bit(k);
    };
    // An xmm register holding FPU register n: its cache register, or
    // `scratch` loaded from memory.
    auto fp_src = [&](unsigned n, bool dbl, unsigned scratch) -> unsigned {
        const int k = fp_slot(n, dbl);
        if (k >= 0) return kFpCacheRegs[k];
        if (dbl) e.movq_load(scratch, fp_mem(n, dbl));
        else e.movd_load(scratch, fp_mem(n, dbl));
        return scratch;
    };
    auto fp_write_xmm = [&](unsigned n, bool dbl, unsigned x) {
        const int k = fp_slot(n, dbl);
        if (k < 0) {
            if (dbl) e.movq_store(fp_mem(n, dbl), x);
            else e.movd_store(fp_mem(n, dbl), x);
            return;
        }
        e.movapd(kFpCacheRegs[k], x);
        loaded |= fp_bit(k);
        dirty |= fp_bit(k);
    };
    if (region_mode) {
        restore(alloc);
        loaded = alloc;
        dirty = written;
    }
    const std::size_t body = e.size();

    auto set_time = [&](unsigned n) {  // time_fp_ = entry time + n instructions
        e.imul_imm(rax, m_weight, static_cast<s32>(n));
        e.alu(Alu::Add, rax, kTime);
        e.mov(m_time, rax);
    };
    auto pc_plus = [&](Reg dst, s64 bytes) {  // dst = entry pc + bytes
        e.mov(dst, kPc);
        e.alu_imm(Alu::Add, dst, static_cast<s32>(bytes));
    };
    auto call_handler = [&](const Op& op) {
        e.mov(kArg0, kCpu);
        e.mov_imm64(kArg1, reinterpret_cast<u64>(jit_op(b, op)));
        e.mov_imm64(rax, reinterpret_cast<u64>(op.fn));
        e.call(rax);
    };

    // Instructions that can raise an exception, inline when they do not: the
    // operation itself where it cannot fault, and a jump to the handler
    // (`slow`) where it might. Loads and stores of RAM do the soft-TLB lookup
    // and the host access, exactly what load()/store() do on a hit with a
    // host page; a miss, an unaligned address, MMIO or a page holding
    // translated code goes to the handler. Returns false for instructions
    // with no inline form.
    const Mem m_store_count{kCpu, offset_of(this, &store_count_)};
    const Mem m_change_epoch{kCpu, offset_of(this, &change_epoch_)};
    const Mem m_scratch{kCpu, offset_of(this, &jit_scratch_)};
    const s32 log_host = offset_of(this, &store_log_[0].host);
    const s32 log_size = offset_of(this, &store_log_[0].size);
    const s32 log_value = offset_of(this, &store_log_[0].value);
    auto write_nz = [&](unsigned g, Reg src) {  // r0 stays zero
        if (g) write(g, src);
    };
    // 64-bit operations are reserved instructions in user and supervisor
    // mode with 32-bit addressing (ops64_enabled()).
    auto need64 = [&](std::vector<std::size_t>& slow) {
        if (addr64) return;
        e.mov(rax, m_entry_mode, false);
        e.alu_imm(Alu::Sub, rax, kSupervisor, false);
        e.alu_imm(Alu::Cmp, rax, kUser - kSupervisor, false);
        slow.push_back(e.jcc(Cond::Be));
    };
    auto emit_alu = [&](u32 w, std::vector<std::size_t>& slow) -> bool {
        const unsigned opc = w >> 26, rs = f_rs(w), rt = f_rt(w), rd = f_rd(w), sa = f_sa(w);
        const s32 simm = static_cast<s16>(w & 0xFFFF);
        if (opc == 0x08 || opc == 0x18 || opc == 0x19) {  // ADDI, DADDI, DADDIU
            const bool w64 = opc != 0x08;
            if (w64) need64(slow);
            ensure(rs);
            read(rax, rs, w64);
            e.alu_imm(Alu::Add, rax, simm, w64);
            if (opc != 0x19) slow.push_back(e.jcc(Cond::O));
            if (!rt) return true;
            if (w64) write(rt, rax);
            else sext32_to(rt);
            return true;
        }
        if (opc == 0x01 && rt >= 0x08 && rt <= 0x0E && rt != 0x0D) {  // TGEI, TGEIU, TLTI, TLTIU, TEQI, TNEI
            static constexpr Cond kTrap[] = {Cond::Ge, Cond::Ae, Cond::L, Cond::B, Cond::E, Cond::E, Cond::Ne};
            ensure(rs);
            read(rcx, rs);
            e.alu_imm(Alu::Cmp, rcx, simm);
            slow.push_back(e.jcc(kTrap[rt - 0x08]));  // the trap: the handler raises it
            return true;
        }
        if (opc != 0x00) return false;
        const unsigned funct = w & 63;
        switch (funct) {
            case 0x30:  // TGE
            case 0x31:  // TGEU
            case 0x32:  // TLT
            case 0x33:  // TLTU
            case 0x34:  // TEQ
            case 0x36: {  // TNE
                static constexpr Cond kTrap[] = {Cond::Ge, Cond::Ae, Cond::L, Cond::B, Cond::E, Cond::E, Cond::Ne};
                ensure(rs), ensure(rt);
                read(rcx, rs);
                alu_g(Alu::Cmp, rcx, rt);
                slow.push_back(e.jcc(kTrap[funct - 0x30]));  // the trap: the handler raises it
                return true;
            }
            case 0x14:  // DSLLV
            case 0x16:  // DSRLV
            case 0x17:  // DSRAV (x86 masks 64-bit shift counts to 6 bits, as MIPS does)
                need64(slow);
                ensure(rs), ensure(rt);
                read(rcx, rs);
                if (rd) shift64(funct == 0x14 ? Shift::Shl : funct == 0x16 ? Shift::Shr : Shift::Sar, rd, rt, true, 0);
                return true;
            case 0x38: case 0x3A: case 0x3B:  // DSLL, DSRL, DSRA
            case 0x3C: case 0x3E: case 0x3F:  // DSLL32, DSRL32, DSRA32
                need64(slow);
                ensure(rt);
                if (rd)
                    shift64((funct & 3) == 0 ? Shift::Shl : (funct & 3) == 2 ? Shift::Shr : Shift::Sar, rd, rt, false,
                            static_cast<u8>(sa + (funct >= 0x3C ? 32 : 0)));
                return true;
            case 0x20:  // ADD
            case 0x22:  // SUB
            case 0x2C:  // DADD
            case 0x2D:  // DADDU
            case 0x2E:  // DSUB
            case 0x2F: {  // DSUBU
                const bool w64 = funct >= 0x2C;
                if (w64) need64(slow);
                ensure(rs), ensure(rt);
                read(rax, rs, w64);
                alu_g((funct & 2) ? Alu::Sub : Alu::Add, rax, rt, w64);
                if (!(funct & 1)) slow.push_back(e.jcc(Cond::O));  // overflow: the handler raises it
                if (!rd) return true;
                if (w64) write(rd, rax);
                else sext32_to(rd);
                return true;
            }
            case 0x1C:  // DMULT
            case 0x1D:  // DMULTU
                need64(slow);
                ensure(rs), ensure(rt);
                read(rax, rs);
                read(rcx, rt);
                if (funct == 0x1C) e.imul1(rcx, true);
                else e.mul(rcx, true);
                e.mov(m_lo, rax);
                e.mov(m_hi, rdx);
                return true;
            case 0x1E:  // DDIV
            case 0x1F: {  // DDIVU: a zero divisor (and DDIV's one overflow) goes to the handler
                need64(slow);
                ensure(rs), ensure(rt);
                read(rcx, rt);
                e.test(rcx, rcx);
                slow.push_back(e.jcc(Cond::E));
                read(rax, rs);
                if (funct == 0x1E) {
                    e.alu_imm(Alu::Cmp, rcx, -1);
                    const std::size_t ok = e.jcc(Cond::Ne);
                    e.mov_imm64(rdx, u64{1} << 63);
                    e.alu(Alu::Cmp, rax, rdx);
                    slow.push_back(e.jcc(Cond::E));
                    e.bind(ok);
                    e.cdq(true);
                    e.idiv(rcx, true);
                } else {
                    e.alu(Alu::Xor, rdx, rdx, false);
                    e.div(rcx, true);
                }
                e.mov(m_lo, rax);
                e.mov(m_hi, rdx);
                return true;
            }
            default: return false;
        }
    };
    // COP1, inline for the common case: the coprocessor usable, Status.FR as
    // at translation, rounding to nearest with the inexact trap off, and
    // operands the FPU handles in hardware. The host computes the result
    // from a clean MXCSR; any flag but inexact, or a denormal result, leaves
    // the result to the handler, which works it out again exactly.
    jit_fp_control_ = host_fp_control();
    const Mem m_status{kCpu, offset_of(this, &status_)};
    const Mem m_fcr31{kCpu, offset_of(this, &fcr31)};
    const Mem m_fp_control{kCpu, offset_of(this, &jit_fp_control_)};
    constexpr u32 kCauseMask = 0x0003F000u, kCauseInexact = 0x1000u, kFlagInexact = 0x4u, kCond = 1u << 23;
    auto fp_usable = [&](std::vector<std::size_t>& slow) {
        e.mov(rax, m_status, false);
        e.alu_imm(Alu::And, rax, static_cast<s32>(sr::CU1 | sr::FR), false);
        e.alu_imm(Alu::Cmp, rax, static_cast<s32>(sr::CU1 | (fr ? sr::FR : 0)), false);
        slow.push_back(e.jcc(Cond::Ne));
    };
    auto fp_nearest = [&](std::vector<std::size_t>& slow) {  // RM nearest, inexact trap off
        e.mov(rax, m_fcr31, false);
        e.test_imm(rax, 0x3u | 0x80u);
        slow.push_back(e.jcc(Cond::Ne));
    };
    auto touch_ = [&] { e.alu_imm(Alu::Add, m_change_epoch, 1); };
    // `r` holds a zero or a normal number (else slow); rcx is clobbered.
    auto fp_ordinary = [&](Reg r, bool dbl, std::vector<std::size_t>& slow) {
        e.mov(rcx, r);
        e.shift_imm(Shift::Shl, rcx, 1, dbl);
        const std::size_t zero = e.jcc(Cond::E);
        e.shift_imm(Shift::Shr, rcx, dbl ? 53 : 24, dbl);
        e.alu_imm(Alu::Sub, rcx, 1, false);
        e.alu_imm(Alu::Cmp, rcx, dbl ? 0x7FD : 0xFD, false);
        slow.push_back(e.jcc(Cond::A));
        e.bind(zero);
    };
    // FPU register n (zero or normal) into rax, and as an xmm register.
    auto fp_operand = [&](unsigned n, bool dbl, unsigned scratch, std::vector<std::size_t>& slow) -> unsigned {
        const unsigned x = fp_src(n, dbl, scratch);
        e.movq_from_xmm(rax, x, dbl);
        fp_ordinary(rax, dbl, slow);
        return x;
    };
    auto fp_not_denormal = [&](Reg r, bool dbl, std::vector<std::size_t>& slow) {  // rcx is clobbered
        e.mov(rcx, r);
        e.shift_imm(Shift::Shl, rcx, 1, dbl);
        const std::size_t zero = e.jcc(Cond::E);
        e.shift_imm(Shift::Shr, rcx, dbl ? 53 : 24, dbl);
        slow.push_back(e.jcc(Cond::E));
        e.bind(zero);
    };
    // Runs one SSE operation from a clean MXCSR; any flag but inexact goes
    // to the handler. The flags stay in the scratch slot.
    auto fp_host = [&](auto&& op, std::vector<std::size_t>& slow) {
        e.ldmxcsr(m_fp_control);
        op();
        e.stmxcsr(m_scratch);
        e.test8_imm(m_scratch, 0x1F);  // invalid, denormal, divide by zero, overflow, underflow
        slow.push_back(e.jcc(Cond::Ne));
    };
    // fp_commit() of an inexact or exact result: Cause, and the sticky flag.
    auto fp_commit_inexact = [&] {
        e.mov(rdx, m_fcr31, false);
        e.alu_imm(Alu::And, rdx, static_cast<s32>(~kCauseMask), false);
        e.test8_imm(m_scratch, 0x20);
        const std::size_t exact = e.jcc(Cond::E);
        e.alu_imm(Alu::Or, rdx, static_cast<s32>(kCauseInexact | kFlagInexact), false);
        e.bind(exact);
        e.mov(m_fcr31, rdx, false);
    };
    auto fp_commit_exact = [&] { e.alu_imm(Alu::And, m_fcr31, static_cast<s32>(~kCauseMask), false); };
    auto emit_fpu = [&](u32 w, std::vector<std::size_t>& slow) -> bool {
        if ((w >> 26) != 0x11) return false;
        const unsigned fmt = f_rs(w), ft = f_rt(w), fs = f_rd(w), fd = f_sa(w), func = w & 63;
        switch (fmt) {
            case 0x00:  // MFC1
            case 0x01:  // DMFC1
                if (fmt == 0x01) need64(slow);
                fp_usable(slow);
                fp_read_gpr(rax, fs, fmt == 0x01);
                if (fmt == 0x00) e.movsxd(rax, rax);
                write_nz(ft, rax);
                touch_();
                return true;
            case 0x04:  // MTC1
            case 0x05:  // DMTC1
                if (fmt == 0x05) need64(slow);
                fp_usable(slow);
                ensure(ft);
                read(rax, ft, fmt == 0x05);
                fp_write_gpr(fs, fmt == 0x05, rax);
                touch_();
                return true;
            case 0x10:
            case 0x11:
            case 0x14:
                break;
            default: return false;
        }
        const bool dbl = fmt == 0x11;
        if (fmt == 0x14) {  // CVT.S.W, CVT.D.W
            if (func != 0x20 && func != 0x21) return false;
            const bool to_d = func == 0x21;
            fp_usable(slow);
            fp_nearest(slow);
            fp_read_gpr(rax, fs, false);
            e.cvt_from_int(to_d, 0, rax);
            if (to_d) {  // exact
                fp_commit_exact();
            } else {  // inexact if the single differs from the integer
                e.sse(Emitter::Sse::Cvt, false, 1, 0);
                e.cvt_from_int(true, 2, rax);
                e.ucomis(true, 1, 2);
                e.setcc(Cond::Ne, rcx);
                e.movzx8(rcx, rcx);
                e.mov(rdx, m_fcr31, false);
                e.alu_imm(Alu::And, rdx, static_cast<s32>(~kCauseMask), false);
                e.imul_imm(rcx, rcx, static_cast<s32>(kCauseInexact | kFlagInexact));
                e.alu(Alu::Or, rdx, rcx, false);
                e.mov(m_fcr31, rdx, false);
            }
            fp_write_xmm(fd, to_d, 0);
            touch_();
            return true;
        }
        // Exponent field of the value in `r` into rcx.
        auto exponent = [&](Reg r) {
            e.mov(rcx, r);
            e.shift_imm(Shift::Shl, rcx, 1, dbl);
            e.shift_imm(Shift::Shr, rcx, dbl ? 53 : 24, dbl);
        };
        const s32 exp_max = dbl ? 0x7FF : 0xFF;
        // A zero result passes only if one of `zeros` is zero; a nonzero
        // one needs an exponent of at least `min_exp` and below the maximum.
        auto result_check = [&](std::initializer_list<unsigned> zeros, s32 min_exp) {
            e.mov(rcx, rax);
            e.shift_imm(Shift::Shl, rcx, 1, dbl);
            const std::size_t zero = e.jcc(Cond::E);
            e.shift_imm(Shift::Shr, rcx, dbl ? 53 : 24, dbl);
            e.alu_imm(Alu::Cmp, rcx, min_exp, false);
            slow.push_back(e.jcc(Cond::B));
            e.alu_imm(Alu::Cmp, rcx, exp_max, false);
            slow.push_back(e.jcc(Cond::E));
            const std::size_t ok = e.jmp();
            e.bind(zero);
            std::vector<std::size_t> exact;
            for (unsigned x : zeros) {
                e.movq_from_xmm(rcx, x, dbl);
                e.shift_imm(Shift::Shl, rcx, 1, dbl);
                exact.push_back(e.jcc(Cond::E));
            }
            slow.push_back(e.jmp());
            for (std::size_t site : exact) e.bind(site);
            e.bind(ok);
        };
        // rcx = 1 if the error term in xmm `x` is nonzero.
        auto nonzero = [&](unsigned x) {
            e.movq_from_xmm(rcx, x, dbl);
            e.shift_imm(Shift::Shl, rcx, 1, dbl);
            e.setcc(Cond::Ne, rcx);
            e.movzx8(rcx, rcx);
        };
        // fp_commit() with rcx = 1 for an inexact result.
        auto commit_inexact_rcx = [&] {
            e.mov(rdx, m_fcr31, false);
            e.alu_imm(Alu::And, rdx, static_cast<s32>(~kCauseMask), false);
            e.imul_imm(rcx, rcx, static_cast<s32>(kCauseInexact | kFlagInexact));
            e.alu(Alu::Or, rdx, rcx, false);
            e.mov(m_fcr31, rdx, false);
        };
        // Exact results and rounding errors leave nothing in doubt at these
        // exponents (far from overflow, and from underflow of the error).
        const s32 min_result = dbl ? 64 : 32, min_dividend = dbl ? 128 : 64;
        // Without FMA, the exact error of a product comes from splitting each
        // factor in two halves (Dekker): x1 = x * y - p, with xmm2, xmm5-7.
        const Mem m_split{kCpu, dbl ? offset_of(this, &jit_split_d_) : offset_of(this, &jit_split_s_)};
        auto split_ok = [&](unsigned x) {  // splitting x cannot overflow
            e.movq_from_xmm(rcx, x, dbl);
            e.shift_imm(Shift::Shl, rcx, 1, dbl);
            e.shift_imm(Shift::Shr, rcx, dbl ? 53 : 24, dbl);
            e.alu_imm(Alu::Cmp, rcx, dbl ? 1995 : 240, false);
            slow.push_back(e.jcc(Cond::A));
        };
        auto two_product = [&](unsigned x, unsigned y, unsigned p) {
            using S = Emitter::Sse;
            auto half = [&](unsigned v, unsigned vh, unsigned vl) {  // v = vh + vl
                if (dbl) e.movq_load(vh, m_split);
                else e.movd_load(vh, m_split);
                e.sse(S::Mul, dbl, vh, v);
                e.movapd(vl, vh), e.sse(S::Sub, dbl, vl, v);
                e.sse(S::Sub, dbl, vh, vl);
                e.movapd(vl, v), e.sse(S::Sub, dbl, vl, vh);
            };
            half(x, 5, 6);
            half(y, 7, 2);
            e.movapd(1, 5), e.sse(S::Mul, dbl, 1, 7), e.sse(S::Sub, dbl, 1, p);  // xh * yh - p
            e.sse(S::Mul, dbl, 5, 2), e.sse(S::Add, dbl, 1, 5);                   // + xh * yl
            e.sse(S::Mul, dbl, 7, 6), e.sse(S::Add, dbl, 1, 7);                   // + xl * yh
            e.sse(S::Mul, dbl, 6, 2), e.sse(S::Add, dbl, 1, 6);                   // + xl * yl
        };
        if (func <= 0x01 || (func <= 0x04 && (jit_fma_ || kDekker))) {  // ADD, SUB; MUL, DIV, SQRT exactly
            fp_usable(slow);
            fp_nearest(slow);
            const unsigned a = fp_operand(fs, dbl, 4, slow);
            if (func == 0x04) {  // SQRT: a zero or a positive number not too small
                e.test(rax, rax);
                if (!dbl) e.test_imm(rax, 0x80000000u);
                const std::size_t positive = e.jcc(dbl ? Cond::Ge : Cond::E);
                e.mov(rcx, rax);
                e.shift_imm(Shift::Shl, rcx, 1, dbl);
                slow.push_back(e.jcc(Cond::Ne));  // the root of a negative number is invalid
                e.bind(positive);
                e.mov(rcx, rax);
                e.shift_imm(Shift::Shl, rcx, 1, dbl);
                const std::size_t zero = e.jcc(Cond::E);
                exponent(rax);
                e.alu_imm(Alu::Cmp, rcx, min_dividend, false);
                slow.push_back(e.jcc(Cond::B));
                e.bind(zero);
                e.movapd(0, a);
                e.sse(Emitter::Sse::Sqrt, dbl, 0, 0);
                if (jit_fma_) {
                    e.movapd(1, a);
                    e.fnmadd231(dbl, 1, 0, 0);  // a - r * r
                } else {  // (a - p) - (r * r - p), p = r * r rounded
                    e.movapd(8, 0), e.sse(Emitter::Sse::Mul, dbl, 8, 0);
                    two_product(0, 0, 8);
                    e.movapd(2, a), e.sse(Emitter::Sse::Sub, dbl, 2, 8);
                    e.sse(Emitter::Sse::Sub, dbl, 2, 1);
                    e.movapd(1, 2);
                }
            } else {
                const unsigned c = fp_operand(ft, dbl, 3, slow);
                if (func == 0x03) {  // DIV: a nonzero divisor, a dividend zero or not too small
                    e.mov(rcx, rax);
                    e.shift_imm(Shift::Shl, rcx, 1, dbl);
                    slow.push_back(e.jcc(Cond::E));
                    e.movq_from_xmm(rax, a, dbl);
                    e.mov(rcx, rax);
                    e.shift_imm(Shift::Shl, rcx, 1, dbl);
                    const std::size_t zero = e.jcc(Cond::E);
                    exponent(rax);
                    e.alu_imm(Alu::Cmp, rcx, min_dividend, false);
                    slow.push_back(e.jcc(Cond::B));
                    e.bind(zero);
                }
                static constexpr Emitter::Sse kOps[] = {Emitter::Sse::Add, Emitter::Sse::Sub, Emitter::Sse::Mul,
                                                        Emitter::Sse::Div};
                e.movapd(0, a);
                e.sse(kOps[func], dbl, 0, c);
                if (func == 0x00) {  // TwoSum: (a - (s - (s - a))) + (b - (s - a))
                    e.movapd(1, 0), e.sse(Emitter::Sse::Sub, dbl, 1, a);
                    e.movapd(2, c), e.sse(Emitter::Sse::Sub, dbl, 2, 1);
                    e.movapd(3, 0), e.sse(Emitter::Sse::Sub, dbl, 3, 1);
                    e.movapd(1, a), e.sse(Emitter::Sse::Sub, dbl, 1, 3);
                    e.sse(Emitter::Sse::Add, dbl, 1, 2);
                } else if (func == 0x01) {  // (a - (s - (s - a))) - (b + (s - a))
                    e.movapd(1, 0), e.sse(Emitter::Sse::Sub, dbl, 1, a);
                    e.movapd(2, c), e.sse(Emitter::Sse::Add, dbl, 2, 1);
                    e.movapd(3, 0), e.sse(Emitter::Sse::Sub, dbl, 3, 1);
                    e.movapd(1, a), e.sse(Emitter::Sse::Sub, dbl, 1, 3);
                    e.sse(Emitter::Sse::Sub, dbl, 1, 2);
                } else if (func == 0x02) {
                    if (jit_fma_) {
                        e.movapd(1, 0);
                        e.fmsub231(dbl, 1, a, c);  // a * b - p
                    } else {
                        split_ok(a), split_ok(c);
                        two_product(a, c, 0);
                    }
                } else if (jit_fma_) {
                    e.movapd(1, a);
                    e.fnmadd231(dbl, 1, 0, c);  // a - q * b
                } else {  // (a - p) - (q * b - p), p = q * b rounded
                    split_ok(0), split_ok(c);
                    e.movapd(8, 0), e.sse(Emitter::Sse::Mul, dbl, 8, c);
                    two_product(0, c, 8);
                    e.movapd(2, a), e.sse(Emitter::Sse::Sub, dbl, 2, 8);
                    e.sse(Emitter::Sse::Sub, dbl, 2, 1);
                    e.movapd(1, 2);
                }
            }
            e.movq_from_xmm(rax, 0, dbl);
            if (func <= 0x01) result_check({0}, 1);  // a zero sum is exact
            else if (func == 0x02) result_check({a, fp_src(ft, dbl, 3)}, min_result);
            else if (func == 0x03) result_check({a}, min_result);
            nonzero(1);
            commit_inexact_rcx();
            fp_write_xmm(fd, dbl, 0);
            touch_();
            return true;
        }
        if (func <= 0x04) {  // MUL, DIV, SQRT without FMA: the host's flags
            fp_usable(slow);
            fp_nearest(slow);
            const unsigned a = fp_operand(fs, dbl, 4, slow);
            const unsigned c = func == 0x04 ? a : fp_operand(ft, dbl, 3, slow);
            e.movapd(0, a);
            static constexpr Emitter::Sse kOps[] = {Emitter::Sse::Add, Emitter::Sse::Sub, Emitter::Sse::Mul,
                                                    Emitter::Sse::Div, Emitter::Sse::Sqrt};
            fp_host([&] { e.sse(kOps[func], dbl, 0, func == 0x04 ? 0 : c); }, slow);
            e.movq_from_xmm(rax, 0, dbl);
            fp_not_denormal(rax, dbl, slow);
            fp_commit_inexact();
            fp_write_xmm(fd, dbl, 0);
            touch_();
            return true;
        }
        switch (func) {
            case 0x06:  // MOV: a plain copy, Cause untouched
                fp_usable(slow);
                fp_write_xmm(fd, dbl, fp_src(fs, dbl, 0));
                touch_();
                return true;
            case 0x05:  // ABS
            case 0x07: {  // NEG
                fp_usable(slow);
                fp_operand(fs, dbl, 0, slow);
                if (dbl) e.mov_imm64(rcx, u64{1} << 63);
                else e.mov_imm32(rcx, 0x80000000u);
                if (func == 0x05) {
                    e.not_(rcx);
                    e.alu(Alu::And, rax, rcx, dbl);
                } else {
                    e.alu(Alu::Xor, rax, rcx, dbl);
                }
                fp_commit_exact();
                fp_write_gpr(fd, dbl, rax);
                touch_();
                return true;
            }
            case 0x20:  // CVT.S.D
            case 0x21: {  // CVT.D.S (exact)
                if ((func == 0x20) != dbl) return false;  // CVT.S.S and CVT.D.D are unimplemented
                fp_usable(slow);
                if (dbl) fp_nearest(slow);
                const unsigned x = fp_operand(fs, dbl, 4, slow);
                if (dbl) {  // a zero, or a normal single, inexact if it differs from the double
                    e.sse(Emitter::Sse::Cvt, true, 0, x);
                    e.movq_from_xmm(rax, 0, false);
                    e.mov(rcx, rax);
                    e.shift_imm(Shift::Shl, rcx, 1, false);
                    const std::size_t zero = e.jcc(Cond::E);
                    e.shift_imm(Shift::Shr, rcx, 24, false);
                    slow.push_back(e.jcc(Cond::E));
                    e.alu_imm(Alu::Cmp, rcx, 0xFF, false);
                    slow.push_back(e.jcc(Cond::E));
                    const std::size_t ok = e.jmp();
                    e.bind(zero);
                    e.movq_from_xmm(rcx, x, true);
                    e.shift_imm(Shift::Shl, rcx, 1, true);
                    slow.push_back(e.jcc(Cond::Ne));  // underflow
                    e.bind(ok);
                    e.sse(Emitter::Sse::Cvt, false, 1, 0);
                    e.ucomis(true, 1, x);
                    e.setcc(Cond::Ne, rcx);
                    e.movzx8(rcx, rcx);
                    commit_inexact_rcx();
                } else {
                    e.sse(Emitter::Sse::Cvt, false, 0, x);
                    fp_commit_exact();
                }
                fp_write_xmm(fd, !dbl, 0);
                touch_();
                return true;
            }
            case 0x0C:  // ROUND.W (to nearest even)
            case 0x0D:  // TRUNC.W
            case 0x24: {  // CVT.W (RM, here nearest)
                fp_usable(slow);
                fp_nearest(slow);
                const unsigned x = fp_operand(fs, dbl, 4, slow);
                e.cvt_to_int(dbl, func == 0x0D, rax, x);
                e.alu_imm(Alu::Cmp, rax, static_cast<s32>(0x80000000u), false);
                slow.push_back(e.jcc(Cond::E));  // out of range (or exactly -2^31: the handler sorts it out)
                e.cvt_from_int(true, 1, rax);    // inexact if the integer differs from the operand
                if (dbl) {
                    e.ucomis(true, 1, x);
                } else {
                    e.sse(Emitter::Sse::Cvt, false, 2, x);
                    e.ucomis(true, 1, 2);
                }
                e.setcc(Cond::Ne, rcx);
                e.movzx8(rcx, rcx);
                commit_inexact_rcx();
                fp_write_gpr(fd, false, rax);
                touch_();
                return true;
            }
            default:
                break;
        }
        if (func >= 0x30 && (fd >> 2) == 0) {  // C.cond, condition code 0: no NaN operand, so ordered
            const unsigned cond = func & 6;  // less (4), equal (2)
            fp_usable(slow);
            auto not_nan = [&](unsigned n, unsigned scratch) {
                const unsigned x = fp_src(n, dbl, scratch);
                e.movq_from_xmm(rcx, x, dbl);
                e.shift_imm(Shift::Shl, rcx, 1, dbl);
                if (dbl) e.mov_imm64(rdx, 0xFFE0000000000000ull);
                else e.mov_imm32(rdx, 0xFF000000u);
                e.alu(Alu::Cmp, rcx, rdx, dbl);
                slow.push_back(e.jcc(Cond::A));
                return x;
            };
            const unsigned xa = not_nan(fs, 0), xb = not_nan(ft, 1);
            e.alu(Alu::Xor, rax, rax, false);
            if (cond) {
                e.ucomis(dbl, xa, xb);
                e.setcc(cond == 6 ? Cond::Be : cond == 4 ? Cond::B : Cond::E, rax);
                e.movzx8(rax, rax);
                e.shift_imm(Shift::Shl, rax, 23, false);
            }
            e.mov(rdx, m_fcr31, false);
            e.alu_imm(Alu::And, rdx, static_cast<s32>(~(kCauseMask | kCond)), false);
            e.alu(Alu::Or, rdx, rax, false);
            e.mov(m_fcr31, rdx, false);
            touch_();
            return true;
        }
        return false;
    };
    auto emit_memory = [&](u32 w, std::vector<std::size_t>& slow) -> bool {
        const unsigned opc = w >> 26, rs = f_rs(w), rt = f_rt(w);
        unsigned size;
        bool is_store = false, merge = false, w64 = false, fp = false;
        switch (opc) {
            case 0x31: size = 4, fp = true; break;                   // LWC1
            case 0x35: size = 8, fp = true; break;                   // LDC1
            case 0x39: size = 4, fp = true, is_store = true; break;  // SWC1
            case 0x3D: size = 8, fp = true, is_store = true; break;  // SDC1
            case 0x20:
            case 0x24: size = 1; break;  // LB, LBU
            case 0x21:
            case 0x25: size = 2; break;  // LH, LHU
            case 0x23: size = 4; break;  // LW
            case 0x27: size = 4, w64 = true; break;  // LWU
            case 0x37: size = 8, w64 = true; break;  // LD
            case 0x28: size = 1, is_store = true; break;  // SB
            case 0x29: size = 2, is_store = true; break;  // SH
            case 0x2B: size = 4, is_store = true; break;  // SW
            case 0x3F: size = 8, is_store = true, w64 = true; break;  // SD
            case 0x22: case 0x26: size = 4, merge = true; break;  // LWL, LWR
            case 0x2A: case 0x2E: size = 4, merge = true, is_store = true; break;  // SWL, SWR
            case 0x1A: case 0x1B: size = 8, merge = true, w64 = true; break;  // LDL, LDR
            case 0x2C: case 0x2D: size = 8, merge = true, is_store = true, w64 = true; break;  // SDL, SDR
            default: return false;
        }
        const bool right = opc == 0x26 || opc == 0x2E || opc == 0x1B || opc == 0x2D;
        if (w64) need64(slow);
        if (fp) fp_usable(slow);
        // Operands are loaded before the paths split, so both agree.
        ensure(rs);
        if ((is_store || merge) && !fp) ensure(rt);
        const s32 table = is_store ? kTableBytes : 0;
        read(rax, rs);
        e.alu_imm(Alu::Add, rax, static_cast<s16>(w & 0xFFFF));
        if (!addr64) e.movsxd(rax, rax);
        if (size > 1 && !merge) {
            e.test_imm(rax, size - 1);
            slow.push_back(e.jcc(Cond::Ne));
        }
        e.mov(rcx, rax);
        e.shift_imm(Shift::Shr, rcx, 12, true);  // tag
        e.mov(rdx, rcx);
        e.alu(Alu::Or, rcx, m_asid_tag);
        e.alu_imm(Alu::And, rdx, kFastEntries - 1, false);
        e.shift_imm(Shift::Shl, rdx, 5, false);
        e.alu(Alu::Add, rdx, kTlb);  // the entry, less `table`
        e.alu(Alu::Cmp, rcx, Mem{rdx, table + static_cast<s32>(offsetof(FastEntry, tag))});
        slow.push_back(e.jcc(Cond::Ne));
        e.alu_imm(Alu::And, rax, 0xFFF, false);
        // Merges: rcx = the shift, 8 * the byte index (counted from the other
        // end for the right-hand forms); rax = offset of the containing unit.
        auto merge_shift = [&] {
            e.mov(rcx, rax);
            e.alu_imm(Alu::And, rcx, size - 1, false);
            if (right) e.alu_imm(Alu::Xor, rcx, size - 1, false);
            e.shift_imm(Shift::Shl, rcx, 3, false);
            e.alu_imm(Alu::And, rax, static_cast<s32>(0xFFF & ~(size - 1)), false);
        };
        if (!is_store) {
            e.mov(rdx, Mem{rdx, static_cast<s32>(offsetof(FastEntry, host))});
            e.test(rdx, rdx);
            slow.push_back(e.jcc(Cond::E));
            if (merge) merge_shift();
            e.alu(Alu::Add, rdx, rax);
            if (fp) {
                e.mov(rax, Mem{rdx, 0}, size == 8);
                if (size == 8) e.bswap64(rax);
                else e.bswap32(rax);
                fp_write_gpr(rt, size == 8, rax);
                touch_();
                return true;
            }
            if (rt == 0) return true;  // the access still happens, for its exceptions
            const Mem p{rdx, 0};
            if (merge) {
                const bool d = size == 8;
                e.mov(rax, p, d);
                if (d) e.bswap64(rax);
                else e.bswap32(rax);
                if (!right) {  // (m << sh) | (rt & ((1 << sh) - 1))
                    e.shift_cl(Shift::Shl, rax, d);
                    e.mov_imm32(rdx, 1);
                    e.shift_cl(Shift::Shl, rdx, true);
                    e.alu_imm(Alu::Sub, rdx, 1);
                    alu_g(Alu::And, rdx, rt, d);
                    e.alu(Alu::Or, rax, rdx, d);
                    if (!d) e.movsxd(rax, rax);
                } else if (d) {  // (m >> sh) | (rt & ~(~0 >> sh))
                    e.shift_cl(Shift::Shr, rax, true);
                    e.mov_imm64(rdx, ~u64{0});
                    e.shift_cl(Shift::Shr, rdx, true);
                    e.not_(rdx);
                    alu_g(Alu::And, rdx, rt);
                    e.alu(Alu::Or, rax, rdx);
                } else {  // LWR: the word merged; a full word sign-extends, a part keeps bits 63:32
                    e.shift_cl(Shift::Shr, rax, false);
                    e.mov_imm32(rdx, 0xFFFFFFFFu);
                    e.shift_cl(Shift::Shr, rdx, false);
                    e.alu_imm(Alu::Xor, rdx, -1, false);
                    alu_g(Alu::And, rdx, rt, false);
                    e.alu(Alu::Or, rax, rdx, false);
                    e.test(rcx, rcx);
                    const std::size_t part = e.jcc(Cond::Ne);
                    e.movsxd(rax, rax);
                    const std::size_t done = e.jmp();
                    e.bind(part);
                    read(rdx, rt);
                    e.shift_imm(Shift::Shr, rdx, 32, true);
                    e.shift_imm(Shift::Shl, rdx, 32, true);
                    e.alu(Alu::Or, rax, rdx);
                    e.bind(done);
                }
                write(rt, rax);
                return true;
            }
            switch (opc) {
                case 0x20: e.movsx8(rax, p); break;
                case 0x24: e.movzx8(rax, p); break;
                case 0x21:
                case 0x25:
                    e.movzx16(rax, p);
                    e.rol16_8(rax);
                    if (opc == 0x21) e.movsx16(rax, rax);
                    else e.movzx16(rax, rax);
                    break;
                case 0x37:
                    e.mov(rax, p);
                    e.bswap64(rax);
                    break;
                default:
                    e.mov(rax, p, false);
                    e.bswap32(rax);
                    if (opc == 0x23) {
                        sext32_to(rt);
                        return true;
                    }
                    break;  // LWU leaves it zero-extended
            }
            write(rt, rax);
            return true;
        }
        e.mov(rcx, Mem{rdx, table + static_cast<s32>(offsetof(FastEntry, host))});
        e.test(rcx, rcx);
        slow.push_back(e.jcc(Cond::E));
        e.mov(rdx, Mem{rdx, table + static_cast<s32>(offsetof(FastEntry, code))});
        e.movzx8(rdx, Mem{rdx, 0});
        e.test(rdx, rdx);
        slow.push_back(e.jcc(Cond::Ne));  // translated code lives in this page
        if (merge) {  // merge_store(): (m & keep) | value, in memory byte order
            const bool d = size == 8;
            e.mov(rdx, rax);
            e.alu_imm(Alu::And, rdx, static_cast<s32>(0xFFF & ~(size - 1)), false);
            e.alu(Alu::Add, rdx, rcx);
            e.mov(m_scratch, rdx);  // the containing unit
            e.mov(rcx, rax);
            e.alu_imm(Alu::And, rcx, size - 1, false);
            if (right) e.alu_imm(Alu::Xor, rcx, size - 1, false);
            e.shift_imm(Shift::Shl, rcx, 3, false);
            // Left forms: value rt >> sh, keep ~(~0 >> sh); right forms:
            // value rt << sh, keep ~(~0 << sh).
            const Shift dir = right ? Shift::Shl : Shift::Shr;
            read(rax, rt, d);
            e.shift_cl(dir, rax, d);
            if (d) e.mov_imm64(rdx, ~u64{0});
            else e.mov_imm32(rdx, 0xFFFFFFFFu);
            e.shift_cl(dir, rdx, d);
            e.alu_imm(Alu::Xor, rdx, -1, d);
            e.mov(rcx, m_scratch);
            if (d) e.bswap64(rax), e.bswap64(rdx);
            else e.bswap32(rax), e.bswap32(rdx);
            e.alu(Alu::And, rdx, Mem{rcx, 0}, d);
            e.alu(Alu::Or, rax, rdx, d);
            e.mov(Mem{rcx, 0}, rax, d);
            e.alu_imm(Alu::Add, m_change_epoch, 1);  // touch()
            return true;
        }
        e.alu(Alu::Add, rcx, rax);  // the host address
        // log_store()
        e.mov(rdx, m_store_count);
        e.mov(rax, rdx);
        e.alu_imm(Alu::Add, rax, 1);
        e.mov(m_store_count, rax);
        e.alu_imm(Alu::And, rdx, (1u << kStoreLogBits) - 1, false);
        e.imul_imm(rdx, rdx, static_cast<s32>(sizeof(StoreRec)));
        e.alu(Alu::Add, rdx, kCpu);
        e.mov(Mem{rdx, log_host}, rcx);
        e.mov_store_imm32(Mem{rdx, log_size}, size);
        e.mov_store_imm(Mem{rdx, log_value}, 0);
        if (fp) fp_read_gpr(rax, rt, size == 8);
        else read(rax, rt, size == 8);
        switch (size) {
            case 1: e.mov8(Mem{rcx, 0}, rax); break;
            case 2:
                e.rol16_8(rax);
                e.mov16(Mem{rcx, 0}, rax);
                break;
            case 8:
                e.bswap64(rax);
                e.mov(Mem{rcx, 0}, rax);
                break;
            default:
                e.bswap32(rax);
                e.mov(Mem{rcx, 0}, rax, false);
                break;
        }
        if (fp) touch_();
        return true;
    };

    // Register operations, inline; others of the kind call their handler
    // (they observe nothing, so they need no bookkeeping).
    auto emit_plain = [&](const Op& op) {
        const u32 w = op.inst;
        // Inline register operations; dest 0 was decoded to a no-op.
        const unsigned rs = f_rs(w), rt = f_rt(w), rd = f_rd(w), sa = f_sa(w);
        const s32 simm = static_cast<s16>(w & 0xFFFF);
        const s32 zimm = static_cast<s32>(w & 0xFFFF);
        const unsigned opc = w >> 26, funct = w & 63;
        auto alu32_store = [&](unsigned dest) { sext32_to(dest); };
        // A 64-bit operation of a and b into dest, straight in dest's cache
        // register where it has one (b is a register, or an immediate when
        // b_reg is false).
        auto alu64 = [&](Alu aop, unsigned d, unsigned a, unsigned b_g, bool b_reg, s32 imm, bool commutes) {
            if (!cached(d)) {
                read(rax, a);
                if (b_reg) alu_g(aop, rax, b_g);
                else e.alu_imm(aop, rax, imm);
                return write(d, rax), rax;
            }
            const Reg h = host_of[d];
            if (b_reg && commutes && b_g == d && a != d) std::swap(a, b_g);
            if (a != d) {
                if (b_reg && b_g == d) {  // d = a op d, not commutative
                    read(rax, a);
                    alu_g(aop, rax, b_g);
                    e.mov(h, rax);
                } else {
                    read(h, a);
                    if (b_reg) alu_g(aop, h, b_g);
                    else e.alu_imm(aop, h, imm);
                }
            } else {
                ensure(d);
                if (b_reg) alu_g(aop, h, b_g);
                else e.alu_imm(aop, h, imm);
            }
            loaded |= u64{1} << d;
            dirty |= u64{1} << d;
            return h;
        };
        auto set_flag = [&](Cond c, unsigned dest) {
            e.setcc(c, rax);
            e.movzx8(rax, rax);
            write(dest, rax);
        };
        // MTHI, MTLO, MULT(U) and DIV(U) write HI/LO rather than rd.
        const bool is_mult = opc == 0x00 && (funct == 0x11 || funct == 0x13 || (funct >= 0x18 && funct <= 0x1B));
        const unsigned dest = opc == 0x00 ? rd : rt;
        bool done = true;
        if (dest == 0 && !is_mult) {  // writes r0: decoded as a no-op
        } else if (opc == 0x00) {
            switch (funct) {
                case 0x00:  // SLL
                case 0x02:  // SRL
                case 0x03:  // SRA
                    read(rax, rt, false);
                    e.shift_imm(funct == 0x00 ? Shift::Shl : funct == 0x02 ? Shift::Shr : Shift::Sar, rax,
                                static_cast<u8>(sa), false);
                    alu32_store(rd);
                    break;
                case 0x04:  // SLLV
                case 0x06:  // SRLV
                case 0x07:  // SRAV (x86 masks 32-bit shift counts to 5 bits, as MIPS does)
                    read(rcx, rs, false);
                    read(rax, rt, false);
                    e.shift_cl(funct == 0x04 ? Shift::Shl : funct == 0x06 ? Shift::Shr : Shift::Sar, rax, false);
                    alu32_store(rd);
                    break;
                case 0x10: e.mov(rax, m_hi), write(rd, rax); break;  // MFHI
                case 0x12: e.mov(rax, m_lo), write(rd, rax); break;  // MFLO
                case 0x11:                                           // MTHI
                case 0x13:                                           // MTLO
                    read(rax, rs);
                    e.mov(funct == 0x11 ? m_hi : m_lo, rax);
                    break;
                case 0x1A:  // DIV
                case 0x1B: {  // DIVU: a zero divisor (and DIV's one overflow) through the handler
                    std::vector<std::size_t> rare;
                    ensure(rs), ensure(rt);
                    read(rcx, rt, false);
                    e.test(rcx, rcx);
                    rare.push_back(e.jcc(Cond::E));
                    read(rax, rs, false);
                    if (funct == 0x1A) {
                        e.alu_imm(Alu::Cmp, rcx, -1, false);
                        const std::size_t ok = e.jcc(Cond::Ne);
                        e.alu_imm(Alu::Cmp, rax, static_cast<s32>(0x80000000u), false);
                        rare.push_back(e.jcc(Cond::E));
                        e.bind(ok);
                        e.cdq(false);
                        e.idiv(rcx, false);
                    } else {
                        e.alu(Alu::Xor, rdx, rdx, false);
                        e.div(rcx, false);
                    }
                    e.movsxd(rax, rax);
                    e.mov(m_lo, rax);
                    e.movsxd(rdx, rdx);
                    e.mov(m_hi, rdx);
                    const std::size_t to_end = e.jmp();
                    for (std::size_t site : rare) e.bind(site);
                    flush(dirty);
                    call_handler(op);
                    restore(loaded);
                    e.bind(to_end);
                    break;
                }
                case 0x18:                                           // MULT
                    movsxd_g(rax, rs);
                    movsxd_g(rcx, rt);
                    e.imul(rax, rcx);
                    e.movsxd(rcx, rax);
                    e.mov(m_lo, rcx);
                    e.shift_imm(Shift::Sar, rax, 32, true);
                    e.mov(m_hi, rax);
                    break;
                case 0x19:  // MULTU
                    read(rax, rs, false);
                    read(rcx, rt, false);
                    e.imul(rax, rcx);
                    e.movsxd(rcx, rax);
                    e.mov(m_lo, rcx);
                    e.shift_imm(Shift::Shr, rax, 32, true);
                    e.movsxd(rax, rax);
                    e.mov(m_hi, rax);
                    break;
                case 0x21:  // ADDU
                case 0x23:  // SUBU
                    read(rax, rs, false);
                    alu_g(funct == 0x21 ? Alu::Add : Alu::Sub, rax, rt, false);
                    alu32_store(rd);
                    break;
                case 0x24:  // AND
                case 0x25:  // OR
                case 0x26:  // XOR
                case 0x27: {  // NOR
                    const Reg r = alu64(funct == 0x24 ? Alu::And : funct == 0x26 ? Alu::Xor : Alu::Or, rd, rs, rt,
                                        true, 0, true);
                    if (funct == 0x27) {
                        if (r == rax) {  // written already: complement in memory
                            e.not_(rax);
                            write(rd, rax);
                        } else {
                            e.not_(r);
                        }
                    }
                    break;
                }
                case 0x2A:  // SLT
                case 0x2B:  // SLTU
                    ensure(rt);
                    read(rcx, rs);
                    e.alu(Alu::Xor, rax, rax, false);
                    alu_g(Alu::Cmp, rcx, rt);
                    set_flag(funct == 0x2A ? Cond::L : Cond::B, rd);
                    break;
                default: done = false; break;
            }
        } else {
            switch (opc) {
                case 0x09:  // ADDIU
                    read(rax, rs, false);
                    e.alu_imm(Alu::Add, rax, simm, false);
                    alu32_store(rt);
                    break;
                case 0x0A:  // SLTI
                case 0x0B:  // SLTIU (the immediate is sign-extended, then compared unsigned)
                    read(rcx, rs);
                    e.alu(Alu::Xor, rax, rax, false);
                    e.alu_imm(Alu::Cmp, rcx, simm);
                    set_flag(opc == 0x0A ? Cond::L : Cond::B, rt);
                    break;
                case 0x0C:  // ANDI
                case 0x0D:  // ORI
                case 0x0E:  // XORI
                    alu64(opc == 0x0C ? Alu::And : opc == 0x0D ? Alu::Or : Alu::Xor, rt, rs, 0, false, zimm, false);
                    break;
                case 0x0F: write_imm(rt, static_cast<s32>(static_cast<u32>(zimm) << 16)); break;  // LUI
                default: done = false; break;
            }
        }
        if (!done) {
            flush(dirty);
            call_handler(op);
            loaded = dirty = 0;
        }
    };

    // The block of the region being emitted.
    const Block* cur = &b;
    std::size_t cur_k = 0;
    struct Exit {
        std::size_t site;
        u32 completed;
        u64 flush;  // cached registers to write back
        std::size_t k;  // in rb[k]
    };
    std::vector<Exit> exits;
    auto exit_if = [&](Cond c, u32 completed, u64 mask) { exits.push_back({e.jcc(c), completed, mask, cur_k}); };
    // Leave when run_block() would after ops[i]: a device or CP0 access, the
    // block's memory written, a confirmed idle loop, or control leaving the
    // straight line (an exception, or a taken branch after its delay slot).
    // It follows a handler call, so memory holds every guest register.
    auto exit_checks = [&](u32 i) {
        e.movzx8(rax, m_io);
        e.alu8(Alu::Or, rax, m_invalidated);
        e.alu8(Alu::Or, rax, m_idle_ready);  // ZF: all three clear
        exit_if(Cond::Ne, i + 1, 0);
        pc_plus(rax, 4 * i + 4);
        e.alu(Alu::Cmp, rax, m_pc);
        exit_if(Cond::Ne, i + 1, 0);
    };
    // The delay slot ran without bookkeeping: move on to the branch's choice.
    auto leave_delay_slot = [&] {
        e.mov(rax, m_npc);
        e.mov(m_pc, rax);
        e.alu_imm(Alu::Add, rax, 4);
        e.mov(m_npc, rax);
        e.mov_store_imm8(m_next_delay, 0);
    };

    // Guest RAM need not be page-aligned in host memory: page positions come
    // from the guest address.
    u32 page_offset = b.offset;  // of the block being emitted
    const u8* const page_host = b.host - page_offset;
    // Successor blocks this one can jump straight into: pc deltas from the
    // block's entry, within the same page.
    std::vector<s64> successors;
    auto add_successor = [&](s64 delta) {
        const s64 off = static_cast<s64>(page_offset) + delta;
        if (off >= 0 && off < 0x1000) successors.push_back(delta);
    };

    bool synced = true;  // pc and npc_ are up to date after the last op

    // Branches and jumps, inline: the same state as branch()/jump() (pc at
    // the delay slot, npc_ at the next instruction after it, next_delay_),
    // and for a taken branch backwards the idle-loop bookkeeping of
    // backward_branch(): its cheap cooldown case inline, the rest by a call.
    auto emit_branch = [&](const Op& op, u32 i) -> bool {
        const u32 w = op.inst;
        const unsigned opc = w >> 26, rs = f_rs(w), rt = f_rt(w);
        const s64 simm = static_cast<s16>(w & 0xFFFF);
        enum Kind { kCompare, kSign, kFpCond, kJump, kJumpReg } kind;
        Cond not_taken = Cond::E;
        bool likely = false;
        switch (opc) {
            case 0x04: case 0x14: kind = kCompare, not_taken = Cond::Ne; break;  // BEQ(L)
            case 0x05: case 0x15: kind = kCompare, not_taken = Cond::E; break;   // BNE(L)
            case 0x06: case 0x16: kind = kSign, not_taken = Cond::G; break;      // BLEZ(L)
            case 0x07: case 0x17: kind = kSign, not_taken = Cond::Le; break;     // BGTZ(L)
            case 0x01:
                switch (rt) {
                    case 0x00: case 0x02: kind = kSign, not_taken = Cond::Ge; break;  // BLTZ(L)
                    case 0x01: case 0x03: kind = kSign, not_taken = Cond::L; break;   // BGEZ(L)
                    default: return false;
                }
                likely = rt >= 2;
                break;
            case 0x11:  // BC1F, BC1T, BC1FL, BC1TL
                if (rs != 0x08 || (rt >> 2) != 0) return false;  // condition code 0 only
                kind = kFpCond, not_taken = (rt & 1) ? Cond::E : Cond::Ne, likely = (rt & 2) != 0;
                break;
            case 0x02: case 0x03: kind = kJump; break;  // J, JAL
            case 0x00:
                if ((w & 63) != 0x08 && (w & 63) != 0x09) return false;
                kind = kJumpReg;  // JR, JALR
                break;
            default: return false;
        }
        if (opc >= 0x14 && opc <= 0x17) likely = true;

        if (kind == kFpCond) {  // the FPU unusable: leave, for the handler to raise it
            e.test8_imm(Mem{kCpu, offset_of(this, &status_) + 3}, static_cast<u8>(sr::CU1 >> 24));
            const std::size_t usable = e.jcc(Cond::Ne);
            pc_plus(rax, 4 * i);
            e.mov(m_pc, rax);
            set_time(i);
            exits.push_back({e.jmp(), i, dirty, cur_k});
            e.bind(usable);
            e.alu_imm(Alu::Add, Mem{kCpu, offset_of(this, &change_epoch_)}, 1);  // touch()
        }
        pc_plus(rax, 4 * i + 4);
        e.mov(m_pc, rax);
        std::size_t to_not_taken = 0;
        if (kind == kFpCond) {
            e.test8_imm(Mem{kCpu, offset_of(this, &fcr31) + 2}, 0x80);  // the condition, FCR31 bit 23
            to_not_taken = e.jcc(not_taken);
        } else if (kind == kCompare) {
            read(rcx, rs);
            alu_g(Alu::Cmp, rcx, rt);
            to_not_taken = e.jcc(not_taken);
        } else if (kind == kSign) {
            read(rcx, rs);
            e.test(rcx, rcx);
            to_not_taken = e.jcc(not_taken);
        }

        // Taken: the target in rcx.
        const s64 delta = 4 * static_cast<s64>(i) + 4 + 4 * simm;
        if (kind == kCompare || kind == kSign || kind == kFpCond) {
            pc_plus(rcx, delta);
            add_successor(delta);
        } else if (kind == kJump) {
            pc_plus(rcx, 4 * i + 4);
            e.alu_imm(Alu::And, rcx, static_cast<s32>(0xF0000000u));
            e.alu_imm(Alu::Or, rcx, static_cast<s32>((w & 0x03FFFFFF) << 2));
            if (opc == 0x03) {
                pc_plus(rax, 4 * i + 8);
                write(31, rax);
            }
        } else {
            read(rcx, rs);
            const unsigned rd = f_rd(w);
            if ((w & 63) == 0x09 && rd != 0) {
                pc_plus(rax, 4 * i + 8);
                write(rd, rax);
            }
        }
        e.mov(m_npc, rcx);
        e.mov_store_imm8(m_next_delay, 1);
        const bool maybe_backward = ((kind == kCompare || kind == kSign || kind == kFpCond) && simm < 0) || opc == 0x02;
        if (maybe_backward) {
            const unsigned idx = ((page_offset + 4 * i) >> 2) & ((1u << kLoopTableBits) - 1);
            const Mem m_loop_pc{kCpu, offset_of(this, &loops_[idx].branch_pc)};
            const Mem m_cooldown{kCpu, offset_of(this, &loops_[idx].cooldown)};
            std::vector<std::size_t> done;
            pc_plus(rax, 4 * i);  // cur_pc_
            if (opc == 0x02) {
                e.alu(Alu::Cmp, rcx, rax);
                done.push_back(e.jcc(Cond::A));  // forwards
            }
            e.alu(Alu::Cmp, rax, m_loop_pc);
            const std::size_t visit1 = e.jcc(Cond::Ne);
            e.mov(rdx, m_cooldown, false);
            e.test(rdx, rdx);
            const std::size_t visit2 = e.jcc(Cond::E);
            e.alu_imm(Alu::Sub, rdx, 1, false);
            e.mov(m_cooldown, rdx, false);
            done.push_back(e.jcc(Cond::Ne));
            e.mov_store_imm(m_loop_pc, -1);  // cooldown over: start afresh
            done.push_back(e.jmp());
            e.bind(visit1);
            e.bind(visit2);
            e.mov(m_cur_pc, rax);
            set_time(i + 1);
            flush(dirty);  // the loop bookkeeping compares registers
            e.mov(kArg1, rcx);  // the target (before rcx may become the first argument)
            e.mov(kArg0, kCpu);
            e.mov_imm64(rax, reinterpret_cast<u64>(&R4000::jit_backward_branch));
            e.call(rax);
            e.movzx8(rax, m_idle_ready);
            e.test(rax, rax);
            exit_if(Cond::Ne, i + 1, 0);
            restore(loaded);
            for (std::size_t site : done) e.bind(site);
        }

        if (to_not_taken) {
            const std::size_t to_done = e.jmp();
            e.bind(to_not_taken);
            if (likely) {  // the delay slot is skipped: leave, as run_block() does
                pc_plus(rax, 4 * i + 8);
                e.mov(m_pc, rax);
                e.alu_imm(Alu::Add, rax, 4);
                e.mov(m_npc, rax);
                set_time(i + 1);
                exits.push_back({e.jmp(), i + 1, dirty, cur_k});
            } else {
                pc_plus(rax, 4 * i + 8);
                e.mov(m_npc, rax);
                e.mov_store_imm8(m_next_delay, 1);
            }
            e.bind(to_done);
        }
        return true;
    };
    // The bookkeeping of run_block(), then the handler, for ops[i]; `mask`:
    // the cached registers to write back first.
    auto emit_handler = [&](u32 i, bool delay_slot, const Op& op, u64 mask) {
        flush(mask);
        set_time(i + 1);
        pc_plus(rax, 4 * i);
        e.mov(m_cur_pc, rax);
        if (delay_slot) {
            e.mov_store_imm8(m_in_delay, 1);
            e.mov_store_imm8(m_next_delay, 0);
            e.mov(rax, m_npc);
            e.mov(m_pc, rax);
            e.alu_imm(Alu::Add, rax, 4);
            e.mov(m_npc, rax);
        } else {
            e.mov_store_imm8(m_in_delay, 0);  // next_delay_ is already clear
            pc_plus(rax, 4 * i + 4);
            e.mov(m_pc, rax);
            e.alu_imm(Alu::Add, rax, 4);
            e.mov(m_npc, rax);
        }
        e.mov_store_imm8(m_io, 0);
        call_handler(op);
        e.mov_store_imm(gpr_at(0), 0);
        // The last op is checked by the block's end. (A branch continues
        // into its delay slot; a branch-likely not taken does not.)
        if (i + 1 < cur->count) exit_checks(i);
    };
    // An inline operation's way to its handler, emitted after the block.
    struct Cold {
        std::vector<std::size_t> slow;
        u32 i = 0;
        bool delay_slot = false;
        const Op* op = nullptr;
        u64 dirty_before = 0, loaded_after = 0;
        std::size_t resume = 0;
        std::size_t k = 0;  // in rb[k]
    };
    std::vector<Cold> colds;
    // Each block of the region in turn: its code, then the way out of it,
    // to another block of the region or the code that links or returns.
    std::vector<std::size_t> label(rb.size(), 0);
    std::vector<std::vector<std::size_t>> to_label(rb.size()), to_tail(rb.size());
    for (std::size_t k = 0; k < rb.size(); ++k) {
        cur = rb[k], cur_k = k, page_offset = cur->offset;
        const Block& blk = *cur;
        label[k] = k == 0 ? body : e.size();
        for (std::size_t site : to_label[k]) e.bind(site);
        if (k > 0) loaded = alloc, dirty = written;  // as every way in leaves them
        successors.clear();
        synced = true;
        bool last_handler = false;  // the last op ran its handler in the straight line
        for (u32 i = 0; i < blk.count; ++i) {
            const Op& op = blk.ops[i];
            const u32 w = op.inst;
            const bool delay_slot = i > 0 && is_branch_word(blk.ops[i - 1].inst);
            bool plain;
            decode(w, plain);
            last_handler = false;

            if (plain) {
                emit_plain(op);
                if (delay_slot) leave_delay_slot();
                synced = delay_slot;
            } else if (!delay_slot && emit_branch(op, i)) {
                synced = true;
            } else {
                // The inline form first, if there is one; its way to the handler
                // is placed after the region's code, out of the straight line.
                Cold cold;
                cold.dirty_before = dirty;
                const bool fast = emit_alu(w, cold.slow) || emit_fpu(w, cold.slow) || emit_memory(w, cold.slow);
                if (fast) {
                    if (delay_slot) leave_delay_slot();
                    cold.i = i, cold.delay_slot = delay_slot, cold.op = &op, cold.loaded_after = loaded, cold.k = k;
                    cold.resume = e.size();
                    colds.push_back(std::move(cold));
                    synced = delay_slot;
                    continue;
                }
                emit_handler(i, delay_slot, op, dirty);
                loaded = dirty = 0;
                synced = true;
                last_handler = true;
            }
        }
        if (!synced) {  // plain operations or inline accesses ran last
            pc_plus(rax, 4 * blk.count);
            e.mov(m_pc, rax);
            e.alu_imm(Alu::Add, rax, 4);
            e.mov(m_npc, rax);
        }
        if (!is_branch_word(blk.ops[blk.count - 1].inst)) add_successor(4 * static_cast<s64>(blk.count));
        std::sort(successors.begin(), successors.end());
        successors.erase(std::unique(successors.begin(), successors.end()), successors.end());

        // Block linking: run the next block's code straight away when
        // run_block() would run it next with nothing checked in between. The
        // pass must fit below the safe time (run_block() stores it in
        // jit_safe_), no access may have happened that needs the checks, and
        // the mode must be unchanged. Within the region the registers stay
        // as they are and only the time needs checking, unless a handler
        // ran last (an inline operation's handler checks after itself).
        // Elsewhere in the page the translation is known to be the same;
        // beyond, the fetch soft-TLB must map the pc as run_block() would
        // find it.
        std::vector<std::size_t> to_finish_dirty, to_finish;
        auto flag_checks = [&] {
            e.movzx8(rax, m_io);
            e.alu8(Alu::Or, rax, m_invalidated);
            e.alu8(Alu::Or, rax, m_idle_ready);
            e.alu8(Alu::Or, rax, m_next_delay);
            to_finish_dirty.push_back(e.jcc(Cond::Ne));
            e.mov(rax, m_entry_mode, false);
            e.alu(Alu::Cmp, rax, m_mode, false);
            to_finish_dirty.push_back(e.jcc(Cond::Ne));
        };
        if (last_handler) flag_checks();
        e.imul_imm(rdx, m_weight, static_cast<s32>(blk.count));
        e.alu(Alu::Add, rdx, kTime);  // entry time of the next block
        e.mov(rax, m_pc);
        std::vector<s64> outside;  // successors not in the region
        for (s64 delta : successors) {
            std::size_t to = rb.size();
            for (std::size_t j = 0; j < rb.size(); ++j)
                if (static_cast<s64>(rb[j]->offset) == static_cast<s64>(page_offset) + delta) to = j;
            if (to == rb.size() || !region_mode) {
                outside.push_back(delta);
                continue;
            }
            // The next block of the region: carry on with the registers held.
            pc_plus(rcx, delta);
            e.alu(Alu::Cmp, rax, rcx);
            const std::size_t other = e.jcc(Cond::Ne);
            e.imul_imm(rcx, m_weight, static_cast<s32>(rb[to]->count - 1));
            e.alu(Alu::Add, rcx, rdx);
            e.alu(Alu::Cmp, rcx, m_safe);
            to_finish_dirty.push_back(e.jcc(Cond::Ae));
            restore(alloc & ~loaded);
            e.mov(kTime, rdx);
            if (delta) e.alu_imm(Alu::Add, kPc, static_cast<s32>(delta));
            if (to <= k) e.jmp_to(label[to]);
            else to_label[to].push_back(e.jmp());
            e.bind(other);
        }
        if (!last_handler) {
            flag_checks();
            e.mov(rax, m_pc);
        }
        const u64 dirty_end = dirty;
        flush(dirty_end);
        // Enters the translation at rsi + top for pc rax if the block (at rcx)
        // fits; the registers are written back.
        auto enter = [&](Reg block) {
            e.mov(rdi, Mem{block, static_cast<s32>(offsetof(Block, code))});
            e.test(rdi, rdi);
            to_finish.push_back(e.jcc(Cond::E));
            e.cmp8_imm(Mem{block, static_cast<s32>(offsetof(Block, jit_addr64))}, addr64 ? 1 : 0);
            to_finish.push_back(e.jcc(Cond::Ne));
            e.mov(rcx, Mem{block, static_cast<s32>(offsetof(Block, count))}, false);
            e.alu_imm(Alu::Sub, rcx, 1, false);
            e.imul(rcx, m_weight);
            e.alu(Alu::Add, rcx, rdx);
            e.alu(Alu::Cmp, rcx, m_safe);
            to_finish.push_back(e.jcc(Cond::Ae));
            e.mov(kTime, rdx);
            e.mov(kPc, rax);
            e.alu_imm(Alu::Add, rdi, static_cast<s32>(top));
            e.jmp(rdi);
        };
        for (s64 delta : outside) {
            pc_plus(rcx, delta);
            e.alu(Alu::Cmp, rax, rcx);
            const std::size_t next = e.jcc(Cond::Ne);
            const u8* const host = page_host + static_cast<u32>(static_cast<s64>(page_offset) + delta);
            e.mov_imm64(rsi, reinterpret_cast<u64>(&blocks_[block_index(host)]));
            e.mov_imm64(rcx, reinterpret_cast<u64>(host));
            e.alu(Alu::Cmp, rcx, Mem{rsi, static_cast<s32>(offsetof(Block, host))});
            to_finish.push_back(e.jcc(Cond::Ne));
            enter(rsi);
            e.bind(next);
        }
        // Anywhere else (another page, or a jump through a register): look up
        // the fetch translation and the block as run_block() would.
        e.test_imm(rax, 3);
        to_finish.push_back(e.jcc(Cond::Ne));
        e.mov(rcx, rax);
        e.shift_imm(Shift::Shr, rcx, 12, true);  // tag
        e.mov(rsi, rcx);
        e.alu(Alu::Or, rcx, m_asid_tag);
        e.alu_imm(Alu::And, rsi, kFastEntries - 1, false);
        e.shift_imm(Shift::Shl, rsi, 5, false);
        e.alu(Alu::Add, rsi, kTlb);  // the load entry; the fetch table precedes it
        e.alu(Alu::Cmp, rcx, Mem{rsi, static_cast<s32>(offsetof(FastEntry, tag)) - kTableBytes});
        to_finish.push_back(e.jcc(Cond::Ne));
        e.mov(rsi, Mem{rsi, static_cast<s32>(offsetof(FastEntry, host)) - kTableBytes});
        e.test(rsi, rsi);
        to_finish.push_back(e.jcc(Cond::E));
        e.mov(rcx, rax);
        e.alu_imm(Alu::And, rcx, 0xFFF, false);
        e.alu(Alu::Add, rsi, rcx);  // host address of the instruction
        e.mov(rcx, rsi);
        e.shift_imm(Shift::Shr, rcx, 2, true);
        e.alu_imm(Alu::And, rcx, (1u << kBlockBits) - 1, false);
        e.imul_imm(rcx, rcx, static_cast<s32>(sizeof(Block)));
        e.mov_imm64(rdi, reinterpret_cast<u64>(blocks_.data()));
        e.alu(Alu::Add, rcx, rdi);
        e.alu(Alu::Cmp, rsi, Mem{rcx, static_cast<s32>(offsetof(Block, host))});
        to_finish.push_back(e.jcc(Cond::Ne));
        e.mov(rsi, rcx);
        enter(rsi);

        // Completed the block.
        for (std::size_t site : to_finish_dirty) e.bind(site);
        flush(dirty_end);
        for (std::size_t site : to_finish) e.bind(site);
        set_time(blk.count);
        e.mov_imm32(rax, blk.count);
        to_tail[k].push_back(e.jmp());
    }

    // Inline operations that need their handler after all: it runs, then
    // the checks after it, then back to where the inline form ends.
    for (const Cold& c : colds) {
        cur = rb[c.k], cur_k = c.k, page_offset = cur->offset;
        for (std::size_t site : c.slow) e.bind(site);
        emit_handler(c.i, c.delay_slot, *c.op, c.dirty_before);
        if (c.i + 1 == cur->count) exit_checks(c.i);
        restore(c.loaded_after);  // as the inline path leaves them
        e.jmp_to(c.resume);
    }

    for (const Exit& x : exits) {
        e.bind(x.site);
        flush(x.flush);
        e.mov_imm32(rax, x.completed);
        to_tail[x.k].push_back(e.jmp());
    }
    // Tell run_block() which block (its slot, and the block that belongs
    // there) and entry pc the count belongs to.
    std::vector<std::size_t> to_epilogue;
    for (std::size_t k = 0; k < rb.size(); ++k) {
        if (to_tail[k].empty()) continue;
        for (std::size_t site : to_tail[k]) e.bind(site);
        e.mov_imm64(rcx, reinterpret_cast<u64>(&blocks_[block_index(rb[k]->host)]));
        e.mov(Mem{kCpu, offset_of(this, &jit_block_)}, rcx);
        e.mov_imm64(rcx, reinterpret_cast<u64>(rb[k]->host));
        e.mov(Mem{kCpu, offset_of(this, &jit_block_host_)}, rcx);
        to_epilogue.push_back(e.jmp());
    }
    for (std::size_t site : to_epilogue) e.bind(site);
    e.mov(Mem{kCpu, offset_of(this, &jit_entry_pc_)}, kPc);
    e.add_rsp(8 + kShadow);
    if (kSaveSiDi) e.pop(rdi), e.pop(rsi);
    e.pop(r15), e.pop(r14), e.pop(r13), e.pop(r12), e.pop(rbp), e.pop(rbx);
    e.ret();

    const void* fn = jit_buf_->add(e.code().data(), e.size());
    if (!fn) return false;
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

#elif !defined(__aarch64__)  // no back end for this host (AArch64: jit_a64.cpp)

namespace sg::mips {
bool R4000::jit_compile_supported() { return false; }
bool R4000::jit_compile(Block&) { return false; }
}  // namespace sg::mips

#endif
