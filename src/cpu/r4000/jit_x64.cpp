// SPDX-License-Identifier: GPL-2.0-or-later
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

inline unsigned f_rs(u32 i) { return (i >> 21) & 31; }
inline unsigned f_rt(u32 i) { return (i >> 16) & 31; }
inline unsigned f_rd(u32 i) { return (i >> 11) & 31; }
inline unsigned f_sa(u32 i) { return (i >> 6) & 31; }

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
                case 0x18:
                case 0x19: return {bit(rs) | bit(rt), 0};
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
        case 0x01: return rt < 4 ? Uses{bit(rs), 0} : Uses{};
        case 0x03: return {0, bit(31)};
        case 0x04:
        case 0x05:
        case 0x14:
        case 0x15: return {bit(rs) | bit(rt), 0};
        case 0x06:
        case 0x07:
        case 0x16:
        case 0x17: return {bit(rs), 0};
        case 0x09:
        case 0x0A:
        case 0x0B:
        case 0x0C:
        case 0x0D:
        case 0x0E: return {bit(rs), bit(rt)};
        case 0x0F: return {0, bit(rt)};
        case 0x20:
        case 0x21:
        case 0x23:
        case 0x24:
        case 0x25: return {bit(rs), bit(rt)};
        case 0x28:
        case 0x29:
        case 0x2B: return {bit(rs) | bit(rt), 0};
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

template <typename T>
static s32 offset_of(const R4000* cpu, const T* member) {
    return static_cast<s32>(reinterpret_cast<const u8*>(member) - reinterpret_cast<const u8*>(cpu));
}

bool R4000::jit_compile_supported() { return true; }

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
    const Mem m_entry_mode{rsp, 0};

    // Prologue (six pushes and 8 bytes keep calls 16-byte aligned).
    e.push(rbx), e.push(rbp), e.push(r12), e.push(r13), e.push(r14), e.push(r15);
    e.sub_rsp(8);
    e.mov(kCpu, rdi);
    e.mov(kPc, rsi);
    e.mov(kTime, rdx);
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
    Reg host_of[32] = {};
    u32 alloc = 0, loaded = 0, dirty = 0;
    u32 written = 0;  // allocated registers the block writes
    bool self_loop = false;  // a branch back to the block's start
    {
        unsigned uses[32] = {};
        for (u32 i = 0; i < b.count; ++i) {
            if (i > 0 && is_branch_word(b.ops[i - 1].inst)) {
                if (is_branch_word(b.ops[i].inst)) continue;  // a branch in a delay slot goes to its handler
            }
            const Uses u = inline_uses(b.ops[i].inst);
            for (unsigned r = 1; r < 32; ++r) uses[r] += ((u.reads >> r) & 1) + ((u.writes >> r) & 1);
            written |= u.writes;
            s64 delta;
            if (branch_offset(b.ops[i].inst, delta) && !(i > 0 && is_branch_word(b.ops[i - 1].inst)) &&
                4 * static_cast<s64>(i) + delta == 0)
                self_loop = true;
        }
        unsigned order[31];
        for (unsigned r = 0; r < 31; ++r) order[r] = r + 1;
        std::stable_sort(order, order + 31, [&](unsigned x, unsigned y) { return uses[x] > uses[y]; });
        unsigned n = 0;
        for (unsigned r : order) {
            if (uses[r] < 2 || n == std::size(kCacheRegs)) break;
            host_of[r] = kCacheRegs[n++];
            alloc |= 1u << r;
        }
        written &= alloc;
    }
    auto cached = [&](unsigned g) { return ((alloc >> g) & 1) != 0; };
    auto ensure = [&](unsigned g) {  // load a cached register on first use
        if (cached(g) && !((loaded >> g) & 1)) {
            e.mov(host_of[g], gpr_at(g));
            loaded |= 1u << g;
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
        loaded |= 1u << g;
        dirty |= 1u << g;
    };
    auto write_imm = [&](unsigned g, s32 v) {  // sign-extended
        if (!cached(g)) return e.mov_store_imm(gpr_at(g), v);
        e.mov_imm64(host_of[g], static_cast<u64>(static_cast<s64>(v)));
        loaded |= 1u << g;
        dirty |= 1u << g;
    };
    auto flush = [&](u32 mask) {  // write back
        for (unsigned g = 1; g < 32; ++g)
            if ((mask >> g) & 1) e.mov(gpr_at(g), host_of[g]);
    };
    auto restore = [&](u32 mask) {  // reload after a call
        for (unsigned g = 1; g < 32; ++g)
            if ((mask >> g) & 1) e.mov(host_of[g], gpr_at(g));
    };
    // A block that branches to itself loops in place with the registers
    // held, so every pass starts with all of them loaded.
    if (self_loop) {
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
        e.mov(rdi, kCpu);
        e.mov_imm64(rsi, reinterpret_cast<u64>(&op));
        e.mov_imm64(rax, reinterpret_cast<u64>(op.fn));
        e.call(rax);
    };

    // Loads and stores of RAM, inline: the soft-TLB lookup and the host
    // access, exactly what load()/store() do on a hit with a host page. A
    // miss, an unaligned address, MMIO or a page holding translated code goes
    // to the handler (`slow`). Returns false for other instructions.
    const Mem m_store_count{kCpu, offset_of(this, &store_count_)};
    const s32 log_host = offset_of(this, &store_log_[0].host);
    const s32 log_size = offset_of(this, &store_log_[0].size);
    const s32 log_value = offset_of(this, &store_log_[0].value);
    auto emit_memory = [&](u32 w, std::vector<std::size_t>& slow) -> bool {
        const unsigned opc = w >> 26, rs = f_rs(w), rt = f_rt(w);
        unsigned size;
        bool is_store = false;
        switch (opc) {
            case 0x20:
            case 0x24: size = 1; break;  // LB, LBU
            case 0x21:
            case 0x25: size = 2; break;  // LH, LHU
            case 0x23: size = 4; break;  // LW
            case 0x28: size = 1, is_store = true; break;  // SB
            case 0x29: size = 2, is_store = true; break;  // SH
            case 0x2B: size = 4, is_store = true; break;  // SW
            default: return false;
        }
        // Operands are loaded before the paths split, so both agree.
        ensure(rs);
        if (is_store) ensure(rt);
        const s32 table = is_store ? kTableBytes : 0;
        read(rax, rs);
        e.alu_imm(Alu::Add, rax, static_cast<s16>(w & 0xFFFF));
        if (!addr64) e.movsxd(rax, rax);
        if (size > 1) {
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
        if (!is_store) {
            e.mov(rdx, Mem{rdx, static_cast<s32>(offsetof(FastEntry, host))});
            e.test(rdx, rdx);
            slow.push_back(e.jcc(Cond::E));
            e.alu(Alu::Add, rdx, rax);
            if (rt == 0) return true;  // the access still happens, for its exceptions
            const Mem p{rdx, 0};
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
                default:
                    e.mov(rax, p, false);
                    e.bswap32(rax);
                    e.movsxd(rax, rax);
                    break;
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
        read(rax, rt, false);
        switch (size) {
            case 1: e.mov8(Mem{rcx, 0}, rax); break;
            case 2:
                e.rol16_8(rax);
                e.mov16(Mem{rcx, 0}, rax);
                break;
            default:
                e.bswap32(rax);
                e.mov(Mem{rcx, 0}, rax, false);
                break;
        }
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
        auto alu32_store = [&](unsigned dest) {  // rax <- sext32(eax), stored to dest
            e.movsxd(rax, rax);
            write(dest, rax);
        };
        auto set_flag = [&](Cond c, unsigned dest) {
            e.setcc(c, rax);
            e.movzx8(rax, rax);
            write(dest, rax);
        };
        const bool is_mult = opc == 0x00 && (funct == 0x18 || funct == 0x19);
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
                case 0x27:  // NOR
                    read(rax, rs);
                    alu_g(funct == 0x24 ? Alu::And : funct == 0x26 ? Alu::Xor : Alu::Or, rax, rt);
                    if (funct == 0x27) e.not_(rax);
                    write(rd, rax);
                    break;
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
                    read(rax, rs);
                    e.alu_imm(opc == 0x0C ? Alu::And : opc == 0x0D ? Alu::Or : Alu::Xor, rax, zimm);
                    write(rt, rax);
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

    struct Exit {
        std::size_t site;
        u32 completed;
        u32 flush;  // cached registers to write back
    };
    std::vector<Exit> exits;
    auto exit_if = [&](Cond c, u32 completed, u32 mask) { exits.push_back({e.jcc(c), completed, mask}); };
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
    const u32 page_offset = b.offset;
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
        enum Kind { kCompare, kSign, kJump, kJumpReg } kind;
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
            case 0x02: case 0x03: kind = kJump; break;  // J, JAL
            case 0x00:
                if ((w & 63) != 0x08 && (w & 63) != 0x09) return false;
                kind = kJumpReg;  // JR, JALR
                break;
            default: return false;
        }
        if (opc >= 0x14 && opc <= 0x17) likely = true;

        pc_plus(rax, 4 * i + 4);
        e.mov(m_pc, rax);
        std::size_t to_not_taken = 0;
        if (kind == kCompare) {
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
        if (kind == kCompare || kind == kSign) {
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
        const bool maybe_backward = ((kind == kCompare || kind == kSign) && simm < 0) || opc == 0x02;
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
            e.mov(rdi, kCpu);
            e.mov(rsi, rcx);
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
                exits.push_back({e.jmp(), i + 1, dirty});
            } else {
                pc_plus(rax, 4 * i + 8);
                e.mov(m_npc, rax);
                e.mov_store_imm8(m_next_delay, 1);
            }
            e.bind(to_done);
        }
        return true;
    };
    for (u32 i = 0; i < b.count; ++i) {
        const Op& op = b.ops[i];
        const u32 w = op.inst;
        const bool delay_slot = i > 0 && is_branch_word(b.ops[i - 1].inst);
        bool plain;
        decode(w, plain);

        if (plain) {
            emit_plain(op);
            if (delay_slot) leave_delay_slot();
            synced = delay_slot;
        } else if (!delay_slot && emit_branch(op, i)) {
            synced = true;
        } else {
            // Loads and stores try the inline path first.
            std::vector<std::size_t> slow;
            std::size_t skip_slow = 0;
            const u32 dirty_before = dirty;
            const bool fast = emit_memory(w, slow);
            const u32 loaded_after = loaded;
            if (fast) {
                if (delay_slot) leave_delay_slot();
                skip_slow = e.jmp();
                for (std::size_t site : slow) e.bind(site);
            }

            // Anything else: the bookkeeping of run_block(), then the handler.
            flush(fast ? dirty_before : dirty);
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
            if (i + 1 < b.count || fast) exit_checks(i);
            if (fast) {
                restore(loaded_after);  // as the inline path leaves them
                e.bind(skip_slow);
            } else {
                loaded = dirty = 0;
            }
            synced = !fast || delay_slot;
        }
    }
    if (!synced) {  // plain operations or inline accesses ran last
        pc_plus(rax, 4 * b.count);
        e.mov(m_pc, rax);
        e.alu_imm(Alu::Add, rax, 4);
        e.mov(m_npc, rax);
    }
    if (!is_branch_word(b.ops[b.count - 1].inst)) add_successor(4 * static_cast<s64>(b.count));
    std::sort(successors.begin(), successors.end());
    successors.erase(std::unique(successors.begin(), successors.end()), successors.end());

    // Block linking: run the next block's translation straight away when
    // run_block() would run it next with nothing checked in between. The
    // pass must fit below the safe time (run_block() stores it in
    // jit_safe_), no access may have happened that needs the checks, and the
    // mode must be unchanged. Within the page the translation is known to be
    // the same; elsewhere the fetch soft-TLB must map the pc as run_block()
    // would find it.
    std::vector<std::size_t> to_finish_dirty, to_finish;
    e.movzx8(rax, m_io);
    e.alu8(Alu::Or, rax, m_invalidated);
    e.alu8(Alu::Or, rax, m_idle_ready);
    e.alu8(Alu::Or, rax, m_next_delay);
    to_finish_dirty.push_back(e.jcc(Cond::Ne));
    e.mov(rax, m_entry_mode, false);
    e.alu(Alu::Cmp, rax, m_mode, false);
    to_finish_dirty.push_back(e.jcc(Cond::Ne));
    e.imul_imm(rdx, m_weight, static_cast<s32>(b.count));
    e.alu(Alu::Add, rdx, kTime);  // entry time of the next block
    e.mov(rax, m_pc);
    // The same block again: carry on with the registers held.
    if (self_loop) {
        e.alu(Alu::Cmp, rax, kPc);
        const std::size_t other = e.jcc(Cond::Ne);
        e.imul_imm(rcx, m_weight, static_cast<s32>(b.count - 1));
        e.alu(Alu::Add, rcx, rdx);
        e.alu(Alu::Cmp, rcx, m_safe);
        to_finish_dirty.push_back(e.jcc(Cond::Ae));
        restore(alloc & ~loaded);
        e.mov(kTime, rdx);
        e.jmp_to(body);
        e.bind(other);
    }
    const u32 dirty_end = dirty;
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
    for (s64 delta : successors) {
        if (delta == 0 && self_loop) continue;
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
    set_time(b.count);
    e.mov_imm32(rax, b.count);
    std::vector<std::size_t> to_epilogue{e.jmp()};
    for (const Exit& x : exits) {
        e.bind(x.site);
        flush(x.flush);
        e.mov_imm32(rax, x.completed);
        to_epilogue.push_back(e.jmp());
    }
    for (std::size_t site : to_epilogue) e.bind(site);
    // Tell run_block() which block (and entry pc) the count belongs to.
    e.mov_imm64(rcx, reinterpret_cast<u64>(&b));
    e.mov(Mem{kCpu, offset_of(this, &jit_block_)}, rcx);
    e.mov(Mem{kCpu, offset_of(this, &jit_entry_pc_)}, kPc);
    e.add_rsp(8);
    e.pop(r15), e.pop(r14), e.pop(r13), e.pop(r12), e.pop(rbp), e.pop(rbx);
    e.ret();

    const void* fn = jit_buf_->add(e.code().data(), e.size());
    if (!fn) return false;
    b.code = reinterpret_cast<JitFn>(const_cast<void*>(fn));
    return true;
}

}  // namespace sg::mips

#elif !defined(__aarch64__)  // no back end for this host (AArch64: jit_a64.cpp)

namespace sg::mips {
bool R4000::jit_compile_supported() { return false; }
bool R4000::jit_compile(Block&) { return false; }
}  // namespace sg::mips

#endif
