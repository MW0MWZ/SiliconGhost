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
#include <vector>

#include "jit/a64_emitter.h"
#include "jit/exec_buffer.h"

namespace sg::mips {

namespace {

using namespace sg::jit::a64;
using Site = Emitter::Site;

// Callee-saved registers live across the whole function.
constexpr Reg kCpu = x19;        // R4000*
constexpr Reg kTime = x20;       // time_fp_ at entry
constexpr Reg kPc = x21;         // pc at entry (blocks are shared by aliases)
constexpr Reg kTlb = x22;        // this mode's soft-TLB load table (fetch before it, store after it)
constexpr Reg kEntryMode = x23;  // mode_ at entry
// Host registers that hold guest registers (caller-saved: reloaded after
// calls, as on x86-64). x0-x4 are scratch; x16 and x17 belong to the emitter.
constexpr Reg kCacheRegs[] = {x8, x9, x10, x11, x12, x13, x14, x15};

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

bool R4000::jit_compile(Block& b) {
    Emitter e;
    auto gpr_off = [&](unsigned r) { return offset_of(this, &gpr[r]); };
    const s64 o_time = offset_of(this, &time_fp_), o_pc = offset_of(this, &pc), o_npc = offset_of(this, &npc_);
    const s64 o_cur_pc = offset_of(this, &cur_pc_), o_in_delay = offset_of(this, &in_delay_);
    const s64 o_next_delay = offset_of(this, &next_delay_), o_io = offset_of(this, &io_);
    const s64 o_invalidated = offset_of(this, &block_invalidated_), o_idle_ready = offset_of(this, &idle_ready_);
    const s64 o_hi = offset_of(this, &hi), o_lo = offset_of(this, &lo), o_mode = offset_of(this, &mode_);
    const s64 o_safe = offset_of(this, &jit_safe_), o_weight = offset_of(this, &weight_);
    const s64 o_asid_tag = offset_of(this, &asid_tag_);
    static_assert(sizeof(FastEntry) == 32 && kFastEntries == 1024);
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
    const bool addr64 = addr64_;
    b.jit_addr64 = addr64;

    // Guest register cache (as on x86-64).
    const JitPlan plan = jit_plan(b);
    Reg host_of[32] = {};
    for (unsigned g = 1; g < 32; ++g)
        if (plan.slot[g] >= 0) host_of[g] = kCacheRegs[plan.slot[g]];
    const u32 alloc = plan.alloc, written = plan.written;
    const bool self_loop = plan.self_loop;
    u32 loaded = 0, dirty = 0;
    auto cached = [&](unsigned g) { return ((alloc >> g) & 1) != 0; };
    auto ensure = [&](unsigned g) {
        if (cached(g) && !((loaded >> g) & 1)) {
            e.ldr(host_of[g], kCpu, gpr_off(g));
            loaded |= 1u << g;
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
        loaded |= 1u << g;
        dirty |= 1u << g;
    };
    auto write_imm = [&](unsigned g, s64 v) {
        if (!cached(g)) {
            e.mov_imm(x0, static_cast<u64>(v));
            return e.str(x0, kCpu, gpr_off(g));
        }
        e.mov_imm(host_of[g], static_cast<u64>(v));
        loaded |= 1u << g;
        dirty |= 1u << g;
    };
    auto flush = [&](u32 mask) {
        for (unsigned g = 1; g < 32; ++g)
            if ((mask >> g) & 1) e.str(host_of[g], kCpu, gpr_off(g));
    };
    auto restore = [&](u32 mask) {
        for (unsigned g = 1; g < 32; ++g)
            if ((mask >> g) & 1) e.ldr(host_of[g], kCpu, gpr_off(g));
    };
    if (self_loop) {
        restore(alloc);
        loaded = alloc;
        dirty = written;
    }
    const std::size_t body = e.size();

    auto set_time = [&](unsigned n) {  // time_fp_ = entry time + n instructions
        e.ldr(x0, kCpu, o_weight);
        e.mov_imm(kScratch0, n);
        e.madd(x0, x0, kScratch0, kTime);
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
        e.load_constant(x1, reinterpret_cast<u64>(&op));
        call(reinterpret_cast<const void*>(op.fn));
    };

    // Loads and stores of RAM, inline (see jit_x64.cpp).
    const s64 o_store_count = offset_of(this, &store_count_);
    const s64 log_host = offset_of(this, &store_log_[0].host);
    const s64 log_size = offset_of(this, &store_log_[0].size);
    const s64 log_value = offset_of(this, &store_log_[0].value);
    auto emit_memory = [&](u32 w, std::vector<Site>& slow) -> bool {
        const unsigned opc = w >> 26, rs = f_rs(w), rt = f_rt(w);
        unsigned size;
        bool is_store = false;
        switch (opc) {
            case 0x20:
            case 0x24: size = 1; break;
            case 0x21:
            case 0x25: size = 2; break;
            case 0x23: size = 4; break;
            case 0x28: size = 1, is_store = true; break;
            case 0x29: size = 2, is_store = true; break;
            case 0x2B: size = 4, is_store = true; break;
            default: return false;
        }
        ensure(rs);
        if (is_store) ensure(rt);
        read(x0, rs);
        e.add_imm(x0, x0, static_cast<s16>(w & 0xFFFF));
        if (!addr64) e.sxtw(x0, x0);
        if (size > 1) {
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
        if (!is_store) {
            e.ldr(x2, x2, offsetof(FastEntry, host));
            slow.push_back(e.cbz(x2));
            if (rt == 0) {  // the access still happens, for its exceptions
                e.ldr_index(xzr, x2, x0, size);
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
                default:
                    e.ldr_index(x0, x2, x0, 4);
                    e.rev32(x0, x0);
                    e.sxtw(x0, x0);
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
        // log_store()
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
        const Reg v = src(rt, x0);
        switch (size) {
            case 1: e.str(v, x1, 0, 1); break;
            case 2:
                e.rev16(x0, v);
                e.str(x0, x1, 0, 2);
                break;
            default:
                e.rev32(x0, v);
                e.str(x0, x1, 0, 4);
                break;
        }
        return true;
    };

    // Register operations, inline; others of the kind call their handler.
    auto emit_plain = [&](const Op& op) {
        const u32 w = op.inst;
        const unsigned rs = f_rs(w), rt = f_rt(w), rd = f_rd(w), sa = f_sa(w);
        const s32 simm = static_cast<s16>(w & 0xFFFF);
        const u32 zimm = w & 0xFFFF;
        const unsigned opc = w >> 26, funct = w & 63;
        auto sext_store = [&](unsigned dest) {  // x0 <- sext32(w0), stored to dest
            e.sxtw(x0, x0);
            write(dest, x0);
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
                    if (funct == 0x00) e.lsl(x0, t, sa, false);
                    else if (funct == 0x02) e.lsr(x0, t, sa, false);
                    else e.asr(x0, t, sa, false);
                    sext_store(rd);
                    break;
                }
                case 0x04:  // SLLV
                case 0x06:  // SRLV
                case 0x07: {  // SRAV (32-bit register shifts take the count mod 32, as MIPS)
                    const Reg s = src(rs, x1), t = src(rt, x0);
                    if (funct == 0x04) e.lslv(x0, t, s, false);
                    else if (funct == 0x06) e.lsrv(x0, t, s, false);
                    else e.asrv(x0, t, s, false);
                    sext_store(rd);
                    break;
                }
                case 0x10: e.ldr(x0, kCpu, o_hi), write(rd, x0); break;  // MFHI
                case 0x12: e.ldr(x0, kCpu, o_lo), write(rd, x0); break;  // MFLO
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
                    if (funct == 0x21) e.add(x0, s, t, false);
                    else e.sub(x0, s, t, false);
                    sext_store(rd);
                    break;
                }
                case 0x24:  // AND
                case 0x25:  // OR
                case 0x26:  // XOR
                case 0x27: {  // NOR
                    const Reg s = src(rs, x0), t = src(rt, x1);
                    if (funct == 0x24) e.and_(x0, s, t);
                    else if (funct == 0x26) e.eor(x0, s, t);
                    else e.orr(x0, s, t);
                    if (funct == 0x27) e.mvn(x0, x0);
                    write(rd, x0);
                    break;
                }
                case 0x2A:  // SLT
                case 0x2B: {  // SLTU
                    const Reg s = src(rs, x0), t = src(rt, x1);
                    e.cmp(s, t);
                    e.cset(x0, funct == 0x2A ? Cond::Lt : Cond::Lo);
                    write(rd, x0);
                    break;
                }
                default: done = false; break;
            }
        } else {
            switch (opc) {
                case 0x09:  // ADDIU
                    e.add_imm(x0, src_imm(rs, x0), simm, false);
                    sext_store(rt);
                    break;
                case 0x0A:  // SLTI
                case 0x0B:  // SLTIU (the immediate is sign-extended, then compared unsigned)
                    e.cmp_imm(src_imm(rs, x1), simm);
                    e.cset(x0, opc == 0x0A ? Cond::Lt : Cond::Lo);
                    write(rt, x0);
                    break;
                case 0x0C:  // ANDI
                case 0x0D:  // ORI
                case 0x0E: {  // XORI
                    const Reg s = src(rs, x0);
                    if (zimm == 0) {
                        if (opc == 0x0C) e.mov(x0, xzr);
                        else e.mov(x0, s);
                    } else if (opc == 0x0C) {
                        e.and_imm(x0, s, zimm);
                    } else if (opc == 0x0D) {
                        e.orr_imm(x0, s, zimm);
                    } else {
                        e.eor_imm(x0, s, zimm);
                    }
                    write(rt, x0);
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

    struct Exit {
        Site site;
        u32 completed;
        u32 flush;
    };
    std::vector<Exit> exits;
    auto exit_if = [&](Site s, u32 completed, u32 mask) { exits.push_back({s, completed, mask}); };
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
    auto leave_delay_slot = [&] {
        e.ldr(x0, kCpu, o_npc);
        e.str(x0, kCpu, o_pc);
        e.add_imm(x0, x0, 4);
        e.str(x0, kCpu, o_npc);
        e.str(xzr, kCpu, o_next_delay, 1);
    };

    const u32 page_offset = b.offset;
    const u8* const page_host = b.host - page_offset;
    std::vector<s64> successors;
    auto add_successor = [&](s64 delta) {
        const s64 off = static_cast<s64>(page_offset) + delta;
        if (off >= 0 && off < 0x1000) successors.push_back(delta);
    };

    bool synced = true;

    // Branches and jumps, inline (see jit_x64.cpp).
    auto emit_branch = [&](const Op& op, u32 i) -> bool {
        const u32 w = op.inst;
        const unsigned opc = w >> 26, rs = f_rs(w), rt = f_rt(w);
        const s64 simm = static_cast<s16>(w & 0xFFFF);
        enum Kind { kCompare, kSign, kJump, kJumpReg } kind;
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
            case 0x02: case 0x03: kind = kJump; break;  // J, JAL
            case 0x00:
                if ((w & 63) != 0x08 && (w & 63) != 0x09) return false;
                kind = kJumpReg;  // JR, JALR
                break;
            default: return false;
        }
        if (opc >= 0x14 && opc <= 0x17) likely = true;

        pc_plus(x0, 4 * i + 4);
        e.str(x0, kCpu, o_pc);
        Site to_not_taken;
        bool conditional = false;
        if (kind == kCompare) {
            e.cmp(src(rs, x1), src(rt, x2));
            to_not_taken = e.b(not_taken), conditional = true;
        } else if (kind == kSign) {
            e.cmp_imm(src_imm(rs, x1), 0);
            to_not_taken = e.b(not_taken), conditional = true;
        }

        // Taken: the target in x1.
        const s64 delta = 4 * static_cast<s64>(i) + 4 + 4 * simm;
        if (kind == kCompare || kind == kSign) {
            pc_plus(x1, delta);
            add_successor(delta);
        } else if (kind == kJump) {
            pc_plus(x1, 4 * i + 4);
            e.and_imm(x1, x1, ~u64{0x0FFFFFFF});
            e.orr_imm(x1, x1, (w & 0x03FFFFFFu) << 2);
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
        }
        e.str(x1, kCpu, o_npc);
        store_byte_imm(o_next_delay, 1);
        const bool maybe_backward = ((kind == kCompare || kind == kSign) && simm < 0) || opc == 0x02;
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
            done.push_back(e.cbnz(x2, false));
            e.mov_imm(x2, ~u64{0});
            e.str(x2, kCpu, o_loop_pc);  // cooldown over: start afresh
            done.push_back(e.b());
            e.bind(visit1);
            e.bind(visit2);
            e.str(x0, kCpu, o_cur_pc);
            e.mov(x24, x1);  // the target, kept over the time update
            set_time(i + 1);
            flush(dirty);  // the loop bookkeeping compares registers
            e.mov(x0, kCpu);
            e.mov(x1, x24);
            call(reinterpret_cast<const void*>(&R4000::jit_backward_branch));
            e.ldr(x0, kCpu, o_idle_ready, 1);
            exit_if(e.cbnz(x0, false), i + 1, 0);
            restore(loaded);
            for (const Site& s : done) e.bind(s);
        }

        if (conditional) {
            const Site to_done = e.b();
            e.bind(to_not_taken);
            if (likely) {  // the delay slot is skipped: leave, as run_block() does
                pc_plus(x0, 4 * i + 8);
                e.str(x0, kCpu, o_pc);
                e.add_imm(x0, x0, 4);
                e.str(x0, kCpu, o_npc);
                set_time(i + 1);
                exits.push_back({e.b(), i + 1, dirty});
            } else {
                pc_plus(x0, 4 * i + 8);
                e.str(x0, kCpu, o_npc);
                store_byte_imm(o_next_delay, 1);
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
            std::vector<Site> slow;
            Site skip_slow;
            const u32 dirty_before = dirty;
            const bool fast = emit_memory(w, slow);
            const u32 loaded_after = loaded;
            if (fast) {
                if (delay_slot) leave_delay_slot();
                skip_slow = e.b();
                for (const Site& s : slow) e.bind(s);
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
            if (i + 1 < b.count || fast) exit_checks(i);
            if (fast) {
                restore(loaded_after);
                e.bind(skip_slow);
            } else {
                loaded = dirty = 0;
            }
            synced = !fast || delay_slot;
        }
    }
    if (!synced) {
        pc_plus(x0, 4 * b.count);
        e.str(x0, kCpu, o_pc);
        e.add_imm(x0, x0, 4);
        e.str(x0, kCpu, o_npc);
    }
    if (!is_branch_word(b.ops[b.count - 1].inst)) add_successor(4 * static_cast<s64>(b.count));
    std::sort(successors.begin(), successors.end());
    successors.erase(std::unique(successors.begin(), successors.end()), successors.end());

    // Block linking (see jit_x64.cpp). x0: the next pc, x2: its entry time.
    std::vector<Site> to_finish_dirty, to_finish;
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
    e.ldr(x3, kCpu, o_weight);
    e.mov_imm(kScratch0, b.count);
    e.madd(x2, x3, kScratch0, kTime);  // entry time of the next block
    e.ldr(x0, kCpu, o_pc);
    if (self_loop) {
        e.cmp(x0, kPc);
        const Site other = e.b(Cond::Ne);
        e.mov_imm(kScratch0, b.count - 1);
        e.madd(x1, x3, kScratch0, x2);
        e.ldr(kScratch0, kCpu, o_safe);
        e.cmp(x1, kScratch0);
        to_finish_dirty.push_back(e.b(Cond::Hs));
        restore(alloc & ~loaded);
        e.mov(kTime, x2);
        e.b_to(body);
        e.bind(other);
    }
    const u32 dirty_end = dirty;
    flush(dirty_end);
    // Enters the translation of the block at `block` for pc x0 if it fits.
    // (x5 is its own scratch: `block` may be any of x1, x3.)
    auto enter = [&](Reg block) {
        e.ldr(x4, block, offsetof(Block, code));
        to_finish.push_back(e.cbz(x4));
        e.ldr(x5, block, offsetof(Block, jit_addr64), 1);
        to_finish.push_back(addr64 ? e.cbz(x5, false) : e.cbnz(x5, false));
        e.ldr(x5, block, offsetof(Block, count), 4);
        e.sub_imm(x5, x5, 1, false);
        e.ldr(kScratch0, kCpu, o_weight);
        e.madd(x5, x5, kScratch0, x2);
        e.ldr(kScratch0, kCpu, o_safe);
        e.cmp(x5, kScratch0);
        to_finish.push_back(e.b(Cond::Hs));
        e.mov(kTime, x2);
        e.mov(kPc, x0);
        e.add_imm(x4, x4, static_cast<s64>(top));
        e.br(x4);
    };
    for (s64 delta : successors) {
        if (delta == 0 && self_loop) continue;
        pc_plus(x1, delta);
        e.cmp(x0, x1);
        const Site next = e.b(Cond::Ne);
        const u8* const host = page_host + static_cast<u32>(static_cast<s64>(page_offset) + delta);
        e.load_constant(x3, reinterpret_cast<u64>(&blocks_[block_index(host)]));
        e.load_constant(x1, reinterpret_cast<u64>(host));
        e.ldr(kScratch0, x3, offsetof(Block, host));
        e.cmp(x1, kScratch0);
        to_finish.push_back(e.b(Cond::Ne));
        enter(x3);
        e.bind(next);
    }
    // Anywhere else: look up the fetch translation and the block.
    e.tst_imm(x0, 3);
    to_finish.push_back(e.b(Cond::Ne));
    e.lsr(x1, x0, 12);
    e.ubfiz(x3, x1, 5, 10);
    e.ldr(kScratch0, kCpu, o_asid_tag);
    e.orr(x1, x1, kScratch0);
    e.add(x3, x3, kTlb);
    e.sub_imm(x3, x3, kTableBytes);  // the fetch table precedes the load table
    e.ldr(kScratch0, x3, offsetof(FastEntry, tag));
    e.cmp(x1, kScratch0);
    to_finish.push_back(e.b(Cond::Ne));
    e.ldr(x3, x3, offsetof(FastEntry, host));
    to_finish.push_back(e.cbz(x3));
    e.and_imm(x1, x0, 0xFFF);
    e.add(x3, x3, x1);  // host address of the instruction
    e.lsr(x1, x3, 2);
    e.and_imm(x1, x1, (1u << kBlockBits) - 1);
    e.mov_imm(kScratch0, sizeof(Block));
    e.load_constant(x4, reinterpret_cast<u64>(blocks_.data()));
    e.madd(x1, x1, kScratch0, x4);
    e.ldr(kScratch0, x1, offsetof(Block, host));
    e.cmp(x3, kScratch0);
    to_finish.push_back(e.b(Cond::Ne));
    enter(x1);

    // Completed the block.
    for (const Site& s : to_finish_dirty) e.bind(s);
    flush(dirty_end);
    for (const Site& s : to_finish) e.bind(s);
    set_time(b.count);
    e.mov_imm32(x0, b.count);
    std::vector<Site> to_epilogue{e.b()};
    for (const Exit& x : exits) {
        e.bind(x.site);
        flush(x.flush);
        e.mov_imm32(x0, x.completed);
        to_epilogue.push_back(e.b());
    }
    for (const Site& s : to_epilogue) e.bind(s);
    // Tell run_block() which block (and entry pc) the count belongs to.
    e.load_constant(kScratch0, reinterpret_cast<u64>(&b));
    e.str(kScratch0, kCpu, offset_of(this, &jit_block_));
    e.str(kPc, kCpu, offset_of(this, &jit_entry_pc_));
    e.ldp(x23, x24, sp, 48);
    e.ldp(x21, x22, sp, 32);
    e.ldp(x19, x20, sp, 16);
    e.ldp_post(x29, x30, sp, 64);
    e.ret();
    e.finish_literals();

    const void* fn = jit_buf_->add(e.code().data(), e.size());
    if (!fn) return false;
    b.code = reinterpret_cast<JitFn>(const_cast<void*>(fn));
    return true;
}

}  // namespace sg::mips

#endif
