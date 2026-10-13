// SPDX-License-Identifier: GPL-2.0-or-later
// R4000 dynamic recompiler, x86-64 back end.
//
// A hot decoded block is translated into one host function that runs the
// block from its first instruction, exactly as run_block() would when no
// time check is due and no interrupt check is pending (the caller makes
// sure of both). Plain register operations are emitted inline; everything
// else calls its pre-decoded handler with the usual bookkeeping and leaves
// the function on the same conditions that end a straight-line run in
// run_block(). The function returns how many instructions it completed;
// run_block() carries on from there.
#include "cpu/r4000/r4000.h"

#if defined(__x86_64__) || defined(_M_X64)

#include <cstddef>
#include <vector>

#include "jit/exec_buffer.h"
#include "jit/x64_emitter.h"

namespace sg::mips {

namespace {

using namespace sg::jit::x64;

// Registers live across the whole function (callee-saved).
constexpr Reg kCpu = rbx;     // R4000*
constexpr Reg kTime = r12;    // time_fp_ at entry
constexpr Reg kWeight = r13;  // weight_
constexpr Reg kPc = r14;      // pc at entry (blocks are shared by aliases)
constexpr Reg kMode = r15;    // mode_ at entry
constexpr Reg kTlb = rbp;     // this mode's soft-TLB load table (the store table follows it)

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

    // Prologue (six pushes and 8 bytes keep calls 16-byte aligned).
    e.push(rbx), e.push(rbp), e.push(r12), e.push(r13), e.push(r14), e.push(r15);
    e.sub_rsp(8);
    e.mov(kCpu, rdi);
    e.mov(kPc, rsi);
    e.mov(kTime, rdx);
    e.mov(kWeight, Mem{kCpu, offset_of(this, &weight_)});
    e.mov(kMode, m_mode, false);
    static_assert(sizeof(FastEntry) == 32 && kFastEntries == 1024);
    constexpr s32 kTableBytes = kFastEntries * sizeof(FastEntry);
    e.imul_imm(kTlb, kMode, 3 * kTableBytes);
    e.alu(Alu::Add, kTlb, kCpu);
    e.alu_imm(Alu::Add, kTlb, offset_of(this, &fast_[0][static_cast<unsigned>(Access::Load)][0]));
    const std::size_t top = e.size();
    // Effective addresses follow the addressing mode the block is compiled
    // for; run_block() only calls it in that mode.
    const bool addr64 = addr64_;
    b.jit_addr64 = addr64;

    auto set_time = [&](unsigned n) {  // time_fp_ = entry time + n instructions
        e.imul_imm(rax, kWeight, static_cast<s32>(n));
        e.alu(Alu::Add, rax, kTime);
        e.mov(m_time, rax);
    };
    auto pc_plus = [&](Reg dst, unsigned bytes) {
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
        const s32 table = is_store ? kTableBytes : 0;
        e.mov(rax, gpr_at(rs));
        e.alu_imm(Alu::Add, rax, static_cast<s16>(w & 0xFFFF));
        if (!addr64) e.movsxd(rax, rax);
        if (size > 1) {
            e.test_imm(rax, size - 1);
            slow.push_back(e.jcc(Cond::Ne));
        }
        e.mov(rcx, rax);
        e.shift_imm(Shift::Shr, rcx, 12, true);  // tag
        e.mov(rdx, rcx);
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
            e.mov(gpr_at(rt), rax);
            return true;
        }
        e.mov(r9, Mem{rdx, table + static_cast<s32>(offsetof(FastEntry, host))});
        e.test(r9, r9);
        slow.push_back(e.jcc(Cond::E));
        e.mov(r8, Mem{rdx, table + static_cast<s32>(offsetof(FastEntry, code))});
        e.movzx8(r8, Mem{r8, 0});
        e.test(r8, r8);
        slow.push_back(e.jcc(Cond::Ne));  // translated code lives in this page
        e.alu(Alu::Add, r9, rax);  // the host address
        // log_store()
        e.mov(rcx, m_store_count);
        e.mov(rdx, rcx);
        e.alu_imm(Alu::Add, rdx, 1);
        e.mov(m_store_count, rdx);
        e.alu_imm(Alu::And, rcx, (1u << kStoreLogBits) - 1, false);
        e.imul_imm(rcx, rcx, static_cast<s32>(sizeof(StoreRec)));
        e.alu(Alu::Add, rcx, kCpu);
        e.mov(Mem{rcx, log_host}, r9);
        e.mov_imm32(rdx, size);
        e.mov(Mem{rcx, log_size}, rdx, false);
        e.mov_store_imm(Mem{rcx, log_value}, 0);
        e.mov(rax, gpr_at(rt), false);
        switch (size) {
            case 1: e.mov8(Mem{r9, 0}, rax); break;
            case 2:
                e.rol16_8(rax);
                e.mov16(Mem{r9, 0}, rax);
                break;
            default:
                e.bswap32(rax);
                e.mov(Mem{r9, 0}, rax, false);
                break;
        }
        return true;
    };

    struct Exit {
        std::size_t site;
        u32 completed;
    };
    std::vector<Exit> exits;
    bool fast_last = false;  // the last op may complete without syncing pc
    bool delay_slot = false;  // ops[i] is the delay slot of ops[i - 1]
    for (u32 i = 0; i < b.count; ++i) {
        const Op& op = b.ops[i];
        const u32 w = op.inst;
        if ((b.plain >> i) & 1) {
            // Inline register operations; dest 0 was decoded to a no-op.
            const unsigned rs = f_rs(w), rt = f_rt(w), rd = f_rd(w), sa = f_sa(w);
            const s32 simm = static_cast<s16>(w & 0xFFFF);
            const s32 zimm = static_cast<s32>(w & 0xFFFF);
            const unsigned opc = w >> 26, funct = w & 63;
            auto alu32_store = [&](unsigned dest) {  // rax <- sext32(eax), stored to dest
                e.movsxd(rax, rax);
                e.mov(gpr_at(dest), rax);
            };
            auto set_flag = [&](Cond c, unsigned dest) {
                e.setcc(c, rax);
                e.movzx8(rax, rax);
                e.mov(gpr_at(dest), rax);
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
                        e.mov(rax, gpr_at(rt), false);
                        e.shift_imm(funct == 0x00 ? Shift::Shl : funct == 0x02 ? Shift::Shr : Shift::Sar, rax,
                                    static_cast<u8>(sa), false);
                        alu32_store(rd);
                        break;
                    case 0x04:  // SLLV
                    case 0x06:  // SRLV
                    case 0x07:  // SRAV (x86 masks 32-bit shift counts to 5 bits, as MIPS does)
                        e.mov(rcx, gpr_at(rs), false);
                        e.mov(rax, gpr_at(rt), false);
                        e.shift_cl(funct == 0x04 ? Shift::Shl : funct == 0x06 ? Shift::Shr : Shift::Sar, rax, false);
                        alu32_store(rd);
                        break;
                    case 0x10: e.mov(rax, m_hi), e.mov(gpr_at(rd), rax); break;  // MFHI
                    case 0x12: e.mov(rax, m_lo), e.mov(gpr_at(rd), rax); break;  // MFLO
                    case 0x18:                                                   // MULT
                        e.movsxd(rax, gpr_at(rs));
                        e.movsxd(rcx, gpr_at(rt));
                        e.imul(rax, rcx);
                        e.movsxd(rcx, rax);
                        e.mov(m_lo, rcx);
                        e.shift_imm(Shift::Sar, rax, 32, true);
                        e.mov(m_hi, rax);
                        break;
                    case 0x19:  // MULTU
                        e.mov(rax, gpr_at(rs), false);
                        e.mov(rcx, gpr_at(rt), false);
                        e.imul(rax, rcx);
                        e.movsxd(rcx, rax);
                        e.mov(m_lo, rcx);
                        e.shift_imm(Shift::Shr, rax, 32, true);
                        e.movsxd(rax, rax);
                        e.mov(m_hi, rax);
                        break;
                    case 0x21:  // ADDU
                    case 0x23:  // SUBU
                        e.mov(rax, gpr_at(rs), false);
                        e.alu(funct == 0x21 ? Alu::Add : Alu::Sub, rax, gpr_at(rt), false);
                        alu32_store(rd);
                        break;
                    case 0x24:  // AND
                    case 0x25:  // OR
                    case 0x26:  // XOR
                    case 0x27:  // NOR
                        e.mov(rax, gpr_at(rs));
                        e.alu(funct == 0x24 ? Alu::And : funct == 0x26 ? Alu::Xor : Alu::Or, rax, gpr_at(rt));
                        if (funct == 0x27) e.not_(rax);
                        e.mov(gpr_at(rd), rax);
                        break;
                    case 0x2A:  // SLT
                    case 0x2B:  // SLTU
                        e.mov(rcx, gpr_at(rs));
                        e.alu(Alu::Xor, rax, rax, false);
                        e.alu(Alu::Cmp, rcx, gpr_at(rt));
                        set_flag(funct == 0x2A ? Cond::L : Cond::B, rd);
                        break;
                    default: done = false; break;
                }
            } else {
                switch (opc) {
                    case 0x09:  // ADDIU
                        e.mov(rax, gpr_at(rs), false);
                        e.alu_imm(Alu::Add, rax, simm, false);
                        alu32_store(rt);
                        break;
                    case 0x0A:  // SLTI
                    case 0x0B:  // SLTIU (the immediate is sign-extended, then compared unsigned)
                        e.mov(rcx, gpr_at(rs));
                        e.alu(Alu::Xor, rax, rax, false);
                        e.alu_imm(Alu::Cmp, rcx, simm);
                        set_flag(opc == 0x0A ? Cond::L : Cond::B, rt);
                        break;
                    case 0x0C:  // ANDI
                    case 0x0D:  // ORI
                    case 0x0E:  // XORI
                        e.mov(rax, gpr_at(rs));
                        e.alu_imm(opc == 0x0C ? Alu::And : opc == 0x0D ? Alu::Or : Alu::Xor, rax, zimm);
                        e.mov(gpr_at(rt), rax);
                        break;
                    case 0x0F: e.mov_store_imm(gpr_at(rt), static_cast<s32>(static_cast<u32>(zimm) << 16)); break;  // LUI
                    default: done = false; break;
                }
            }
            if (!done) call_handler(op);
            delay_slot = false;
            continue;
        }

        // Loads and stores try the inline path first (not in a delay slot,
        // where pc follows the branch).
        std::vector<std::size_t> slow;
        std::size_t skip_slow = 0;
        const bool fast = !delay_slot && emit_memory(w, slow);
        if (fast) {
            skip_slow = e.jmp();
            for (std::size_t site : slow) e.bind(site);
        }
        fast_last = fast;

        // Anything else: the bookkeeping of run_block(), then the handler.
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
        if (i + 1 < b.count || fast) {
            // Leave when run_block() would: a device or CP0 access, the
            // block's memory written, a confirmed idle loop, or control
            // leaving the straight line (an exception or a taken branch
            // after its delay slot; a branch itself continues into its slot).
            e.movzx8(rax, m_io);
            e.alu8(Alu::Or, rax, m_invalidated);
            e.alu8(Alu::Or, rax, m_idle_ready);  // ZF: all three clear
            exits.push_back({e.jcc(Cond::Ne), i + 1});
            pc_plus(rax, 4 * i + 4);
            e.alu(Alu::Cmp, rax, m_pc);
            exits.push_back({e.jcc(Cond::Ne), i + 1});
        }
        if (fast) e.bind(skip_slow);
        delay_slot = is_branch_word(w);
    }

    // A block that branched back to its own start runs again straight away
    // when run_block() would run it again with nothing checked in between:
    // the next pass must fit below the safe time (run_block() stores it in
    // jit_safe_), and no access may have happened that needs the checks.
    std::vector<std::size_t> to_finish;
    if (b.count >= 2 && is_branch_word(b.ops[b.count - 2].inst)) {
        e.mov(rax, m_pc);
        e.alu(Alu::Cmp, rax, kPc);
        to_finish.push_back(e.jcc(Cond::Ne));
        e.movzx8(rax, m_io);
        e.alu8(Alu::Or, rax, m_invalidated);
        e.alu8(Alu::Or, rax, m_idle_ready);
        e.alu8(Alu::Or, rax, m_next_delay);
        to_finish.push_back(e.jcc(Cond::Ne));
        e.alu(Alu::Cmp, kMode, m_mode, false);
        to_finish.push_back(e.jcc(Cond::Ne));
        e.imul_imm(rcx, kWeight, static_cast<s32>(b.count));
        e.alu(Alu::Add, rcx, kTime);  // entry time of the next pass
        e.imul_imm(rax, kWeight, static_cast<s32>(b.count - 1));
        e.alu(Alu::Add, rax, rcx);
        e.alu(Alu::Cmp, rax, m_safe);
        to_finish.push_back(e.jcc(Cond::Ae));
        e.mov(kTime, rcx);
        e.jmp_to(top);
    }
    for (std::size_t site : to_finish) e.bind(site);

    // Completed the block: the time, and pc if plain operations ran last.
    set_time(b.count);
    if (((b.plain >> (b.count - 1)) & 1) || fast_last) {
        pc_plus(rax, 4 * b.count);
        e.mov(m_pc, rax);
        e.alu_imm(Alu::Add, rax, 4);
        e.mov(m_npc, rax);
    }
    e.mov_imm32(rax, b.count);
    std::vector<std::size_t> to_epilogue{e.jmp()};
    for (const Exit& x : exits) {
        e.bind(x.site);
        e.mov_imm32(rax, x.completed);
        to_epilogue.push_back(e.jmp());
    }
    for (std::size_t site : to_epilogue) e.bind(site);
    e.add_rsp(8);
    e.pop(r15), e.pop(r14), e.pop(r13), e.pop(r12), e.pop(rbp), e.pop(rbx);
    e.ret();

    const void* fn = jit_buf_->add(e.code().data(), e.size());
    if (!fn) return false;
    b.code = reinterpret_cast<JitFn>(const_cast<void*>(fn));
    return true;
}

}  // namespace sg::mips

#else

namespace sg::mips {
bool R4000::jit_compile_supported() { return false; }
bool R4000::jit_compile(Block&) { return false; }
}  // namespace sg::mips

#endif
