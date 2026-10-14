// SPDX-License-Identifier: GPL-2.0-or-later
// MIPS R4000/R4400 reference interpreter.
//
// References: MIPS R4000 Microprocessor User's Manual (2nd ed., Heinrich),
// MIPS IV Instruction Set (rev 3.2) for MIPS III semantics.
#include "cpu/r4000/r4000.h"

#include <climits>
#include <type_traits>

#include "core/int128.h"
#include "core/log.h"
#include "jit/exec_buffer.h"
#include "mem/bus.h"

namespace sg::mips {

namespace {

constexpr u64 kBevBase = 0xFFFFFFFFBFC00200ull;
constexpr u64 kNormalBase = 0xFFFFFFFF80000000ull;

constexpr u64 kEntryHiMask = 0xC00000FFFFFFE0FFull;  // R | VPN2 | ASID
constexpr u64 kVpn2Mask = 0xC00000FFFFFFE000ull;     // R | VPN2
constexpr u64 kEntryLoMask = 0x3FFFFFFFull;          // PFN | C | D | V | G
constexpr u32 kPageMaskMask = 0x01FFE000u;
constexpr u32 kStatusWriteMask = 0xFE77FFFFu;
constexpr u32 kCauseWriteMask = 0x00000300u;  // IP1:IP0 (software interrupts)
constexpr u32 kCauseBD = 0x80000000u;
constexpr u32 kCauseIP7 = 1u << 15;
constexpr u32 kConfigWriteMask = 0x3Fu;  // IB, DB, CU, K0
constexpr u32 kConfigNoScache = 1u << 17;
constexpr u32 kStatusCH = 1u << 18;

constexpr u64 k32BitCompatBase = 0xFFFFFFFF80000000ull;
constexpr u64 kKseg2Base = 0xFFFFFFFFC0000000ull;
constexpr u64 kXuseg = u64{1} << 40;  // R4000 implements 40 virtual address bits
// How long (in cycles) a secondary tag written with Index Store Tag is reported
// back verbatim by Index Load Tag; see execute_cache().
constexpr u64 kScacheTagWindow = 10'000'000;
constexpr u64 kNeverStored = ~u64{0};

inline unsigned f_rs(u32 i) { return (i >> 21) & 31; }
inline unsigned f_rt(u32 i) { return (i >> 16) & 31; }
inline unsigned f_rd(u32 i) { return (i >> 11) & 31; }
inline unsigned f_sa(u32 i) { return (i >> 6) & 31; }
inline u64 f_simm(u32 i) { return sext16(i & 0xFFFF); }
inline u64 f_zimm(u32 i) { return i & 0xFFFF; }

}  // namespace

R4000::R4000(Bus& bus, const Options& options) : bus_(bus), opts_(options), blocks_(1u << kBlockBits) {
    bus_.set_code_write_handler([this](const u8* page) { invalidate_code_page(page); });
    bus_.set_remap_handler([this] { flush_fast_tlb(); });  // cached host pointers are stale
    set_jit(true);
    reset();
}

R4000::~R4000() = default;

void R4000::set_jit(bool enabled) {
    if (enabled && !jit_buf_) {
        constexpr std::size_t kJitBufferSize = 32u << 20;
        jit_buf_ = std::make_unique<jit::ExecBuffer>(kJitBufferSize);
        if (!jit_buf_->ok() || !jit_compile_supported()) {
            jit_buf_.reset();
            SG_LOG(Info, "cpu", "recompiler not available on this host: running the block cache");
        } else {
#if defined(__aarch64__)
            SG_LOG(Info, "cpu", "recompiler on (AArch64)");
#else
            SG_LOG(Info, "cpu", "recompiler on (x86-64)");
#endif
        }
    } else if (!enabled && jit_buf_) {
        jit_flush();
        jit_buf_.reset();
        SG_LOG(Info, "cpu", "recompiler off");
    }
}

void R4000::jit_flush() {
    for (Block& b : blocks_) {
        b.code = nullptr;
        b.runs = 0;
    }
    if (jit_buf_) jit_buf_->clear();
}

void R4000::reset() {
    gpr.fill(0);
    fpr.fill(0);
    hi = lo = 0;
    fcr31 = 0;

    status_ = sr::BEV | sr::ERL;
    cause_ = 0;
    config_ = opts_.scache_bytes ? (opts_.config & ~kConfigNoScache) : (opts_.config | kConfigNoScache);
    prid_ = opts_.prid;
    index_ = wired_ = compare_ = pagemask_ = count_offset_ = 0;
    watchlo_ = watchhi_ = ecc_ = cacheerr_ = taglo_ = taghi_ = lladdr_ = 0;
    entrylo0_ = entrylo1_ = context_ = xcontext_ = badvaddr_ = 0;
    entryhi_ = epc_ = errorepc_ = 0;
    asid_tag_ = 0;
    llbit_ = false;
    random_base_ = cycles();                               // Random restarts at 47
    count_offset_ = 0u - static_cast<u32>(cycles() >> 1);  // Count restarts at 0
    tlb_.fill(TlbEntry{});

    // Tag arrays sized for the smallest line size the Config register allows.
    const u32 icache = 1u << (12 + ((config_ >> 9) & 7));
    const u32 dcache = 1u << (12 + ((config_ >> 6) & 7));
    itag_.assign(icache / 16, 0);
    dtag_.assign(dcache / 16, 0);
    const u32 sline = 16u << ((config_ >> 22) & 3);
    stag_.assign(opts_.scache_bytes / sline, 0);
    stag_stored_at_.assign(stag_.size(), kNeverStored);

    // The cycle counter is the machine's master clock and keeps running
    // through a reset (Count's value after reset is undefined anyway).
    loops_.fill(LoopEntry{});
    idle_ready_ = false;
    in_delay_ = false;
    update_mode();
    flush_fast_tlb();
    reschedule_timer();
    set_pc(kResetVector);
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

SG_FORCEINLINE void R4000::step_inline() {
    time_fp_ += weight_;
    if (SG_UNLIKELY(time_fp_ >= timer_event_fp_)) timer_fire();

    cur_pc_ = pc;
    in_delay_ = next_delay_;
    next_delay_ = false;

    if (SG_UNLIKELY(interrupt_pending())) {
        raise(Exc::Int);
        return;
    }

    u32 inst;
    if (SG_UNLIKELY(!fetch(inst))) return;
    pc = npc_;
    npc_ += 4;
    execute(inst);
    gpr[0] = 0;
}

void R4000::step() { step_inline(); }

u64 R4000::run(u64 max_instructions) {
    stop_.store(false, std::memory_order_relaxed);
    u64 n = 0;
    while (n < max_instructions) {
        const u64 chunk_end = (max_instructions - n > 4096) ? n + 4096 : max_instructions;
        for (; n < chunk_end; ++n) step();
        if (stop_.load(std::memory_order_relaxed)) break;
    }
    return n;
}

void R4000::run_until(u64 cycle, const u64* deadline) {
    static constexpr u64 kNever = ~u64{0};
    const u64* const limit = deadline ? deadline : &kNever;
    u64 n = 0;
    while (cycles() < cycle && cycles() < *limit) {
        if (SG_LIKELY(block_cache_)) run_block(cycle, limit);
        else step_inline();
        if (SG_UNLIKELY(idle_ready_)) {
            idle_ready_ = false;
            skip_idle(std::min(cycle, *limit));
        }
        if ((++n & 4095) == 0 && stop_.load(std::memory_order_relaxed)) break;
    }
}

// ---------------------------------------------------------------------------
// Decoded blocks
// ---------------------------------------------------------------------------

namespace {

// Control transfers: the block ends after the delay slot.
bool is_branch(u32 w) {
    const unsigned op = w >> 26;
    switch (op) {
        case 0x00: return (w & 0x3E) == 0x08;  // JR, JALR
        case 0x01: return ((w >> 16) & 0x0C) == 0x00;  // BLTZ/BGEZ(L)(AL)
        case 0x02:
        case 0x03:
        case 0x04:
        case 0x05:
        case 0x06:
        case 0x07:
        case 0x14:
        case 0x15:
        case 0x16:
        case 0x17: return true;
        case 0x10:
        case 0x11:
        case 0x12:
        case 0x13: return ((w >> 21) & 0x1F) == 0x08;  // BCz
        default: return false;
    }
}

// Instructions after which fetch translation or the code itself may differ:
// all of CP0 (MTC0, ERET, TLB writes...) and CACHE.
bool ends_block(u32 w) {
    const unsigned op = w >> 26;
    return (op == 0x10 && ((w >> 21) & 0x1F) != 0x08) || op == 0x2F;
}

}  // namespace

// Decodes a straight-line run from `host` up to (and including) a control
// transfer and its delay slot, a block-ending instruction, the end of the
// 4 KiB page, or the length limit.
void R4000::build_block(Block& b, const u8* host, u32 page_offset) {
    b.host = host;
    b.offset = page_offset;
    b.count = 0;
    b.runs = 0;
    b.code = nullptr;
    const u32 room = (0x1000 - page_offset) / 4;  // instructions left in the page
    const u32 max = std::min<u32>(room, kMaxBlockLength);
    b.plain = 0;
    auto add = [&](u32 w, bool delay_slot) {
        bool plain;
        const Handler fn = decode(w, plain);
        if (plain && !delay_slot) b.plain |= 1u << b.count;
        b.ops[b.count++] = Op{fn, w, static_cast<u8>(f_rs(w)), static_cast<u8>(f_rt(w)), static_cast<u8>(f_rd(w)),
                              static_cast<u8>(f_sa(w))};
    };
    while (b.count < max) {
        const u32 w = load_be<u32>(host + 4 * b.count);
        add(w, false);
        if (is_branch(w)) {
            if (b.count < max) add(load_be<u32>(host + 4 * b.count), true);
            break;
        }
        if (ends_block(w)) break;
    }
    if (u8* flag = bus_.code_flag(host)) *flag = 1;  // writes to this page now drop its blocks
}

// `page` is the host address of a 4 KiB page that was written.
void R4000::invalidate_code_page(const u8* page) {
    if (u8* flag = bus_.code_flag(page)) *flag = 0;
    for (u32 off = 0; off < 0x1000; off += 4) {
        Block& b = blocks_[block_index(page + off)];
        if (b.host >= page && b.host < page + 0x1000) b.host = nullptr;
    }
    block_invalidated_ = true;
}

void R4000::flush_blocks() {
    for (Block& b : blocks_) b.host = nullptr;
    jit_flush();
}

bool R4000::is_branch_word(u32 w) { return is_branch(w); }

// Runs blocks from pc, with exactly the per-instruction semantics of
// step(). A block is left when control leaves the straight line (branch,
// exception, interrupt) or its memory is written; execution then carries on
// directly with the block at the new pc while that is in the same virtual
// page, translated the same way (blocks are linked within a page). It
// returns when control leaves the page, an idle loop is confirmed, or the
// run limit is reached.
//
// The per-instruction checks of step() are only needed where their outcome
// can change: the run limit and the Compare timer are turned into one time
// below which neither can trigger, and interrupt lines only change when the
// timer fires, a device is accessed or CP0 is written.
void R4000::run_block(u64 cycle, const u64* limit) {
    const unsigned mode = mode_;
    const u64 vpage = pc >> 12;
    const FastEntry& fe = fast_[mode][static_cast<unsigned>(Access::Fetch)][vpage & (kFastEntries - 1)];
    if (SG_UNLIKELY(next_delay_ || (pc & 3) || fe.tag != (vpage | asid_tag_) || fe.host == nullptr)) {
        step_inline();  // delay slot on a new page, misaligned or untranslated pc, uncached fetch
        return;
    }
    const u8* const page = fe.host;

    // Below `safe` an instruction neither reaches the run limit nor fires
    // the timer; recomputed after anything that could move either.
    auto safe_time = [&] {
        const u64 stop = std::min(cycle, *limit) << kCycleFracBits;
        const u64 timer = timer_event_fp_ > weight_ ? timer_event_fp_ - weight_ : 0;
        return std::min(stop, timer);
    };
    u64 safe = safe_time();
    bool check_irq = true;
    bool first = true;  // the caller has checked the run limit for the first instruction
    for (unsigned linked = 0; linked < kMaxLinkedBlocks; ++linked) {
        const u32 offset = static_cast<u32>(pc & 0xFFF);
        const u8* const host = page + offset;
        Block* blk = &blocks_[block_index(host)];
        if (blk->host != host) build_block(*blk, host, offset);
        block_invalidated_ = false;
        if (jit_buf_ && !blk->code && ++blk->runs == kJitThreshold && !jit_compile(*blk)) {
            jit_flush();  // the code buffer is full: start afresh
            jit_compile(*blk);
        }

        // Plain operations only advance the time (kept in `t`), so pc, npc_
        // and the delay-slot state are brought up to date (`synced`) just
        // before anything that can observe them, and when the block is left.
        u64 ipc = pc;  // address of ops[i]
        u64 t = time_fp_;
        bool synced = true;
        u32 i = 0;
        bool leave = false;
        // Translated code runs the block when no time check falls inside it
        // and no interrupt is pending; it stops early where the loop below
        // would break, or after an access that may need the checks redone,
        // and may carry on directly into other translated blocks.
        if (blk->code && blk->jit_addr64 == addr64_ && t + (blk->count - 1) * weight_ < safe) {
            if (check_irq && !interrupt_pending()) check_irq = false;
            if (!check_irq) {
                jit_safe_ = safe;
                i = blk->code(this, ipc, t);
                // It may have gone on into other blocks, in this page or others.
                blk = jit_block_;
                ipc = jit_entry_pc_ + 4 * u64{i};
                t = time_fp_;
                first = false;
                if (SG_UNLIKELY(io_)) {
                    check_irq = true;
                    safe = safe_time();
                }
                leave = pc != ipc || block_invalidated_ || idle_ready_;
            }
        }
        for (; !leave && i < blk->count; ++i, ipc += 4) {
            if (SG_UNLIKELY(t >= safe)) {
                time_fp_ = t;
                if (!first && !(cycles() < cycle && cycles() < *limit)) {
                    if (!synced) set_pc(ipc);
                    return;
                }
                time_fp_ = t += weight_;
                if (time_fp_ >= timer_event_fp_) timer_fire();
                check_irq = true;
                safe = safe_time();
            } else {
                t += weight_;
            }
            first = false;
            if (SG_UNLIKELY(check_irq)) {
                check_irq = false;
                if (interrupt_pending()) {
                    time_fp_ = t;
                    cur_pc_ = ipc;
                    in_delay_ = next_delay_;
                    next_delay_ = false;
                    raise(Exc::Int);
                    break;
                }
            }
            const Op& op = blk->ops[i];
            if ((blk->plain >> i) & 1) {
                op.fn(*this, op);
                synced = false;
                continue;
            }
            time_fp_ = t;
            cur_pc_ = ipc;
            in_delay_ = next_delay_;
            next_delay_ = false;
            if (in_delay_) {
                pc = npc_;
                npc_ += 4;
            } else {
                pc = ipc + 4;
                npc_ = ipc + 8;
            }
            io_ = false;
            op.fn(*this, op);
            gpr[0] = 0;
            t = time_fp_;
            synced = true;
            if (SG_UNLIKELY(io_)) {  // a device or CP0 may have raised an interrupt or moved an event
                check_irq = true;
                safe = safe_time();
            }
            if (pc != ipc + 4 || block_invalidated_ || idle_ready_) break;
        }
        if (i == blk->count) {  // ran off the end of the block
            time_fp_ = t;
            if (!synced) set_pc(ipc);
        }

        // Link to the block at the new pc if nothing it relies on changed.
        if (idle_ready_ || next_delay_ || (pc >> 12) != vpage || (pc & 3) || mode_ != mode || fe.tag != (vpage | asid_tag_) ||
            fe.host != page)
            return;
    }
}

void R4000::set_cycles_per_instruction(double cpi) {
    const double w = cpi * (1u << kCycleFracBits);
    const u64 weight = w < 1 ? 1 : static_cast<u64>(w + 0.5);
    if (weight == weight_) return;
    instret_base_ = instructions();
    instret_time_fp_ = time_fp_;
    instret_skip_fp_ = idle_skipped_fp_;
    weight_ = weight;
    ++weight_epoch_;  // iteration lengths measured in guest time are no longer comparable
}

// ---------------------------------------------------------------------------
// Idle detection
// ---------------------------------------------------------------------------

namespace {
constexpr u64 kMaxIdleLoopInstructions = 4096;
constexpr unsigned kIdleCooldown = 1024;  // visits to ignore after a loop proved not idle
}  // namespace

namespace {
u64 read_host(const u8* p, unsigned size) {
    switch (size) {
        case 1: return *p;
        case 2: return load_be<u16>(p);
        case 4: return load_be<u32>(p);
        default: return load_be<u64>(p);
    }
}
}  // namespace

// The previous iteration's registers, stores (locations and the values they
// were left holding) and length, compared with the iteration just finished.
bool R4000::snapshot_matches(const LoopEntry& e, u64 period, unsigned nstores) const {
    if (!e.have_snapshot || period != e.period_fp || nstores != e.nstores || e.regs[32] != hi || e.regs[33] != lo ||
        !std::equal(gpr.begin(), gpr.end(), e.regs.begin()))
        return false;
    constexpr u64 mask = (1u << kStoreLogBits) - 1;
    for (unsigned i = 0; i < nstores; ++i) {
        const StoreRec& now = store_log_[(store_count_ - nstores + i) & mask];
        const StoreRec& then = e.stores[i];
        if (now.host != then.host || now.size != then.size || read_host(now.host, now.size) != then.value) return false;
    }
    return true;
}

void R4000::take_snapshot(LoopEntry& e, u64 period, unsigned nstores) {
    std::copy(gpr.begin(), gpr.end(), e.regs.begin());
    e.regs[32] = hi;
    e.regs[33] = lo;
    constexpr u64 mask = (1u << kStoreLogBits) - 1;
    for (unsigned i = 0; i < nstores; ++i) {
        StoreRec r = store_log_[(store_count_ - nstores + i) & mask];
        r.value = read_host(r.host, r.size);
        e.stores[i] = r;
    }
    e.nstores = nstores;
    e.period_fp = period;
    e.have_snapshot = true;
}

// Called when a branch to an earlier address is taken; see set_idle_skip().
void R4000::loop_visit(LoopEntry& e, u64 target) {
    (void)target;
    if (!idle_skip_) return;
    const u64 period = time_fp_ - e.time_fp;
    const u64 stores = store_count_ - e.store_pos;
    const bool same = e.branch_pc == cur_pc_ && e.epoch == change_epoch_ && e.weight_epoch == weight_epoch_ &&
                      period <= kMaxIdleLoopInstructions * weight_ && stores <= (1u << kStoreLogBits);
    e.time_fp = time_fp_;
    e.epoch = change_epoch_;
    e.weight_epoch = weight_epoch_;
    e.store_pos = store_count_;
    if (!same) {
        e.branch_pc = cur_pc_;
        e.have_snapshot = false;
        e.cooldown = 0;
        return;
    }
    const unsigned n = static_cast<unsigned>(stores);
    if (snapshot_matches(e, period, n)) {
        idle_period_fp_ = period;
        idle_ready_ = true;
        return;
    }
    if (e.have_snapshot) e.cooldown = kIdleCooldown;  // a loop, but one that makes progress
    take_snapshot(e, period, n);
}

// Fast-forwards whole iterations of a confirmed spin loop, stopping strictly
// before the next event (the Compare timer, or `limit_cycle`: the next
// device event or the end of the run slice), so that event is taken by the
// same instruction it would have been without skipping.
void R4000::skip_idle(u64 limit_cycle) {
    if (idle_period_fp_ == 0 || interrupt_pending()) return;
    const u64 limit_fp = std::min(limit_cycle << kCycleFracBits, timer_event_fp_);
    if (limit_fp <= time_fp_ + 1) return;
    const u64 iterations = (limit_fp - 1 - time_fp_) / idle_period_fp_;
    if (iterations == 0) return;
    const u64 advance = iterations * idle_period_fp_;
    time_fp_ += advance;
    for (LoopEntry& e : loops_) e.time_fp += advance;  // keep visit times consistent
    idle_skipped_fp_ += advance;
}

void R4000::set_int_line(unsigned line, bool asserted) {
    if (line > 4) return;
    const u32 bit = 1u << (10 + line);
    cause_ = asserted ? (cause_ | bit) : (cause_ & ~bit);
}

void R4000::timer_fire() {
    cause_ |= kCauseIP7;
    reschedule_timer();
}

// Count increments every other pipeline cycle; compute the cycle at which it
// next equals Compare so the hot loop only has to do one comparison.
void R4000::reschedule_timer() {
    const u64 now_half = cycles() >> 1;
    const u32 k = compare_ - count();
    const u64 delta = k ? k : (u64{1} << 32);
    timer_event_fp_ = ((now_half + delta) << 1) << kCycleFracBits;
}

// ---------------------------------------------------------------------------
// Exceptions
// ---------------------------------------------------------------------------

void R4000::raise(Exc code, bool tlb_refill, bool xtlb, unsigned ce) {
    touch();
    if (exception_hook) [[unlikely]]
        exception_hook(static_cast<unsigned>(code), cur_pc_, status_);
    const bool exl = (status_ & sr::EXL) != 0;
    if (!exl) {
        epc_ = in_delay_ ? cur_pc_ - 4 : cur_pc_;
        cause_ = in_delay_ ? (cause_ | kCauseBD) : (cause_ & ~kCauseBD);
    }
    cause_ = (cause_ & ~0x3000007Cu) | (static_cast<u32>(code) << 2) | ((ce & 3u) << 28);

    const u64 base = (status_ & sr::BEV) ? kBevBase : kNormalBase;
    const u64 offset = (tlb_refill && !exl) ? (xtlb ? 0x080 : 0x000) : 0x180;
    status_ |= sr::EXL;
    update_mode();
    set_pc(base + offset);
}

void R4000::address_error(u64 vaddr, Access acc) {
    badvaddr_ = vaddr;
    raise(acc == Access::Store ? Exc::AdES : Exc::AdEL);
}

void R4000::tlb_exception(Exc code, u64 vaddr, bool refill, bool xtlb) {
    badvaddr_ = vaddr;
    context_ = (context_ & ~0x7FFFFFull) | ((vaddr >> 9) & 0x7FFFF0ull);
    xcontext_ = (xcontext_ & ~0x1FFFFFFFFull) | (((vaddr >> 62) & 3) << 31) | ((vaddr >> 9) & 0x7FFFFFF0ull);
    entryhi_ = (entryhi_ & 0xFF) | (vaddr & kVpn2Mask);
    raise(code, refill, xtlb);
}

// ---------------------------------------------------------------------------
// Address translation
// ---------------------------------------------------------------------------

void R4000::update_mode() {
    if (status_ & sr::ERL) {
        mode_ = kKernelErl;
    } else if (status_ & sr::EXL) {
        mode_ = kKernel;
    } else {
        switch ((status_ & sr::KSU_MASK) >> 3) {
            case 0: mode_ = kKernel; break;
            case 1: mode_ = kSupervisor; break;
            default: mode_ = kUser; break;
        }
    }
    switch (mode_) {
        case kUser: addr64_ = (status_ & sr::UX) != 0; break;
        case kSupervisor: addr64_ = (status_ & sr::SX) != 0; break;
        default: addr64_ = (status_ & sr::KX) != 0; break;
    }
}

bool R4000::ops64_enabled() const {
    switch (mode_) {
        case kUser: return (status_ & sr::UX) != 0;
        case kSupervisor: return (status_ & sr::SX) != 0;
        default: return true;  // 64-bit operations are always valid in kernel mode
    }
}

void R4000::flush_fast_tlb() {
    for (auto& mode : fast_)
        for (auto& table : mode)
            for (auto& e : table) e = FastEntry{};
}

// Drops the soft-TLB slots a TLB entry's pages can occupy (in every mode).
// Slots are indexed by the low bits of the 4 KiB page number, which the
// entry determines whatever the upper address bits are; anything else
// sharing those slots is merely re-translated.
void R4000::flush_fast_tlb_entry(const TlbEntry& e) {
    const u64 span = (e.mask | 0x1FFF) + 1;  // both pages of the pair
    const u64 pages = span >> 12;
    if (pages >= kFastEntries) {
        flush_fast_tlb();
        return;
    }
    const u64 first = (e.hi >> 12) & ~(pages - 1);
    for (u64 p = 0; p < pages; ++p) {
        const unsigned slot = static_cast<unsigned>((first + p) & (kFastEntries - 1));
        for (auto& mode : fast_)
            for (auto& table : mode) table[slot] = FastEntry{};
    }
}

template <bool kRaise>
bool R4000::translate(u64 vaddr, Access acc, u64& paddr) {
    switch (mode_) {
        case kUser:
            if (status_ & sr::UX) {
                if (vaddr < kXuseg) return tlb_lookup<kRaise>(vaddr, acc, true, paddr);
            } else if (vaddr < 0x80000000ull) {
                return tlb_lookup<kRaise>(vaddr, acc, false, paddr);
            }
            break;

        case kSupervisor: {
            const bool csseg = vaddr >= kKseg2Base && vaddr < 0xFFFFFFFFE0000000ull;
            if (status_ & sr::SX) {
                const bool xsseg = (vaddr >> 62) == 1 && (vaddr & 0x3FFFFF0000000000ull) == 0;
                if (vaddr < kXuseg || xsseg || csseg) return tlb_lookup<kRaise>(vaddr, acc, true, paddr);
            } else if (vaddr < 0x80000000ull || csseg) {
                return tlb_lookup<kRaise>(vaddr, acc, false, paddr);
            }
            break;
        }

        default: {  // kernel
            const bool kx = (status_ & sr::KX) != 0;
            if (vaddr >= k32BitCompatBase) {
                if (vaddr < kKseg2Base) {  // kseg0 / kseg1
                    paddr = vaddr & 0x1FFFFFFFull;
                    return true;
                }
                return tlb_lookup<kRaise>(vaddr, acc, kx, paddr);  // ksseg / kseg3
            }
            if (vaddr < 0x80000000ull) {
                if (mode_ == kKernelErl) {  // ERL: kuseg is unmapped and uncached
                    paddr = vaddr;
                    return true;
                }
                return tlb_lookup<kRaise>(vaddr, acc, kx, paddr);
            }
            if (!kx) break;
            if (vaddr < kXuseg) return tlb_lookup<kRaise>(vaddr, acc, true, paddr);  // xkuseg
            switch (vaddr >> 62) {
                case 1:  // xksseg
                    if ((vaddr & 0x3FFFFF0000000000ull) == 0) return tlb_lookup<kRaise>(vaddr, acc, true, paddr);
                    break;
                case 2:  // xkphys: bits 61:59 select cache algorithm, 58:36 must be zero
                    if ((vaddr & 0x07FFFFF000000000ull) == 0) {
                        paddr = vaddr & 0xFFFFFFFFFull;
                        return true;
                    }
                    break;
                case 3:  // xkseg
                    if (vaddr <= 0xC00000FF7FFFFFFFull) return tlb_lookup<kRaise>(vaddr, acc, true, paddr);
                    break;
            }
            break;
        }
    }
    if constexpr (kRaise) address_error(vaddr, acc);
    return false;
}

template <bool kRaise>
bool R4000::tlb_lookup(u64 vaddr, Access acc, bool xtlb, u64& paddr) {
    const u64 asid = entryhi_ & 0xFF;
    for (const TlbEntry& e : tlb_) {
        if (((vaddr ^ e.hi) & kVpn2Mask & ~e.mask) != 0) continue;
        if (!e.global && (e.hi & 0xFF) != asid) continue;

        const u64 offset_mask = (e.mask >> 1) | 0xFFF;
        const u64 entry_lo = e.lo[(vaddr & (offset_mask + 1)) ? 1 : 0];
        if (!(entry_lo & 2)) {  // V
            if constexpr (kRaise)
                tlb_exception(acc == Access::Store ? Exc::TLBS : Exc::TLBL, vaddr, false, xtlb);
            return false;
        }
        if (acc == Access::Store && !(entry_lo & 4)) {  // D
            if constexpr (kRaise) tlb_exception(Exc::Mod, vaddr, false, xtlb);
            return false;
        }
        paddr = ((((entry_lo >> 6) & 0xFFFFFF) << 12) & ~offset_mask) | (vaddr & offset_mask);
        return true;
    }
    if constexpr (kRaise) tlb_exception(acc == Access::Store ? Exc::TLBS : Exc::TLBL, vaddr, true, xtlb);
    return false;
}

bool R4000::lookup_slow(u64 vaddr, Access acc, u8*& host, u64& paddr) {
    if (!translate<true>(vaddr, acc, paddr)) return false;
    const u64 ppage = paddr & ~u64{0xFFF};
    const bool is_store = acc == Access::Store;
    host = is_store ? bus_.host_write_ptr(ppage) : bus_.host_read_ptr(ppage);
    FastEntry& e = fast_[mode_][static_cast<unsigned>(acc)][(vaddr >> 12) & (kFastEntries - 1)];
    e.tag = (vaddr >> 12) | asid_tag_;
    e.host = host;
    e.ppage = ppage;
    e.code = host ? bus_.code_flag(host) : nullptr;
    if (is_store) store_code_ = e.code;
    return true;
}

// On success `host` is the host pointer to the 4 KiB page (nullptr for MMIO)
// and `paddr` the full physical address.
template <R4000::Access A>
SG_FORCEINLINE bool R4000::lookup(u64 vaddr, u8*& host, u64& paddr) {
    const FastEntry& e = fast_[mode_][static_cast<unsigned>(A)][(vaddr >> 12) & (kFastEntries - 1)];
    if (SG_LIKELY(e.tag == ((vaddr >> 12) | asid_tag_))) {
        host = e.host;
        paddr = e.ppage | (vaddr & 0xFFF);
        if constexpr (A == Access::Store) store_code_ = e.code;
        return true;
    }
    return lookup_slow(vaddr, A, host, paddr);
}

bool R4000::debug_translate(u64 vaddr, u64& paddr) const {
    // translate<false> performs no writes; the cast only satisfies the template's signature.
    return const_cast<R4000*>(this)->translate<false>(vaddr, Access::Load, paddr);
}

bool R4000::debug_read32(u64 vaddr, u32& value) const {
    u64 paddr;
    if ((vaddr & 3) || !debug_translate(vaddr, paddr)) return false;
    const u8* host = bus_.host_read_ptr(paddr);
    if (!host) return false;  // never touch MMIO from the debugger
    value = load_be<u32>(host);
    return true;
}

// ---------------------------------------------------------------------------
// Memory access
// ---------------------------------------------------------------------------

template <typename T>
SG_FORCEINLINE bool R4000::phys_read(u64 paddr, T& out, Access acc) {
    io_ = true;
    if (acc != Access::Load || !bus_.read_is_pure(paddr, sizeof(T))) touch();
    if (SG_LIKELY(bus_.read<T>(paddr, out))) return true;
    raise(acc == Access::Fetch ? Exc::IBE : Exc::DBE);
    return false;
}

template <typename T>
SG_FORCEINLINE bool R4000::phys_write(u64 paddr, T value) {
    io_ = true;
    touch();
    if (SG_LIKELY(bus_.write<T>(paddr, value))) return true;
    raise(Exc::DBE);
    return false;
}

SG_FORCEINLINE bool R4000::fetch(u32& inst) {
    if (SG_UNLIKELY(pc & 3)) {
        address_error(pc, Access::Fetch);
        return false;
    }
    u8* host;
    u64 paddr;
    if (SG_UNLIKELY(!lookup<Access::Fetch>(pc, host, paddr))) return false;
    if (SG_LIKELY(host != nullptr)) {
        inst = load_be<u32>(host + (pc & 0xFFF));
        return true;
    }
    return phys_read<u32>(paddr, inst, Access::Fetch);
}

template <typename T>
SG_FORCEINLINE bool R4000::load(u64 vaddr, T& out) {
    if (SG_UNLIKELY(vaddr & (sizeof(T) - 1))) {
        address_error(vaddr, Access::Load);
        return false;
    }
    u8* host;
    u64 paddr;
    if (SG_UNLIKELY(!lookup<Access::Load>(vaddr, host, paddr))) return false;
    if (SG_LIKELY(host != nullptr)) {
        out = load_be<T>(host + (vaddr & 0xFFF));
        return true;
    }
    return phys_read<T>(paddr, out, Access::Load);
}

template <typename T>
SG_FORCEINLINE bool R4000::store(u64 vaddr, T value) {
    if (SG_UNLIKELY(vaddr & (sizeof(T) - 1))) {
        address_error(vaddr, Access::Store);
        return false;
    }
    u8* host;
    u64 paddr;
    if (SG_UNLIKELY(!lookup<Access::Store>(vaddr, host, paddr))) return false;
    if (SG_LIKELY(host != nullptr)) {
        u8* const p = host + (vaddr & 0xFFF);
        log_store(p, sizeof(T));
        store_be<T>(p, value);
        if (SG_UNLIKELY(*store_code_)) invalidate_code_page(host);  // decoded code lives here
        return true;
    }
    return phys_write<T>(paddr, value);
}

template <typename T>
bool R4000::load_containing(u64 vaddr, T& out) {
    u8* host;
    u64 paddr;
    if (!lookup<Access::Load>(vaddr, host, paddr)) return false;
    constexpr u64 align = sizeof(T) - 1;
    if (host) {
        out = load_be<T>(host + (vaddr & 0xFFF & ~align));
        return true;
    }
    return phys_read<T>(paddr & ~align, out, Access::Load);
}

template <typename T>
bool R4000::merge_store(u64 vaddr, T value, T keep_mask) {
    touch();
    u8* host;
    u64 paddr;
    if (!lookup<Access::Store>(vaddr, host, paddr)) return false;
    constexpr u64 align = sizeof(T) - 1;
    if (host) {
        u8* p = host + (vaddr & 0xFFF & ~align);
        store_be<T>(p, static_cast<T>((load_be<T>(p) & keep_mask) | value));
        if (SG_UNLIKELY(*store_code_)) invalidate_code_page(host);
        return true;
    }
    T old;
    if (!phys_read<T>(paddr & ~align, old, Access::Load)) return false;
    return phys_write<T>(paddr & ~align, static_cast<T>((old & keep_mask) | value));
}

// ---------------------------------------------------------------------------
// Instruction execution
// ---------------------------------------------------------------------------

#define SG_REQUIRE_64BIT()      \
    if (!ops64_enabled()) {     \
        raise(Exc::RI);         \
        return;                 \
    }

void R4000::execute(u32 inst) {
    const unsigned rs = f_rs(inst), rt = f_rt(inst);
    const u64 simm = f_simm(inst), zimm = f_zimm(inst);
    u64* const r = gpr.data();
    const u64 btarget = cur_pc_ + 4 + (simm << 2);

    switch (inst >> 26) {
        case 0x00: execute_special(inst); break;
        case 0x01: execute_regimm(inst); break;
        case 0x02: {  // J
            const u64 target = ((cur_pc_ + 4) & ~0x0FFFFFFFull) | (u64{inst & 0x03FFFFFF} << 2);
            jump(target);
            if (target <= cur_pc_) backward_branch(target);
            break;
        }
        case 0x03:                                                                              // JAL
            r[31] = cur_pc_ + 8;
            jump(((cur_pc_ + 4) & ~0x0FFFFFFFull) | (u64{inst & 0x03FFFFFF} << 2));
            break;
        case 0x04: branch(r[rs] == r[rt], btarget); break;                              // BEQ
        case 0x05: branch(r[rs] != r[rt], btarget); break;                              // BNE
        case 0x06: branch(static_cast<s64>(r[rs]) <= 0, btarget); break;                // BLEZ
        case 0x07: branch(static_cast<s64>(r[rs]) > 0, btarget); break;                 // BGTZ
        case 0x08: {                                                                    // ADDI
            const s64 res = s64{static_cast<s32>(r[rs])} + s64{static_cast<s32>(simm)};
            if (res != static_cast<s32>(res)) {
                raise(Exc::Ov);
                return;
            }
            r[rt] = sext32(static_cast<u64>(res));
            break;
        }
        case 0x09: r[rt] = sext32(static_cast<u32>(r[rs]) + static_cast<u32>(simm)); break;  // ADDIU
        case 0x0A: r[rt] = static_cast<s64>(r[rs]) < static_cast<s64>(simm); break;           // SLTI
        case 0x0B: r[rt] = r[rs] < simm; break;                                               // SLTIU
        case 0x0C: r[rt] = r[rs] & zimm; break;                                               // ANDI
        case 0x0D: r[rt] = r[rs] | zimm; break;                                               // ORI
        case 0x0E: r[rt] = r[rs] ^ zimm; break;                                               // XORI
        case 0x0F: r[rt] = sext32(zimm << 16); break;                                         // LUI
        case 0x10: execute_cop0(inst); break;
        case 0x11: execute_cop1(inst); break;
        case 0x12:  // COP2
        case 0x13:  // COP3
        {
            const unsigned z = (inst >> 26) & 3;
            if (!(status_ & (sr::CU0 << z))) raise(Exc::CpU, false, false, z);
            else raise(Exc::RI);
            return;
        }
        case 0x14: branch_likely(r[rs] == r[rt], btarget); break;                         // BEQL
        case 0x15: branch_likely(r[rs] != r[rt], btarget); break;                         // BNEL
        case 0x16: branch_likely(static_cast<s64>(r[rs]) <= 0, btarget); break;           // BLEZL
        case 0x17: branch_likely(static_cast<s64>(r[rs]) > 0, btarget); break;            // BGTZL
        case 0x18: {                                                                      // DADDI
            SG_REQUIRE_64BIT();
            const u64 res = r[rs] + simm;
            if (((r[rs] ^ res) & (simm ^ res)) >> 63) {
                raise(Exc::Ov);
                return;
            }
            r[rt] = res;
            break;
        }
        case 0x19:  // DADDIU
            SG_REQUIRE_64BIT();
            r[rt] = r[rs] + simm;
            break;
        case 0x1A: {  // LDL
            SG_REQUIRE_64BIT();
            const u64 va = effective_address(r[rs], simm);
            const unsigned sh = 8 * (va & 7);
            u64 d;
            if (!load_containing<u64>(va, d)) return;
            r[rt] = (d << sh) | (r[rt] & ((u64{1} << sh) - 1));
            break;
        }
        case 0x1B: {  // LDR
            SG_REQUIRE_64BIT();
            const u64 va = effective_address(r[rs], simm);
            const unsigned sh = 8 * (7 - (va & 7));
            u64 d;
            if (!load_containing<u64>(va, d)) return;
            r[rt] = (d >> sh) | (r[rt] & ~(~u64{0} >> sh));
            break;
        }
        case 0x20: {  // LB
            u8 v;
            if (load<u8>(effective_address(r[rs], simm), v)) r[rt] = sext8(v);
            break;
        }
        case 0x21: {  // LH
            u16 v;
            if (load<u16>(effective_address(r[rs], simm), v)) r[rt] = sext16(v);
            break;
        }
        case 0x22: {  // LWL
            const u64 va = effective_address(r[rs], simm);
            const unsigned sh = 8 * (va & 3);
            u32 w;
            if (!load_containing<u32>(va, w)) return;
            const u32 keep = static_cast<u32>((u64{1} << sh) - 1);
            r[rt] = sext32((w << sh) | (static_cast<u32>(r[rt]) & keep));
            break;
        }
        case 0x23: {  // LW
            u32 v;
            if (load<u32>(effective_address(r[rs], simm), v)) r[rt] = sext32(v);
            break;
        }
        case 0x24: {  // LBU
            u8 v;
            if (load<u8>(effective_address(r[rs], simm), v)) r[rt] = v;
            break;
        }
        case 0x25: {  // LHU
            u16 v;
            if (load<u16>(effective_address(r[rs], simm), v)) r[rt] = v;
            break;
        }
        case 0x26: {  // LWR
            const u64 va = effective_address(r[rs], simm);
            const unsigned b = va & 3;
            const unsigned sh = 8 * (3 - b);
            u32 w;
            if (!load_containing<u32>(va, w)) return;
            const u32 merged = (w >> sh) | (static_cast<u32>(r[rt]) & ~(0xFFFFFFFFu >> sh));
            // A full word load sign-extends; a partial one leaves bits 63:32 alone.
            r[rt] = (b == 3) ? sext32(merged) : ((r[rt] & 0xFFFFFFFF00000000ull) | merged);
            break;
        }
        case 0x27: {  // LWU
            SG_REQUIRE_64BIT();
            u32 v;
            if (load<u32>(effective_address(r[rs], simm), v)) r[rt] = v;
            break;
        }
        case 0x28: store<u8>(effective_address(r[rs], simm), static_cast<u8>(r[rt])); break;    // SB
        case 0x29: store<u16>(effective_address(r[rs], simm), static_cast<u16>(r[rt])); break;  // SH
        case 0x2A: {                                                           // SWL
            const u64 va = effective_address(r[rs], simm);
            const unsigned sh = 8 * (va & 3);
            merge_store<u32>(va, static_cast<u32>(r[rt]) >> sh, ~(0xFFFFFFFFu >> sh));
            break;
        }
        case 0x2B: store<u32>(effective_address(r[rs], simm), static_cast<u32>(r[rt])); break;  // SW
        case 0x2C: {                                                           // SDL
            SG_REQUIRE_64BIT();
            const u64 va = effective_address(r[rs], simm);
            const unsigned sh = 8 * (va & 7);
            merge_store<u64>(va, r[rt] >> sh, ~(~u64{0} >> sh));
            break;
        }
        case 0x2D: {  // SDR
            SG_REQUIRE_64BIT();
            const u64 va = effective_address(r[rs], simm);
            const unsigned sh = 8 * (7 - (va & 7));
            merge_store<u64>(va, r[rt] << sh, ~(~u64{0} << sh));
            break;
        }
        case 0x2E: {  // SWR
            const u64 va = effective_address(r[rs], simm);
            const unsigned sh = 8 * (3 - (va & 3));
            merge_store<u32>(va, static_cast<u32>(r[rt]) << sh, ~(0xFFFFFFFFu << sh));
            break;
        }
        case 0x2F: execute_cache(inst); break;  // CACHE
        case 0x30: {  // LL
            const u64 va = effective_address(r[rs], simm);
            u32 v;
            if (!load<u32>(va, v)) return;
            u64 paddr = 0;
            debug_translate(va, paddr);
            r[rt] = sext32(v);
            lladdr_ = static_cast<u32>(paddr >> 4);
            llbit_ = true;
            break;
        }
        case 0x34: {  // LLD
            SG_REQUIRE_64BIT();
            const u64 va = effective_address(r[rs], simm);
            u64 v;
            if (!load<u64>(va, v)) return;
            u64 paddr = 0;
            debug_translate(va, paddr);
            r[rt] = v;
            lladdr_ = static_cast<u32>(paddr >> 4);
            llbit_ = true;
            break;
        }
        case 0x37: {  // LD
            SG_REQUIRE_64BIT();
            u64 v;
            if (load<u64>(effective_address(r[rs], simm), v)) r[rt] = v;
            break;
        }
        case 0x38:  // SC
            if (llbit_) {
                if (!store<u32>(effective_address(r[rs], simm), static_cast<u32>(r[rt]))) return;
                r[rt] = 1;
            } else {
                r[rt] = 0;
            }
            break;
        case 0x3C:  // SCD
            SG_REQUIRE_64BIT();
            if (llbit_) {
                if (!store<u64>(effective_address(r[rs], simm), r[rt])) return;
                r[rt] = 1;
            } else {
                r[rt] = 0;
            }
            break;
        case 0x3F:  // SD
            SG_REQUIRE_64BIT();
            store<u64>(effective_address(r[rs], simm), r[rt]);
            break;

        // FPU loads/stores
        case 0x31:  // LWC1
        case 0x35:  // LDC1
        case 0x39:  // SWC1
        case 0x3D:  // SDC1
        {
            touch();  // FPU registers are not part of the idle-loop comparison
            if (!(status_ & sr::CU1)) {
                raise(Exc::CpU, false, false, 1);
                return;
            }
            const u64 va = effective_address(r[rs], simm);
            switch (inst >> 26) {
                case 0x31: {
                    u32 v;
                    if (load<u32>(va, v)) fpr_write32(rt, v);
                    break;
                }
                case 0x35: {
                    u64 v;
                    if (load<u64>(va, v)) fpr_write64(rt, v);
                    break;
                }
                case 0x39: store<u32>(va, fpr_read32(rt)); break;
                case 0x3D: store<u64>(va, fpr_read64(rt)); break;
            }
            break;
        }

        // COP2 loads/stores: no COP2 on IP22
        case 0x32:
        case 0x36:
        case 0x3A:
        case 0x3E:
            if (!(status_ & sr::CU2)) raise(Exc::CpU, false, false, 2);
            else raise(Exc::RI);
            return;

        default: raise(Exc::RI); return;
    }
}

void R4000::execute_special(u32 inst) {
    const unsigned rs = f_rs(inst), rt = f_rt(inst), rd = f_rd(inst), sa = f_sa(inst);
    u64* const r = gpr.data();

    switch (inst & 63) {
        case 0x00: r[rd] = sext32(static_cast<u32>(r[rt]) << sa); break;                              // SLL
        case 0x02: r[rd] = sext32(static_cast<u32>(r[rt]) >> sa); break;                              // SRL
        case 0x03: r[rd] = sext32(static_cast<u32>(static_cast<s32>(r[rt]) >> sa)); break;            // SRA
        case 0x04: r[rd] = sext32(static_cast<u32>(r[rt]) << (r[rs] & 31)); break;                    // SLLV
        case 0x06: r[rd] = sext32(static_cast<u32>(r[rt]) >> (r[rs] & 31)); break;                    // SRLV
        case 0x07: r[rd] = sext32(static_cast<u32>(static_cast<s32>(r[rt]) >> (r[rs] & 31))); break;  // SRAV
        case 0x08: jump(r[rs]); break;                                                                // JR
        case 0x09: {                                                                                  // JALR
            const u64 target = r[rs];
            r[rd] = cur_pc_ + 8;
            jump(target);
            break;
        }
        case 0x0C: raise(Exc::Sys); return;  // SYSCALL
        case 0x0D: raise(Exc::Bp); return;   // BREAK
        case 0x0F: break;                    // SYNC
        case 0x10: r[rd] = hi; break;        // MFHI
        case 0x11: hi = r[rs]; break;        // MTHI
        case 0x12: r[rd] = lo; break;        // MFLO
        case 0x13: lo = r[rs]; break;        // MTLO
        case 0x14:                           // DSLLV
            SG_REQUIRE_64BIT();
            r[rd] = r[rt] << (r[rs] & 63);
            break;
        case 0x16:  // DSRLV
            SG_REQUIRE_64BIT();
            r[rd] = r[rt] >> (r[rs] & 63);
            break;
        case 0x17:  // DSRAV
            SG_REQUIRE_64BIT();
            r[rd] = static_cast<u64>(static_cast<s64>(r[rt]) >> (r[rs] & 63));
            break;
        case 0x18: {  // MULT
            const s64 p = s64{static_cast<s32>(r[rs])} * s64{static_cast<s32>(r[rt])};
            lo = sext32(static_cast<u64>(p));
            hi = sext32(static_cast<u64>(p) >> 32);
            break;
        }
        case 0x19: {  // MULTU
            const u64 p = u64{static_cast<u32>(r[rs])} * u64{static_cast<u32>(r[rt])};
            lo = sext32(p);
            hi = sext32(p >> 32);
            break;
        }
        case 0x1A: {  // DIV
            const s32 n = static_cast<s32>(r[rs]), d = static_cast<s32>(r[rt]);
            if (d == 0) {  // result undefined; this matches observed R4000 behaviour
                lo = n >= 0 ? ~u64{0} : 1;
                hi = sext32(static_cast<u32>(n));
            } else if (n == INT32_MIN && d == -1) {
                lo = sext32(0x80000000u);
                hi = 0;
            } else {
                lo = sext32(static_cast<u32>(n / d));
                hi = sext32(static_cast<u32>(n % d));
            }
            break;
        }
        case 0x1B: {  // DIVU
            const u32 n = static_cast<u32>(r[rs]), d = static_cast<u32>(r[rt]);
            if (d == 0) {
                lo = ~u64{0};
                hi = sext32(n);
            } else {
                lo = sext32(n / d);
                hi = sext32(n % d);
            }
            break;
        }
        case 0x1C:  // DMULT
            SG_REQUIRE_64BIT();
            mul_s64(static_cast<s64>(r[rs]), static_cast<s64>(r[rt]), hi, lo);
            break;
        case 0x1D:  // DMULTU
            SG_REQUIRE_64BIT();
            mul_u64(r[rs], r[rt], hi, lo);
            break;
        case 0x1E: {  // DDIV
            SG_REQUIRE_64BIT();
            const s64 n = static_cast<s64>(r[rs]), d = static_cast<s64>(r[rt]);
            if (d == 0) {
                lo = n >= 0 ? ~u64{0} : 1;
                hi = static_cast<u64>(n);
            } else if (n == INT64_MIN && d == -1) {
                lo = static_cast<u64>(n);
                hi = 0;
            } else {
                lo = static_cast<u64>(n / d);
                hi = static_cast<u64>(n % d);
            }
            break;
        }
        case 0x1F:  // DDIVU
            SG_REQUIRE_64BIT();
            if (r[rt] == 0) {
                lo = ~u64{0};
                hi = r[rs];
            } else {
                lo = r[rs] / r[rt];
                hi = r[rs] % r[rt];
            }
            break;
        case 0x20: {  // ADD
            const s64 res = s64{static_cast<s32>(r[rs])} + s64{static_cast<s32>(r[rt])};
            if (res != static_cast<s32>(res)) {
                raise(Exc::Ov);
                return;
            }
            r[rd] = sext32(static_cast<u64>(res));
            break;
        }
        case 0x21: r[rd] = sext32(static_cast<u32>(r[rs]) + static_cast<u32>(r[rt])); break;  // ADDU
        case 0x22: {                                                                           // SUB
            const s64 res = s64{static_cast<s32>(r[rs])} - s64{static_cast<s32>(r[rt])};
            if (res != static_cast<s32>(res)) {
                raise(Exc::Ov);
                return;
            }
            r[rd] = sext32(static_cast<u64>(res));
            break;
        }
        case 0x23: r[rd] = sext32(static_cast<u32>(r[rs]) - static_cast<u32>(r[rt])); break;  // SUBU
        case 0x24: r[rd] = r[rs] & r[rt]; break;                                               // AND
        case 0x25: r[rd] = r[rs] | r[rt]; break;                                               // OR
        case 0x26: r[rd] = r[rs] ^ r[rt]; break;                                               // XOR
        case 0x27: r[rd] = ~(r[rs] | r[rt]); break;                                            // NOR
        case 0x2A: r[rd] = static_cast<s64>(r[rs]) < static_cast<s64>(r[rt]); break;           // SLT
        case 0x2B: r[rd] = r[rs] < r[rt]; break;                                               // SLTU
        case 0x2C: {                                                                           // DADD
            SG_REQUIRE_64BIT();
            const u64 res = r[rs] + r[rt];
            if (((r[rs] ^ res) & (r[rt] ^ res)) >> 63) {
                raise(Exc::Ov);
                return;
            }
            r[rd] = res;
            break;
        }
        case 0x2D:  // DADDU
            SG_REQUIRE_64BIT();
            r[rd] = r[rs] + r[rt];
            break;
        case 0x2E: {  // DSUB
            SG_REQUIRE_64BIT();
            const u64 res = r[rs] - r[rt];
            if (((r[rs] ^ r[rt]) & (r[rs] ^ res)) >> 63) {
                raise(Exc::Ov);
                return;
            }
            r[rd] = res;
            break;
        }
        case 0x2F:  // DSUBU
            SG_REQUIRE_64BIT();
            r[rd] = r[rs] - r[rt];
            break;
        case 0x30: if (static_cast<s64>(r[rs]) >= static_cast<s64>(r[rt])) raise(Exc::Tr); break;  // TGE
        case 0x31: if (r[rs] >= r[rt]) raise(Exc::Tr); break;                                      // TGEU
        case 0x32: if (static_cast<s64>(r[rs]) < static_cast<s64>(r[rt])) raise(Exc::Tr); break;   // TLT
        case 0x33: if (r[rs] < r[rt]) raise(Exc::Tr); break;                                       // TLTU
        case 0x34: if (r[rs] == r[rt]) raise(Exc::Tr); break;                                      // TEQ
        case 0x36: if (r[rs] != r[rt]) raise(Exc::Tr); break;                                      // TNE
        case 0x38:                                                                                 // DSLL
            SG_REQUIRE_64BIT();
            r[rd] = r[rt] << sa;
            break;
        case 0x3A:  // DSRL
            SG_REQUIRE_64BIT();
            r[rd] = r[rt] >> sa;
            break;
        case 0x3B:  // DSRA
            SG_REQUIRE_64BIT();
            r[rd] = static_cast<u64>(static_cast<s64>(r[rt]) >> sa);
            break;
        case 0x3C:  // DSLL32
            SG_REQUIRE_64BIT();
            r[rd] = r[rt] << (sa + 32);
            break;
        case 0x3E:  // DSRL32
            SG_REQUIRE_64BIT();
            r[rd] = r[rt] >> (sa + 32);
            break;
        case 0x3F:  // DSRA32
            SG_REQUIRE_64BIT();
            r[rd] = static_cast<u64>(static_cast<s64>(r[rt]) >> (sa + 32));
            break;
        default: raise(Exc::RI); return;
    }
}

void R4000::execute_regimm(u32 inst) {
    const unsigned rs = f_rs(inst);
    const u64 simm = f_simm(inst);
    const u64 v = gpr[rs];
    const s64 sv = static_cast<s64>(v);
    const u64 target = cur_pc_ + 4 + (simm << 2);

    switch (f_rt(inst)) {
        case 0x00: branch(sv < 0, target); break;          // BLTZ
        case 0x01: branch(sv >= 0, target); break;         // BGEZ
        case 0x02: branch_likely(sv < 0, target); break;   // BLTZL
        case 0x03: branch_likely(sv >= 0, target); break;  // BGEZL
        case 0x08: if (sv >= static_cast<s64>(simm)) raise(Exc::Tr); break;  // TGEI
        case 0x09: if (v >= simm) raise(Exc::Tr); break;                     // TGEIU
        case 0x0A: if (sv < static_cast<s64>(simm)) raise(Exc::Tr); break;   // TLTI
        case 0x0B: if (v < simm) raise(Exc::Tr); break;                      // TLTIU
        case 0x0C: if (v == simm) raise(Exc::Tr); break;                     // TEQI
        case 0x0E: if (v != simm) raise(Exc::Tr); break;                     // TNEI
        case 0x10:                                                           // BLTZAL
            gpr[31] = cur_pc_ + 8;
            branch(sv < 0, target);
            break;
        case 0x11:  // BGEZAL
            gpr[31] = cur_pc_ + 8;
            branch(sv >= 0, target);
            break;
        case 0x12:  // BLTZALL
            gpr[31] = cur_pc_ + 8;
            branch_likely(sv < 0, target);
            break;
        case 0x13:  // BGEZALL
            gpr[31] = cur_pc_ + 8;
            branch_likely(sv >= 0, target);
            break;
        default: raise(Exc::RI); return;
    }
}

// ---------------------------------------------------------------------------
// Pre-decoded handlers
// ---------------------------------------------------------------------------

// The most frequent instructions, with the same semantics as execute(); the
// rest go through execute() itself.
struct R4000::Ops {
    static u64 simm(const Op& o) { return sext16(o.inst); }
    static u64 zimm(const Op& o) { return o.inst & 0xFFFF; }
    static u64 btarget(const R4000& c, const Op& o) { return c.cur_pc_ + 4 + (simm(o) << 2); }
    static bool require64(R4000& c) {
        if (SG_LIKELY(c.ops64_enabled())) return true;
        c.raise(Exc::RI);
        return false;
    }

    static void other(R4000& c, const Op& o) { c.execute(o.inst); }

    // FPU (COP1) operations and the FPU loads and stores, straight to their
    // implementation rather than through execute() again.
    static bool cop1_usable(R4000& c) {
        c.touch();  // FPU state is not part of the idle-loop comparison
        if (c.status_ & sr::CU1) return true;
        c.raise(Exc::CpU, false, false, 1);
        return false;
    }
    template <typename F>
    static void cop1_fmt(R4000& c, const Op& o) {  // arithmetic, compares, conversions (.S, .D)
        if (cop1_usable(c)) c.fp_compute<F>(o.inst & 63, o.rt, o.rd, o.sa);
    }
    static void mfc1(R4000& c, const Op& o) {
        if (cop1_usable(c)) c.gpr[o.rt] = sext32(c.fpr_read32(o.rd));
    }
    static void mtc1(R4000& c, const Op& o) {
        if (cop1_usable(c)) c.fpr_write32(o.rd, static_cast<u32>(c.gpr[o.rt]));
    }
    template <unsigned kOpcode>
    static void fp_mem(R4000& c, const Op& o) {  // LWC1, LDC1, SWC1, SDC1
        if (!cop1_usable(c)) return;
        const u64 va = c.effective_address(c.gpr[o.rs], simm(o));
        if constexpr (kOpcode == 0x31) {
            u32 v;
            if (c.load<u32>(va, v)) c.fpr_write32(o.rt, v);
        } else if constexpr (kOpcode == 0x35) {
            u64 v;
            if (c.load<u64>(va, v)) c.fpr_write64(o.rt, v);
        } else if constexpr (kOpcode == 0x39) {
            c.store<u32>(va, c.fpr_read32(o.rt));
        } else {
            c.store<u64>(va, c.fpr_read64(o.rt));
        }
    }
    static void nop(R4000&, const Op&) {}

    // Immediate ALU
    static void addiu(R4000& c, const Op& o) {
        c.gpr[o.rt] = sext32(static_cast<u32>(c.gpr[o.rs]) + static_cast<u32>(simm(o)));
    }
    static void daddiu(R4000& c, const Op& o) {
        if (require64(c)) c.gpr[o.rt] = c.gpr[o.rs] + simm(o);
    }
    static void slti(R4000& c, const Op& o) { c.gpr[o.rt] = static_cast<s64>(c.gpr[o.rs]) < static_cast<s64>(simm(o)); }
    static void sltiu(R4000& c, const Op& o) { c.gpr[o.rt] = c.gpr[o.rs] < simm(o); }
    static void andi(R4000& c, const Op& o) { c.gpr[o.rt] = c.gpr[o.rs] & zimm(o); }
    static void ori(R4000& c, const Op& o) { c.gpr[o.rt] = c.gpr[o.rs] | zimm(o); }
    static void xori(R4000& c, const Op& o) { c.gpr[o.rt] = c.gpr[o.rs] ^ zimm(o); }
    static void lui(R4000& c, const Op& o) { c.gpr[o.rt] = sext32(zimm(o) << 16); }

    // Loads and stores
    template <typename T, bool kSigned, bool k64 = false>
    static void load(R4000& c, const Op& o) {
        if (k64 && !require64(c)) return;
        T v;
        if (!c.load<T>(c.effective_address(c.gpr[o.rs], simm(o)), v)) return;
        if constexpr (kSigned) c.gpr[o.rt] = static_cast<u64>(static_cast<s64>(static_cast<std::make_signed_t<T>>(v)));
        else c.gpr[o.rt] = v;
    }
    template <typename T, bool k64 = false>
    static void store(R4000& c, const Op& o) {
        if (k64 && !require64(c)) return;
        c.store<T>(c.effective_address(c.gpr[o.rs], simm(o)), static_cast<T>(c.gpr[o.rt]));
    }

    // Branches and jumps
    static void beq(R4000& c, const Op& o) { c.branch(c.gpr[o.rs] == c.gpr[o.rt], btarget(c, o)); }
    static void bne(R4000& c, const Op& o) { c.branch(c.gpr[o.rs] != c.gpr[o.rt], btarget(c, o)); }
    static void blez(R4000& c, const Op& o) { c.branch(static_cast<s64>(c.gpr[o.rs]) <= 0, btarget(c, o)); }
    static void bgtz(R4000& c, const Op& o) { c.branch(static_cast<s64>(c.gpr[o.rs]) > 0, btarget(c, o)); }
    static void bltz(R4000& c, const Op& o) { c.branch(static_cast<s64>(c.gpr[o.rs]) < 0, btarget(c, o)); }
    static void bgez(R4000& c, const Op& o) { c.branch(static_cast<s64>(c.gpr[o.rs]) >= 0, btarget(c, o)); }
    static void beql(R4000& c, const Op& o) { c.branch_likely(c.gpr[o.rs] == c.gpr[o.rt], btarget(c, o)); }
    static void bnel(R4000& c, const Op& o) { c.branch_likely(c.gpr[o.rs] != c.gpr[o.rt], btarget(c, o)); }
    static u64 jtarget(const R4000& c, const Op& o) {
        return ((c.cur_pc_ + 4) & ~0x0FFFFFFFull) | (u64{o.inst & 0x03FFFFFF} << 2);
    }
    static void j(R4000& c, const Op& o) {
        const u64 target = jtarget(c, o);
        c.jump(target);
        if (target <= c.cur_pc_) c.backward_branch(target);
    }
    static void jal(R4000& c, const Op& o) {
        c.gpr[31] = c.cur_pc_ + 8;
        c.jump(jtarget(c, o));
    }
    static void jr(R4000& c, const Op& o) { c.jump(c.gpr[o.rs]); }
    static void jalr(R4000& c, const Op& o) {
        const u64 target = c.gpr[o.rs];
        c.gpr[o.rd] = c.cur_pc_ + 8;
        c.jump(target);
    }

    // Register ALU
    static void sll(R4000& c, const Op& o) { c.gpr[o.rd] = sext32(static_cast<u32>(c.gpr[o.rt]) << o.sa); }
    static void srl(R4000& c, const Op& o) { c.gpr[o.rd] = sext32(static_cast<u32>(c.gpr[o.rt]) >> o.sa); }
    static void sra(R4000& c, const Op& o) {
        c.gpr[o.rd] = sext32(static_cast<u32>(static_cast<s32>(c.gpr[o.rt]) >> o.sa));
    }
    static void sllv(R4000& c, const Op& o) {
        c.gpr[o.rd] = sext32(static_cast<u32>(c.gpr[o.rt]) << (c.gpr[o.rs] & 31));
    }
    static void srlv(R4000& c, const Op& o) {
        c.gpr[o.rd] = sext32(static_cast<u32>(c.gpr[o.rt]) >> (c.gpr[o.rs] & 31));
    }
    static void srav(R4000& c, const Op& o) {
        c.gpr[o.rd] = sext32(static_cast<u32>(static_cast<s32>(c.gpr[o.rt]) >> (c.gpr[o.rs] & 31)));
    }
    static void mfhi(R4000& c, const Op& o) { c.gpr[o.rd] = c.hi; }
    static void mflo(R4000& c, const Op& o) { c.gpr[o.rd] = c.lo; }
    static void mult(R4000& c, const Op& o) {
        const s64 p = s64{static_cast<s32>(c.gpr[o.rs])} * s64{static_cast<s32>(c.gpr[o.rt])};
        c.lo = sext32(static_cast<u64>(p));
        c.hi = sext32(static_cast<u64>(p) >> 32);
    }
    static void multu(R4000& c, const Op& o) {
        const u64 p = u64{static_cast<u32>(c.gpr[o.rs])} * u64{static_cast<u32>(c.gpr[o.rt])};
        c.lo = sext32(p);
        c.hi = sext32(p >> 32);
    }
    static void addu(R4000& c, const Op& o) {
        c.gpr[o.rd] = sext32(static_cast<u32>(c.gpr[o.rs]) + static_cast<u32>(c.gpr[o.rt]));
    }
    static void subu(R4000& c, const Op& o) {
        c.gpr[o.rd] = sext32(static_cast<u32>(c.gpr[o.rs]) - static_cast<u32>(c.gpr[o.rt]));
    }
    static void and_(R4000& c, const Op& o) { c.gpr[o.rd] = c.gpr[o.rs] & c.gpr[o.rt]; }
    static void or_(R4000& c, const Op& o) { c.gpr[o.rd] = c.gpr[o.rs] | c.gpr[o.rt]; }
    static void xor_(R4000& c, const Op& o) { c.gpr[o.rd] = c.gpr[o.rs] ^ c.gpr[o.rt]; }
    static void nor(R4000& c, const Op& o) { c.gpr[o.rd] = ~(c.gpr[o.rs] | c.gpr[o.rt]); }
    static void slt(R4000& c, const Op& o) {
        c.gpr[o.rd] = static_cast<s64>(c.gpr[o.rs]) < static_cast<s64>(c.gpr[o.rt]);
    }
    static void sltu(R4000& c, const Op& o) { c.gpr[o.rd] = c.gpr[o.rs] < c.gpr[o.rt]; }
    static void daddu(R4000& c, const Op& o) {
        if (require64(c)) c.gpr[o.rd] = c.gpr[o.rs] + c.gpr[o.rt];
    }
    static void dsubu(R4000& c, const Op& o) {
        if (require64(c)) c.gpr[o.rd] = c.gpr[o.rs] - c.gpr[o.rt];
    }
    template <unsigned kExtra>
    static void dsll(R4000& c, const Op& o) {
        if (require64(c)) c.gpr[o.rd] = c.gpr[o.rt] << (o.sa + kExtra);
    }
    template <unsigned kExtra>
    static void dsrl(R4000& c, const Op& o) {
        if (require64(c)) c.gpr[o.rd] = c.gpr[o.rt] >> (o.sa + kExtra);
    }
    template <unsigned kExtra>
    static void dsra(R4000& c, const Op& o) {
        if (require64(c)) c.gpr[o.rd] = static_cast<u64>(static_cast<s64>(c.gpr[o.rt]) >> (o.sa + kExtra));
    }
};

R4000::Handler R4000::decode(u32 w, bool& plain) {
    plain = false;
    // Plain register operations; one that writes r0 does nothing at all.
    auto pure = [&](Handler fn, unsigned dest) -> Handler {
        plain = true;
        return dest ? fn : Ops::nop;
    };
    switch (w >> 26) {
        case 0x00:
            switch (w & 63) {
                case 0x00: return pure(Ops::sll, f_rd(w));
                case 0x02: return pure(Ops::srl, f_rd(w));
                case 0x03: return pure(Ops::sra, f_rd(w));
                case 0x04: return pure(Ops::sllv, f_rd(w));
                case 0x06: return pure(Ops::srlv, f_rd(w));
                case 0x07: return pure(Ops::srav, f_rd(w));
                case 0x08: return Ops::jr;
                case 0x09: return Ops::jalr;
                case 0x10: return pure(Ops::mfhi, f_rd(w));
                case 0x12: return pure(Ops::mflo, f_rd(w));
                case 0x18: return pure(Ops::mult, 1);
                case 0x19: return pure(Ops::multu, 1);
                case 0x21: return pure(Ops::addu, f_rd(w));
                case 0x23: return pure(Ops::subu, f_rd(w));
                case 0x24: return pure(Ops::and_, f_rd(w));
                case 0x25: return pure(Ops::or_, f_rd(w));
                case 0x26: return pure(Ops::xor_, f_rd(w));
                case 0x27: return pure(Ops::nor, f_rd(w));
                case 0x2A: return pure(Ops::slt, f_rd(w));
                case 0x2B: return pure(Ops::sltu, f_rd(w));
                case 0x2D: return Ops::daddu;
                case 0x2F: return Ops::dsubu;
                case 0x38: return Ops::dsll<0>;
                case 0x3A: return Ops::dsrl<0>;
                case 0x3B: return Ops::dsra<0>;
                case 0x3C: return Ops::dsll<32>;
                case 0x3E: return Ops::dsrl<32>;
                case 0x3F: return Ops::dsra<32>;
                default: return Ops::other;
            }
        case 0x01:
            switch (f_rt(w)) {
                case 0x00: return Ops::bltz;
                case 0x01: return Ops::bgez;
                default: return Ops::other;
            }
        case 0x02: return Ops::j;
        case 0x03: return Ops::jal;
        case 0x04: return Ops::beq;
        case 0x05: return Ops::bne;
        case 0x06: return Ops::blez;
        case 0x07: return Ops::bgtz;
        case 0x09: return pure(Ops::addiu, f_rt(w));
        case 0x0A: return pure(Ops::slti, f_rt(w));
        case 0x0B: return pure(Ops::sltiu, f_rt(w));
        case 0x0C: return pure(Ops::andi, f_rt(w));
        case 0x0D: return pure(Ops::ori, f_rt(w));
        case 0x0E: return pure(Ops::xori, f_rt(w));
        case 0x0F: return pure(Ops::lui, f_rt(w));
        case 0x11:  // COP1
            switch (f_rs(w)) {
                case 0x00: return Ops::mfc1;
                case 0x04: return Ops::mtc1;
                case 0x10: return Ops::cop1_fmt<float>;
                case 0x11: return Ops::cop1_fmt<double>;
                default: return Ops::other;  // branches, control registers, W/L conversions
            }
        case 0x14: return Ops::beql;
        case 0x15: return Ops::bnel;
        case 0x19: return Ops::daddiu;
        case 0x20: return Ops::load<u8, true>;
        case 0x21: return Ops::load<u16, true>;
        case 0x23: return Ops::load<u32, true>;
        case 0x24: return Ops::load<u8, false>;
        case 0x25: return Ops::load<u16, false>;
        case 0x27: return Ops::load<u32, false, true>;  // LWU
        case 0x28: return Ops::store<u8>;
        case 0x29: return Ops::store<u16>;
        case 0x2B: return Ops::store<u32>;
        case 0x31: return Ops::fp_mem<0x31>;
        case 0x35: return Ops::fp_mem<0x35>;
        case 0x39: return Ops::fp_mem<0x39>;
        case 0x3D: return Ops::fp_mem<0x3D>;
        case 0x37: return Ops::load<u64, false, true>;  // LD
        case 0x3F: return Ops::store<u64, true>;        // SD
        default: return Ops::other;
    }
}

// ---------------------------------------------------------------------------
// COP0
// ---------------------------------------------------------------------------

void R4000::execute_cop0(u32 inst) {
    if (!cop0_usable()) {
        raise(Exc::CpU, false, false, 0);
        return;
    }
    const unsigned rt = f_rt(inst), rd = f_rd(inst);
    // Status is written all the time (IRIX's spl calls raise and lower the
    // interrupt mask): only a change of mode or addressing, or an interrupt
    // that can now be taken, needs the run loop to look again.
    if ((f_rs(inst) == 0x04 || (f_rs(inst) == 0x05 && ops64_enabled())) && rd == cop0::Status) {
        const u32 old = status_;
        cp0_write(rd, f_rs(inst) == 0x04 ? sext32(gpr[rt]) : gpr[rt]);
        touch();
        if (((old ^ status_) & ~(sr::IE | sr::IM_MASK)) || interrupt_pending()) io_ = true;
        return;
    }
    // Reading a constant register (PRId, Config, Status...) is pure; Count
    // and Random change with time, and everything else alters CP0 state.
    if (f_rs(inst) > 0x01 || rd == cop0::Count || rd == cop0::Random) touch();
    if (f_rs(inst) > 0x01) io_ = true;  // interrupt enables, Compare or translation may change
    switch (f_rs(inst)) {
        case 0x00: gpr[rt] = sext32(cp0_read(rd)); break;  // MFC0
        case 0x01:                                          // DMFC0
            SG_REQUIRE_64BIT();
            gpr[rt] = cp0_read(rd);
            break;
        case 0x04: cp0_write(rd, sext32(gpr[rt])); break;  // MTC0
        case 0x05:                                          // DMTC0
            SG_REQUIRE_64BIT();
            cp0_write(rd, gpr[rt]);
            break;
        default:
            if (inst & (1u << 25)) {  // CO
                switch (inst & 63) {
                    case 0x01: tlb_read(); return;                // TLBR
                    case 0x02: tlb_write(index_ & 63); return;    // TLBWI
                    case 0x06: tlb_write(random()); return;       // TLBWR
                    case 0x08: tlb_probe(); return;               // TLBP
                    case 0x18:                                    // ERET
                        if (status_ & sr::ERL) {
                            set_pc(errorepc_);
                            status_ &= ~sr::ERL;
                        } else {
                            set_pc(epc_);
                            status_ &= ~sr::EXL;
                        }
                        update_mode();
                        llbit_ = false;
                        return;
                }
            }
            SG_LOG(Warn, "cpu", "unimplemented COP0 op %08x @ %016llx", inst,
                   static_cast<unsigned long long>(cur_pc_));
            raise(Exc::RI);
            return;
    }
}

u32 R4000::random() const {
    if (wired_ >= kTlbEntries) return kTlbEntries - 1;
    const u32 range = kTlbEntries - wired_;
    return (kTlbEntries - 1) - static_cast<u32>((cycles() - random_base_) % range);
}

u64 R4000::cp0_read(unsigned reg) const {
    switch (reg) {
        case cop0::Index: return index_;
        case cop0::Random: return random();
        case cop0::EntryLo0: return entrylo0_;
        case cop0::EntryLo1: return entrylo1_;
        case cop0::Context: return context_;
        case cop0::PageMask: return pagemask_;
        case cop0::Wired: return wired_;
        case cop0::BadVAddr: return badvaddr_;
        case cop0::Count: return count();
        case cop0::EntryHi: return entryhi_;
        case cop0::Compare: return compare_;
        case cop0::Status: return status_;
        case cop0::Cause: return cause_;
        case cop0::EPC: return epc_;
        case cop0::PRId: return prid_;
        case cop0::Config: return config_;
        case cop0::LLAddr: return lladdr_;
        case cop0::WatchLo: return watchlo_;
        case cop0::WatchHi: return watchhi_;
        case cop0::XContext: return xcontext_;
        case cop0::ECC: return ecc_;
        case cop0::CacheErr: return cacheerr_;
        case cop0::TagLo: return taglo_;
        case cop0::TagHi: return taghi_;
        case cop0::ErrorEPC: return errorepc_;
        default: return 0;
    }
}

void R4000::cp0_write(unsigned reg, u64 v) {
    const u32 v32 = static_cast<u32>(v);
    switch (reg) {
        case cop0::Index: index_ = (index_ & 0x80000000u) | (v32 & 0x3F); break;
        case cop0::EntryLo0: entrylo0_ = v & kEntryLoMask; break;
        case cop0::EntryLo1: entrylo1_ = v & kEntryLoMask; break;
        case cop0::Context: context_ = (v & ~0x7FFFFFull) | (context_ & 0x7FFFF0ull); break;
        case cop0::PageMask: pagemask_ = v32 & kPageMaskMask; break;
        case cop0::Wired:
            wired_ = v32 & 0x3F;
            random_base_ = cycles();  // Random restarts at 47
            break;
        case cop0::Count:
            count_offset_ = v32 - static_cast<u32>(cycles() >> 1);
            reschedule_timer();
            break;
        case cop0::EntryHi: {
            set_entryhi(v & kEntryHiMask);
            break;
        }
        case cop0::Compare:
            compare_ = v32;
            cause_ &= ~kCauseIP7;
            reschedule_timer();
            break;
        case cop0::Status: {
            const u32 old = status_;
            status_ = (status_ & ~kStatusWriteMask) | (v32 & kStatusWriteMask);
            if ((old ^ status_) & (sr::KX | sr::SX | sr::UX)) flush_fast_tlb();
            if ((old ^ status_) & sr::RE) SG_LOG(Warn, "cpu", "reverse-endian user mode not supported");
            update_mode();
            break;
        }
        case cop0::Cause: cause_ = (cause_ & ~kCauseWriteMask) | (v32 & kCauseWriteMask); break;
        case cop0::EPC: epc_ = v; break;
        case cop0::Config: config_ = (config_ & ~kConfigWriteMask) | (v32 & kConfigWriteMask); break;
        case cop0::LLAddr: lladdr_ = v32; break;
        case cop0::WatchLo: watchlo_ = v32; break;
        case cop0::WatchHi: watchhi_ = v32; break;
        case cop0::XContext: xcontext_ = (v & ~0x1FFFFFFFFull) | (xcontext_ & 0x1FFFFFFF0ull); break;
        case cop0::ECC: ecc_ = v32; break;
        case cop0::CacheErr: cacheerr_ = v32; break;
        case cop0::TagLo: taglo_ = v32; break;
        case cop0::TagHi: taghi_ = v32; break;
        case cop0::ErrorEPC: errorepc_ = v; break;
        default: break;  // Random, BadVAddr, PRId: read-only
    }
}

void R4000::tlb_write(unsigned index) {
    if (index >= kTlbEntries) {
        SG_LOG(Warn, "cpu", "TLB write to invalid index %u", index);
        return;
    }
    TlbEntry& e = tlb_[index];
    flush_fast_tlb_entry(e);  // the mapping being replaced
    e.mask = pagemask_;
    e.hi = entryhi_ & ~u64{pagemask_};
    e.lo[0] = entrylo0_ & ~u64{1};
    e.lo[1] = entrylo1_ & ~u64{1};
    e.global = (entrylo0_ & entrylo1_ & 1) != 0;
    flush_fast_tlb_entry(e);  // and any slot the new one could shadow
}

void R4000::tlb_read() {
    const unsigned index = index_ & 63;
    if (index >= kTlbEntries) return;
    const TlbEntry& e = tlb_[index];
    pagemask_ = static_cast<u32>(e.mask);
    set_entryhi(e.hi);
    entrylo0_ = e.lo[0] | (e.global ? 1 : 0);
    entrylo1_ = e.lo[1] | (e.global ? 1 : 0);
}

void R4000::tlb_probe() {
    const u64 asid = entryhi_ & 0xFF;
    for (unsigned i = 0; i < kTlbEntries; ++i) {
        const TlbEntry& e = tlb_[i];
        if (((entryhi_ ^ e.hi) & kVpn2Mask & ~e.mask) == 0 && (e.global || (e.hi & 0xFF) == asid)) {
            index_ = i;
            return;
        }
    }
    index_ = 0x80000000u;
}

// ---------------------------------------------------------------------------
// COP1 (FPU) - register moves and control only so far; arithmetic is M1.
// ---------------------------------------------------------------------------

u32 R4000::fpr_read32(unsigned n) const {
    if (status_ & sr::FR) return static_cast<u32>(fpr[n]);
    return static_cast<u32>(fpr[n & ~1u] >> ((n & 1) * 32));
}

void R4000::fpr_write32(unsigned n, u32 v) {
    if (status_ & sr::FR) {
        fpr[n] = (fpr[n] & 0xFFFFFFFF00000000ull) | v;
    } else {
        const unsigned shift = (n & 1) * 32;
        u64& reg = fpr[n & ~1u];
        reg = (reg & ~(0xFFFFFFFFull << shift)) | (u64{v} << shift);
    }
}

u64 R4000::fpr_read64(unsigned n) const { return (status_ & sr::FR) ? fpr[n] : fpr[n & ~1u]; }

void R4000::fpr_write64(unsigned n, u64 v) {
    if (status_ & sr::FR) fpr[n] = v;
    else fpr[n & ~1u] = v;
}

// ---------------------------------------------------------------------------
// CACHE instruction. Cache contents are not modelled (all accesses go to
// memory, so the system is always coherent); only the tag arrays exist, so
// that tag diagnostics and cache-size probing see real hardware behaviour.
// ---------------------------------------------------------------------------

namespace {
// Primary TagLo: PTagLo (paddr 35:12) in 31:8, PState in 7:6.
constexpr u32 kPStateMask = 0xC0u;
// Secondary TagLo: STagLo (paddr 35:17) in 31:13, SState 12:10, VIndex 9:7, ECC 6:0.
constexpr u32 kSStateMask = 0x1C00u;
constexpr u32 kSStateCleanExclusive = 4u << 10;
constexpr u32 kSStateDirtyExclusive = 5u << 10;

u32 primary_tag(u64 paddr, u32 state) { return static_cast<u32>(((paddr >> 12) & 0xFFFFFF) << 8) | (state << 6); }
u32 secondary_tag(u64 paddr, u32 state_bits, u64 vaddr) {
    return static_cast<u32>(((paddr >> 17) & 0x7FFFF) << 13) | state_bits | static_cast<u32>(((vaddr >> 12) & 7) << 7);
}
bool primary_hit(u32 tag, u64 paddr) { return (tag & kPStateMask) && (tag >> 8) == ((paddr >> 12) & 0xFFFFFF); }
bool secondary_hit(u32 tag, u64 paddr) { return (tag & kSStateMask) && (tag >> 13) == ((paddr >> 17) & 0x7FFFF); }
}  // namespace

void R4000::execute_cache(u32 inst) {
    touch();
    if (!cop0_usable()) {
        raise(Exc::CpU);
        return;
    }
    const unsigned op = f_rt(inst);
    const u64 vaddr = effective_address(gpr[f_rs(inst)], f_simm(inst));
    u64 paddr;
    if (!translate<true>(vaddr, Access::Load, paddr)) return;

    const unsigned cache = op & 3, operation = op >> 2;
    if (cache >= 2 && stag_.empty()) return;  // no secondary cache: undefined, ignore

    // Primary caches are virtually indexed, the secondary physically.
    const u32 iline = 16u << ((config_ >> 5) & 1), dline = 16u << ((config_ >> 4) & 1);
    const u32 icache = 1u << (12 + ((config_ >> 9) & 7)), dcache = 1u << (12 + ((config_ >> 6) & 7));
    const u32 sline = 16u << ((config_ >> 22) & 3);
    u32* tag;
    unsigned sindex = 0;
    switch (cache) {
        case 0: tag = &itag_[(vaddr & (icache - 1)) / iline]; break;
        case 1: tag = &dtag_[(vaddr & (dcache - 1)) / dline]; break;
        default:
            sindex = static_cast<unsigned>((paddr / sline) % stag_.size());
            tag = &stag_[sindex];
            break;
    }
    const bool secondary = cache >= 2;
    const bool hit = secondary ? secondary_hit(*tag, paddr) : primary_hit(*tag, paddr);
    auto set_ch = [this](bool h) { status_ = h ? (status_ | kStatusCH) : (status_ & ~kStatusCH); };

    switch (operation) {
        case 0:  // Index Invalidate / Index Writeback Invalidate
            *tag &= secondary ? ~kSStateMask : ~kPStateMask;
            break;
        case 1:  // Index Load Tag
            if (secondary && !(*tag & kSStateMask) &&
                (stag_stored_at_[sindex] == kNeverStored || cycles() - stag_stored_at_[sindex] > kScacheTagWindow)) {
                // Lines not written through Index Store Tag recently are reported
                // as holding the probed address, as if a cached access had filled
                // them. Size probes (store a zero tag at the base, then look for
                // the index that wraps back to it) depend on this.
                taglo_ = secondary_tag(paddr, kSStateCleanExclusive, vaddr);
            } else {
                taglo_ = *tag;
            }
            taghi_ = 0;
            ecc_ = 0;
            break;
        case 2:  // Index Store Tag
            if (secondary) {
                *tag = (taglo_ & 0xFFFFFC00u) | static_cast<u32>(((vaddr >> 12) & 7) << 7);
                stag_stored_at_[sindex] = cycles();
            } else {
                *tag = taglo_ & 0xFFFFFFC0u;
            }
            break;
        case 3:  // Create Dirty Exclusive (D, SD)
            if (cache == 1) *tag = primary_tag(paddr, 3);
            else if (cache == 3) *tag = secondary_tag(paddr, kSStateDirtyExclusive, vaddr);
            break;
        case 4:  // Hit Invalidate
            if (hit) *tag &= secondary ? ~kSStateMask : ~kPStateMask;
            if (secondary) set_ch(hit);
            break;
        case 5:  // Fill I / Hit Writeback Invalidate D, SD
            if (cache == 0) {
                *tag = primary_tag(paddr, 1);
            } else if (hit) {
                *tag &= secondary ? ~kSStateMask : ~kPStateMask;
            }
            if (secondary) set_ch(hit);
            break;
        case 6:  // Hit Writeback: data is never held, nothing to write
            if (secondary) set_ch(hit);
            break;
        case 7:  // Hit Set Virtual (SI, SD)
            if (secondary && hit) *tag = (*tag & ~0x380u) | static_cast<u32>(((vaddr >> 12) & 7) << 7);
            if (secondary) set_ch(hit);
            break;
    }
}

#undef SG_REQUIRE_64BIT

}  // namespace sg::mips
