// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
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

// DIV and DIVU. A zero divisor leaves an undefined result; this matches
// observed R4000 behaviour.
void div32(u64 rs, u64 rt, u64& hi, u64& lo) {
    const s32 n = static_cast<s32>(rs), d = static_cast<s32>(rt);
    if (d == 0) {
        lo = n >= 0 ? ~u64{0} : 1;
        hi = sext32(static_cast<u32>(n));
    } else if (n == INT32_MIN && d == -1) {
        lo = sext32(0x80000000u);
        hi = 0;
    } else {
        lo = sext32(static_cast<u32>(n / d));
        hi = sext32(static_cast<u32>(n % d));
    }
}

void divu32(u64 rs, u64 rt, u64& hi, u64& lo) {
    const u32 n = static_cast<u32>(rs), d = static_cast<u32>(rt);
    if (d == 0) {
        lo = ~u64{0};
        hi = sext32(n);
    } else {
        lo = sext32(n / d);
        hi = sext32(n % d);
    }
}

constexpr u64 kBevBase = 0xFFFFFFFFBFC00200ull;
constexpr u64 kNormalBase = 0xFFFFFFFF80000000ull;

constexpr u64 kEntryHiMask = 0xC00000FFFFFFE0FFull;  // R | VPN2 | ASID
constexpr u64 kVpn2Mask = 0xC00000FFFFFFE000ull;     // R | VPN2
constexpr u64 kEntryLoMask = 0x3FFFFFFFull;          // PFN | C | D | V | G
// The R10000's: 44-bit virtual addresses, 40-bit physical, and EntryLo's
// uncached attribute in bits 63:62.
constexpr u64 kEntryHiMaskR10k = 0xC0000FFFFFFFE0FFull;
constexpr u64 kVpn2MaskR10k = 0xC0000FFFFFFFE000ull;
constexpr u64 kEntryLoMaskR10k = 0xC0000003FFFFFFFFull;
constexpr u32 kConfigK0 = 0x7u;
constexpr u32 kConfigR10kSS = 7u << 16;  // secondary cache size
constexpr u32 kPageMaskMask = 0x01FFE000u;
constexpr u32 kStatusWriteMask = 0xFE77FFFFu;
constexpr u32 kCauseWriteMask = 0x00000300u;  // IP1:IP0 (software interrupts)
constexpr u32 kCauseBD = 0x80000000u;
constexpr u32 kCauseIP7 = 1u << 15;
constexpr u32 kConfigWriteMask = 0x3Fu;  // IB, DB, CU, K0
constexpr u32 kConfigWriteMaskR4600 = 0x0Fu;  // CU, K0 (the line sizes are fixed)
constexpr u32 kConfigNoScache = 1u << 17;
constexpr u32 kConfigR5000SE = 1u << 12;     // secondary cache enable (writable)
constexpr u32 kConfigR5000SS = 3u << 20;     // secondary cache size
constexpr u32 kConfigWriteMaskR5000 = kConfigR5000SE | 0x7u;
constexpr u32 kStatusCH = 1u << 18;

constexpr u64 k32BitCompatBase = 0xFFFFFFFF80000000ull;
constexpr u64 kKseg2Base = 0xFFFFFFFFC0000000ull;
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

R4000::R4000(Bus& bus, const Options& options)
    : bus_(bus), opts_(options), blocks_(1u << kBlockBits), jit_victims_(1u << kBlockBits) {
    r10k_ = opts_.core == Core::R10000;
    mips4_ = opts_.core == Core::R5000 || r10k_;
    two_way_ = opts_.core != Core::R4000;
    if (r10k_) {
        vseg_size_ = u64{1} << 44;
        pa_mask_ = 0xFFFFFFFFFFull;
        vpn2_mask_ = kVpn2MaskR10k;
        entryhi_mask_ = kEntryHiMaskR10k;
        entrylo_mask_ = kEntryLoMaskR10k;
        pfn_mask_ = 0xFFFFFFF;
        tlb_entries_ = 64;
    } else {
        vpn2_mask_ = kVpn2Mask;
        entryhi_mask_ = kEntryHiMask;
        entrylo_mask_ = kEntryLoMask;
    }
    // FCR0: the R4000/R4400 FPU is implementation 5; the R4600's, R5000's and R10000's
    // carry the CPU's implementation and revision numbers.
    fir_ = opts_.core == Core::R4000 ? 0x0500u : (opts_.prid & 0xFFFFu);
    bus_.set_code_write_handler([this](const u8* page) { invalidate_code_page(page); });
    bus_.set_remap_handler([this] { flush_fast_tlb(); });  // cached host pointers are stale
    set_jit(true);
    reset();
}

R4000::~R4000() {
    if (jit_compiles_ || code_page_writes_)
        SG_LOG(Info, "cpu", "recompiler: %llu translations, %llu reused, %llu code pages written, %llu buffer flushes",
               static_cast<unsigned long long>(jit_compiles_), static_cast<unsigned long long>(jit_reuses_),
               static_cast<unsigned long long>(code_page_writes_), static_cast<unsigned long long>(jit_buffer_flushes_));
}

void R4000::set_jit(bool enabled) {
    if (enabled && !jit_buf_) {
        constexpr std::size_t kJitBufferSize = 128u << 20;
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
        b.jit_host = nullptr;
        b.runs = 0;
    }
    if (jit_buf_) jit_buf_->clear();
    jit_ops_.clear();
    ++jit_generation_;  // kept translations are gone too
}

void R4000::keep_translation(Block& b) {
    if (!b.code || !b.jit_host) return;
    auto& v = jit_victims_[static_cast<std::size_t>(&b - blocks_.data())];
    for (unsigned i = kJitVictims - 1; i > 0; --i) v[i] = v[i - 1];
    JitVictim& k = v[0];
    k.host = b.jit_host;
    k.code = b.code;
    k.generation = jit_generation_;
    k.count = b.count;
    k.addr64 = b.jit_addr64;
    k.is_region = b.jit_region;
    k.region = b.region;
    for (u32 i = 0; i < b.count; ++i) k.words[i] = b.ops[i].inst;
}

u64 R4000::jit_region_hash(const u8* page, const JitRegion& r) {
    u64 h = 0xCBF29CE484222325ull;
    for (unsigned m = 0; m < r.n; ++m)
        for (unsigned i = 0; i < r.count[m]; ++i) {
            h ^= load_be<u32>(page + r.offset[m] + 4 * i);
            h *= 0x100000001B3ull;
        }
    return h;
}

// The code refers only to the slot, its decoded operations and host
// addresses, so it is valid again for the same instructions in the slot.
void R4000::reuse_translation(Block& b) {
    for (JitVictim& k : jit_victims_[static_cast<std::size_t>(&b - blocks_.data())]) {
        if (k.host != b.host || k.count != b.count || k.generation != jit_generation_ || !k.code) continue;
        bool same = true;
        for (u32 i = 0; i < b.count && same; ++i) same = k.words[i] == b.ops[i].inst;
        // A region also needs its other blocks unchanged.
        if (same && k.is_region) same = jit_region_hash(b.host - b.offset, k.region) == k.region.hash;
        if (!same) continue;
        b.code = k.code;
        b.jit_addr64 = k.addr64;
        b.jit_region = k.is_region;
        b.region = k.region;
        b.jit_host = b.host;
        b.runs = kJitThreshold;
        k.code = nullptr;
        ++jit_reuses_;
        return;
    }
}

void R4000::reset() {
    gpr.fill(0);
    fpr.fill(0);
    hi = lo = 0;
    fcr31 = 0;

    status_ = sr::BEV | sr::ERL;
    cause_ = 0;
    switch (opts_.core) {
        case Core::R4000:
            config_ = opts_.scache_bytes ? (opts_.config & ~kConfigNoScache) : (opts_.config | kConfigNoScache);
            break;
        case Core::R4600:  // no secondary cache interface: an SC module's cache is the module's own
            config_ = opts_.config | kConfigNoScache;
            break;
        case Core::R5000: {  // SC, and SS: 512 KiB, 1 MiB, 2 MiB or none; SE (enable) clear
            const u32 ss = opts_.scache_bytes >= (2u << 20) ? 2 : opts_.scache_bytes >= (1u << 20) ? 1 : 0;
            config_ = opts_.config & ~(kConfigNoScache | kConfigR5000SS | kConfigR5000SE);
            config_ |= opts_.scache_bytes ? (ss << 20) : (kConfigNoScache | kConfigR5000SS);
            break;
        }
        case Core::R10000: {  // SS: 512 KiB to 16 MiB (there is always a secondary cache)
            u32 ss = 0;
            while (ss < 5 && (512u << 10 << ss) < opts_.scache_bytes) ++ss;
            config_ = (opts_.config & ~kConfigR10kSS) | (ss << 16);
            break;
        }
    }
    waiting_ = false;
    prid_ = opts_.prid;
    index_ = wired_ = compare_ = pagemask_ = count_offset_ = 0;
    watchlo_ = watchhi_ = ecc_ = cacheerr_ = taglo_ = taghi_ = lladdr_ = 0;
    watch_ = 0, watch_addr_ = 0, watch_delayed_ = false;
    framemask_ = 0;
    perf_.fill(0);
    perf_active_ = false, perf_bit_ = 0, perf_mark_fp_ = 0, perf_insn_rem_ = 0;
    timer_ip7_ = false;
    entrylo0_ = entrylo1_ = context_ = xcontext_ = badvaddr_ = 0;
    entryhi_ = epc_ = errorepc_ = 0;
    asid_tag_ = 0;
    llbit_ = false;
    random_base_ = cycles();                               // Random restarts at its top
    count_offset_ = 0u - static_cast<u32>(cycles() >> 1);  // Count restarts at 0
    tlb_.fill(TlbEntry{});
    for (unsigned i = tlb_entries_; i < kTlbEntries; ++i) tlb_[i].present = false;

    if (r10k_) {
        // 32 KiB 2-way primary caches (I: 64-byte lines, D: 32-byte), the
        // secondary 2-way with 64- or 128-byte lines (Config.SB).
        auto make = [](R10kCache& c, u32 bytes, u32 line, u32 check_unit) {
            c.line = line;
            c.sets = bytes / 2 / line;
            c.tags.assign(2 * c.sets, 0);
            c.data.assign(bytes / (check_unit == 16 ? 8 : 4), 0);
            c.check.assign(bytes / check_unit, 0);
        };
        make(r10k_icache_, 32u << 10, 64, 4);
        make(r10k_dcache_, 32u << 10, 32, 4);
        make(r10k_scache_, 512u << 10 << ((config_ >> 16) & 7), (config_ & (1u << 13)) ? 128 : 64, 16);
        itag_.clear();
        dtag_.clear();
        ilru_.clear();
        dlru_.clear();
        stag_.clear();
        stag_stored_at_.clear();
    } else {
        // Tag arrays sized for the smallest line size the Config register allows.
        const u32 icache = 1u << (12 + ((config_ >> 9) & 7));
        const u32 dcache = 1u << (12 + ((config_ >> 6) & 7));
        itag_.assign(icache / 16, 0);
        dtag_.assign(dcache / 16, 0);
        ilru_.assign(two_way_ ? icache / 64 : 0, 0);
        dlru_.assign(two_way_ ? dcache / 64 : 0, 0);
        const u32 sline = opts_.core == Core::R5000 ? 32u : 16u << ((config_ >> 22) & 3);
        stag_.assign(opts_.core == Core::R4600 ? 0 : opts_.scache_bytes / sline, 0);
        stag_stored_at_.assign(stag_.size(), kNeverStored);
    }
    stag_valid_ = false;

    // The cycle counter is the machine's master clock and keeps running
    // through a reset (Count's value after reset is undefined anyway).
    loops_.fill(LoopEntry{});
    snaps_.fill(LoopSnapshot{});
    idle_ready_ = false;
    in_delay_ = false;
    update_mode();
    flush_fast_tlb();
    reschedule_timer();
    set_pc(kResetVector);
}

void R4000::soft_reset() {
    errorepc_ = next_delay_ ? pc - 4 : pc;  // in a delay slot, the branch restarts
    status_ = (status_ | sr::SR | sr::BEV | sr::ERL) & ~sr::TS;
    waiting_ = false;
    in_delay_ = next_delay_ = false;
    update_mode();
    flush_fast_tlb();
    set_pc(kResetVector);
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

SG_FORCEINLINE void R4000::step_inline() {
    time_fp_ += weight_;
    if (SG_UNLIKELY(time_fp_ >= timer_event_fp_)) timer_fire();
    if (SG_UNLIKELY(waiting_)) {  // in WAIT: time passes, nothing runs
        idle_skipped_fp_ += weight_;
        if (cause_ & status_ & sr::IM_MASK) waiting_ = false;
        return;
    }

    cur_pc_ = pc;
    in_delay_ = next_delay_;
    next_delay_ = false;

    if (SG_UNLIKELY(interrupt_pending())) {
        take_interrupt();
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
        if (SG_UNLIKELY(waiting_)) {
            wait_for_interrupt(std::min(cycle, *limit));
            continue;
        }
        if (SG_LIKELY(block_cache_)) run_block(cycle, limit);
        else step_inline();
        if (SG_UNLIKELY(idle_ready_)) {
            idle_ready_ = false;
            if (!waiting_) skip_idle(std::min(cycle, *limit));
        }
        if ((++n & 4095) == 0 && stop_.load(std::memory_order_relaxed)) break;
    }
    resolve_fp_lazy();  // FCR31 as it stands, for anyone looking from outside
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
        case 0x12: return ((w >> 21) & 0x1F) == 0x08;  // BC0, BC1, BC2 (0x13 is COP1X on the R5000)
        default: return false;
    }
}

// ERET, which goes elsewhere with no delay slot. (Other CP0 operations stay
// in the block; the run loops leave it if one changes the fetch translation.)
bool ends_block(u32 w) { return (w >> 26) == 0x10 && (w & 0x0200003Fu) == 0x02000018u; }

}  // namespace

// Decodes a straight-line run from `host` up to (and including) a control
// transfer and its delay slot, a block-ending instruction, the end of the
// 4 KiB page, or the length limit.
void R4000::build_block(Block& b, const u8* host, u32 page_offset) {
    keep_translation(b);
    b.jit_host = nullptr;
    b.jr_n = 0;
    b.hot = false;
    b.runs = 0;
    b.code = nullptr;
    b.jit_region = false;
    decode_ops(b, host, page_offset);
    if (u8* flag = bus_.code_flag(host)) *flag = 1;  // writes to this page now drop its blocks
    if (jit_buf_) reuse_translation(b);
}

void R4000::decode_ops(Block& b, const u8* host, u32 page_offset) {
    b.host = host;
    b.offset = page_offset;
    b.count = 0;
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
    b.delay = delay_loop(b.ops, b.count, page_offset);
}

R4000::DelayLoop R4000::delay_loop(const Op* ops, u32 n, u32 page_offset) {
    DelayLoop d;
    if (n < 2 || n > 4) return d;
    const u32 br = ops[n - 2].inst;
    const unsigned opc = br >> 26, rs = f_rs(br), rt = f_rt(br);
    if (static_cast<s64>(page_offset) + 4 * (n - 1) + 4 * static_cast<s16>(br & 0xFFFF) != page_offset) return d;
    // One instruction counts; any others are NOPs.
    int count = -1;
    for (u32 i = 0; i < n; ++i) {
        if (i == n - 2 || ops[i].inst == 0) continue;
        if (count >= 0) return d;
        count = static_cast<int>(i);
    }
    if (count < 0) return d;
    const u32 w = ops[count].inst;
    switch (w >> 26) {
        case 0x08: case 0x09:  // ADDI, ADDIU
            if (f_rs(w) != f_rt(w)) return d;
            d.counter = static_cast<u8>(f_rt(w));
            d.imm = static_cast<u32>(static_cast<s32>(static_cast<s16>(w & 0xFFFF)));
            d.trap = (w >> 26) == 0x08;
            if (d.imm == 0) return d;
            break;
        case 0x00:
            switch (w & 63) {
                case 0x20: case 0x21: case 0x22: case 0x23:  // ADD, ADDU, SUB, SUBU
                    if (f_rd(w) != f_rs(w) || f_rt(w) == f_rd(w) || f_rt(w) == 0 || (w >> 6 & 31)) return d;
                    d.counter = static_cast<u8>(f_rd(w));
                    d.step_reg = static_cast<u8>(f_rt(w));
                    d.sub = (w & 63) >= 0x22;
                    d.trap = !(w & 1);
                    break;
                default: return d;
            }
            break;
        default: return d;
    }
    if (d.counter == 0) return d;
    DelayLoop::Test test = DelayLoop::None;
    switch (opc) {
        case 0x04: case 0x14:  // BEQ(L), BNE(L): the counter against a register
        case 0x05: case 0x15:
            if (rs == rt || (rs != d.counter && rt != d.counter)) return d;
            d.other = static_cast<u8>(rs == d.counter ? rt : rs);
            test = (opc & 1) ? DelayLoop::Ne : DelayLoop::Eq;
            break;
        case 0x06: case 0x16: case 0x07: case 0x17:  // BLEZ(L), BGTZ(L)
            if (rt != 0 || rs != d.counter) return d;
            test = (opc & 1) ? DelayLoop::Gtz : DelayLoop::Lez;
            break;
        case 0x01:  // BLTZ(L), BGEZ(L)
            if (rs != d.counter || (rt & ~3u)) return d;
            test = (rt & 1) ? DelayLoop::Gez : DelayLoop::Ltz;
            break;
        default: return d;
    }
    d.in_slot = static_cast<u32>(count) == n - 1;
    d.test = test;
    return d;
}

u64 R4000::run_delay_loop(const Block& b, u64 t, u64 safe) {
    const DelayLoop& d = b.delay;
    const u64 w = std::max(weight_, delay_weight_);
    const u64 per = w * b.count;
    if (t + per >= safe) return t;
    const u64 max = (safe - t - 1) / per;
    u64 c = gpr[d.counter];
    const u32 step = d.step_reg ? static_cast<u32>(gpr[d.step_reg]) : d.imm;
    const u64 other = gpr[d.other];
    u64 k = 0;
    for (; k < max; ++k) {
        const u32 a = static_cast<u32>(c);
        const u32 r = d.sub ? a - step : a + step;
        if (d.trap && ((d.sub ? (a ^ step) : ~(a ^ step)) & (a ^ r) & 0x80000000u)) break;  // overflow
        const u64 next = sext32(r);
        const s64 v = static_cast<s64>(d.in_slot ? c : next);
        bool taken = false;
        switch (d.test) {
            case DelayLoop::Eq: taken = static_cast<u64>(v) == other; break;
            case DelayLoop::Ne: taken = static_cast<u64>(v) != other; break;
            case DelayLoop::Lez: taken = v <= 0; break;
            case DelayLoop::Gtz: taken = v > 0; break;
            case DelayLoop::Ltz: taken = v < 0; break;
            case DelayLoop::Gez: taken = v >= 0; break;
            case DelayLoop::None: break;
        }
        if (!taken) break;
        c = next;
    }
    gpr[d.counter] = c;
    // Time beyond the ordinary charge is not instructions (see instructions()).
    const u64 n = k * b.count;
    idle_skipped_fp_ += n * (w - weight_);
    return t + n * w;
}

// `page` is the host address of a 4 KiB page that was written.
void R4000::invalidate_code_page(const u8* page) {
    ++code_page_writes_;
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
    // Whether the page of `at` (in block b) no longer fetches from b's page
    // in the current mode: a CP0 operation changed the mode or translation.
    auto fetch_moved = [&](u64 at, const Block& b) {
        const u64 vp = at >> 12;
        const FastEntry& f = fast_[mode_][static_cast<unsigned>(Access::Fetch)][vp & (kFastEntries - 1)];
        return f.tag != (vp | asid_tag_) || f.host != b.host - b.offset;
    };
    bool check_irq = true;
    bool first = true;  // the caller has checked the run limit for the first instruction
    for (unsigned linked = 0; linked < kMaxLinkedBlocks; ++linked) {
        const u32 offset = static_cast<u32>(pc & 0xFFF);
        const u8* const host = page + offset;
        // A run that stopped inside a block carries on in it: a block made
        // from where it stopped would only duplicate the end of that one.
        u32 start = 0;
        Block* blk = nullptr;
        if (resume_host_) {
            Block& r = blocks_[block_index(resume_host_)];
            if (linked == 0 && r.host == resume_host_ && resume_i_ < r.count && r.host + 4 * resume_i_ == host)
                blk = &r, start = resume_i_;
            resume_host_ = nullptr;
        }
        if (!blk) {
            blk = &blocks_[block_index(host)];
            if (blk->host != host) build_block(*blk, host, offset);
        }
        block_invalidated_ = false;
        if (SG_UNLIKELY(blk->delay.test != DelayLoop::None) && !start && !(check_irq && interrupt_pending())) {
            check_irq = false;
            time_fp_ = run_delay_loop(*blk, time_fp_, safe);
        }
        if (jit_buf_ && !start && !blk->code && blk->delay.test == DelayLoop::None && ++blk->runs == kJitThreshold) {
            if (!jit_compile(*blk)) {
                jit_flush();  // the code buffer is full: start afresh
                ++jit_buffer_flushes_;
                jit_compile(*blk);
            }
            if (blk->code) {
                blk->jit_host = blk->host;
                ++jit_compiles_;
            }
        }

        // Plain operations only advance the time (kept in `t`), so pc, npc_
        // and the delay-slot state are brought up to date (`synced`) just
        // before anything that can observe them, and when the block is left.
        u64 ipc = pc;  // address of ops[i]
        u64 t = time_fp_;
        bool synced = true;
        u32 i = start;
        bool leave = false;
        // Translated code runs the block when no time check falls inside it
        // and no interrupt is pending; it stops early where the loop below
        // would break, or after an access that may need the checks redone,
        // and may carry on directly into other translated blocks.
        if (!start && blk->code && blk->jit_addr64 == addr64_ && t + (blk->count - 1) * weight_ < safe) {
            if (check_irq && !interrupt_pending()) check_irq = false;
            if (!check_irq) {
                jit_time0_ = t;
                jit_budget0_ = (safe - t + weight_ - 1) / weight_;  // instructions starting before `safe`
                i = blk->code(this, ipc, jit_budget0_);
                // It may have gone on into other blocks, in this page or others.
                blk = jit_block_;
                ipc = jit_entry_pc_ + 4 * u64{i};
                // A region's other blocks run from the code alone: decoded again
                // (the same, as the page is unchanged) if the slot holds another.
                if (SG_UNLIKELY(blk->host != jit_block_host_))
                    build_block(*blk, jit_block_host_, static_cast<u32>(jit_entry_pc_ & 0xFFF));
                t = time_fp_;
                first = false;
                if (SG_UNLIKELY(io_)) {
                    check_irq = true;
                    safe = safe_time();
                }
                leave = pc != ipc || block_invalidated_ || idle_ready_ || (io_ && fetch_moved(jit_entry_pc_, *blk));
            }
        }
        for (; !leave && i < blk->count; ++i, ipc += 4) {
            if (SG_UNLIKELY(t >= safe)) {
                time_fp_ = t;
                if (!first && !(cycles() < cycle && cycles() < *limit)) {
                    if (!synced) set_pc(ipc);
                    if (i > 0) resume_host_ = blk->host, resume_i_ = i;
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
                    take_interrupt();
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
            bool moved = false;
            if (SG_UNLIKELY(io_)) {  // a device or CP0 may have raised an interrupt or moved an event
                check_irq = true;
                safe = safe_time();
                moved = fetch_moved(ipc, *blk);  // the rest of the block was fetched another way
            }
            if (moved || pc != ipc + 4 || block_invalidated_ || idle_ready_) break;
        }
        if (i == blk->count) {  // ran off the end of the block
            time_fp_ = t;
            if (!synced) set_pc(ipc);
            // A jump through a register within the page, while interpreted:
            // remember where to, for the translation.
            if (!blk->code && blk->count >= 2 && (pc >> 12) == vpage) {
                const u32 w = blk->ops[blk->count - 2].inst;
                if ((w >> 26) == 0 && ((w & 63) == 0x08 || (w & 63) == 0x09)) note_jr(*blk, static_cast<u32>(pc & 0xFFF));
            }
        }

        // Link to the block at the new pc if nothing it relies on changed.
        if (idle_ready_ || next_delay_ || (pc >> 12) != vpage || (pc & 3) || mode_ != mode || fe.tag != (vpage | asid_tag_) ||
            fe.host != page)
            return;
    }
}

void R4000::set_delay_cycles_per_instruction(double cpi) {
    delay_weight_ = cpi > 0 ? static_cast<u64>(cpi * (1u << kCycleFracBits) + 0.5) : 0;
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
// A loop is looked at every kIdleSpan iterations: one visit starts watching
// it, the next records what an iteration span did, the one after compares.
// Loops that run only a few times are never looked at closely.
constexpr unsigned kIdleSpan = 5;
constexpr u64 kMaxIdleLoopInstructions = 4096 * kIdleSpan;  // in a span
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
    const LoopSnapshot& s = snaps_[e.snap];
    if (!e.have_snapshot || s.owner != &e || period != e.period_fp || nstores != s.nstores || s.regs[32] != hi ||
        s.regs[33] != lo || !std::equal(gpr.begin(), gpr.end(), s.regs.begin()))
        return false;
    constexpr u64 mask = (1u << kStoreLogBits) - 1;
    for (unsigned i = 0; i < nstores; ++i) {
        const StoreRec& now = store_log_[(store_count_ - nstores + i) & mask];
        const StoreRec& then = s.stores[i];
        if (now.host != then.host || now.size != then.size || read_host(now.host, now.size) != then.value) return false;
    }
    return true;
}

void R4000::take_snapshot(LoopEntry& e, u64 period, unsigned nstores) {
    if (!e.have_snapshot || snaps_[e.snap].owner != &e) {
        e.snap = static_cast<u8>(next_snap_);
        next_snap_ = (next_snap_ + 1) % kLoopSnapshots;
    }
    LoopSnapshot& s = snaps_[e.snap];
    s.owner = &e;
    std::copy(gpr.begin(), gpr.end(), s.regs.begin());
    s.regs[32] = hi;
    s.regs[33] = lo;
    constexpr u64 mask = (1u << kStoreLogBits) - 1;
    for (unsigned i = 0; i < nstores; ++i) {
        StoreRec r = store_log_[(store_count_ - nstores + i) & mask];
        r.value = read_host(r.host, r.size);
        s.stores[i] = r;
    }
    s.nstores = nstores;
    e.period_fp = period;
    e.have_snapshot = true;
}

// Called when a branch to an earlier address is taken; see set_idle_skip().
void R4000::loop_visit(LoopEntry& e, u64 target) {
    if (!idle_skip_) return;
    const u64 period = time_fp_ - e.time_fp;
    const u64 stores = store_count_ - e.store_pos;
    const bool logged = log_stores_ && e.log_epoch == log_epoch_;  // every store since the last visit
    const bool same = e.branch_pc == cur_pc_ && e.epoch == change_epoch_ && e.weight_epoch == weight_epoch_ &&
                      logged && period <= kMaxIdleLoopInstructions * weight_ && stores <= (1u << kStoreLogBits);
    if (!log_stores_) {
        log_stores_ = true;
        ++log_epoch_;
    }
    e.time_fp = time_fp_;
    e.epoch = change_epoch_;
    e.weight_epoch = weight_epoch_;
    e.store_pos = store_count_;
    e.log_epoch = log_epoch_;
    e.cooldown = kIdleSpan - 1;
    if (!same) {
        e.branch_pc = cur_pc_;
        e.have_snapshot = false;
        return;
    }
    const unsigned n = static_cast<unsigned>(stores);
    if (snapshot_matches(e, period, n)) {
        idle_period_fp_ = period;
        idle_ready_ = true;
        return;
    }
    if (e.have_snapshot) {  // a loop, but one that makes progress
        e.cooldown = kIdleCooldown;
        e.have_snapshot = false;
        log_stores_ = false;
        promote_loop(target);
        return;
    }
    take_snapshot(e, period, n);
}

// A loop that keeps running: its head's translation is made again, as a
// large region (see jit_region()).
void R4000::promote_loop(u64 target) {
    if (!jit_buf_) return;
    const FastEntry& fe = fast_[mode_][static_cast<unsigned>(Access::Fetch)][(target >> 12) & (kFastEntries - 1)];
    if (fe.tag != ((target >> 12) | asid_tag_) || !fe.host || (target & 3)) return;
    const u8* const host = fe.host + (target & 0xFFF);
    Block& b = blocks_[block_index(host)];
    if (b.host != host || b.hot) return;
    b.hot = true;
    if (b.code) {
        b.code = nullptr;
        b.jit_host = nullptr;
        b.runs = kJitThreshold - 1;
    }
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

// WAIT (R4600, R5000): the pipeline stops until an interrupt is requested,
// enabled or not. The pc is already past the WAIT, so an interrupt taken
// then returns to the instruction after it.
void R4000::execute_wait() {
    touch();
    if (cause_ & status_ & sr::IM_MASK) return;
    waiting_ = true;
    idle_ready_ = true;  // leave the block loops; run_until() does the waiting
}

// Time passes in WAIT up to `limit_cycle` or the Compare timer, whichever
// comes first, as if idle-skipped.
void R4000::wait_for_interrupt(u64 limit_cycle) {
    if (!(cause_ & status_ & sr::IM_MASK)) {
        const u64 target = std::min(limit_cycle << kCycleFracBits, timer_event_fp_);
        if (target > time_fp_) {
            idle_skipped_fp_ += target - time_fp_;
            time_fp_ = target;
        }
        if (time_fp_ >= timer_event_fp_) timer_fire();
    }
    if (cause_ & status_ & sr::IM_MASK) waiting_ = false;
}

void R4000::set_int_line(unsigned line, bool asserted) {
    if (line > 4) return;
    const u32 bit = 1u << (10 + line);
    cause_ = asserted ? (cause_ | bit) : (cause_ & ~bit);
}

void R4000::timer_fire() {
    if (time_fp_ >= compare_event_fp_) timer_ip7_ = true;
    if (perf_active_) perf_sync();
    update_ip7();
    reschedule_timer();
}

// IP7 is the timer's, and on the R10000 the performance counters' too.
void R4000::update_ip7() {
    if (timer_ip7_ || perf_overflow()) cause_ |= kCauseIP7;
    else cause_ &= ~kCauseIP7;
}

// Count increments every other pipeline cycle; compute the cycle at which it
// next equals Compare so the hot loop only has to do one comparison.
void R4000::reschedule_timer() {
    const u64 now_half = cycles() >> 1;
    const u32 k = compare_ - count();
    const u64 delta = k ? k : (u64{1} << 32);
    compare_event_fp_ = ((now_half + delta) << 1) << kCycleFracBits;
    timer_event_fp_ = compare_event_fp_;
    if (perf_active_) timer_event_fp_ = std::min(timer_event_fp_, perf_overflow_time());
}

// ---------------------------------------------------------------------------
// R10000 performance counters
// ---------------------------------------------------------------------------
//
// Each counter counts its event in the modes its control register names:
// U, S, K (kernel with EXL and ERL clear) or EXL; nothing while ERL is set.
// The architectural events are counted: cycles, instructions (issued and
// graduated alike: there is no out-of-order pipeline here), and TLB refill
// exceptions. Events of the pipeline and caches (misses, mispredictions,
// write-backs, interventions) are not modelled and stay at zero. Counters
// catch up from the cycle count whenever they are read or written and
// whenever the processor mode changes; an overflow (bit 31 becoming set)
// with IE raises IP7 when it happens.

namespace {
constexpr u32 kPerfIe = 1u << 4;
unsigned perf_event(u32 control) { return (control >> 5) & 0xF; }
}  // namespace

// The control-register mode bit counting applies in now.
u32 R4000::perf_mode_bit() const {
    if (status_ & sr::ERL) return 0;
    if (status_ & sr::EXL) return 1;
    switch ((status_ & sr::KSU_MASK) >> 3) {
        case 0: return 2;
        case 1: return 4;
        default: return 8;
    }
}

// Whether counter k (0 or 1) counts cycles (1), instructions (2) or neither (0).
unsigned R4000::perf_kind(unsigned k) const {
    const u32 ev = perf_event(perf_[2 * k]);
    if (ev == 0) return 1;
    if (k == 0 ? (ev == 1 || ev == 15) : ev == 1) return 2;
    return 0;
}

void R4000::perf_sync() {
    const u64 t = time_fp_;
    const u64 dc = (t >> kCycleFracBits) - (perf_mark_fp_ >> kCycleFracBits);
    const u64 di = (t - perf_mark_fp_ + perf_insn_rem_) / weight_;
    perf_insn_rem_ = (t - perf_mark_fp_ + perf_insn_rem_) % weight_;
    perf_mark_fp_ = t;
    for (unsigned k = 0; k < 2; ++k) {
        if (!(perf_[2 * k] & perf_bit_)) continue;
        const unsigned kind = perf_kind(k);
        if (kind) perf_[2 * k + 1] += static_cast<u32>(kind == 1 ? dc : di);
    }
    perf_bit_ = perf_mode_bit();
}

bool R4000::perf_overflow() const {
    for (unsigned k = 0; k < 2; ++k)
        if ((perf_[2 * k] & kPerfIe) && (perf_[2 * k + 1] & 0x80000000u)) return true;
    return false;
}

// The guest time at which a counting counter with IE next sets bit 31.
u64 R4000::perf_overflow_time() const {
    u64 when = ~u64{0};
    for (unsigned k = 0; k < 2; ++k) {
        const u32 c = perf_[2 * k], n = perf_[2 * k + 1];
        if (!(c & kPerfIe) || !(c & perf_bit_) || (n & 0x80000000u)) continue;
        const unsigned kind = perf_kind(k);
        if (!kind) continue;
        const u64 left = 0x80000000u - n;
        const u64 dt = kind == 1 ? left << kCycleFracBits : left * weight_;
        when = std::min(when, perf_mark_fp_ + dt);
    }
    return when;
}

// A counter or control register written, or the counters' activity changed.
void R4000::perf_changed() {
    perf_active_ = ((perf_[0] | perf_[2]) & 0xF) != 0;
    perf_mark_fp_ = time_fp_;
    perf_insn_rem_ = 0;
    perf_bit_ = perf_mode_bit();
    update_ip7();
    reschedule_timer();
}

// A TLB refill exception, for counter 1's event 7.
void R4000::perf_tlb_refill() {
    perf_sync();
    if ((perf_[2] & perf_bit_) && perf_event(perf_[2]) == 7) perf_[3] += 1;
}

// ---------------------------------------------------------------------------
// Exceptions
// ---------------------------------------------------------------------------

void R4000::raise(Exc code, bool tlb_refill, bool xtlb, unsigned ce) {
    touch();
    if (SG_UNLIKELY(perf_active_) && tlb_refill) perf_tlb_refill();
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
    if (r10k_)  // R in 36:35, BadVPN2 (vaddr 43:13) in 34:4
        xcontext_ = (xcontext_ & ~0x1FFFFFFFFFull) | (((vaddr >> 62) & 3) << 35) | ((vaddr >> 9) & 0x7FFFFFFF0ull);
    else
        xcontext_ = (xcontext_ & ~0x1FFFFFFFFull) | (((vaddr >> 62) & 3) << 31) | ((vaddr >> 9) & 0x7FFFFFF0ull);
    entryhi_ = (entryhi_ & 0xFF) | (vaddr & vpn2_mask_);
    raise(code, refill, xtlb);
}

// ---------------------------------------------------------------------------
// Address translation
// ---------------------------------------------------------------------------

void R4000::update_mode() {
    if (SG_UNLIKELY(perf_active_)) {  // count what went before in the old mode
        perf_sync();
        update_ip7();
        reschedule_timer();
    }
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
    fast_gen_ = (fast_gen_ + 1) % kFastGens;
    asid_tag_ = (asid_tag_ & ~(u64{0xF} << kFastGenShift)) | (fast_gen_ << kFastGenShift);
    if (fast_gen_ != 0) return;
    for (auto& mode : fast_)
        for (auto& table : mode)
            for (auto& e : table) e = FastEntry{};
    wide_cached_ = 0;
}

// Status's KX, SX or UX was cleared: 32-bit addressing for that mode. Its
// soft-TLB entries stay right, as any address 32-bit addressing reaches
// translates the same in 64-bit addressing, except those of 64-bit addresses
// beyond it. Setting a bit needs nothing at all, which matters: IRIX runs
// its kernel with UX clear and sets it on every return to a user process.
void R4000::narrow_addressing(u32 cleared) {
    auto drop = [&](unsigned mode) {
        if (!(wide_cached_ & (1u << mode))) return;
        wide_cached_ &= static_cast<u8>(~(1u << mode));
        for (auto& table : fast_[mode])
            for (auto& e : table) e = FastEntry{};
    };
    if (cleared & sr::UX) drop(kUser);
    if (cleared & sr::SX) drop(kSupervisor);
    if (cleared & sr::KX) {
        drop(kKernel);
        drop(kKernelErl);
    }
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

// The R10000 differs from the R4400 in two ways here (R10000 manual, 16.3):
// the 64-bit user space is reachable from 64-bit Supervisor and Kernel
// modes only with UX set, the 64-bit supervisor space from Kernel mode only
// with SX set; and a TLB refill takes the 32- or 64-bit vector by the
// extended-addressing bit of the space the address lies in (UX, SX or KX),
// not of the current mode.
template <bool kRaise>
bool R4000::translate(u64 vaddr, Access acc, u64& paddr) {
    const bool ux = (status_ & sr::UX) != 0, sx = (status_ & sr::SX) != 0;
    switch (mode_) {
        case kUser:
            if (status_ & sr::UX) {
                if (vaddr < vseg_size_) return tlb_lookup<kRaise>(vaddr, acc, true, paddr);
            } else if (vaddr < 0x80000000ull) {
                return tlb_lookup<kRaise>(vaddr, acc, false, paddr);
            }
            break;

        case kSupervisor: {
            const bool csseg = vaddr >= kKseg2Base && vaddr < 0xFFFFFFFFE0000000ull;
            if (status_ & sr::SX) {
                const bool xsseg = (vaddr >> 62) == 1 && (vaddr & 0x3FFFFFFFFFFFFFFFull) < vseg_size_;
                if (r10k_ && vaddr < 0x80000000ull) return tlb_lookup<kRaise>(vaddr, acc, ux, paddr);
                if (r10k_ && vaddr < vseg_size_ && !ux) break;  // xsuseg needs UX
                if (vaddr < vseg_size_ || xsseg || csseg) return tlb_lookup<kRaise>(vaddr, acc, true, paddr);
            } else if (vaddr < 0x80000000ull || csseg) {
                return tlb_lookup<kRaise>(vaddr, acc, r10k_ && vaddr < 0x80000000ull && ux, paddr);
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
                // ksseg (supervisor space) / kseg3
                if (r10k_ && vaddr < 0xFFFFFFFFE0000000ull) return tlb_lookup<kRaise>(vaddr, acc, sx, paddr);
                return tlb_lookup<kRaise>(vaddr, acc, kx, paddr);
            }
            if (vaddr < 0x80000000ull) {
                if (mode_ == kKernelErl) {  // ERL: kuseg is unmapped and uncached
                    paddr = vaddr;
                    return true;
                }
                return tlb_lookup<kRaise>(vaddr, acc, r10k_ ? ux : kx, paddr);
            }
            if (!kx) break;
            if (vaddr < vseg_size_) {  // xkuseg
                if (r10k_ && !ux) break;
                return tlb_lookup<kRaise>(vaddr, acc, true, paddr);
            }
            switch (vaddr >> 62) {
                case 1:  // xksseg
                    if (r10k_ && !sx) break;
                    if ((vaddr & 0x3FFFFFFFFFFFFFFFull) < vseg_size_) return tlb_lookup<kRaise>(vaddr, acc, true, paddr);
                    break;
                case 2: {  // xkphys: bits 61:59 select cache algorithm, those above the physical address must be zero
                    // (on the R10000, uncached algorithms 2 and 7 carry the uncached attribute in 58:57)
                    const unsigned cca = static_cast<unsigned>(vaddr >> 59) & 7;
                    const u64 attr = r10k_ && (cca == 2 || cca == 7) ? 0x0600000000000000ull : 0;
                    if ((vaddr & 0x07FFFFFFFFFFFFFFull & ~pa_mask_ & ~attr) == 0) {
                        paddr = vaddr & pa_mask_;
                        return true;
                    }
                    break;
                }
                case 3:  // xkseg, up to the 32-bit compatibility space's size below its end
                    if ((vaddr & 0x3FFFFFFFFFFFFFFFull) < vseg_size_ - 0x80000000ull)
                        return tlb_lookup<kRaise>(vaddr, acc, true, paddr);
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
        if (((vaddr ^ e.hi) & vpn2_mask_ & ~e.mask) != 0 || !e.present) continue;
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
        paddr = ((((entry_lo >> 6) & pfn_mask_) << 12) & ~offset_mask) | (vaddr & offset_mask);
        return true;
    }
    if constexpr (kRaise) tlb_exception(acc == Access::Store ? Exc::TLBS : Exc::TLBL, vaddr, true, xtlb);
    return false;
}

unsigned R4000::cache_algorithm(u64 vaddr) const {
    if (vaddr >= k32BitCompatBase && vaddr < kKseg2Base)
        return vaddr < 0xFFFFFFFFA0000000ull ? config_ & kConfigK0 : 2;  // kseg0, kseg1
    if ((vaddr >> 62) == 2) return static_cast<unsigned>(vaddr >> 59) & 7;  // xkphys
    if (mode_ == kKernelErl && vaddr < 0x80000000ull) return 2;              // ERL: kuseg unmapped
    const u64 asid = entryhi_ & 0xFF;
    for (const TlbEntry& e : tlb_) {
        if (((vaddr ^ e.hi) & vpn2_mask_ & ~e.mask) != 0 || !e.present) continue;
        if (!e.global && (e.hi & 0xFF) != asid) continue;
        const u64 entry_lo = e.lo[(vaddr & (((e.mask >> 1) | 0xFFF) + 1)) ? 1 : 0];
        return static_cast<unsigned>(entry_lo >> 3) & 7;
    }
    return 2;
}

void R4000::set_uncached_store_check(bool on) {
    if (on == uncached_store_check_) return;
    uncached_store_check_ = on;
    flush_fast_tlb();  // uncached stores made while it was off may have entries
}

// The watch registers changed: whether loads or stores are watched, and
// where. The watched page gets no soft-TLB entries, so every access to it
// comes through lookup_slow to be compared.
void R4000::watch_update() {
    watch_addr_ = (u64{watchhi_} << 32) | (watchlo_ & ~7u);
    watch_ = watchlo_ & 3;
    flush_fast_tlb();
}

bool R4000::lookup_slow(u64 vaddr, Access acc, u8*& host, u64& paddr) {
    if (!translate<true>(vaddr, acc, paddr)) return false;
    bool watched_page = false;
    if (SG_UNLIKELY(watch_) && acc != Access::Fetch && (paddr & ~u64{0xFFF}) == (watch_addr_ & ~u64{0xFFF})) {
        watched_page = true;
        if ((paddr & ~u64{7}) == watch_addr_ && (watch_ & (acc == Access::Store ? 1u : 2u))) {
            // Postponed while EXL or ERL is set: the access completes and the
            // exception waits for both to clear.
            if (status_ & (sr::EXL | sr::ERL)) {
                watch_delayed_ = true;
            } else {
                touch();
                raise(Exc::WATCH);
                return false;
            }
        }
    }
    if (SG_UNLIKELY(paddr >= opts_.uncached_only_base && paddr < opts_.uncached_only_end)) {
        const unsigned alg = cache_algorithm(vaddr);
        if (alg != 2 && alg != 7) {  // cached: refused by the system (no soft-TLB entry, so every access asks)
            touch();
            raise(acc == Access::Fetch ? Exc::IBE : Exc::DBE);
            return false;
        }
    }
    const u64 ppage = paddr & ~u64{0xFFF};
    const bool is_store = acc == Access::Store;
    host = is_store ? bus_.host_write_ptr(ppage) : bus_.host_read_ptr(ppage);
    if (SG_UNLIKELY(uncached_store_check_) && is_store && host) {
        const unsigned alg = cache_algorithm(vaddr);
        if (alg == 2 || alg == 7) {  // uncached: reported each time, so no soft-TLB entry
            if (uncached_store_hook) uncached_store_hook(paddr);
            watched_page = true;
        }
    }
    if (SG_UNLIKELY(watched_page)) {  // no soft-TLB entry: every access comes back here
        if (is_store) store_code_ = host ? bus_.code_flag(host) : nullptr;
        return true;
    }
    FastEntry& e = fast_[mode_][static_cast<unsigned>(acc)][(vaddr >> 12) & (kFastEntries - 1)];
    if (vaddr >= 0x80000000ull && vaddr < k32BitCompatBase) wide_cached_ |= 1u << mode_;
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
    if (SG_LIKELY(bus_.read<T>(paddr, out))) {
        if (bus_.read_stall) {  // an uncached read across a slower bus: the pipeline waits
            time_fp_ += u64{bus_.read_stall} << kCycleFracBits;
            bus_.read_stall = 0;
        }
        return true;
    }
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
        if (log_stores_) log_store(p, sizeof(T));
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
        case 0x13:  // COP1X on the R5000, COP3 otherwise
            if (mips4_) {
                if (mips4_enabled()) execute_cop1x(inst);
                else raise(Exc::RI);
                return;
            }
            [[fallthrough]];
        case 0x12:  // COP2
        {
            const unsigned z = (inst >> 26) & 3;
            if (!(status_ & (sr::CU0 << z))) raise(Exc::CpU, false, false, z);
            else if (z == 2 && opts_.core == Core::R4000 && ((inst >> 21) & 0x1F) != 0x08) return;  // no COP2: its operations do nothing
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
            llvaddr_ = va;
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
            llvaddr_ = va;
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

        case 0x33:  // PREF (MIPS IV): a hint, no exceptions
            if (!mips4_enabled()) raise(Exc::RI);
            return;

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
        case 0x01:  // MOVF, MOVT (MIPS IV): on an FPU condition code
            if (!mips4_enabled()) {
                raise(Exc::RI);
                return;
            }
            touch();
            if (!(status_ & sr::CU1)) {
                raise(Exc::CpU, false, false, 1);
                return;
            }
            if (((fcr31 & fcc_bit((inst >> 18) & 7)) != 0) == ((inst >> 16) & 1)) r[rd] = r[rs];
            break;
        case 0x0A:  // MOVZ (MIPS IV)
        case 0x0B:  // MOVN
            if (!mips4_enabled()) {
                raise(Exc::RI);
                return;
            }
            if ((r[rt] == 0) == ((inst & 1) == 0)) r[rd] = r[rs];
            break;
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
        case 0x1A: div32(r[rs], r[rt], hi, lo); break;   // DIV
        case 0x1B: divu32(r[rs], r[rt], hi, lo); break;  // DIVU
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

// COP1X (MIPS IV): the indexed FPU loads and stores here; multiply-add in fpu.cpp.
void R4000::execute_cop1x(u32 inst) {
    touch();  // FPU state is not part of the idle-loop comparison
    if (!(status_ & sr::CU1)) {
        raise(Exc::CpU, false, false, 1);
        return;
    }
    const unsigned fr = (inst >> 21) & 31, ft = (inst >> 16) & 31, fs = (inst >> 11) & 31;
    const unsigned fd = (inst >> 6) & 31, func = inst & 63;
    const u64 va = effective_address(gpr[fr], gpr[ft]);  // base + index
    switch (func) {
        case 0x00: {  // LWXC1
            u32 v;
            if (load<u32>(va, v)) fpr_write32(fd, v);
            return;
        }
        case 0x01: {  // LDXC1
            u64 v;
            if (load<u64>(va, v)) fpr_write64(fd, v);
            return;
        }
        case 0x08: store<u32>(va, fpr_read32(fs)); return;  // SWXC1
        case 0x09: store<u64>(va, fpr_read64(fs)); return;  // SDXC1
        case 0x0F: return;                                  // PREFX: a hint
        case 0x20: case 0x28: case 0x30: case 0x38:          // MADD, MSUB, NMADD, NMSUB .S
            fp_multiply_add<float>(func, fr, ft, fs, fd);
            return;
        case 0x21: case 0x29: case 0x31: case 0x39:          // .D
            fp_multiply_add<double>(func, fr, ft, fs, fd);
            return;
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
    static void mthi(R4000& c, const Op& o) { c.hi = c.gpr[o.rs]; }
    static void mtlo(R4000& c, const Op& o) { c.lo = c.gpr[o.rs]; }
    static void div(R4000& c, const Op& o) { div32(c.gpr[o.rs], c.gpr[o.rt], c.hi, c.lo); }
    static void divu(R4000& c, const Op& o) { divu32(c.gpr[o.rs], c.gpr[o.rt], c.hi, c.lo); }
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
                case 0x11: return pure(Ops::mthi, 1);
                case 0x12: return pure(Ops::mflo, f_rd(w));
                case 0x13: return pure(Ops::mtlo, 1);
                case 0x18: return pure(Ops::mult, 1);
                case 0x19: return pure(Ops::multu, 1);
                case 0x1A: return pure(Ops::div, 1);
                case 0x1B: return pure(Ops::divu, 1);
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
    // Interrupts, the timer or the fetch translation may change: the run
    // loops look again. (The other registers stage a TLB write or are read
    // back; TLBP only reads the TLB.)
    switch (f_rs(inst)) {
        case 0x00: case 0x01: break;
        case 0x04: case 0x05:
            if (rd == cop0::EntryHi || rd == cop0::Count || rd == cop0::Compare || rd == cop0::Cause ||
                rd == cop0::Config || rd == cop0::WatchLo || rd == cop0::WatchHi)
                io_ = true;
            break;
        default:
            if ((inst & 0x0200003Fu) != 0x02000008u) io_ = true;
            break;
    }
    if (r10k_ && rd == cop0::PerfCounter && f_rs(inst) <= 0x05 && (f_rs(inst) & 3) <= 1) {
        // The R10000's performance counters: the instruction's low bits pick
        // control 0, counter 0, control 1 or counter 1.
        if (perf_active_) perf_sync();
        u32& r = perf_[inst & 3];
        if (f_rs(inst) <= 0x01) {
            gpr[rt] = sext32(r);
        } else {
            r = static_cast<u32>(gpr[rt]) & ((inst & 1) ? 0xFFFFFFFFu : 0x1FFu);
            perf_changed();
        }
        return;
    }
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
                    case 0x20:                                    // WAIT (R4600, R5000)
                        if (opts_.core == Core::R4000 || r10k_) break;
                        execute_wait();
                        return;
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
    if (wired_ >= tlb_entries_) return tlb_entries_ - 1;
    const u32 range = tlb_entries_ - wired_;
    return (tlb_entries_ - 1) - static_cast<u32>((cycles() - random_base_) % range);
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
        case cop0::FrameMask: return r10k_ ? framemask_ : 0;
        default: return 0;  // the R10000's Diagnostic register reads as zero (no ITLB or branch state)
    }
}

void R4000::cp0_write(unsigned reg, u64 v) {
    const u32 v32 = static_cast<u32>(v);
    switch (reg) {
        case cop0::Index: index_ = (index_ & 0x80000000u) | (v32 & 0x3F); break;
        case cop0::EntryLo0: entrylo0_ = v & entrylo_mask_; break;
        case cop0::EntryLo1: entrylo1_ = v & entrylo_mask_; break;
        case cop0::Context: context_ = (v & ~0x7FFFFFull) | (context_ & 0x7FFFF0ull); break;
        case cop0::PageMask: pagemask_ = v32 & kPageMaskMask; break;
        case cop0::Wired:
            wired_ = v32 & 0x3F;
            random_base_ = cycles();  // Random restarts at its top
            break;
        case cop0::Count:
            count_offset_ = v32 - static_cast<u32>(cycles() >> 1);
            reschedule_timer();
            break;
        case cop0::EntryHi: {
            set_entryhi(v & entryhi_mask_);
            break;
        }
        case cop0::Compare:
            compare_ = v32;
            timer_ip7_ = false;
            update_ip7();
            reschedule_timer();
            break;
        case cop0::Status: {
            const u32 old = status_;
            status_ = (status_ & ~kStatusWriteMask) | (v32 & kStatusWriteMask);
            if (const u32 cleared = old & ~status_ & (sr::KX | sr::SX | sr::UX)) narrow_addressing(cleared);
            if ((old ^ status_) & sr::RE) SG_LOG(Warn, "cpu", "reverse-endian user mode not supported");
            update_mode();
            break;
        }
        case cop0::Cause: cause_ = (cause_ & ~kCauseWriteMask) | (v32 & kCauseWriteMask); break;
        case cop0::EPC: epc_ = v; break;
        case cop0::Config: {
            const u32 mask = opts_.core == Core::R4000    ? kConfigWriteMask
                             : opts_.core == Core::R4600  ? kConfigWriteMaskR4600
                             : opts_.core == Core::R10000 ? kConfigK0  // the rest is the boot mode bits
                                                          : kConfigWriteMaskR5000;
            config_ = (config_ & ~mask) | (v32 & mask);
            break;
        }
        case cop0::LLAddr: lladdr_ = v32; break;  // (the R10000's is a scratch register)
        case cop0::WatchLo:  // the R4600 and R5000 have no watch registers
            if (opts_.core != Core::R4000 && !r10k_) break;
            watchlo_ = v32 & ~4u;
            watch_delayed_ = false;  // a write clears a delayed watch
            watch_update();
            break;
        case cop0::WatchHi:  // physical address bits 35:32 (R4000), 39:32 (R10000)
            if (opts_.core != Core::R4000 && !r10k_) break;
            watchhi_ = v32 & (r10k_ ? 0xFFu : 0xFu);
            watch_update();
            break;
        case cop0::XContext:
            if (r10k_) xcontext_ = (v & ~0x1FFFFFFFFFull) | (xcontext_ & 0x1FFFFFFFF0ull);
            else xcontext_ = (v & ~0x1FFFFFFFFull) | (xcontext_ & 0x1FFFFFFF0ull);
            break;
        case cop0::ECC: ecc_ = r10k_ ? v32 & 0x3FF : v32; break;
        case cop0::CacheErr:  // read-only on the R10000
            if (!r10k_) cacheerr_ = v32;
            break;
        case cop0::FrameMask: framemask_ = v32 & 0xFFFF; break;
        case cop0::TagLo: taglo_ = v32; break;
        case cop0::TagHi: taghi_ = v32; break;
        case cop0::ErrorEPC: errorepc_ = v; break;
        default: break;  // Random, BadVAddr, PRId: read-only
    }
}

void R4000::tlb_write(unsigned index) {
    if (index >= tlb_entries_) {
        SG_LOG(Warn, "cpu", "TLB write to invalid index %u", index);
        return;
    }
    TlbEntry& e = tlb_[index];
    flush_fast_tlb_entry(e);  // the mapping being replaced
    // The R10000 passes EntryLo through FrameMask (bits 15:0 clear PFN bits 33:18).
    const u64 frame = r10k_ ? ~(u64{framemask_} << 18) : ~u64{0};
    e.mask = pagemask_;
    e.hi = entryhi_ & ~u64{pagemask_};
    e.lo[0] = entrylo0_ & ~u64{1} & frame;
    e.lo[1] = entrylo1_ & ~u64{1} & frame;
    e.global = (entrylo0_ & entrylo1_ & 1) != 0;
    e.present = true;
    if (r10k_) {
        // An entry that would match the same addresses as others: those are
        // invalidated (even wired ones) and Status.TS is set; TS is cleared
        // by a write that finds none.
        bool shadowed = false;
        for (unsigned i = 0; i < tlb_entries_; ++i) {
            TlbEntry& o = tlb_[i];
            if (i == index || !o.present) continue;
            if (((o.hi ^ e.hi) & vpn2_mask_ & ~(o.mask | e.mask)) != 0) continue;
            if (!o.global && !e.global && ((o.hi ^ e.hi) & 0xFF) != 0) continue;
            flush_fast_tlb_entry(o);
            o.present = false;
            shadowed = true;
        }
        status_ = shadowed ? (status_ | sr::TS) : (status_ & ~sr::TS);
    }
    flush_fast_tlb_entry(e);  // and any slot the new one could shadow
}

void R4000::tlb_read() {
    const unsigned index = index_ & 63;
    if (index >= tlb_entries_) return;
    const TlbEntry& e = tlb_[index];
    pagemask_ = static_cast<u32>(e.mask);
    set_entryhi(e.hi);
    entrylo0_ = e.lo[0] | (e.global ? 1 : 0);
    entrylo1_ = e.lo[1] | (e.global ? 1 : 0);
}

void R4000::tlb_probe() {
    const u64 asid = entryhi_ & 0xFF;
    for (unsigned i = 0; i < tlb_entries_; ++i) {
        const TlbEntry& e = tlb_[i];
        if (e.present && ((entryhi_ ^ e.hi) & vpn2_mask_ & ~e.mask) == 0 && (e.global || (e.hi & 0xFF) == asid)) {
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
        // The R10000 clears the upper half (MTC1, LWC1, single and word
        // results); the others keep it.
        fpr[n] = r10k_ ? v : (fpr[n] & 0xFFFFFFFF00000000ull) | v;
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
    const unsigned op = f_rt(inst);
    // The R10000's Cache Barrier changes nothing (it only orders accesses),
    // so a loop holding one can still be an idle spin (IP28 code puts them
    // ahead of stores and at branch targets, poll loops included).
    if (!(r10k_ && op == 0x14)) touch();
    if (!cop0_usable()) {
        raise(Exc::CpU);
        return;
    }
    const u64 vaddr = effective_address(gpr[f_rs(inst)], f_simm(inst));
    u64 paddr;
    if (!translate<true>(vaddr, Access::Load, paddr)) return;

    if (r10k_) return cache_op_r10k(op, vaddr, paddr);
    const unsigned cache = op & 3, operation = op >> 2;
    // The LLbit is cleared by a change of state of the cache line holding the
    // link address (R4000 manual, Load Linked Store Conditional Operation).
    if (llbit_ && cache_op_changes_link(cache, operation, vaddr, paddr)) llbit_ = false;
    if (two_way_) return cache_op_2way(cache, operation, vaddr, paddr);
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

// Whether a CACHE operation invalidates, retags or creates the data (or
// secondary) cache line that holds the link address: Hit operations on that
// line, Index operations on its index (in either way of a 2-way cache), and
// the R5000's secondary cache flash and page invalidates.
bool R4000::cache_op_changes_link(unsigned cache, unsigned operation, u64 vaddr, u64 paddr) const {
    if (cache == 0) return false;  // the instruction cache holds no data
    const u64 link = u64{lladdr_} << 4;
    if (opts_.core == Core::R5000 && cache == 3) {
        if (stag_.empty()) return false;
        if (operation == 0) return true;                                              // Flash Invalidate
        if (operation == 5) return (paddr >> 12) == (link >> 12);                     // Page Invalidate
        if (operation == 2) return (paddr / 32) % stag_.size() == (link / 32) % stag_.size();  // Index Store Tag
        return false;
    }
    if (operation == 1 || operation >= 6) return false;  // Load Tag, Hit Writeback, Hit Set Virtual: no change
    const bool index_op = operation == 0 || operation == 2;
    if (cache == 1) {
        const u32 line = 16u << ((config_ >> 4) & 1);
        u32 size = 1u << (12 + ((config_ >> 6) & 7));
        if (two_way_) size /= 2;
        if (index_op) return ((vaddr & (size - 1)) / line) == ((llvaddr_ & (size - 1)) / line);
        return paddr / line == link / line;
    }
    if (stag_.empty()) return false;
    const u32 sline = 16u << ((config_ >> 22) & 3);
    if (index_op) return (paddr / sline) % stag_.size() == (link / sline) % stag_.size();
    return paddr / sline == link / sline;
}

// The R4600 and R5000: 2-way primary caches (the way an Index operation
// works on is the address bit above a way's size, so the tag array is laid
// out as way 0 then way 1), and on the R5000 a direct-mapped secondary cache
// of 32-byte lines whose Index Store Tag takes the tag from the address.
void R4000::cache_op_2way(unsigned cache, unsigned operation, u64 vaddr, u64 paddr) {
    if (cache == 3) {
        if (opts_.core != Core::R5000 || stag_.empty()) return;  // none: undefined, ignore
        constexpr u32 kLine = 32, kValid = 1u << 12;              // SState 4
        auto stag_of = [](u64 pa) { return static_cast<u32>(((pa >> 19) & 0x1FFFF) << 15); };
        const std::size_t lines = stag_.size();
        const std::size_t index = (paddr / kLine) % lines;
        switch (operation) {
            case 0:  // Flash Invalidate
                if (stag_valid_) std::fill(stag_.begin(), stag_.end(), 0);
                stag_valid_ = false;
                break;
            case 1:  // Index Load Tag
                // As on the R4000: a line not written through Index Store Tag
                // recently is reported as holding the probed address, as a
                // cached access would have left it (size probes rely on it).
                if (!stag_[index] && (stag_stored_at_[index] == kNeverStored ||
                                      cycles() - stag_stored_at_[index] > kScacheTagWindow))
                    taglo_ = stag_of(paddr) | kValid;
                else
                    taglo_ = stag_[index];
                taghi_ = 0;
                ecc_ = 0;
                break;
            case 2:  // Index Store Tag: the tag from the address, the state from TagLo (invalid reads as zero)
                stag_[index] = (taglo_ & kValid) ? stag_of(paddr) | kValid : 0;
                stag_valid_ |= stag_[index] != 0;
                stag_stored_at_[index] = cycles();
                break;
            case 5: {  // Page Invalidate: the page's lines, one run of tags (lines is a multiple of a page's)
                if (!stag_valid_) break;
                const std::size_t first = ((paddr & ~u64{0xFFF}) / kLine) % lines;
                std::fill_n(stag_.begin() + static_cast<std::ptrdiff_t>(first), 0x1000 / kLine, 0u);
                break;
            }
            default: break;
        }
        return;
    }
    if (cache == 2) return;  // reserved

    const bool icache = cache == 0;
    const u32 line = icache ? 16u << ((config_ >> 5) & 1) : 16u << ((config_ >> 4) & 1);
    const u32 size = 1u << (12 + ((config_ >> (icache ? 9 : 6)) & 7));
    std::vector<u32>& tags = icache ? itag_ : dtag_;
    std::vector<u8>& lru = icache ? ilru_ : dlru_;
    const u32 sets = size / 2 / line;
    const u32 set = static_cast<u32>((vaddr & (size / 2 - 1)) / line);
    u32* const way[2] = {&tags[set], &tags[sets + set]};
    u32* const indexed = &tags[(vaddr & (size - 1)) / line];
    int hit_way = -1;
    for (int w = 0; w < 2; ++w)
        if (primary_hit(*way[w], paddr)) hit_way = w;
    auto allocate = [&](u32 state) {  // the way holding the line, else the one to replace
        const int w = hit_way >= 0 ? hit_way : lru[set];
        *way[w] = primary_tag(paddr, state);
        lru[set] = static_cast<u8>(w ^ 1);
    };
    switch (operation) {
        case 0: *indexed &= ~kPStateMask; break;  // Index Invalidate / Index Writeback Invalidate
        case 1:                                   // Index Load Tag
            taglo_ = *indexed;
            taghi_ = 0;
            ecc_ = 0;
            break;
        case 2: *indexed = taglo_ & 0xFFFFFFC0u; break;  // Index Store Tag
        case 3:                                          // Create Dirty Exclusive (D)
            if (!icache) allocate(3);
            break;
        case 4:  // Hit Invalidate
            if (hit_way >= 0) *way[hit_way] &= ~kPStateMask;
            break;
        case 5:  // Fill (I), Hit Writeback Invalidate (D)
            if (icache) allocate(2);
            else if (hit_way >= 0) *way[hit_way] &= ~kPStateMask;
            break;
        case 6: break;  // Hit Writeback: data is never held, nothing to write
        default: break;
    }
}

// The R10000's CACHE operations (R10000 manual, chapter 10). The primary
// caches are indexed by the virtual address, the secondary by the physical;
// an Index operation's way is address bit 0. Tags read back as Index Store
// Tag wrote them (the fields the processor keeps) and Hit operations compare
// them; the data arrays hold only what Index Store Data put there, as
// cached accesses go to memory, which is always current.
void R4000::cache_op_r10k(unsigned op, u64 vaddr, u64 paddr) {
    const unsigned cache = op & 3, operation = op >> 2;
    if (cache == 2 || op == 0x14) return;  // reserved; Cache Barrier (only orders accesses)
    const bool icache = cache == 0, secondary = cache == 3;
    R10kCache& c = icache ? r10k_icache_ : cache == 1 ? r10k_dcache_ : r10k_scache_;
    const u64 index_addr = secondary ? paddr : vaddr;
    const unsigned way = static_cast<unsigned>(index_addr & 1);
    const u32 set = static_cast<u32>((index_addr / c.line) % c.sets);
    // The fields each array keeps, as TagHi << 32 | TagLo.
    constexpr u64 kIFields = 0x0000000FFFFFFF4Dull;   // Tag[39:12], state, LRU, parities
    constexpr u64 kDFields = 0xE000000FFFFFFFCFull;   // ... state 7:6, SCWay, StateMod 63:61
    constexpr u64 kSFields = 0x8000000FFFFFCDFFull;   // MRU, Tag[39:18], state 11:10, PIdx 8:7, ECC 6:0
    const u64 fields = icache ? kIFields : secondary ? kSFields : kDFields;
    const u64 tag_mask = secondary ? 0xFFFFFC0000ull : 0xFFFFFFF000ull;  // the address bits a tag holds
    auto tag_addr = [&](u64 t) {
        return ((t >> 32 & 0xF) << 36) | (secondary ? (t >> 14 & 0x3FFFF) << 18 : (t >> 8 & 0xFFFFFF) << 12);
    };
    auto valid = [&](u64 t) { return (t & (secondary ? 0xC00u : icache ? 0x40u : 0xC0u)) != 0; };
    auto hit = [&](unsigned w) {
        const u64 t = c.tags[w * c.sets + set];
        return valid(t) && tag_addr(t) == (paddr & tag_mask);
    };
    // Invalid, with the primary data cache's SCWay clear and StateMod Normal.
    auto invalidate = [&](u64& t) { t = icache ? t & ~0x40ull : secondary ? t & ~0xC00ull : (t & ~0xE0000000000000C2ull) | (1ull << 61); };
    // A secondary line's primary subset blocks, found from the address and
    // the virtual index the secondary tag records.
    auto invalidate_subsets = [&](u64 stag) {
        const u64 pa = tag_addr(stag) | (paddr & ~tag_mask & 0xFFFFFFFFFFull & ~u64{c.line - 1});
        const u64 va = ((stag >> 7 & 3) << 12) | (pa & 0xFFF);
        for (R10kCache* p : {&r10k_icache_, &r10k_dcache_})
            for (u64 off = 0; off < c.line; off += p->line) {
                const u32 ps = static_cast<u32>(((va + off) / p->line) % p->sets);
                for (unsigned w = 0; w < 2; ++w) {
                    u64& t = p->tags[w * p->sets + ps];
                    const bool is_i = p == &r10k_icache_;
                    const u64 ta = ((t >> 32 & 0xF) << 36) | ((t >> 8 & 0xFFFFFF) << 12);
                    if ((t & (is_i ? 0x40u : 0xC0u)) && ta == ((pa + off) & 0xFFFFFFF000ull))
                        t = is_i ? t & ~0x40ull : (t & ~0xE0000000000000C2ull) | (1ull << 61);
                }
            }
    };
    auto set_ch = [this](bool h) { status_ = h ? (status_ | kStatusCH) : (status_ & ~kStatusCH); };
    u64& tag = c.tags[way * c.sets + set];

    switch (operation) {
        case 0:  // Index Invalidate (I), Index Writeback Invalidate (D, S)
            if (secondary && valid(tag)) {
                invalidate_subsets(tag);
                // The tag is rewritten with the instruction's address and virtual index.
                tag = (tag & 0x8000000000000000ull) | ((paddr >> 36 & 0xF) << 32) | ((paddr >> 18 & 0x3FFFF) << 14) |
                      ((vaddr >> 12 & 3) << 7);
            } else {
                invalidate(tag);
            }
            break;
        case 1:  // Index Load Tag
            taglo_ = static_cast<u32>(tag);
            taghi_ = static_cast<u32>(tag >> 32);
            break;
        case 2: {  // Index Store Tag
            const u64 t = ((u64{taghi_} << 32) | taglo_) & fields;
            tag = t;
            if (secondary) {  // MRU belongs to the set: both ways show it
                u64& other = c.tags[(way ^ 1) * c.sets + set];
                other = (other & ~0x8000000000000000ull) | (t & 0x8000000000000000ull);
            }
            break;
        }
        case 4:    // Hit Invalidate
        case 5: {  // Hit Writeback Invalidate (D, S); 0x14 (Cache Barrier) is handled above
            int w = hit(0) ? 0 : hit(1) ? 1 : -1;
            if (w >= 0) {
                u64& t = c.tags[static_cast<unsigned>(w) * c.sets + set];
                if (secondary) {
                    invalidate_subsets(t);
                    t = (t & 0x8000000000000000ull) | ((paddr >> 36 & 0xF) << 32) | ((paddr >> 18 & 0x3FFFF) << 14) |
                        ((vaddr >> 12 & 3) << 7);
                } else {
                    invalidate(t);
                }
            }
            if (secondary) set_ch(w >= 0);
            break;
        }
        case 6:  // Index Load Data
        case 7: {  // Index Store Data
            const bool store = operation == 7;
            if (secondary) {
                const u64 way_bytes = u64{c.sets} * c.line;
                const std::size_t dw = static_cast<std::size_t>(way * (way_bytes / 8) + (paddr % way_bytes) / 8);
                if (store) {
                    c.data[dw] = (u64{taghi_} << 32) | taglo_;
                    c.data[dw ^ 1] = 0;  // the rest of the quadword
                    c.check[dw / 2] = static_cast<u16>(ecc_ & 0x3FF);
                } else {
                    taghi_ = static_cast<u32>(c.data[dw] >> 32);
                    taglo_ = static_cast<u32>(c.data[dw]);
                    ecc_ = c.check[dw / 2];
                }
            } else {
                const std::size_t word = way * 4096u + ((vaddr >> 2) & 0xFFF);  // 16 KiB a way
                if (store) {
                    c.data[word] = icache ? (u64{taghi_ & 0xF} << 32) | taglo_ : taglo_;
                    c.check[word] = static_cast<u16>(ecc_ & (icache ? 1 : 0xF));
                } else {
                    taglo_ = static_cast<u32>(c.data[word]);
                    if (icache) taghi_ = static_cast<u32>(c.data[word] >> 32);
                    ecc_ = c.check[word];
                }
            }
            break;
        }
        default: break;  // not an R10000 operation: undefined
    }
}

#undef SG_REQUIRE_64BIT

}  // namespace sg::mips
