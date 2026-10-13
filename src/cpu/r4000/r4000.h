// SPDX-License-Identifier: GPL-2.0-or-later
// MIPS R4000/R4400 CPU - reference interpreter.
//
// This interpreter is the correctness baseline for the whole project: the
// future JIT is validated against it instruction-for-instruction. It favours
// clarity, but keeps the hot paths (fetch, load/store via the soft-TLB) cheap.
#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <vector>

#include "core/types.h"

namespace sg {
class Bus;
}

namespace sg::mips {

namespace cop0 {
enum Reg : unsigned {
    Index = 0,
    Random = 1,
    EntryLo0 = 2,
    EntryLo1 = 3,
    Context = 4,
    PageMask = 5,
    Wired = 6,
    BadVAddr = 8,
    Count = 9,
    EntryHi = 10,
    Compare = 11,
    Status = 12,
    Cause = 13,
    EPC = 14,
    PRId = 15,
    Config = 16,
    LLAddr = 17,
    WatchLo = 18,
    WatchHi = 19,
    XContext = 20,
    ECC = 26,
    CacheErr = 27,
    TagLo = 28,
    TagHi = 29,
    ErrorEPC = 30,
};
}  // namespace cop0

// Status register bits.
namespace sr {
constexpr u32 IE = 1u << 0;
constexpr u32 EXL = 1u << 1;
constexpr u32 ERL = 1u << 2;
constexpr u32 KSU_MASK = 3u << 3;
constexpr u32 KSU_SUPERVISOR = 1u << 3;
constexpr u32 KSU_USER = 2u << 3;
constexpr u32 UX = 1u << 5;
constexpr u32 SX = 1u << 6;
constexpr u32 KX = 1u << 7;
constexpr u32 IM_MASK = 0xFFu << 8;
constexpr u32 DE = 1u << 16;
constexpr u32 CE = 1u << 17;
constexpr u32 CH = 1u << 18;
constexpr u32 SR = 1u << 20;
constexpr u32 TS = 1u << 21;
constexpr u32 BEV = 1u << 22;
constexpr u32 RE = 1u << 25;
constexpr u32 FR = 1u << 26;
constexpr u32 RP = 1u << 27;
constexpr u32 CU0 = 1u << 28;
constexpr u32 CU1 = 1u << 29;
constexpr u32 CU2 = 1u << 30;
constexpr u32 CU3 = 1u << 31;
}  // namespace sr

enum class Exc : u32 {
    Int = 0,
    Mod = 1,
    TLBL = 2,
    TLBS = 3,
    AdEL = 4,
    AdES = 5,
    IBE = 6,
    DBE = 7,
    Sys = 8,
    Bp = 9,
    RI = 10,
    CpU = 11,
    Ov = 12,
    Tr = 13,
    VCEI = 14,
    FPE = 15,
    WATCH = 23,
    VCED = 31,
};

struct TlbEntry {
    u64 mask = 0;  // PageMask bits 24:13
    u64 hi = 0;    // R | VPN2 | ASID
    u64 lo[2] = {0, 0};
    bool global = false;
};

class R4000 {
public:
    static constexpr unsigned kTlbEntries = 48;
    static constexpr u64 kResetVector = 0xFFFFFFFFBFC00000ull;

    // R4400SC as fitted to the Indy: SysClock = PClock/3, 128-byte secondary
    // lines, big-endian, 16 KiB primary caches (I: 32-byte lines, D: 16-byte),
    // kseg0 cacheable-noncoherent. SC (bit 17) is derived from scache_bytes.
    static constexpr u32 kDefaultConfig = 0x10C084A3u;

    struct Options {
        u32 prid = 0x00000450;          // R4400 rev 5.0 (150 MHz Indy)
        u32 config = kDefaultConfig;
        u32 scache_bytes = 1u << 20;    // 0 = no secondary cache (R4x00PC)
    };

    explicit R4000(Bus& bus) : R4000(bus, Options{}) {}
    R4000(Bus& bus, const Options& options);

    void reset();
    void step();
    // Executes up to `max_instructions`; returns the number executed.
    u64 run(u64 max_instructions);
    // Executes until the cycle counter reaches `cycle` (or `*deadline`, which
    // may move earlier while running) or a stop is requested.
    void run_until(u64 cycle, const u64* deadline = nullptr);
    bool stop_requested() const { return stop_.load(std::memory_order_relaxed); }
    void clear_stop() { stop_.store(false, std::memory_order_relaxed); }
    void request_stop() { stop_.store(true, std::memory_order_relaxed); }

    // External interrupt lines Int0..Int4 drive Cause.IP2..IP6 (IP7 is the timer).
    void set_int_line(unsigned line, bool asserted);

    void set_pc(u64 new_pc) {
        pc = new_pc;
        npc_ = new_pc + 4;
        next_delay_ = false;
    }

    // COP0 access with DMFC0/DMTC0 semantics (masks applied, side effects included).
    u64 cp0_read(unsigned reg) const;
    void cp0_write(unsigned reg, u64 value);
    const TlbEntry& tlb_entry(unsigned i) const { return tlb_[i]; }
    u64 cycles() const { return cycles_; }

    // Debugger hook, called as each exception is taken: (ExcCode, faulting PC,
    // Status before the exception).
    std::function<void(unsigned code, u64 pc, u64 status)> exception_hook;

    // Side-effect-free translation and read, for debuggers and tracing.
    bool debug_translate(u64 vaddr, u64& paddr) const;
    bool debug_read32(u64 vaddr, u32& value) const;

    // Architectural state, public for the debugger and unit tests.
    std::array<u64, 32> gpr{};
    u64 hi = 0;
    u64 lo = 0;
    u64 pc = 0;  // address of the next instruction to execute
    std::array<u64, 32> fpr{};
    u32 fcr31 = 0;

private:
    enum class Access { Fetch, Load, Store };
    enum Mode : unsigned { kKernel = 0, kSupervisor = 1, kUser = 2, kKernelErl = 3, kNumModes = 4 };

    // Soft-TLB: direct-mapped cache of recent 4 KiB virtual->physical
    // translations per (mode, read/write). Entries for RAM/ROM carry a host
    // pointer so the common case is one compare plus a byte-swapped load.
    struct FastEntry {
        u64 tag = ~u64{0};  // vaddr >> 12
        u8* host = nullptr; // host pointer to the 4 KiB page, or nullptr for MMIO
        u64 ppage = 0;      // physical page address
    };
    static constexpr unsigned kFastBits = 8;
    static constexpr unsigned kFastEntries = 1u << kFastBits;

    // Execution
    void execute(u32 inst);
    void execute_special(u32 inst);
    void execute_regimm(u32 inst);
    void execute_cop0(u32 inst);
    void execute_cop1(u32 inst);
    void execute_cache(u32 inst);

    // FPU (fpu.cpp)
    template <typename F>
    void fp_compute(unsigned func, unsigned ft, unsigned fs, unsigned fd);
    void fp_convert_from_int(unsigned fmt, unsigned func, unsigned fs, unsigned fd);
    bool fp_commit(u32 cause);  // updates FCR31; false if an FPE was raised
    void fp_unimplemented();
    void fp_check_ctc1_trap();
    void branch(bool taken, u64 target) {
        next_delay_ = true;
        if (taken) npc_ = target;
    }
    void branch_likely(bool taken, u64 target) {
        if (taken) {
            next_delay_ = true;
            npc_ = target;
        } else {  // nullify the delay slot
            pc = npc_;
            npc_ = pc + 4;
        }
    }
    void jump(u64 target) {
        next_delay_ = true;
        npc_ = target;
    }

    // Exceptions
    void raise(Exc code, bool tlb_refill = false, bool xtlb = false, unsigned ce = 0);
    void address_error(u64 vaddr, Access acc);
    void tlb_exception(Exc code, u64 vaddr, bool refill, bool xtlb);
    bool interrupt_pending() const {
        return (cause_ & status_ & sr::IM_MASK) != 0 && (status_ & (sr::IE | sr::EXL | sr::ERL)) == sr::IE;
    }
    void timer_fire();
    void reschedule_timer();

    // Address translation
    template <bool kRaise>
    bool translate(u64 vaddr, Access acc, u64& paddr);
    template <bool kRaise>
    bool tlb_lookup(u64 vaddr, Access acc, bool xtlb, u64& paddr);
    bool lookup_slow(u64 vaddr, Access acc, u8*& host, u64& paddr);
    template <Access A>
    bool lookup(u64 vaddr, u8*& host, u64& paddr);
    void flush_fast_tlb();
    void update_mode();
    bool ops64_enabled() const;
    // Load/store address: a full 64-bit sum with 64-bit addressing, otherwise
    // the 32-bit sum sign-extended (32-bit binaries rely on the wrap).
    u64 effective_address(u64 base, u64 offset) const {
        const u64 a = base + offset;
        return addr64_ ? a : static_cast<u64>(static_cast<s64>(static_cast<s32>(a)));
    }
    bool cop0_usable() const { return mode_ == kKernel || mode_ == kKernelErl || (status_ & sr::CU0); }

    // Memory access (return false if an exception was raised)
    bool fetch(u32& inst);
    template <typename T>
    bool load(u64 vaddr, T& out);
    template <typename T>
    bool store(u64 vaddr, T value);
    template <typename T>
    bool load_containing(u64 vaddr, T& out);  // aligned T containing vaddr (LWL/LWR/LDL/LDR)
    template <typename T>
    bool merge_store(u64 vaddr, T value, T keep_mask);  // SWL/SWR/SDL/SDR
    template <typename T>
    bool phys_read(u64 paddr, T& out, Access acc);
    template <typename T>
    bool phys_write(u64 paddr, T value);

    // TLB instructions
    void tlb_write(unsigned index);
    void tlb_read();
    void tlb_probe();
    u32 random() const;
    u32 count() const { return static_cast<u32>(count_offset_ + static_cast<u32>(cycles_ >> 1)); }

    // FPU register file views (FR=0: 16 64-bit regs as 32 32-bit halves)
    u32 fpr_read32(unsigned n) const;
    void fpr_write32(unsigned n, u32 v);
    u64 fpr_read64(unsigned n) const;
    void fpr_write64(unsigned n, u64 v);

    Bus& bus_;
    Options opts_;
    std::atomic<bool> stop_{false};

    u64 npc_ = 0;
    u64 cur_pc_ = 0;
    bool in_delay_ = false;
    bool next_delay_ = false;
    u64 cycles_ = 0;
    u64 timer_event_ = ~u64{0};
    unsigned mode_ = kKernelErl;
    bool addr64_ = false;  // 64-bit addressing in the current mode (KX/SX/UX)

    // COP0
    u32 index_ = 0, wired_ = 0, status_ = 0, cause_ = 0, compare_ = 0;
    u32 config_ = 0, prid_ = 0, pagemask_ = 0, count_offset_ = 0;
    u32 watchlo_ = 0, watchhi_ = 0, ecc_ = 0, cacheerr_ = 0, taglo_ = 0, taghi_ = 0, lladdr_ = 0;
    u64 entrylo0_ = 0, entrylo1_ = 0, context_ = 0, xcontext_ = 0, badvaddr_ = 0;
    u64 entryhi_ = 0, epc_ = 0, errorepc_ = 0;
    bool llbit_ = false;
    u64 random_base_ = 0;
    std::array<TlbEntry, kTlbEntries> tlb_{};

    // Cache tag arrays (no data is cached; tags exist for CACHE-instruction
    // diagnostics and size probing).
    std::vector<u32> itag_, dtag_, stag_;
    std::vector<u64> stag_stored_at_;  // cycle of the last Index Store Tag per S$ line

    FastEntry fast_[kNumModes][2][kFastEntries];
};

}  // namespace sg::mips
