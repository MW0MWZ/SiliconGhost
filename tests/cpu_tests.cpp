// SPDX-License-Identifier: GPL-2.0-or-later
// R4000 interpreter tests: hand-assembled programs run from kseg0 in kernel mode.
#include <array>
#include <cstddef>
#include <initializer_list>
#include <iterator>
#include <tuple>
#include <vector>

#include "core/endian.h"
#include "cpu/r4000/r4000.h"
#include "mem/bus.h"
#include "mips_asm.h"
#include "test.h"

using namespace sg;
using namespace sg::mips;
using namespace sg::test::as;

namespace {

constexpr u64 kCode = 0xFFFFFFFF80001000ull;  // physical 0x1000
constexpr u64 kGeneralVector = 0xFFFFFFFF80000180ull;
constexpr u64 kRefillVector = 0xFFFFFFFF80000000ull;

struct Rig {
    static constexpr u64 kRamSize = 8u << 20;
    std::vector<u8> ram = std::vector<u8>(kRamSize);
    Bus bus;
    R4000 cpu{bus};

    Rig() {
        bus.map_ram(0, kRamSize, ram.data());
        cpu.cp0_write(cop0::Status, 0);  // kernel mode, BEV=0, ERL=0, interrupts off
        cpu.set_pc(kCode);
    }
    void load(std::initializer_list<u32> prog, u64 paddr = 0x1000) {
        for (u32 w : prog) {
            store_be<u32>(&ram[paddr], w);
            paddr += 4;
        }
    }
    void steps(int n) {
        for (int i = 0; i < n; ++i) cpu.step();
    }
    u32 r32(u64 paddr) const { return load_be<u32>(&ram[paddr]); }
    void w32(u64 paddr, u32 v) { store_be<u32>(&ram[paddr], v); }
    unsigned exc_code() const { return (cpu.cp0_read(cop0::Cause) >> 2) & 31; }
    bool bd() const { return (cpu.cp0_read(cop0::Cause) >> 31) & 1; }
};

}  // namespace

TEST(cpu_basic_arithmetic) {
    Rig t;
    t.load({LUI(t0, 0x1234), ORI(t0, t0, 0x5678), ADDIU(t1, zero, -1), ADDU(t2, t0, t1), ADDIU(zero, zero, 5)});
    t.steps(5);
    CHECK_EQ(t.cpu.gpr[t0], 0x12345678ull);
    CHECK_EQ(t.cpu.gpr[t1], ~0ull);
    CHECK_EQ(t.cpu.gpr[t2], 0x12345677ull);
    CHECK_EQ(t.cpu.gpr[zero], 0ull);
    CHECK_EQ(t.cpu.pc, kCode + 20);
}

TEST(cpu_32bit_results_are_sign_extended) {
    Rig t;
    t.load({LUI(t0, 0x7FFF), ORI(t0, t0, 0xFFFF), ADDIU(t1, t0, 1), DSLL32(t2, t0, 0)});
    t.steps(4);
    CHECK_EQ(t.cpu.gpr[t1], 0xFFFFFFFF80000000ull);
    CHECK_EQ(t.cpu.gpr[t2], 0x7FFFFFFF00000000ull);
}

TEST(cpu_add_overflow_traps_without_writing) {
    Rig t;
    t.load({LUI(t0, 0x7FFF), ORI(t0, t0, 0xFFFF), ADDIU(t1, zero, 1), ADD(t2, t0, t1)});
    t.cpu.gpr[t2] = 0x55;
    t.steps(4);
    CHECK_EQ(t.cpu.pc, kGeneralVector);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::Ov));
    CHECK_EQ(t.cpu.cp0_read(cop0::EPC), kCode + 12);
    CHECK(!t.bd());
    CHECK(t.cpu.cp0_read(cop0::Status) & sr::EXL);
    CHECK_EQ(t.cpu.gpr[t2], 0x55ull);
}

TEST(cpu_branch_delay_slot_executes) {
    Rig t;
    t.load({ADDIU(t0, zero, 0), BEQ(zero, zero, 2), ADDIU(t0, t0, 1), ADDIU(t0, t0, 10), ADDIU(t0, t0, 100)});
    t.steps(4);
    CHECK_EQ(t.cpu.gpr[t0], 101ull);
    CHECK_EQ(t.cpu.pc, kCode + 0x14);
}

TEST(cpu_branch_likely_nullifies_delay_slot) {
    Rig t;
    t.load({ADDIU(t0, zero, 0), ADDIU(t1, zero, 1), BEQL(t1, zero, 5), ADDIU(t0, t0, 1), ADDIU(t0, t0, 10)});
    t.steps(4);
    CHECK_EQ(t.cpu.gpr[t0], 10ull);
    CHECK_EQ(t.cpu.pc, kCode + 0x14);
}

TEST(cpu_exception_in_delay_slot_sets_bd) {
    Rig t;
    t.load({LUI(t0, 0x7FFF), ORI(t0, t0, 0xFFFF), BEQ(zero, zero, 4), ADD(t1, t0, t0)});
    t.steps(4);
    CHECK_EQ(t.cpu.pc, kGeneralVector);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::Ov));
    CHECK(t.bd());
    CHECK_EQ(t.cpu.cp0_read(cop0::EPC), kCode + 8);
}

TEST(cpu_unaligned_word_load_store) {
    Rig t;
    const u8 bytes[8] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
    for (int i = 0; i < 8; ++i) t.ram[0x2000 + i] = bytes[i];
    for (int i = 0; i < 8; ++i) t.ram[0x3000 + i] = 0xFF;
    t.cpu.gpr[t0] = 0xFFFFFFFF80002000ull;
    t.cpu.gpr[t3] = 0xFFFFFFFF80003000ull;
    t.cpu.gpr[t2] = sext32(0xDEADBEEF);
    t.load({LWL(t1, 1, t0), LWR(t1, 4, t0), SWL(t2, 1, t3), SWR(t2, 4, t3)});
    t.steps(4);
    CHECK_EQ(t.cpu.gpr[t1], 0x11223344ull);
    CHECK_EQ(t.r32(0x3000), 0xFFDEADBEu);
    CHECK_EQ(t.r32(0x3004), 0xEFFFFFFFu);
}

TEST(cpu_partial_lwr_preserves_upper_bits) {
    Rig t;
    t.w32(0x2000, 0xAABBCCDD);
    t.cpu.gpr[t0] = 0xFFFFFFFF80002000ull;
    t.cpu.gpr[t1] = 0x0123456789ABCDEFull;
    t.load({LWR(t1, 1, t0)});  // loads bytes AA BB into the low half
    t.steps(1);
    CHECK_EQ(t.cpu.gpr[t1], 0x0123456789ABAABBull);
}

TEST(cpu_multiply_divide) {
    Rig t;
    t.cpu.gpr[t0] = sext32(static_cast<u32>(-3));
    t.cpu.gpr[t1] = 7;
    t.load({MULT(t0, t1), MFLO(t2), MFHI(t3), DIV(t1, t0), MFLO(t4), MFHI(t5)});
    t.steps(6);
    CHECK_EQ(t.cpu.gpr[t2], static_cast<u64>(-21));
    CHECK_EQ(t.cpu.gpr[t3], ~0ull);
    CHECK_EQ(t.cpu.gpr[t4], static_cast<u64>(-2));
    CHECK_EQ(t.cpu.gpr[t5], 1ull);

    Rig u;
    u.cpu.gpr[t0] = ~0ull;
    u.cpu.gpr[t1] = 2;
    u.cpu.gpr[t2] = 0x8000000000000000ull;
    u.cpu.gpr[t3] = ~0ull;
    u.load({DMULTU(t0, t1), MFHI(t4), MFLO(t5), DDIV(t2, t3), MFHI(t6), MFLO(t7)});
    u.steps(6);
    CHECK_EQ(u.cpu.gpr[t4], 1ull);
    CHECK_EQ(u.cpu.gpr[t5], 0xFFFFFFFFFFFFFFFEull);
    CHECK_EQ(u.cpu.gpr[t6], 0ull);
    CHECK_EQ(u.cpu.gpr[t7], 0x8000000000000000ull);
}

TEST(cpu_misaligned_load_raises_adel) {
    Rig t;
    t.cpu.gpr[t0] = 0xFFFFFFFF80002001ull;
    t.load({LW(t1, 0, t0)});
    t.steps(1);
    CHECK_EQ(t.cpu.pc, kGeneralVector);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::AdEL));
    CHECK_EQ(t.cpu.cp0_read(cop0::BadVAddr), 0xFFFFFFFF80002001ull);
}

TEST(cpu_32bit_addressing_wraps_effective_address) {
    Rig t;
    t.w32(0x2010, 0xCAFEF00D);
    t.cpu.gpr[t0] = 0x000000007FFFFFF0ull;  // base + offset crosses 2 GiB
    t.load({LW(t1, 0x2020, t0)});
    t.steps(1);
    CHECK_EQ(t.cpu.gpr[t1], 0xFFFFFFFFCAFEF00Dull);  // read from kseg0 0x80002010
    CHECK_EQ(t.cpu.pc, kCode + 4);
}

TEST(cpu_64bit_addressing_does_not_wrap) {
    Rig t;
    t.cpu.cp0_write(cop0::Status, sr::KX);
    t.cpu.gpr[t0] = 0x000000007FFFFFF0ull;
    t.load({LW(t1, 0x2020, t0)});
    t.steps(1);
    CHECK_EQ(t.cpu.pc, kRefillVector + 0x80);  // xkuseg 0x80002010: XTLB refill
}

TEST(cpu_kseg0_kseg1_alias) {
    Rig t;
    t.cpu.gpr[t0] = 0xFFFFFFFF80002000ull;
    t.cpu.gpr[t1] = 0xFFFFFFFFA0002000ull;
    t.cpu.gpr[t2] = 0x1234;
    t.load({SW(t2, 0, t0), LW(t3, 0, t1)});
    t.steps(2);
    CHECK_EQ(t.cpu.gpr[t3], 0x1234ull);
}

namespace {
// Maps VA 0x00400000 (even page, valid, dirty per `dirty`) -> PA 0x00200000;
// the odd page 0x00401000 is invalid. ASID 5, written via a TLBWI instruction.
void setup_tlb(Rig& t, bool dirty) {
    t.cpu.cp0_write(cop0::Index, 3);
    t.cpu.cp0_write(cop0::PageMask, 0);
    t.cpu.cp0_write(cop0::EntryHi, 0x00400000 | 5);
    t.cpu.cp0_write(cop0::EntryLo0, (0x200u << 6) | (3u << 3) | (dirty ? 4u : 0u) | 2u);
    t.cpu.cp0_write(cop0::EntryLo1, 0);
}
}  // namespace

TEST(cpu_tlb_mapped_access_and_invalid_page) {
    Rig t;
    setup_tlb(t, true);
    t.w32(0x200010, 0xCAFEBABE);
    t.load({TLBWI, LUI(t0, 0x0040), LW(t1, 0x10, t0), SW(t1, 0x14, t0), LW(t2, 0x1000, t0)});
    t.steps(5);
    CHECK_EQ(t.cpu.gpr[t1], sext32(0xCAFEBABE));
    CHECK_EQ(t.r32(0x200014), 0xCAFEBABEu);
    CHECK_EQ(t.cpu.pc, kGeneralVector);  // invalid entry: not a refill
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::TLBL));
    CHECK_EQ(t.cpu.cp0_read(cop0::BadVAddr), 0x00401000ull);
    CHECK_EQ(t.cpu.cp0_read(cop0::EntryHi), 0x00400000ull | 5);
}

TEST(cpu_tlb_refill_sets_context) {
    Rig t;
    setup_tlb(t, true);
    t.load({TLBWI, LUI(t0, 0x0080), LW(t1, 0, t0)});
    t.steps(3);
    CHECK_EQ(t.cpu.pc, kRefillVector);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::TLBL));
    CHECK_EQ(t.cpu.cp0_read(cop0::Context) & 0x7FFFF0, (0x00800000ull >> 9) & 0x7FFFF0);
}

TEST(cpu_tlb_asid_change_invalidates_soft_tlb) {
    Rig t;
    setup_tlb(t, true);
    t.load({TLBWI, LUI(t0, 0x0040), LW(t1, 0, t0), ADDIU(t2, zero, 6), MTC0(t2, cop0::EntryHi), LW(t1, 0, t0)});
    t.steps(4);
    CHECK_EQ(t.cpu.pc, kCode + 0x10);  // first access hit
    t.steps(2);
    CHECK_EQ(t.cpu.pc, kRefillVector);  // ASID 6 must not reuse the cached ASID 5 translation
}

TEST(cpu_tlb_store_to_clean_page_raises_mod) {
    Rig t;
    setup_tlb(t, false);
    t.load({TLBWI, LUI(t0, 0x0040), LW(t1, 0, t0), SW(t1, 0, t0)});
    t.steps(4);
    CHECK_EQ(t.cpu.pc, kGeneralVector);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::Mod));
}

TEST(cpu_tlbp_finds_entry) {
    Rig t;
    setup_tlb(t, true);
    t.load({TLBWI, TLBP});
    t.steps(1);
    t.cpu.cp0_write(cop0::Index, 0);
    t.steps(1);
    CHECK_EQ(t.cpu.cp0_read(cop0::Index), 3ull);
}

TEST(cpu_eret_to_user_mode_and_64bit_op_is_reserved) {
    Rig t;
    // Global, valid, dirty mapping of user VA 0x10000 -> PA 0x10000.
    t.cpu.cp0_write(cop0::Index, 0);
    t.cpu.cp0_write(cop0::EntryHi, 0x00010000);
    t.cpu.cp0_write(cop0::EntryLo0, (0x10u << 6) | (3u << 3) | 7u);
    t.cpu.cp0_write(cop0::EntryLo1, 1);
    t.cpu.cp0_write(cop0::EPC, 0x10000);
    t.cpu.cp0_write(cop0::Status, sr::KSU_USER | sr::EXL);
    t.load({TLBWI, ERET});
    t.load({ADDIU(t0, zero, 1), DADDU(t1, t0, t0)}, 0x10000);
    t.steps(2);
    CHECK_EQ(t.cpu.pc, 0x10000ull);
    CHECK(!(t.cpu.cp0_read(cop0::Status) & sr::EXL));
    t.steps(2);
    CHECK_EQ(t.cpu.gpr[t0], 1ull);
    CHECK_EQ(t.cpu.pc, kGeneralVector);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::RI));
    CHECK_EQ(t.cpu.cp0_read(cop0::EPC), 0x10004ull);
}

TEST(cpu_user_mode_cannot_touch_kseg0) {
    Rig t;
    t.cpu.cp0_write(cop0::Index, 0);
    t.cpu.cp0_write(cop0::EntryHi, 0x00010000);
    t.cpu.cp0_write(cop0::EntryLo0, (0x10u << 6) | (3u << 3) | 7u);
    t.cpu.cp0_write(cop0::EntryLo1, 1);
    t.cpu.cp0_write(cop0::EPC, 0x10000);
    t.cpu.cp0_write(cop0::Status, sr::KSU_USER | sr::EXL);
    t.cpu.gpr[t0] = 0xFFFFFFFF80002000ull;
    t.load({TLBWI, ERET});
    t.load({LW(t1, 0, t0)}, 0x10000);
    t.steps(3);
    CHECK_EQ(t.cpu.pc, kGeneralVector);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::AdEL));
}

TEST(cpu_syscall_and_eret_round_trip) {
    Rig t;
    t.load({SYSCALL, ADDIU(t0, zero, 42)});
    t.load({MFC0(k0, cop0::EPC), ADDIU(k0, k0, 4), MTC0(k0, cop0::EPC), ERET}, 0x180);
    t.steps(1);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::Sys));
    t.steps(5);
    CHECK_EQ(t.cpu.gpr[t0], 42ull);
    CHECK_EQ(t.cpu.pc, kCode + 8);
}

TEST(cpu_timer_interrupt) {
    Rig t;
    t.cpu.cp0_write(cop0::Count, 0);
    t.cpu.cp0_write(cop0::Compare, 10);
    t.cpu.cp0_write(cop0::Status, sr::IE | (0x80u << 8));  // IM7
    t.steps(19);
    CHECK(t.cpu.pc < kGeneralVector || t.cpu.pc >= kCode);  // not yet
    t.steps(2);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::Int));
    CHECK(t.cpu.cp0_read(cop0::Cause) & (1u << 15));
    CHECK(t.cpu.cp0_read(cop0::Status) & sr::EXL);
    // Writing Compare acknowledges the interrupt.
    t.cpu.cp0_write(cop0::Compare, 0);
    CHECK(!(t.cpu.cp0_read(cop0::Cause) & (1u << 15)));
}

TEST(cpu_external_interrupt_line) {
    Rig t;
    t.load({NOP, NOP, NOP});
    t.cpu.cp0_write(cop0::Status, sr::IE | (0x04u << 8));  // IM2 = Int0
    t.steps(1);
    t.cpu.set_int_line(0, true);
    t.steps(1);
    CHECK_EQ(t.cpu.pc, kGeneralVector);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::Int));
    CHECK_EQ(t.cpu.cp0_read(cop0::EPC), kCode + 4);
}

TEST(cpu_bev_vectors_and_reset_state) {
    Bus bus;
    R4000 cpu(bus);
    CHECK_EQ(cpu.pc, R4000::kResetVector);
    CHECK(cpu.cp0_read(cop0::Status) & sr::BEV);
    CHECK(cpu.cp0_read(cop0::Status) & sr::ERL);
    CHECK_EQ(cpu.cp0_read(cop0::PRId), 0x450ull);
    // Fetch from an unmapped PROM address -> instruction bus error via BEV vector.
    cpu.step();
    CHECK_EQ(cpu.pc, 0xFFFFFFFFBFC00380ull);
    CHECK_EQ((cpu.cp0_read(cop0::Cause) >> 2) & 31, static_cast<u64>(Exc::IBE));
}

namespace {
// Polls a RAM word that stays zero until the Compare interrupt (IP7) fires;
// the exception vector then spins too.
void load_spin_program(Rig& t) {
    t.cpu.gpr[t0] = 0xFFFFFFFF80002000ull;
    t.load({LW(t1, 0, t0), BEQ(t1, zero, -2), NOP});
    t.load({BEQ(zero, zero, -1), NOP}, 0x180);
    t.cpu.cp0_write(cop0::Compare, 50000);  // Count == Compare at cycle 100000
    t.cpu.cp0_write(cop0::Status, sr::IE | (1u << 15));
}
}  // namespace

TEST(cpu_idle_skip_is_exact) {
    Rig fast, slow;
    load_spin_program(fast);
    load_spin_program(slow);
    slow.cpu.set_idle_skip(false);
    fast.cpu.run_until(150000);
    slow.cpu.run_until(150000);
    CHECK_EQ(fast.cpu.cycles(), slow.cpu.cycles());
    CHECK_EQ(fast.cpu.pc, slow.cpu.pc);
    CHECK_EQ(fast.cpu.cp0_read(cop0::EPC), slow.cpu.cp0_read(cop0::EPC));
    CHECK_EQ(fast.cpu.cp0_read(cop0::Cause), slow.cpu.cp0_read(cop0::Cause));
    CHECK_EQ(fast.cpu.cp0_read(cop0::Count), slow.cpu.cp0_read(cop0::Count));
    CHECK_EQ(fast.cpu.cp0_read(cop0::Random), slow.cpu.cp0_read(cop0::Random));
    CHECK(fast.cpu.cp0_read(cop0::EPC) == kCode || fast.cpu.cp0_read(cop0::EPC) == kCode + 4 ||
          fast.cpu.cp0_read(cop0::EPC) == kCode + 8);
    CHECK(fast.cpu.idle_skipped_cycles() > 140000);
    CHECK(fast.cpu.instructions() < 1000);
    CHECK_EQ(slow.cpu.idle_skipped_cycles(), 0u);
}

TEST(cpu_idle_skip_ignores_loops_that_progress_or_store) {
    // Count t2 up to 1000, storing it each time, then spin.
    auto storing = [](Rig& t) {
        t.cpu.gpr[t0] = 0xFFFFFFFF80002000ull;
        t.cpu.gpr[t3] = 1000;
        t.load({ADDIU(t2, t2, 1), SW(t2, 0, t0), BNE(t2, t3, -3), NOP, BEQ(zero, zero, -1), NOP});
    };
    Rig a, b;
    storing(a);
    storing(b);
    b.cpu.set_idle_skip(false);
    a.cpu.run_until(3999);  // still counting
    b.cpu.run_until(3999);
    CHECK_EQ(a.cpu.instructions(), 3999u);
    CHECK_EQ(a.cpu.gpr[t2], b.cpu.gpr[t2]);
    a.cpu.run_until(100000);  // finished; the final spin is skipped
    b.cpu.run_until(100000);
    CHECK_EQ(a.cpu.gpr[t2], 1000u);
    CHECK_EQ(a.r32(0x2000), 1000u);
    CHECK_EQ(a.cpu.pc, b.cpu.pc);
    CHECK(a.cpu.instructions() < 4100);

    Rig u;  // a counting loop without stores makes progress: never skipped
    u.load({ADDIU(t2, t2, 1), BNE(t2, t3, -2), NOP, BEQ(zero, zero, -1), NOP});
    u.cpu.gpr[t3] = 5000;
    u.cpu.run_until(10002);
    CHECK_EQ(u.cpu.gpr[t2], 3334u);
    CHECK_EQ(u.cpu.instructions(), 10002u);
}

TEST(cpu_fractional_cycles_per_instruction) {
    Rig t;
    t.load({BEQ(zero, zero, -1), NOP});
    t.cpu.set_idle_skip(false);
    t.cpu.set_cycles_per_instruction(0.25);
    t.cpu.run_until(1000);
    CHECK_EQ(t.cpu.cycles(), 1000u);
    CHECK_EQ(t.cpu.instructions(), 4000u);
    CHECK_EQ(t.cpu.cp0_read(cop0::Count), 500u);  // Count follows guest cycles, not instructions
}

namespace {
// A device register whose value never changes; `pure` says whether reading
// it is declared side-effect free.
struct StatusDevice final : Device {
    bool pure;
    unsigned reads = 0;
    explicit StatusDevice(bool p) : pure(p) {}
    const char* name() const override { return "status"; }
    bool read(u32, unsigned, u64& v) override {
        ++reads;
        v = 0x80;
        return true;
    }
    bool write(u32, unsigned, u64) override { return true; }
    bool read_is_pure(u32, unsigned) const override { return pure; }
};

// The PROM's console wait in miniature: poll a status register, store two
// different values to the same stack slot (as calls at the same depth do),
// count down a delay, repeat.
void load_poll_program(Rig& t, StatusDevice& dev) {
    t.bus.map_device(0x1F000000, Bus::kPageSize, &dev);
    t.cpu.gpr[t0] = 0xFFFFFFFFBF000000ull;  // kseg1: the device
    t.cpu.gpr[sp] = 0xFFFFFFFF80003000ull;
    t.cpu.gpr[s0] = 0x1234;
    t.cpu.gpr[s1] = 0x5678;
    t.load({
        LW(t1, 0, t0),             // 1000: poll
        SW(s0, 0, sp),             // 1004: the slot changes within a pass...
        SW(s1, 0, sp),             // 1008: ...but ends each pass the same
        ADDIU(t4, zero, 20),       // 100c: delay count
        ADDIU(t4, t4, -1),         // 1010: delay loop
        BNE(t4, zero, -2), NOP,    // 1014
        BEQ(zero, zero, -8), NOP,  // 101c: back to the poll
    });
    t.load({BEQ(zero, zero, -1), NOP}, 0x180);
    t.cpu.cp0_write(cop0::Compare, 50000);
    t.cpu.cp0_write(cop0::Status, sr::IE | (1u << 15));
}
}  // namespace

TEST(cpu_idle_skip_nested_poll_loop_with_pure_device_read) {
    Rig fast, slow;
    StatusDevice dev_fast(true), dev_slow(true);
    load_poll_program(fast, dev_fast);
    load_poll_program(slow, dev_slow);
    slow.cpu.set_idle_skip(false);
    fast.cpu.run_until(150000);
    slow.cpu.run_until(150000);
    CHECK_EQ(fast.cpu.cycles(), slow.cpu.cycles());
    CHECK_EQ(fast.cpu.pc, slow.cpu.pc);
    CHECK_EQ(fast.cpu.cp0_read(cop0::EPC), slow.cpu.cp0_read(cop0::EPC));
    CHECK_EQ(fast.cpu.cp0_read(cop0::Cause), slow.cpu.cp0_read(cop0::Cause));
    CHECK_EQ(fast.cpu.gpr[t4], slow.cpu.gpr[t4]);
    CHECK(fast.cpu.idle_skipped_cycles() > 140000);
    CHECK(dev_fast.reads < dev_slow.reads / 100);
}

TEST(cpu_idle_skip_never_skips_impure_device_reads) {
    Rig t;
    StatusDevice dev(false);
    load_poll_program(t, dev);
    t.cpu.run_until(40000);
    CHECK_EQ(t.cpu.idle_skipped_cycles(), 0u);
    CHECK_EQ(t.cpu.instructions(), 40000u);
}

TEST(cpu_block_cache_sees_code_written_inside_the_running_block) {
    for (bool blocks : {false, true}) {
        Rig t;
        t.cpu.set_block_cache(blocks);
        t.cpu.set_idle_skip(false);
        t.w32(0x2000, ADDIU(t2, zero, 42));  // the replacement instruction
        t.load({
            LUI(t0, 0x8000), ORI(t0, t0, 0x1018),
            LUI(t3, 0x8000), LW(t1, 0x2000, t3),   // 1008, 100c
            SW(t1, 0, t0),                         // 1010: overwrite 0x1018
            NOP,                                   // 1014
            ADDIU(t2, zero, 7),                    // 1018: replaced before it runs
            BEQ(zero, zero, -1), NOP,              // 101c
        });
        t.cpu.run_until(20);
        CHECK_EQ(t.cpu.gpr[t2], 42u);
    }
}

TEST(cpu_block_cache_drops_code_overwritten_later_and_by_dma) {
    Rig t;
    t.cpu.set_idle_skip(false);
    t.load({ADDIU(t2, zero, 1), BEQ(zero, zero, -1), NOP});
    t.cpu.run_until(10);
    CHECK_EQ(t.cpu.gpr[t2], 1u);
    // Overwritten by the CPU (from elsewhere), then run again
    t.cpu.gpr[t0] = kCode;
    t.load({SW(t1, 0, t0), BEQ(zero, zero, -1), NOP}, 0x3000);
    t.cpu.gpr[t1] = ADDIU(t2, zero, 2);
    t.cpu.set_pc(0xFFFFFFFF80003000ull);
    t.cpu.run_until(20);
    t.cpu.set_pc(kCode);
    t.cpu.run_until(30);
    CHECK_EQ(t.cpu.gpr[t2], 2u);
    // Overwritten by DMA
    u8 word[4];
    store_be<u32>(word, ADDIU(t2, zero, 3));
    CHECK(t.bus.write_block(0x1000, word, 4));
    t.cpu.set_pc(kCode);
    t.cpu.run_until(40);
    CHECK_EQ(t.cpu.gpr[t2], 3u);
}

TEST(cpu_block_cache_matches_interpreter_on_exception_in_delay_slot) {
    auto run = [](bool blocks) {
        Rig t;
        t.cpu.set_block_cache(blocks);
        t.cpu.set_idle_skip(false);
        t.cpu.gpr[t0] = 0xFFFFFFFF80002001ull;  // misaligned
        t.load({ADDIU(t1, zero, 5), BNE(t1, zero, 4), LW(t2, 0, t0), NOP, NOP, NOP, NOP});
        t.load({BEQ(zero, zero, -1), NOP}, 0x180);
        t.cpu.run_until(50);
        return std::make_tuple(t.cpu.pc, t.cpu.cp0_read(cop0::EPC), t.cpu.cp0_read(cop0::Cause), t.cpu.cycles(),
                               t.cpu.gpr[t1]);
    };
    CHECK(run(false) == run(true));
    CHECK_EQ(std::get<1>(run(true)), kCode + 4);  // EPC at the branch, BD set
    CHECK(std::get<2>(run(true)) >> 31);
}

TEST(cpu_tlb_rewrite_takes_effect_for_cached_translation) {
    Rig t;
    setup_tlb(t, true);
    t.w32(0x200010, 0x11111111);
    t.w32(0x300010, 0x22222222);
    // Read through the mapping (now in the soft TLB), remap the same entry
    // to PA 0x300000 and read again.
    t.load({TLBWI, LUI(t0, 0x0040), LW(t1, 0x10, t0), TLBWI, LW(t2, 0x10, t0), NOP});
    t.steps(3);
    t.cpu.cp0_write(cop0::EntryLo0, (0x300u << 6) | (3u << 3) | 4u | 2u);
    t.steps(2);
    CHECK_EQ(t.cpu.gpr[t1], 0x11111111u);
    CHECK_EQ(t.cpu.gpr[t2], 0x22222222u);
}

TEST(cpu_bus_remap_drops_cached_host_pointers) {
    Rig t;
    std::vector<u8> other(Bus::kPageSize);
    store_be<u32>(&other[0x2010], 0x22222222);
    t.w32(0x22010, 0x11111111);
    t.cpu.gpr[t0] = 0xFFFFFFFF80022000ull;  // data in the second 64 KiB bus page
    t.load({LW(t1, 0x10, t0), LW(t2, 0x10, t0), NOP});
    t.cpu.step();  // translation of 0x80002000 now cached
    t.bus.map_ram(0x20000, Bus::kPageSize, other.data());  // the memory controller remaps
    t.cpu.step();
    CHECK_EQ(t.cpu.gpr[t1], 0x11111111u);
    CHECK_EQ(t.cpu.gpr[t2], 0x22222222u);
}

namespace {

// Final state of a run, for comparing the block cache with step().
struct CpuState {
    std::array<u64, 32> gpr{};
    u64 hi = 0, lo = 0, pc = 0, cycles = 0, epc = 0, cause = 0;
    std::vector<u8> data;
    bool operator==(const CpuState&) const = default;
};

CpuState capture(Rig& t, u64 data_paddr, std::size_t data_size) {
    CpuState s;
    s.gpr = t.cpu.gpr;
    s.hi = t.cpu.hi;
    s.lo = t.cpu.lo;
    s.pc = t.cpu.pc;
    s.cycles = t.cpu.cycles();
    s.epc = t.cpu.cp0_read(cop0::EPC);
    s.cause = t.cpu.cp0_read(cop0::Cause);
    s.data.assign(t.ram.begin() + static_cast<std::ptrdiff_t>(data_paddr),
                  t.ram.begin() + static_cast<std::ptrdiff_t>(data_paddr + data_size));
    return s;
}

// A random straight-line program with forward branches, using every
// instruction the block cache decodes ahead (and a few it does not).
std::vector<u32> random_program(u32 seed, unsigned length) {
    u64 x = seed * 0x9E3779B97F4A7C15ull + 1;
    auto rnd = [&](u32 n) {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        return static_cast<u32>(x % n);
    };
    auto reg = [&] { return rnd(32); };
    auto dst = [&] {  // never s0 (the data pointer)
        unsigned r;
        do r = rnd(32);
        while (r == s0);
        return r;
    };
    static constexpr unsigned kRFuncts[] = {0x00, 0x02, 0x03, 0x04, 0x06, 0x07, 0x10, 0x12, 0x18, 0x19,
                                            0x21, 0x23, 0x24, 0x25, 0x26, 0x27, 0x2A, 0x2B, 0x2D, 0x2F,
                                            0x38, 0x3A, 0x3B, 0x3C, 0x3E, 0x3F, 0x11, 0x13, 0x1A, 0x1B};
    static constexpr unsigned kImmOps[] = {0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x19};
    static constexpr unsigned kLoadStore[] = {0x20, 0x21, 0x23, 0x24, 0x25, 0x27, 0x37, 0x28, 0x29, 0x2B, 0x3F, 0x22, 0x26};
    static constexpr unsigned kBranches[] = {0x04, 0x05, 0x06, 0x07, 0x14, 0x15};

    std::vector<u32> prog;
    while (prog.size() + 2 < length) {
        const unsigned room = length - 2 - static_cast<unsigned>(prog.size());
        switch (rnd(8)) {
            case 0:
            case 1:
            case 2: {
                const unsigned f = kRFuncts[rnd(std::size(kRFuncts))];
                prog.push_back(R(f, reg(), reg(), dst(), rnd(32)));
                break;
            }
            case 3:
            case 4: prog.push_back(I(kImmOps[rnd(std::size(kImmOps))], reg(), dst(), static_cast<s32>(rnd(65536)) - 32768)); break;
            case 5: {  // aligned for every size, within the data area
                const unsigned op = kLoadStore[rnd(std::size(kLoadStore))];
                prog.push_back(I(op, s0, dst(), static_cast<s32>(rnd(128) * 8 + rnd(2) * (op == 0x22 || op == 0x26 ? 3 : 0))));
                break;
            }
            case 6: {  // forward branch or jump; the delay slot is a plain instruction
                if (room < 12) break;
                const s32 off = static_cast<s32>(1 + rnd(8));
                if (rnd(4) == 0) {
                    const u64 target = kCode + 4 * (prog.size() + 1 + static_cast<unsigned>(off));
                    prog.push_back((0x02u << 26) | static_cast<u32>((target >> 2) & 0x03FFFFFF));
                } else if (rnd(3) == 0) {
                    prog.push_back(I(0x01, reg(), rnd(2), off));  // BLTZ/BGEZ
                } else {
                    prog.push_back(I(kBranches[rnd(std::size(kBranches))], reg(), reg(), off));
                }
                prog.push_back(I(0x09, reg(), dst(), static_cast<s32>(rnd(256))));
                break;
            }
            default: prog.push_back(rnd(2) ? NOP : I(0x0D, reg(), dst(), static_cast<s32>(rnd(65536)))); break;
        }
    }
    prog.push_back(BEQ(zero, zero, -1));
    prog.push_back(NOP);
    return prog;
}

}  // namespace

TEST(cpu_block_cache_matches_interpreter_on_random_programs) {
    constexpr u64 kData = 0x10000;
    for (u32 seed = 1; seed <= 200; ++seed) {
        const std::vector<u32> prog = random_program(seed, 600);
        auto run = [&](bool blocks) {
            Rig t;
            t.cpu.set_block_cache(blocks);
            t.cpu.set_idle_skip(false);
            u64 x = seed;
            for (unsigned i = 1; i < 32; ++i) {
                x = x * 6364136223846793005ull + 1442695040888963407ull;
                t.cpu.gpr[i] = (i & 1) ? sext32(x >> 17) : x;
            }
            t.cpu.gpr[s0] = 0xFFFFFFFF80000000ull + kData;
            for (u64 a = 0; a < 0x800; a += 4) t.w32(kData + a, static_cast<u32>(a * 0x01010101u ^ seed));
            u64 paddr = 0x1000;
            for (u32 w : prog) {
                t.w32(paddr, w);
                paddr += 4;
            }
            t.load({BEQ(zero, zero, -1), NOP}, 0x180);  // exceptions (overflow) end up here
            t.cpu.run_until(3000);
            return capture(t, kData, 0x800);
        };
        const CpuState reference = run(false);
        if (!(run(true) == reference)) {
            CHECK_EQ(seed, 0u);  // report the failing seed
            break;
        }
    }
}

TEST(cpu_linked_blocks_match_interpreter_with_timer_interrupts) {
    auto run = [](bool blocks) {
        Rig t;
        t.cpu.set_block_cache(blocks);
        t.cpu.set_idle_skip(false);
        // Main loop: re-arm Compare every 8 iterations; t3 counts interrupts.
        t.load({
            ADDIU(t2, zero, 3000),
            ADDIU(t0, t0, 1),                       // 1004: loop
            ANDI(t1, t0, 7),
            BNE(t1, zero, 4), NOP,
            MFC0(t4, cop0::Count), ADDIU(t4, t4, 37), MTC0(t4, cop0::Compare), NOP,
            BNE(t0, t2, -9), NOP,                    // 1024 -> 1004
            BEQ(zero, zero, -1), NOP,
        });
        t.load({ADDIU(t3, t3, 1), MFC0(k0, cop0::Count), ADDIU(k0, k0, 500), MTC0(k0, cop0::Compare), ERET}, 0x180);
        t.cpu.cp0_write(cop0::Compare, 100);
        t.cpu.cp0_write(cop0::Status, sr::IE | (0x80u << 8));
        t.cpu.run_until(40000);
        return capture(t, 0, 0);
    };
    const CpuState reference = run(false);
    CHECK(run(true) == reference);
    CHECK(reference.gpr[t3] > 10u);
    CHECK_EQ(reference.gpr[t0], 3000u);
}
