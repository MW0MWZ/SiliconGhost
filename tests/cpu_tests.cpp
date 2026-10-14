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
// Runs from `base` (kseg0), which may be anywhere in the first 64 KiB.
std::vector<u32> random_program(u32 seed, unsigned length, u64 base = kCode) {
    u64 x = seed * 0x9E3779B97F4A7C15ull + 1;
    auto rnd = [&](u32 n) {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        return static_cast<u32>(x % n);
    };
    auto reg = [&] { return rnd(32); };
    auto dst = [&] {  // never s0 (the data pointer), s1 (the loop counter) or s2 (jump targets)
        unsigned r;
        do r = rnd(32);
        while (r == s0 || r == s1 || r == s2);
        return r;
    };
    static constexpr unsigned kRFuncts[] = {0x00, 0x02, 0x03, 0x04, 0x06, 0x07, 0x10, 0x12, 0x18, 0x19,
                                            0x21, 0x23, 0x24, 0x25, 0x26, 0x27, 0x2A, 0x2B, 0x2D, 0x2F,
                                            0x38, 0x3A, 0x3B, 0x3C, 0x3E, 0x3F, 0x11, 0x13, 0x1A, 0x1B};
    static constexpr unsigned kImmOps[] = {0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x19};
    static constexpr unsigned kLoadStore[] = {0x20, 0x21, 0x23, 0x24, 0x25, 0x27, 0x37, 0x28, 0x29, 0x2B, 0x3F, 0x22, 0x26};
    static constexpr unsigned kBranches[] = {0x04, 0x05, 0x06, 0x07, 0x14, 0x15, 0x16, 0x17};

    // The body runs several times, so that its blocks become hot.
    std::vector<u32> prog{ADDIU(s1, zero, 12)};
    while (prog.size() + 5 < length) {
        const unsigned room = length - 5 - static_cast<unsigned>(prog.size());
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
            case 5: {  // within the data area; rarely misaligned (an address error)
                const unsigned op = kLoadStore[rnd(std::size(kLoadStore))];
                const s32 skew = (op == 0x22 || op == 0x26) ? static_cast<s32>(rnd(4)) : rnd(150) == 0 ? 1 : 0;
                prog.push_back(I(op, s0, dst(), static_cast<s32>(rnd(128) * 8) + skew));
                break;
            }
            case 6: {  // forward branch or jump; the delay slot is a plain instruction
                if (room < 12) break;
                const s32 off = static_cast<s32>(1 + rnd(8));
                if (rnd(5) == 0) {  // JR or JALR through s2
                    const u64 target = base + 4 * (prog.size() + 3 + static_cast<unsigned>(off));
                    prog.push_back(LUI(s2, static_cast<u16>(target >> 16)));
                    prog.push_back(ORI(s2, s2, static_cast<u16>(target)));
                    prog.push_back(rnd(2) ? JR(s2) : R(0x09, s2, 0, dst()));
                } else if (rnd(4) == 0) {  // J or JAL
                    const u64 target = base + 4 * (prog.size() + 1 + static_cast<unsigned>(off));
                    prog.push_back(((0x02u + rnd(2)) << 26) | static_cast<u32>((target >> 2) & 0x03FFFFFF));
                } else if (rnd(3) == 0) {
                    static constexpr unsigned kRegimm[] = {0x00, 0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13};
                    prog.push_back(I(0x01, reg(), kRegimm[rnd(std::size(kRegimm))], off));  // BLTZ/BGEZ(AL)(L)
                } else {
                    prog.push_back(I(kBranches[rnd(std::size(kBranches))], reg(), reg(), off));
                }
                prog.push_back(I(0x09, reg(), dst(), static_cast<s32>(rnd(256))));
                break;
            }
            default: prog.push_back(rnd(2) ? NOP : I(0x0D, reg(), dst(), static_cast<s32>(rnd(65536)))); break;
        }
    }
    prog.push_back(ADDIU(s1, s1, -1));
    prog.push_back(BNE(s1, zero, 1 - static_cast<s32>(prog.size() + 1)));
    prog.push_back(NOP);
    prog.push_back(BEQ(zero, zero, -1));
    prog.push_back(NOP);
    return prog;
}

}  // namespace

enum class Engine { Interpreter, Blocks, Jit };

void use_engine(Rig& t, Engine engine) {
    t.cpu.set_block_cache(engine != Engine::Interpreter);
    t.cpu.set_jit(engine == Engine::Jit);
}

// Each engine runs `seeds` random programs from physical address `base`.
static void check_random_programs(u32 seeds, unsigned length, u64 base) {
    constexpr u64 kData = 0x10000;
    for (u32 seed = 1; seed <= seeds; ++seed) {
        const std::vector<u32> prog = random_program(seed, length, 0xFFFFFFFF80000000ull + base);
        auto run = [&](Engine engine) {
            Rig t;
            use_engine(t, engine);
            t.cpu.set_idle_skip(false);
            u64 x = seed;
            for (unsigned i = 1; i < 32; ++i) {
                x = x * 6364136223846793005ull + 1442695040888963407ull;
                t.cpu.gpr[i] = (i & 1) ? sext32(x >> 17) : x;
            }
            t.cpu.gpr[s0] = 0xFFFFFFFF80000000ull + kData;
            for (u64 a = 0; a < 0x800; a += 4) t.w32(kData + a, static_cast<u32>(a * 0x01010101u ^ seed));
            u64 paddr = base;
            for (u32 w : prog) {
                t.w32(paddr, w);
                paddr += 4;
            }
            t.load({BEQ(zero, zero, -1), NOP}, 0x180);  // exceptions end up here
            t.cpu.set_pc(0xFFFFFFFF80000000ull + base);
            t.cpu.run_until(20 * length + 8000);
            return capture(t, kData, 0x800);
        };
        const CpuState reference = run(Engine::Interpreter);
        if (!(run(Engine::Blocks) == reference) || !(run(Engine::Jit) == reference)) {
            CHECK_EQ(seed, 0u);  // report the failing seed
            break;
        }
    }
}

TEST(cpu_block_cache_and_jit_match_interpreter_on_random_programs) { check_random_programs(200, 600, 0x1000); }

TEST(cpu_block_cache_and_jit_match_interpreter_on_random_programs_across_pages) {
    check_random_programs(60, 1500, 0x1E00);
}

TEST(cpu_linked_blocks_and_jit_match_interpreter_with_timer_interrupts) {
    auto run = [](Engine engine) {
        Rig t;
        use_engine(t, engine);
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
    const CpuState reference = run(Engine::Interpreter);
    CHECK(run(Engine::Blocks) == reference);
    CHECK(run(Engine::Jit) == reference);
    CHECK(reference.gpr[t3] > 10u);
    CHECK_EQ(reference.gpr[t0], 3000u);
}

TEST(cpu_jit_drops_translated_code_when_it_is_overwritten) {
    for (Engine engine : {Engine::Interpreter, Engine::Blocks, Engine::Jit}) {
        Rig t;
        use_engine(t, engine);
        t.cpu.set_idle_skip(false);
        // A hot loop adds t4 to t2; after 50 passes it patches its own add
        // into "addiu t2, t2, 100" and keeps going until t0 reaches 80.
        t.cpu.gpr[t5] = ADDIU(t2, t2, 100);
        t.cpu.gpr[t6] = 0xFFFFFFFF80001008ull;
        t.cpu.gpr[t4] = 1;
        t.load({
            ADDIU(t1, zero, 50), ADDIU(t3, zero, 80),
            ADDU(t2, t2, t4),          // 1008: patched
            ADDIU(t0, t0, 1),
            BNE(t0, t1, 2), NOP,       // 1010
            SW(t5, 0, t6), NOP,        // 1018: once, at t0 == 50
            BNE(t0, t3, -7), NOP,      // 1020 -> 1008
            BEQ(zero, zero, -1), NOP,
        });
        t.cpu.run_until(5000);
        CHECK_EQ(t.cpu.gpr[t0], 80u);
        CHECK_EQ(t.cpu.gpr[t2], 50u + 30u * 100u);
    }
}

TEST(cpu_jit_self_loop_takes_interrupts_where_the_interpreter_does) {
    auto run = [](Engine engine) {
        Rig t;
        use_engine(t, engine);
        t.cpu.set_idle_skip(false);
        t.load({
            ADDIU(t1, zero, 5000),
            ADDIU(t0, t0, 1),     // 1004: a block that branches to itself
            ADDU(t2, t2, t0),
            BNE(t0, t1, -3), NOP,
            BEQ(zero, zero, -1), NOP,
        });
        t.load({ADDIU(t3, t3, 1), MFC0(k0, cop0::Count), ADDIU(k0, k0, 150), MTC0(k0, cop0::Compare), ERET}, 0x180);
        t.cpu.cp0_write(cop0::Compare, 77);
        t.cpu.cp0_write(cop0::Status, sr::IE | (0x80u << 8));
        t.cpu.run_until(30000);
        return capture(t, 0, 0);
    };
    const CpuState reference = run(Engine::Interpreter);
    CHECK(run(Engine::Blocks) == reference);
    CHECK(run(Engine::Jit) == reference);
    CHECK(reference.gpr[t3] > 20u);
    CHECK_EQ(reference.gpr[t0], 5000u);
}

// A self-looping block whose loads and stores keep missing the soft-TLB
// (two data pages 4 MiB apart share its slots) and which calls a handler
// mid-way: registers held in host registers must survive both.
TEST(cpu_jit_keeps_registers_across_soft_tlb_refills_and_handler_calls) {
    auto run = [](Engine engine) {
        Rig t;
        use_engine(t, engine);
        t.cpu.set_idle_skip(false);
        t.cpu.gpr[s0] = 0xFFFFFFFF80010000ull;
        t.cpu.gpr[s3] = 0xFFFFFFFF80410000ull;
        t.cpu.gpr[t4] = 300;
        t.cpu.gpr[t6] = 7;
        t.w32(0x10000, 5);
        t.load({
            ADDU(t2, t2, t6),       // 1000: loop
            ADDU(t7, t7, t6),
            LW(t1, 0, s0),
            ADDU(t2, t2, t1),
            R(0x2D, t5, t0, t5),    // daddu t5, t5, t0 (a handler)
            ADDIU(t0, t0, 1),
            SW(t0, 0, s3),
            LW(t3, 0, s3),
            ADDU(t2, t2, t3),
            BNE(t0, t4, -10), NOP,  // -> 1000
            BEQ(zero, zero, -1), NOP,
        });
        t.cpu.run_until(10000);
        return capture(t, 0, 0);
    };
    const CpuState reference = run(Engine::Interpreter);
    CHECK(run(Engine::Blocks) == reference);
    CHECK(run(Engine::Jit) == reference);
    CHECK_EQ(reference.gpr[t0], 300u);
    CHECK_EQ(reference.gpr[t7], 2100u);
    CHECK_EQ(reference.gpr[t2], 300u * (7 + 5) + 300u * 301u / 2);
}

// Pages whose fetch translations share a soft-TLB slot: translated code
// jumping from one to the other must not take the slot's translation for
// its own.
TEST(cpu_jit_links_across_pages_only_through_their_own_translation) {
    auto run = [](Engine engine) {
        Rig t;
        use_engine(t, engine);
        t.cpu.set_idle_skip(false);
        // VA 0x402000 -> PA 0x300000; its slot is that of kseg0 0x80002000.
        t.cpu.cp0_write(cop0::Index, 5);
        t.cpu.cp0_write(cop0::PageMask, 0);
        t.cpu.cp0_write(cop0::EntryHi, 0x00402000);
        t.cpu.cp0_write(cop0::EntryLo0, (0x300u << 6) | (3u << 3) | 4u | 2u);
        t.cpu.cp0_write(cop0::EntryLo1, 0);
        t.cpu.gpr[t8] = 0xFFFFFFFF80002000ull;
        t.cpu.gpr[t9] = 0x00402000;
        t.cpu.gpr[t4] = 10;  // the first loop runs ten times, so it is translated
        t.cpu.gpr[t3] = 40;
        t.load({TLBWI, JR(t8), NOP});
        t.load({
                   ADDIU(t1, t1, 1),      // 80002000
                   BNE(t1, t4, -2), NOP,  // -> 80002000 until t1 == t4
                   JR(t9), NOP,           // -> 402000
               },
               0x2000);
        t.load({
                   ADDIU(t2, t2, 1),      // 402000
                   ADDIU(t4, t1, 1),
                   BEQ(t2, t3, 3), NOP,   // -> 402018 when done
                   JR(t8), NOP,           // -> 80002000
                   BEQ(zero, zero, -1), NOP,
               },
               0x300000);
        t.cpu.run_until(5000);
        return capture(t, 0, 0);
    };
    const CpuState reference = run(Engine::Interpreter);
    CHECK(run(Engine::Blocks) == reference);
    CHECK(run(Engine::Jit) == reference);
    CHECK_EQ(reference.gpr[t2], 40u);
    CHECK_EQ(reference.gpr[t1], 49u);
}

TEST(cpu_jit_links_follow_the_guest_translation_when_ram_is_not_host_page_aligned) {
    for (Engine engine : {Engine::Interpreter, Engine::Blocks, Engine::Jit}) {
        // Guest RAM 2 KiB off host page alignment.
        constexpr std::size_t kSize = 8u << 20;
        std::vector<u8> backing(kSize + 0x2000);
        u8* const base = backing.data() + ((0x1000 - reinterpret_cast<uintptr_t>(backing.data()) % 0x1000) % 0x1000) + 0x800;
        Bus bus;
        bus.map_ram(0, kSize, base);
        R4000 cpu(bus);
        cpu.cp0_write(cop0::Status, 0);
        cpu.set_block_cache(engine != Engine::Interpreter);
        cpu.set_jit(engine == Engine::Jit);
        cpu.set_idle_skip(false);
        auto w32 = [&](u64 paddr, u32 v) { store_be<u32>(base + paddr, v); };
        // VA 0x400000 -> PA 0x200000, VA 0x401000 -> PA 0x300000 (not the
        // physically following page, which holds different code).
        cpu.cp0_write(cop0::Index, 3);
        cpu.cp0_write(cop0::PageMask, 0);
        cpu.cp0_write(cop0::EntryHi, 0x00400000);
        cpu.cp0_write(cop0::EntryLo0, (0x200u << 6) | (3u << 3) | 4u | 2u);
        cpu.cp0_write(cop0::EntryLo1, (0x300u << 6) | (3u << 3) | 4u | 2u);
        // First make the physically following code hot (through kseg0, so
        // it has a translation), then clear t1 and enter the mapped pages.
        w32(0x1000, ADDIU(t4, zero, 10));
        w32(0x1004, LUI(t5, 0x8020));
        w32(0x1008, ORI(t5, t5, 0x1000));
        w32(0x100C, R(0x09, t5, 0, ra));  // jalr t5
        w32(0x1010, NOP);
        w32(0x1014, ADDIU(t4, t4, -1));
        w32(0x1018, BNE(t4, zero, -4));   // back to 0x100C
        w32(0x101C, NOP);
        w32(0x1020, ADDIU(t1, zero, 0));
        w32(0x1024, TLBWI);
        w32(0x1028, LUI(t9, 0x0040));
        w32(0x102C, ORI(t9, t9, 0x0FF8));
        w32(0x1030, JR(t9));
        w32(0x1034, ADDIU(t3, zero, 40));
        w32(0x201004, JR(ra));
        w32(0x201008, NOP);
        // Last two words of the first mapped page, then the second page.
        w32(0x200FF8, ADDIU(t0, t0, 1));
        w32(0x200FFC, ADDIU(t2, t2, 1));
        w32(0x201000, ADDIU(t1, t1, 100));  // physically next: must not run from VA 0x401000
        w32(0x300000, ADDIU(t1, t1, 1));
        w32(0x300004, BNE(t0, t3, -4));    // back to 0x400FF8
        w32(0x300008, NOP);
        w32(0x30000C, BEQ(zero, zero, -1));
        w32(0x300010, NOP);
        cpu.set_pc(kCode);
        cpu.run_until(3000);
        CHECK_EQ(cpu.gpr[t0], 40u);
        CHECK_EQ(cpu.gpr[t1], 40u);
    }
}

// The same virtual page mapped to different memory under two ASIDs: code
// switching between the address spaces must always see the current one's
// page, whichever engine runs it.
TEST(cpu_soft_tlb_keeps_address_spaces_apart_across_asid_switches) {
    auto run = [](Engine engine) {
        Rig t;
        use_engine(t, engine);
        t.cpu.set_idle_skip(false);
        t.w32(0x200000, 11);  // ASID 5's page
        t.w32(0x300000, 22);  // ASID 6's page
        t.cpu.gpr[t4] = 200;
        t.load({
            ADDIU(t5, zero, 5), ADDIU(t6, zero, 6), LUI(t0, 0x0040), MTC0(zero, cop0::PageMask),
            MTC0(zero, cop0::EntryLo1),
            ADDIU(a0, zero, 3), MTC0(a0, cop0::Index), ORI(a1, t0, 5), MTC0(a1, cop0::EntryHi),
            ORI(a2, zero, (0x200u << 6) | 0x1E), MTC0(a2, cop0::EntryLo0), TLBWI,
            ADDIU(a0, zero, 4), MTC0(a0, cop0::Index), ORI(a1, t0, 6), MTC0(a1, cop0::EntryHi),
            ORI(a2, zero, (0x300u << 6) | 0x1E), MTC0(a2, cop0::EntryLo0), TLBWI,
            MTC0(t5, cop0::EntryHi),  // loop:
            LW(t1, 0, t0),
            ADDU(t2, t2, t1),
            MTC0(t6, cop0::EntryHi),
            LW(t1, 0, t0),
            ADDU(t3, t3, t1),
            ADDIU(t4, t4, -1),
            BNE(t4, zero, -8), NOP,  // -> loop
            BEQ(zero, zero, -1), NOP,
        });
        t.cpu.run_until(20000);
        return capture(t, 0, 0);
    };
    const CpuState reference = run(Engine::Interpreter);
    CHECK(run(Engine::Blocks) == reference);
    CHECK(run(Engine::Jit) == reference);
    CHECK_EQ(reference.gpr[t2], 200u * 11);
    CHECK_EQ(reference.gpr[t3], 200u * 22);
}

// Status writes that leave a pending interrupt masked (IRIX's spl calls)
// let the code run on; the write that unmasks it takes the interrupt
// straight after it, whichever engine runs the code.
TEST(cpu_status_write_that_unmasks_a_pending_interrupt_takes_it_at_once) {
    auto run = [](Engine engine) {
        Rig t;
        use_engine(t, engine);
        t.cpu.set_idle_skip(false);
        t.load({
            ADDIU(t1, zero, 300),
            ORI(a1, zero, sr::IE),               // interrupts on, Int0 masked
            ORI(a2, zero, sr::IE | (0x04u << 8)), // and unmasked
            MTC0(a1, cop0::Status),              // loop
            ADDIU(t0, t0, 1),
            BNE(t0, t1, -3), NOP,
            MTC0(a2, cop0::Status),
            ADDIU(t2, zero, 7),  // not reached before the interrupt
            BEQ(zero, zero, -1), NOP,
        });
        t.load({MFC0(t3, cop0::EPC), BEQ(zero, zero, -1), NOP}, 0x180);
        t.cpu.cp0_write(cop0::Status, sr::IE);
        t.cpu.set_int_line(0, true);
        t.cpu.run_until(20000);
        return capture(t, 0, 0);
    };
    const CpuState reference = run(Engine::Interpreter);
    CHECK(run(Engine::Blocks) == reference);
    CHECK(run(Engine::Jit) == reference);
    CHECK_EQ(reference.gpr[t0], 300u);
    CHECK_EQ(reference.gpr[t2], 0u);
    CHECK_EQ(reference.gpr[t3], kCode + 8 * 4);
}

// FPU code under every engine: COP1 arithmetic, compares and conversions,
// MTC1, and the FPU loads and stores (pre-decoded handlers) must behave as
// in the interpreter, registers and memory alike.
TEST(cpu_fpu_code_matches_interpreter_in_every_engine) {
    auto fop = [](unsigned fmt, unsigned func, unsigned fd, unsigned fs, unsigned ft = 0) {
        return (0x11u << 26) | (fmt << 21) | (ft << 16) | (fs << 11) | (fd << 6) | func;
    };
    constexpr unsigned S = 16, D = 17, W = 20;
    const u32 kMtc1 = (0x11u << 26) | (4u << 21);
    const u32 kBc1t = (0x11u << 26) | (8u << 21) | (1u << 16);
    auto run = [&](Engine engine) {
        Rig t;
        use_engine(t, engine);
        t.cpu.set_idle_skip(false);
        t.cpu.cp0_write(cop0::Status, sr::CU1);
        t.cpu.gpr[s0] = 0xFFFFFFFF80010000ull;
        t.cpu.gpr[t4] = 200;
        t.load({
            kMtc1 | (t0 << 16) | (0u << 11),   // loop: f0 = i (as an integer)
            fop(W, 0x21, 2, 0),                // cvt.d.w f2, f0
            I(0x35, s0, 4, 0),                 // ldc1 f4, 0(s0)
            fop(D, 0x00, 4, 4, 2),             // add.d f4, f4, f2
            fop(D, 0x02, 6, 2, 2),             // mul.d f6, f2, f2
            fop(D, 0x00, 4, 4, 6),             // add.d f4, f4, f6
            I(0x3D, s0, 4, 0),                 // sdc1 f4, 0(s0)
            I(0x31, s0, 8, 8),                 // lwc1 f8, 8(s0)
            fop(D, 0x20, 10, 2),               // cvt.s.d f10, f2
            fop(S, 0x00, 8, 8, 10),            // add.s f8, f8, f10
            I(0x39, s0, 8, 8),                 // swc1 f8, 8(s0)
            fop(D, 0x3C, 0, 2, 6),             // c.lt.d f2, f6
            kBc1t | 2,                         // bc1t +2
            NOP,
            ADDIU(t5, t5, 1),                  // counts i where !(i < i*i)
            ADDIU(t0, t0, 1),
            BNE(t0, t4, -17), NOP,             // -> loop
            BEQ(zero, zero, -1), NOP,
        });
        t.cpu.run_until(200000);
        return capture(t, 0x10000, 16);
    };
    const CpuState reference = run(Engine::Interpreter);
    CHECK(run(Engine::Blocks) == reference);
    CHECK(run(Engine::Jit) == reference);
    CHECK_EQ(reference.gpr[t0], 200u);
    CHECK_EQ(reference.gpr[t5], 2u);
    double acc;
    std::memcpy(&acc, &reference.data[0], 8);  // stored big-endian by the guest
    u64 bits = 0;
    for (int i = 0; i < 8; ++i) bits = (bits << 8) | reference.data[i];
    std::memcpy(&acc, &bits, 8);
    CHECK(acc == 199.0 * 200 / 2 + 199.0 * 200 * 399 / 6);
}

// Two hot loops 64 KiB apart share a block slot and keep evicting each
// other: each takes its translation back instead of being translated again,
// unless its instructions have changed in the meantime.
TEST(cpu_jit_takes_back_translations_of_evicted_blocks) {
    auto JAL = [](u32 target) { return (0x03u << 26) | ((target >> 2) & 0x03FFFFFFu); };
    Rig t;
    use_engine(t, Engine::Jit);
    t.cpu.set_idle_skip(false);
    // main loop at 0x1000 calls loop A at 0x2000 and loop B at 0x12000 in turn
    t.load({
        ADDIU(s0, zero, 40),
        JAL(0x80002000), NOP,  // loop: A
        JAL(0x80012000), NOP,  // B
        ADDIU(s0, s0, -1),
        BNE(s0, zero, -6), NOP,
        BEQ(zero, zero, -1), NOP,
    });
    t.load({ADDIU(t0, zero, 20), ADDIU(t1, t1, 1), ADDIU(t0, t0, -1), BNE(t0, zero, -3), NOP, JR(ra), NOP}, 0x2000);
    t.load({ADDIU(t0, zero, 20), ADDIU(t2, t2, 1), ADDIU(t0, t0, -1), BNE(t0, zero, -3), NOP, JR(ra), NOP}, 0x12000);
    t.cpu.run_until(60000);
    CHECK_EQ(t.cpu.gpr[s0], 0u);
    CHECK_EQ(t.cpu.gpr[t1], 800u);
    CHECK_EQ(t.cpu.gpr[t2], 800u);
    CHECK(t.cpu.jit_reuses() > 20);
    CHECK(t.cpu.jit_compiles() < 20);
}
