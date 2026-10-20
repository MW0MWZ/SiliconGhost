// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// R10000 tests: identification and Config, the 64-entry TLB with FrameMask
// and duplicate-entry invalidation, 44-bit virtual and 40-bit physical
// addresses, the CACHE operations and the performance counter registers.
#include <initializer_list>
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

constexpr u64 kCode = 0xFFFFFFFF80001000ull;
constexpr u64 kGeneralVector = 0xFFFFFFFF80000180ull;

constexpr u32 CACHE(unsigned op, s32 off, unsigned base) { return I(0x2F, base, op, off); }
constexpr u32 TLBR = 0x42000001;
constexpr u32 CFC1(unsigned rt, unsigned fs) { return (0x11u << 26) | (2u << 21) | (rt << 16) | (fs << 11); }

R4000::Options r10000(u32 scache = 1u << 20) {
    R4000::Options o;
    o.core = Core::R10000;
    o.prid = 0x0926;
    o.config = R4000::kR10000Config;
    o.scache_bytes = scache;
    return o;
}

struct Rig {
    std::vector<u8> ram = std::vector<u8>(8u << 20);
    Bus bus;
    R4000 cpu;
    explicit Rig(const R4000::Options& o = r10000(), u32 status = sr::CU1 | sr::KX) : cpu(bus, o) {
        bus.map_ram(0, ram.size(), ram.data());
        cpu.cp0_write(cop0::Status, status);
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
    unsigned exc_code() const { return (cpu.cp0_read(cop0::Cause) >> 2) & 31; }
    void tlb_write(unsigned index, u64 hi, u64 lo0, u64 lo1) {
        cpu.cp0_write(cop0::Index, index);
        cpu.cp0_write(cop0::PageMask, 0);
        cpu.cp0_write(cop0::EntryHi, hi);
        cpu.cp0_write(cop0::EntryLo0, lo0);
        cpu.cp0_write(cop0::EntryLo1, lo1);
        load({TLBWI}, 0x3000);
        cpu.set_pc(0xFFFFFFFF80003000ull);
        steps(1);
        cpu.set_pc(kCode);
    }
};

// EntryLo for physical page `pa`: cacheable noncoherent, dirty, valid.
constexpr u64 entrylo(u64 pa) { return ((pa >> 12) << 6) | (3u << 3) | 0x6; }

}  // namespace

TEST(r10000_reports_its_identity_and_configuration) {
    Rig t(r10000(2u << 20));
    CHECK_EQ(t.cpu.cp0_read(cop0::PRId), 0x0926ull);
    const u64 config = t.cpu.cp0_read(cop0::Config);
    CHECK_EQ((config >> 16) & 7, 2ull);   // SS: 2 MiB
    CHECK_EQ((config >> 29) & 7, 3ull);   // IC: 32 KiB
    CHECK_EQ((config >> 26) & 7, 3ull);   // DC: 32 KiB
    t.cpu.cp0_write(cop0::Config, 0);     // only K0 is writable
    CHECK_EQ(t.cpu.cp0_read(cop0::Config), config & ~u64{7});
    CHECK_EQ(t.cpu.tlb_entries(), 64u);
    CHECK_EQ(t.cpu.cp0_read(cop0::Random), 63ull);
    t.load({CFC1(t0, 0)});
    t.steps(1);
    CHECK_EQ(t.cpu.gpr[t0], 0x0926ull);  // FPU implementation 0x09
}

TEST(r10000_tlb_maps_44_bit_addresses_and_40_bit_frames) {
    Rig t;
    // xuseg reaches 16 TiB: map 0x00000F0000000000 to physical 0x2000, entry 63.
    const u64 va = 0x00000F0000000000ull;
    t.tlb_write(63, va, entrylo(0x2000), 0);
    t.cpu.gpr[t0] = va;
    store_be<u32>(&t.ram[0x2010], 0xCAFEF00Du);
    t.load({LW(t1, 0x10, t0)});
    t.steps(1);
    CHECK_EQ(t.cpu.gpr[t1], 0xFFFFFFFFCAFEF00Dull);
    // The R4400 has 40 virtual address bits: the same address is an error.
    Rig r(R4000::Options{}, sr::KX);
    r.cpu.gpr[t0] = va;
    r.load({LW(t1, 0, t0)});
    r.steps(1);
    CHECK_EQ(r.exc_code(), static_cast<unsigned>(Exc::AdEL));
    // PFN bits above 36 address bits: a frame at 0x80_0000_0000 (physical bit 39).
    t.tlb_write(1, 0x40000000, entrylo(0x8000000000ull), 0);
    t.cpu.cp0_write(cop0::Index, 1);
    t.load({TLBR}, 0x3000);
    t.cpu.set_pc(0xFFFFFFFF80003000ull);
    t.steps(1);
    CHECK_EQ(t.cpu.cp0_read(cop0::EntryLo0) >> 6, 0x8000000ull);
}

TEST(r10000_framemask_clears_frame_bits_on_tlb_writes) {
    Rig t;
    t.cpu.cp0_write(cop0::FrameMask, 0x10001);  // only bits 15:0 exist
    CHECK_EQ(t.cpu.cp0_read(cop0::FrameMask), 1ull);
    t.tlb_write(5, 0x00400000, entrylo(0x1000000 | 0x2000), 0);  // physical bit 24: EntryLo bit 18
    t.cpu.cp0_write(cop0::Index, 5);
    t.load({TLBR}, 0x3000);
    t.cpu.set_pc(0xFFFFFFFF80003000ull);
    t.steps(1);
    // FrameMask bit 0 masks EntryLo bit 18 (PFN bit 12, physical address bit 24).
    CHECK_EQ((t.cpu.cp0_read(cop0::EntryLo0) >> 6) << 12, u64{0x2000});
}

TEST(r10000_duplicate_tlb_entries_are_invalidated_and_set_ts) {
    Rig t;
    store_be<u32>(&t.ram[0x5000], 0x11111111u);
    store_be<u32>(&t.ram[0x6000], 0x22222222u);
    t.tlb_write(2, 0x00400000, entrylo(0x5000), 0);
    CHECK(!(t.cpu.cp0_read(cop0::Status) & sr::TS));
    t.tlb_write(9, 0x00400000, entrylo(0x6000), 0);  // the same page: entry 2 goes
    CHECK(t.cpu.cp0_read(cop0::Status) & sr::TS);
    t.cpu.gpr[t0] = 0x00400000;
    t.load({LW(t1, 0, t0)});
    t.steps(1);
    CHECK_EQ(t.cpu.gpr[t1], 0x22222222ull);
    t.cpu.cp0_write(cop0::EntryHi, 0x00400000);
    t.load({TLBP}, 0x3000);
    t.cpu.set_pc(0xFFFFFFFF80003000ull);
    t.steps(1);
    CHECK_EQ(t.cpu.cp0_read(cop0::Index), 9ull);
    t.tlb_write(3, 0x00800000, entrylo(0x5000), 0);  // a write with no duplicate clears TS
    CHECK(!(t.cpu.cp0_read(cop0::Status) & sr::TS));
}

TEST(r10000_xkphys_takes_40_bit_addresses_and_the_uncached_attribute) {
    Rig t;
    store_be<u32>(&t.ram[0x1800], 0x5A5A5A5Au);
    t.cpu.gpr[t0] = 0x9600000000001800ull;  // uncached (2), attribute 3 in bits 58:57
    t.cpu.gpr[t2] = 0xA800000000001800ull;  // cacheable coherent exclusive on write (5)
    t.load({LW(t1, 0, t0), LW(t3, 0, t2)});
    t.steps(2);
    CHECK_EQ(t.cpu.gpr[t1], 0x5A5A5A5Aull);
    CHECK_EQ(t.cpu.gpr[t3], 0x5A5A5A5Aull);
    t.cpu.gpr[t0] = 0xA600000000001800ull;  // bits 58:57 with a cached algorithm: an error
    t.load({LW(t1, 0, t0)});
    t.cpu.set_pc(kCode);
    t.steps(1);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::AdEL));
}

TEST(r10000_cache_tags_and_data_read_back_and_hit_operations_compare_tags) {
    Rig t;
    // Primary data: Index Store Tag then Index Load Tag (way 1 from address bit 0).
    t.cpu.cp0_write(cop0::TagLo, 0x123456C3u);  // tag, state 3, LRU, SCWay, parity
    t.cpu.cp0_write(cop0::TagHi, 0xE0000005u);  // StateMod, tag bits 39:36
    t.cpu.gpr[t0] = 0xFFFFFFFF80000041ull;      // index of line 2, way 1
    t.load({CACHE(0x09, 0, t0), MTC0(zero, cop0::TagLo), MTC0(zero, cop0::TagHi), CACHE(0x05, 0, t0),
            MFC0(a0, cop0::TagLo), MFC0(a1, cop0::TagHi)});
    t.steps(6);
    CHECK_EQ(static_cast<u32>(t.cpu.gpr[a0]), 0x123456C3u & 0xFFFFFFCFu);
    CHECK_EQ(static_cast<u32>(t.cpu.gpr[a1]), 0xE0000005u);
    // Secondary: a tag for physical 0x40000 (way 0), then Hit Invalidate (S)
    // there sets CH, and again (now invalid) clears it.
    const u32 stag = ((0x40000u >> 18) << 14) | (3u << 10);  // Tag[35:18], state 3
    t.cpu.cp0_write(cop0::TagLo, stag);
    t.cpu.cp0_write(cop0::TagHi, 0);
    t.cpu.gpr[t1] = 0xFFFFFFFF80040000ull;
    t.load({CACHE(0x0B, 0, t1), CACHE(0x13, 0, t1)});
    t.cpu.set_pc(kCode);
    t.steps(2);
    CHECK(t.cpu.cp0_read(cop0::Status) & sr::CH);
    t.load({CACHE(0x13, 0, t1)});
    t.cpu.set_pc(kCode);
    t.steps(1);
    CHECK(!(t.cpu.cp0_read(cop0::Status) & sr::CH));
    // Secondary data: Index Store Data writes a doubleword and zeroes its pair.
    t.cpu.cp0_write(cop0::TagHi, 0x01234567u);
    t.cpu.cp0_write(cop0::TagLo, 0x89ABCDEFu);
    t.cpu.cp0_write(cop0::ECC, 0x3A5);
    t.cpu.gpr[t2] = 0xFFFFFFFF80000108ull;
    t.cpu.gpr[t3] = 0xFFFFFFFF80000100ull;
    t.load({CACHE(0x1F, 0, t2), MTC0(zero, cop0::ECC), CACHE(0x1B, 0, t2), MFC0(a0, cop0::TagHi),
            MFC0(a1, cop0::TagLo), MFC0(a2, cop0::ECC), CACHE(0x1B, 0, t3), MFC0(a3, cop0::TagLo)});
    t.cpu.set_pc(kCode);
    t.steps(8);
    CHECK_EQ(static_cast<u32>(t.cpu.gpr[a0]), 0x01234567u);
    CHECK_EQ(static_cast<u32>(t.cpu.gpr[a1]), 0x89ABCDEFu);
    CHECK_EQ(t.cpu.gpr[a2], 0x3A5ull);
    CHECK_EQ(t.cpu.gpr[a3], 0ull);
}

TEST(r10000_performance_counters_are_selected_by_the_instruction) {
    Rig t;
    t.cpu.gpr[t0] = 0x1FF;
    t.cpu.gpr[t1] = 0x12345678;
    t.load({MTC0(t0, cop0::PerfCounter) | 0, MTC0(t1, cop0::PerfCounter) | 1, MTC0(t1, cop0::PerfCounter) | 3,
            MFC0(a0, cop0::PerfCounter) | 0, MFC0(a1, cop0::PerfCounter) | 1, MFC0(a2, cop0::PerfCounter) | 2,
            MFC0(a3, cop0::PerfCounter) | 3});
    t.steps(7);
    CHECK_EQ(t.cpu.gpr[a0], 0x1FFull);
    CHECK_EQ(t.cpu.gpr[a1], 0x12345678ull);
    CHECK_EQ(t.cpu.gpr[a2], 0ull);
    CHECK_EQ(t.cpu.gpr[a3], 0x12345678ull);
}

TEST(r10000_has_no_wait) {
    Rig t;
    t.load({0x42000020});  // WAIT: reserved on the R10000
    t.steps(1);
    CHECK_EQ(t.cpu.pc, kGeneralVector);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::RI));
}

TEST(r10000_cached_accesses_to_an_uncached_only_range_fail) {
    R4000::Options o = r10000();
    o.uncached_only_base = 0x100000;
    o.uncached_only_end = 0x200000;
    Rig t(o);
    store_be<u32>(&t.ram[0x100040], 0x600DF00Du);
    t.cpu.gpr[t0] = 0x9000000000100040ull;  // xkphys uncached: fine
    t.cpu.gpr[t2] = 0xA800000000100040ull;  // xkphys cacheable: a bus error
    t.load({LW(t1, 0, t0), LW(t3, 0, t2)});
    t.steps(2);
    CHECK_EQ(t.cpu.gpr[t1], 0x600DF00Dull);
    CHECK_EQ(t.cpu.pc, kGeneralVector);
    CHECK_EQ(t.exc_code(), static_cast<unsigned>(Exc::DBE));
    CHECK_EQ(t.cpu.cp0_read(cop0::EPC), kCode + 4);
}
