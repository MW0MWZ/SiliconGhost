// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include <cmath>
#include <string>
#include <vector>

#include "cpu/r4000/disasm.h"
#include "cpu/r4000/r4000.h"
#include "mem/bus.h"
#include "mips_asm.h"
#include "test.h"

using namespace sg;
using namespace sg::test::as;

namespace {
class RecordingDevice final : public Device {
public:
    const char* name() const override { return "rec"; }
    bool read(u32 offset, unsigned size, u64& value) override {
        value = 0x1000 + offset + size;
        return true;
    }
    bool write(u32 offset, unsigned size, u64 value) override {
        last_offset = offset;
        last_size = size;
        last_value = value;
        return true;
    }
    u32 last_offset = 0;
    unsigned last_size = 0;
    u64 last_value = 0;
};
}  // namespace

TEST(bus_ram_device_rom_dispatch) {
    Bus bus;
    std::vector<u8> ram(0x10000), rom(0x10000, 0xAB);
    RecordingDevice dev;
    bus.map_ram(0x00000000, ram.size(), ram.data());
    bus.map_ram(0x1FC00000, rom.size(), rom.data(), false);
    bus.map_device(0x1FA00000, 0x10000, &dev);

    CHECK(bus.write<u32>(0x10, 0x11223344u));
    CHECK_EQ(ram[0x10], 0x11);  // big-endian in host memory
    u16 h = 0;
    CHECK(bus.read<u16>(0x12, h));
    CHECK_EQ(h, 0x3344);

    CHECK(bus.write<u32>(0x1FC00000, 0));  // ROM write is ignored
    CHECK_EQ(rom[0], 0xAB);

    u32 v = 0;
    CHECK(bus.read<u32>(0x1FA00010, v));
    CHECK_EQ(v, 0x1000u + 0x10 + 4);
    CHECK(bus.write<u64>(0x1FA00008, 0x55));
    CHECK_EQ(dev.last_offset, 8u);
    CHECK_EQ(dev.last_size, 8u);

    CHECK(!bus.read<u32>(0x30000000, v));          // unmapped read: bus error
    CHECK(!bus.read<u32>(0x100000000ull, v));      // beyond 32-bit physical space
}

TEST(disasm_common_forms) {
    using sg::mips::disassemble;
    CHECK(disassemble(NOP, 0) == "nop");
    CHECK(disassemble(ADDIU(sp, sp, -32), 0) == "addiu sp, sp, -32");
    CHECK(disassemble(LW(t0, 16, sp), 0) == "lw t0, 16(sp)");
    CHECK(disassemble(MTC0(t0, mips::cop0::Status), 0) == "mtc0 t0, Status");
    CHECK(disassemble(LUI(at, 0x1fc0), 0) == "lui at, 0x1fc0");
    CHECK(disassemble(BNE(a0, a1, -1), 0xFFFFFFFFBFC00010ull) == "bne a0, a1, 0xbfc00010");
    CHECK(disassemble(ERET, 0) == "eret");
}

#include "core/pacer.h"
#include "core/scheduler.h"

TEST(scheduler_orders_and_reschedules) {
    u64 now = 0;
    Scheduler s;
    s.set_clock([&] { return now; });
    std::vector<int> fired;
    Timer a(s, [&] { fired.push_back(1); });
    Timer b(s, [&] { fired.push_back(2); });
    Timer periodic(s, [&] {
        fired.push_back(3);
        periodic.schedule_in(100);
    });
    a.schedule_at(50);
    b.schedule_at(20);
    periodic.schedule_at(30);
    CHECK_EQ(s.next_event(), 20ull);
    now = 60;
    s.run_due(now);
    CHECK(fired == (std::vector<int>{2, 3, 1}));
    CHECK_EQ(s.next_event(), 160ull);  // periodic rescheduled relative to the clock
    periodic.cancel();
    CHECK_EQ(s.next_event(), Scheduler::kNever);
}

namespace {
struct FakeHost {
    u64 now = 1000000000;
    u64 slept = 0;
    Pacer::HostClock clock() {
        return {[this] { return now; }, [this](u64 ns) { now += ns; slept += ns; }};
    }
};
}  // namespace

TEST(pacer_cycle_mode_waits_when_ahead) {
    FakeHost host;
    Pacer p(Pacer::Mode::Cycle, 100e6, host.clock());
    p.update(0, 0);
    host.now += 2000000;               // 2 ms of host time ran 5 ms of guest time (idle skipping)
    CHECK_EQ(p.update(500000, 1000), 1.0);
    CHECK_EQ(host.slept, 3000000u);    // waits for the wall clock to catch up
    host.now += 10000000;              // a slow stretch: the guest falls behind and is not waited for
    p.update(1000000, 2000);
    CHECK_EQ(host.slept, 3000000u);
}

TEST(pacer_realtime_locks_guest_time_to_the_wall_clock) {
    FakeHost host;
    const double hz = 175e6;
    Pacer p(Pacer::Mode::Realtime, hz, host.clock());
    u64 cycles = 0, instructions = 0;
    double cpi = p.update(0, 0);
    const double host_ips = 350e6;  // the host runs twice as many instructions as a real R4400 would cycles
    double guest_cycles = 0;
    for (int slice = 0; slice < 2000; ++slice) {  // 1 ms slices of host time, fully busy
        const u64 n = static_cast<u64>(host_ips / 1000);
        instructions += n;
        guest_cycles += static_cast<double>(n) * cpi;
        cycles = static_cast<u64>(guest_cycles);
        host.now += 1000000;
        cpi = p.update(cycles, instructions);
    }
    CHECK(cpi > 0.49 && cpi < 0.51);  // each instruction worth half a cycle: a CPU twice as fast
    const double wall = 2.0, guest = static_cast<double>(cycles) / hz;
    CHECK(guest > wall - 0.01 && guest < wall + 0.01);
}

TEST(pacer_realtime_recovers_from_a_stall) {
    FakeHost host;
    const double hz = 175e6, host_ips = 350e6;
    Pacer p(Pacer::Mode::Realtime, hz, host.clock());
    p.update(0, 0);
    // 1 ms of guest time took 200 ms of host time (the host was busy
    // elsewhere): the charge per instruction goes to its maximum.
    u64 cycles = 175000, instructions = 175000;
    host.now += 200000000;
    double cpi = p.update(cycles, instructions);
    CHECK(cpi > 15);
    // Back to full speed, in 1 ms slices of guest time, each now holding
    // fewer instructions than one measurement needs.
    for (int slice = 0; slice < 3000; ++slice) {
        const u64 n = static_cast<u64>(175000 / cpi);
        instructions += n;
        cycles += 175000;
        host.now += static_cast<u64>(static_cast<double>(n) / host_ips * 1e9);
        cpi = p.update(cycles, instructions);
    }
    CHECK(cpi > 0.45 && cpi < 0.55);
}

TEST(pacer_realtime_caps_the_speed_up) {
    FakeHost host;
    const double hz = 175e6, host_ips = 1000e6;  // could run the guest CPU 5.7x as fast
    Pacer p(Pacer::Mode::Realtime, hz, host.clock());
    p.set_max_speed(2.0);
    double cpi = p.update(0, 0);
    u64 cycles = 0, instructions = 0;
    for (int slice = 0; slice < 2000; ++slice) {  // 1 ms slices of guest time
        const u64 n = static_cast<u64>(175000 / cpi);
        instructions += n;
        cycles += 175000;
        host.now += static_cast<u64>(static_cast<double>(n) / host_ips * 1e9);
        cpi = p.update(cycles, instructions);
    }
    CHECK_EQ(cpi, 0.5);
    // Ahead of the wall clock, it waits: guest time still matches.
    const double wall = static_cast<double>(host.now - 1000000000) * 1e-9, guest = static_cast<double>(cycles) / hz;
    CHECK(guest > wall - 0.01 && guest < wall + 0.01);
}

// A stall of seconds (the window's event loop blocked, a slow start) is made
// up for afterwards rather than lost.
TEST(pacer_realtime_catches_up_after_a_long_stall) {
    FakeHost host;
    const double hz = 175e6, host_ips = 700e6;  // room to run the guest CPU 4x as fast
    Pacer p(Pacer::Mode::Realtime, hz, host.clock());
    double cpi = p.update(0, 0);
    u64 cycles = 0, instructions = 0;
    auto run = [&](int slices) {
        for (int slice = 0; slice < slices; ++slice) {  // 1 ms slices of host time, fully busy
            const u64 n = static_cast<u64>(host_ips / 1000);
            instructions += n;
            cycles += static_cast<u64>(static_cast<double>(n) * cpi);
            host.now += 1000000;
            cpi = p.update(cycles, instructions);
        }
    };
    run(500);
    host.now += 3000000000ull;  // 3 s with nothing run
    run(10000);
    const double wall = static_cast<double>(host.now - 1000000000) * 1e-9, guest = static_cast<double>(cycles) / hz;
    CHECK(guest > wall - 0.01 && guest < wall + 0.01);
}

// While the guest times the CPU, it runs at the timed speed even on a host
// too slow to keep up, so its delay loops are calibrated at that speed.
TEST(pacer_realtime_runs_at_the_timed_speed_while_the_guest_times_the_cpu) {
    FakeHost host;
    const double hz = 175e6, host_ips = 100e6;  // slower than the real CPU
    Pacer p(Pacer::Mode::Realtime, hz, host.clock());
    p.set_timed_speed(1.5);
    double cpi = p.update(0, 0);
    u64 cycles = 0, instructions = 0;
    for (int slice = 0; slice < 3000; ++slice) {
        const u64 n = static_cast<u64>(host_ips / 1000);
        instructions += n;
        cycles += static_cast<u64>(static_cast<double>(n) * cpi);
        host.now += 1000000;
        const bool timing = slice >= 1000 && slice < 1100;
        cpi = p.update(cycles, instructions, timing);
        if (slice >= 1000 && slice < 1099) CHECK(std::abs(cpi - 1 / 1.5) < 1e-9);
    }
    CHECK(cpi > 1.7);  // adapting again: the host covers 0.57 of real speed
    CHECK(std::abs(p.delay_cycles_per_instruction() - 1 / 1.5) < 1e-9);
}

// Outside the guest's timing the CPU runs as fast as the host allows, while
// guest time still follows the wall clock.
TEST(pacer_realtime_runs_uncapped_after_the_guest_times_the_cpu) {
    FakeHost host;
    const double hz = 175e6, host_ips = 1000e6;  // could run the guest CPU 5.7x as fast
    Pacer p(Pacer::Mode::Realtime, hz, host.clock());
    double cpi = p.update(0, 0);
    u64 cycles = 0, instructions = 0;
    for (int slice = 0; slice < 3000; ++slice) {  // 1 ms slices of host time, fully busy
        const u64 n = static_cast<u64>(host_ips / 1000);
        instructions += n;
        cycles += static_cast<u64>(static_cast<double>(n) * cpi);
        host.now += 1000000;
        const bool timing = slice >= 1000 && slice < 1100;
        cpi = p.update(cycles, instructions, timing);
        if (slice >= 1000 && slice < 1099) CHECK_EQ(cpi, 0.5);  // the default timed speed, 2x
    }
    CHECK(cpi > 0.17 && cpi < 0.18);
    CHECK_EQ(p.delay_cycles_per_instruction(), 0.5);
    const double wall = static_cast<double>(host.now - 1000000000) * 1e-9, guest = static_cast<double>(cycles) / hz;
    CHECK(guest > wall - 0.01 && guest < wall + 0.01);
}

TEST(pacer_max_mode_never_waits) {
    FakeHost host;
    Pacer p(Pacer::Mode::Max, 100e6, host.clock());
    p.update(0, 0);
    host.now += 1000;
    CHECK_EQ(p.update(100000000, 1000), 1.0);
    CHECK_EQ(host.slept, 0u);
}
