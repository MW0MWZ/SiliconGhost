// SPDX-License-Identifier: GPL-2.0-or-later
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
