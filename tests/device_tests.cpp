// SPDX-License-Identifier: GPL-2.0-or-later
// Tests for the generic peripheral chips (PIT, RTC, EEPROM, SCC).
#include <string>
#include <vector>

#include "core/scheduler.h"
#include "devices/ds1386.h"
#include "devices/eeprom93cxx.h"
#include "devices/pit8254.h"
#include "devices/z85230.h"
#include "test.h"

using namespace sg;

namespace {
// A scheduler driven by a manual clock, advanced event-by-event like the machine loop.
struct Clock {
    u64 now = 0;
    Scheduler sched;
    Clock() { sched.set_clock([this] { return now; }); }
    void advance_to(u64 t) {
        while (sched.next_event() <= t) {
            now = sched.next_event();
            sched.run_due(now);
        }
        now = t;
    }
};
}  // namespace

TEST(pit_mode2_rate_generator_period) {
    Clock clk;
    // 100 CPU cycles per PIT tick.
    Pit8254 pit(clk.sched, 100.0e6, {1.0e6, 1.0e6, 1.0e6});
    std::vector<u64> falls;
    pit.set_output_callback([&](unsigned c, bool level) {
        if (c == 0 && !level) falls.push_back(clk.now);
    });
    pit.write(3, 0x34);  // counter 0, LSB+MSB, mode 2
    pit.write(0, 10);
    pit.write(0, 0);
    clk.advance_to(100 * 45);
    CHECK_EQ(falls.size(), 4u);
    if (falls.size() >= 2) CHECK_EQ(falls[1] - falls[0], 1000ull);  // 10 ticks
}

TEST(pit_mode0_terminal_count_and_latch) {
    Clock clk;
    Pit8254 pit(clk.sched, 1.0e6, {1.0e6, 1.0e6, 1.0e6});  // 1 cycle per tick
    pit.write(3, 0xB0);  // counter 2, LSB+MSB, mode 0
    CHECK(!pit.output(2));
    pit.write(2, 0x00);
    pit.write(2, 0x01);  // 256
    clk.advance_to(101);
    pit.write(3, 0x80);  // latch counter 2
    const unsigned lo = pit.read(2), hi = pit.read(2);
    CHECK_EQ((hi << 8) | lo, 256u - 100u);
    clk.advance_to(300);
    CHECK(pit.output(2));
}

TEST(pit_readback_status) {
    Clock clk;
    Pit8254 pit(clk.sched, 1.0e6, {1.0e6, 1.0e6, 1.0e6});
    pit.write(3, 0x76);  // counter 1, LSB+MSB, mode 3
    pit.write(3, 0xE4);  // read-back status of counter 1
    const u8 status = pit.read(1);
    CHECK_EQ(status & 0x3F, 0x36);  // rw=3, mode=3, binary
    CHECK(status & 0x40);           // null count until loaded
}

TEST(ds1386_time_freeze_and_set) {
    Ds1386 rtc;
    std::time_t host = 1700000000;  // 2023-11-14 22:13:20 UTC (a Tuesday)
    rtc.set_time_source([&] { return host; });
    CHECK_EQ(rtc.read(0x1), 0x20);  // seconds BCD
    CHECK_EQ(rtc.read(0x2), 0x13);
    CHECK_EQ(rtc.read(0x4), 0x22);
    CHECK_EQ(rtc.read(0x8), 0x14);
    CHECK_EQ(rtc.read(0x9) & 0x1F, 0x11);
    CHECK_EQ(rtc.read(0xA), 0x23);
    CHECK_EQ(rtc.read(0x6), 3);  // Tuesday (1 = Sunday)

    // Set 1999-12-31 23:59:50 with TE cleared, then resume.
    rtc.write(Ds1386::kRegCommand, 0x00);
    rtc.write(0xA, 0x99);
    rtc.write(0x9, 0x12);
    rtc.write(0x8, 0x31);
    rtc.write(0x4, 0x23);
    rtc.write(0x2, 0x59);
    rtc.write(0x1, 0x50);
    rtc.write(Ds1386::kRegCommand, 0x80);
    host += 15;  // rolls over into 2000
    CHECK_EQ(rtc.read(0xA), 0x00);
    CHECK_EQ(rtc.read(0x9) & 0x1F, 0x01);
    CHECK_EQ(rtc.read(0x1), 0x05);

    rtc.write(0x100, 0x5A);  // NVRAM
    CHECK_EQ(rtc.read(0x100), 0x5A);
}

namespace {
void ee_cmd(Eeprom93Cxx& ee, u32 bits, unsigned n) {
    ee.set_lines(true, false, false);
    for (int i = static_cast<int>(n) - 1; i >= 0; --i) {
        const bool b = (bits >> i) & 1;
        ee.set_lines(true, false, b);
        ee.set_lines(true, true, b);
    }
}
u16 ee_read(Eeprom93Cxx& ee, unsigned addr) {
    ee_cmd(ee, (0b110u << 8) | addr, 11);
    CHECK(!ee.data_out());  // dummy zero
    u16 v = 0;
    for (int i = 0; i < 16; ++i) {
        ee.set_lines(true, false, false);
        ee.set_lines(true, true, false);
        v = static_cast<u16>((v << 1) | (ee.data_out() ? 1 : 0));
    }
    ee.set_lines(false, false, false);
    return v;
}
}  // namespace

TEST(eeprom_write_enable_write_read) {
    Eeprom93Cxx ee;
    ee_cmd(ee, (0b101u << 8) | 0x10, 11);  // WRITE while disabled
    ee_cmd(ee, 0xBEEF, 16);
    ee.set_lines(false, false, false);
    CHECK_EQ(ee_read(ee, 0x10), 0xFFFF);
    ee_cmd(ee, (0b100u << 8) | 0xC0, 11);  // EWEN
    ee.set_lines(false, false, false);
    ee_cmd(ee, (0b101u << 8) | 0x10, 11);
    ee_cmd(ee, 0xBEEF, 16);
    ee.set_lines(false, false, false);
    CHECK_EQ(ee_read(ee, 0x10), 0xBEEF);
}

TEST(scc_transmit_and_interrupts) {
    Clock clk;
    Z85230 scc(clk.sched, 50);
    std::string out;
    bool irq = false;
    scc.set_tx_sink([&](unsigned ch, u8 b) {
        if (ch == 0) out.push_back(static_cast<char>(b));
    });
    scc.set_irq_callback([&](bool a) { irq = a; });
    scc.write_control(0, 9);
    scc.write_control(0, 0x08);  // WR9: MIE
    scc.write_control(0, 5);
    scc.write_control(0, 0x68);  // WR5: Tx enable, 8 bits
    scc.write_control(0, 1);
    scc.write_control(0, 0x12);  // WR1: Tx int enable, Rx int on all chars
    scc.write_control(0, 3);
    scc.write_control(0, 0xC1);  // WR3: Rx enable, 8 bits
    scc.write_data(0, 'h');
    CHECK(!(scc.read_control(0) & 0x04));  // Tx buffer full
    clk.advance_to(60);
    CHECK(scc.read_control(0) & 0x04);
    CHECK(irq);
    scc.write_control(0, 3);
    CHECK_EQ(scc.read_control(0) & 0x10, 0x10);  // RR3: channel A Tx IP
    scc.write_control(0, 0x28);                   // reset Tx int pending
    CHECK(!irq);
    scc.receive(0, 'x');
    CHECK(irq);
    CHECK(scc.read_control(0) & 0x01);
    CHECK_EQ(scc.read_data(0), 'x');
    CHECK(!irq);
    CHECK(out == "h");
}
