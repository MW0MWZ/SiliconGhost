// SPDX-License-Identifier: GPL-2.0-or-later
// Tests for the generic peripheral chips (PIT, RTC, EEPROM, SCC).
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "core/scheduler.h"
#include "devices/ds1386.h"
#include "devices/eeprom93cxx.h"
#include "devices/pit8254.h"
#include "devices/scsi_disk.h"
#include "devices/wd33c93.h"
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
    double host = 1700000000;  // 2023-11-14 22:13:20 UTC (a Tuesday)
    rtc.set_time_source([&] { return host; });
    rtc.write(Ds1386::kRegCommand, 0x00);  // re-base the clock onto this source
    rtc.write(0xA, 0x23);
    rtc.write(0x9, 0x11);
    rtc.write(0x8, 0x14);
    rtc.write(0x4, 0x22);
    rtc.write(0x2, 0x13);
    rtc.write(0x1, 0x20);
    rtc.write(0x0, 0x00);
    rtc.write(Ds1386::kRegCommand, 0x80);
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

    // Partial writes with TE set: an invalid intermediate date is kept verbatim.
    rtc.write(0x9, 0x00);
    CHECK_EQ(rtc.read(0x9) & 0x1F, 0x00);
    rtc.write(0x9, 0x02);
    CHECK_EQ(rtc.read(0x9) & 0x1F, 0x02);
    host += 0.25;
    CHECK_EQ(rtc.read(0x0), 0x25);  // hundredths tick

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

TEST(wd33c93_reset_and_selection_timeout) {
    Clock clk;
    Wd33c93 wd(clk.sched, 150000.0);  // 150 MHz
    bool irq = false;
    wd.set_irq_callback([&](bool a) { irq = a; });
    wd.write_address(0x00);
    wd.write_data(0x08 | 7);  // own ID 7, advanced features
    wd.write_address(0x02);
    wd.write_data(0x20);      // selection timeout period
    wd.write_address(0x18);
    wd.write_data(0x00);      // RESET
    CHECK(wd.read_aux_status() & 0x10);
    clk.advance_to(100000);
    CHECK(irq);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x01);  // reset with advanced features
    CHECK(!irq);
    wd.write_address(0x15);
    wd.write_data(3);        // destination ID
    wd.write_address(0x18);
    wd.write_data(0x07);     // SELECT without ATN
    CHECK(wd.read_aux_status() & 0x20);
    clk.advance_to(100000 + 150000 * 20);  // ~13 ms timeout at 20 MHz
    CHECK(irq);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x42);  // selection timeout
    wd.write_address(0x18);
    wd.write_data(0x04);  // DISCONNECT while not connected: no interrupt
    clk.advance_to(100000 + 150000 * 40);
    CHECK(!irq);
}

namespace {
std::string make_image(unsigned blocks) {
    const std::string path = (std::filesystem::temp_directory_path() / "sg_scsi_test.img").string();
    std::ofstream f(path, std::ios::binary);
    for (unsigned b = 0; b < blocks; ++b) {
        std::vector<char> block(512, static_cast<char>(b));
        f.write(block.data(), 512);
    }
    return path;
}
void wd_write(Wd33c93& wd, u8 reg, u8 v) {
    wd.write_address(reg);
    wd.write_data(v);
}
}  // namespace

TEST(scsi_cdrom_inquiry_capacity_read) {
    const std::string path = make_image(16);
    ScsiDisk cd(ScsiDisk::Kind::CdRom, path);
    CHECK(cd.ok());
    const u8 inquiry[6] = {0x12, 0, 0, 0, 36, 0};
    auto r = cd.command(inquiry, 6);
    CHECK_EQ(r.data_in.size(), 36u);
    CHECK_EQ(r.data_in[0], 0x05);  // CD-ROM
    const u8 capacity[10] = {0x25};
    r = cd.command(capacity, 10);
    CHECK_EQ(r.data_in[3], 15);    // last LBA
    CHECK_EQ(r.data_in[6], 0x02);  // 512-byte blocks
    const u8 read10[10] = {0x28, 0, 0, 0, 0, 3, 0, 0, 2, 0};
    r = cd.command(read10, 10);
    CHECK_EQ(r.data_in.size(), 1024u);
    CHECK_EQ(r.data_in[0], 3);
    CHECK_EQ(r.data_in[600], 4);
    const u8 write10[10] = {0x2A, 0, 0, 0, 0, 0, 0, 0, 1, 0};
    r = cd.command(write10, 10);
    CHECK_EQ(r.status, 0x02);  // write-protected medium
    std::filesystem::remove(path);
}

TEST(wd33c93_select_and_transfer_dma) {
    const std::string path = make_image(8);
    ScsiDisk disk(ScsiDisk::Kind::Disk, path);
    Clock clk;
    Wd33c93 wd(clk.sched, 150000.0);
    bool irq = false;
    std::vector<u8> memory;
    wd.set_irq_callback([&](bool a) { irq = a; });
    wd.set_dma([&](const u8* d, u32 n) { memory.insert(memory.end(), d, d + n); return n; },
               [&](u8*, u32) { return 0u; });
    wd.attach(1, &disk);
    wd_write(wd, 0x01, 0x80);  // control: DMA mode
    wd_write(wd, 0x15, 1);     // destination ID
    const u8 cdb[10] = {0x28, 0, 0, 0, 0, 5, 0, 0, 1, 0};  // READ(10) block 5
    for (unsigned i = 0; i < 10; ++i) wd_write(wd, static_cast<u8>(0x03 + i), cdb[i]);
    wd_write(wd, 0x12, 0);
    wd_write(wd, 0x13, 0x02);
    wd_write(wd, 0x14, 0x00);  // transfer count 512
    wd_write(wd, 0x18, 0x09);  // SELECT-AND-TRANSFER
    clk.advance_to(150000);
    CHECK(irq);
    CHECK_EQ(memory.size(), 512u);
    if (!memory.empty()) CHECK_EQ(memory[0], 5);
    wd.write_address(0x0F);
    CHECK_EQ(wd.read_data(), 0x00);  // target status GOOD
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x16);  // select-and-transfer complete
    std::filesystem::remove(path);
}

TEST(wd33c93_transfer_count_exhausted_pauses_and_resumes) {
    const std::string path = make_image(8);
    ScsiDisk disk(ScsiDisk::Kind::Disk, path);
    Clock clk;
    Wd33c93 wd(clk.sched, 150000.0);
    bool irq = false;
    std::vector<u8> memory;
    wd.set_irq_callback([&](bool a) { irq = a; });
    wd.set_dma([&](const u8* d, u32 n) { memory.insert(memory.end(), d, d + n); return n; },
               [&](u8*, u32) { return 0u; });
    wd.attach(1, &disk);
    wd_write(wd, 0x01, 0x80);
    wd_write(wd, 0x15, 1);
    const u8 cdb[10] = {0x28, 0, 0, 0, 0, 2, 0, 0, 4, 0};  // READ(10) blocks 2-5
    for (unsigned i = 0; i < 10; ++i) wd_write(wd, static_cast<u8>(0x03 + i), cdb[i]);
    auto start = [&](u32 count) {
        wd_write(wd, 0x12, static_cast<u8>(count >> 16));
        wd_write(wd, 0x13, static_cast<u8>(count >> 8));
        wd_write(wd, 0x14, static_cast<u8>(count));
        wd_write(wd, 0x18, 0x09);
    };
    start(1024);  // host DMA only covers half the data
    clk.advance_to(150000);
    CHECK(irq);
    CHECK_EQ(memory.size(), 1024u);
    wd.write_address(0x10);
    CHECK_EQ(wd.read_data(), 0x46);  // paused: transfer count exhausted
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x49);  // unexpected phase: target still in DATA IN
    CHECK(!irq);
    start(1024);  // reload and continue
    clk.advance_to(300000);
    CHECK(irq);
    CHECK_EQ(memory.size(), 2048u);
    if (memory.size() == 2048) {
        CHECK_EQ(memory[0], 2);
        CHECK_EQ(memory[1024], 4);
        CHECK_EQ(memory[2047], 5);
    }
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x16);
    std::filesystem::remove(path);
}

TEST(wd33c93_select_with_atn_reports_selection_then_message_out) {
    const std::string path = make_image(1);
    ScsiDisk disk(ScsiDisk::Kind::Disk, path);
    Clock clk;
    Wd33c93 wd(clk.sched, 150000.0);
    bool irq = false;
    wd.set_irq_callback([&](bool a) { irq = a; });
    wd.attach(1, &disk);
    wd_write(wd, 0x15, 1);
    wd_write(wd, 0x18, 0x06);  // SELECT WITH ATN
    clk.advance_to(150000);
    CHECK(irq);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x11);  // selected
    CHECK(irq);                      // target requests MESSAGE OUT
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x8E);
    CHECK(!irq);
    wd_write(wd, 0x18, 0x04);  // DISCONNECT: no interrupt
    clk.advance_to(300000);
    CHECK(!irq);
    std::filesystem::remove(path);
}

TEST(wd33c93_sync_negotiation_then_command) {
    const std::string path = make_image(4);
    ScsiDisk disk(ScsiDisk::Kind::Disk, path);
    Clock clk;
    Wd33c93 wd(clk.sched, 150000.0);
    bool irq = false;
    std::vector<u8> memory;
    wd.set_irq_callback([&](bool a) { irq = a; });
    wd.set_dma([&](const u8* d, u32 n) { memory.insert(memory.end(), d, d + n); return n; },
               [&](u8*, u32) { return 0u; });
    wd.attach(1, &disk);
    u64 now = 0;
    auto status = [&]() {
        now += 150000;
        clk.advance_to(now);
        wd.write_address(0x17);
        return wd.read_data();
    };
    auto count = [&](u32 n) {
        wd_write(wd, 0x12, 0);
        wd_write(wd, 0x13, static_cast<u8>(n >> 8));
        wd_write(wd, 0x14, static_cast<u8>(n));
    };
    wd_write(wd, 0x01, 0x80);  // DMA mode: message phases still use the data register
    wd_write(wd, 0x15, 1);
    wd_write(wd, 0x18, 0x06);  // SELECT WITH ATN
    CHECK_EQ(status(), 0x11);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x8E);  // MESSAGE OUT requested
    count(6);
    wd_write(wd, 0x18, 0x20);  // TRANSFER INFO, polled
    for (u8 b : {u8{0xC0}, u8{0x01}, u8{0x03}, u8{0x01}, u8{0x32}, u8{0x0C}}) {
        CHECK(wd.read_aux_status() & 0x01);  // DBR
        wd_write(wd, 0x19, b);               // IDENTIFY + SDTR
    }
    CHECK_EQ(status(), 0x1F);  // done; target requests MESSAGE IN
    std::vector<u8> reply;
    for (;;) {
        count(1);
        wd_write(wd, 0x18, 0x20);
        CHECK(wd.read_aux_status() & 0x01);
        wd.write_address(0x19);
        reply.push_back(wd.read_data());
        CHECK_EQ(status(), 0x20);  // paused with ACK asserted
        wd_write(wd, 0x18, 0x03);  // NEGATE ACK
        const u8 next = status();
        if (next != 0x8F) {
            CHECK_EQ(next, 0x8A);  // COMMAND phase next
            break;
        }
        if (reply.size() > 8) break;
    }
    CHECK_EQ(reply.size(), 5u);
    if (reply.size() == 5) CHECK_EQ(reply[4], 0x00);  // asynchronous: offset 0
    const u8 cdb[10] = {0x28, 0, 0, 0, 0, 3, 0, 0, 1, 0};
    for (unsigned i = 0; i < 10; ++i) wd_write(wd, static_cast<u8>(0x03 + i), cdb[i]);
    count(512);
    wd_write(wd, 0x18, 0x08);  // SELECT-WITH-ATN-AND-TRANSFER continues the connection
    CHECK_EQ(status(), 0x16);
    CHECK_EQ(memory.size(), 512u);
    if (!memory.empty()) CHECK_EQ(memory[0], 3);
    std::filesystem::remove(path);
}

TEST(wd33c93_message_in_whole_message_holds_ack_on_last_byte) {
    const std::string path = make_image(1);
    ScsiDisk disk(ScsiDisk::Kind::Disk, path);
    Clock clk;
    Wd33c93 wd(clk.sched, 150000.0);
    wd.attach(1, &disk);
    u64 now = 0;
    auto status = [&]() {
        now += 150000;
        clk.advance_to(now);
        wd.write_address(0x17);
        return wd.read_data();
    };
    auto count = [&](u32 n) {
        wd_write(wd, 0x12, 0);
        wd_write(wd, 0x13, 0);
        wd_write(wd, 0x14, static_cast<u8>(n));
    };
    wd_write(wd, 0x15, 1);
    wd_write(wd, 0x18, 0x06);
    CHECK_EQ(status(), 0x11);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x8E);
    count(6);
    wd_write(wd, 0x18, 0x20);
    for (u8 b : {u8{0x80}, u8{0x01}, u8{0x03}, u8{0x01}, u8{0x32}, u8{0x0A}}) wd_write(wd, 0x19, b);
    CHECK_EQ(status(), 0x1F);
    count(5);
    wd_write(wd, 0x18, 0x20);  // whole SDTR reply in one Transfer Info
    std::vector<u8> reply;
    for (int i = 0; i < 5 && (wd.read_aux_status() & 0x01); ++i) {
        wd.write_address(0x19);
        reply.push_back(wd.read_data());
    }
    CHECK_EQ(reply.size(), 5u);
    if (reply.size() == 5) {
        CHECK_EQ(reply[0], 0x01);
        CHECK_EQ(reply[3], 0x32);  // period echoed
    }
    CHECK_EQ(status(), 0x20);  // ACK held on the last byte
    wd_write(wd, 0x18, 0x03);
    CHECK_EQ(status(), 0x8A);  // then COMMAND
    std::filesystem::remove(path);
}

TEST(wd33c93_manual_phases_command_status_message) {
    const std::string path = make_image(2);
    ScsiDisk disk(ScsiDisk::Kind::Disk, path);
    Clock clk;
    Wd33c93 wd(clk.sched, 150000.0);
    wd.attach(1, &disk);
    u64 now = 0;
    auto status = [&]() {
        now += 150000;
        clk.advance_to(now);
        wd.write_address(0x17);
        return wd.read_data();
    };
    auto count = [&](u32 n) {
        wd_write(wd, 0x12, 0);
        wd_write(wd, 0x13, static_cast<u8>(n >> 8));
        wd_write(wd, 0x14, static_cast<u8>(n));
    };
    auto read_byte = [&]() {
        wd.write_address(0x19);
        return wd.read_data();
    };
    wd_write(wd, 0x15, 1);
    wd_write(wd, 0x18, 0x06);
    CHECK_EQ(status(), 0x11);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x8E);
    count(1);
    wd_write(wd, 0x18, 0x20);
    wd_write(wd, 0x19, 0x80);  // IDENTIFY only
    CHECK_EQ(status(), 0x1A);  // COMMAND next
    count(6);
    wd_write(wd, 0x18, 0x20);
    for (u8 b : {u8{0x08}, u8{0}, u8{0}, u8{1}, u8{1}, u8{0}}) wd_write(wd, 0x19, b);  // READ(6) block 1
    CHECK_EQ(status(), 0x19);  // DATA IN next
    count(512);
    wd_write(wd, 0x18, 0x20);  // polled data
    std::vector<u8> data;
    while (wd.read_aux_status() & 0x01) data.push_back(read_byte());
    CHECK_EQ(data.size(), 512u);
    if (!data.empty()) CHECK_EQ(data[0], 1);
    CHECK_EQ(status(), 0x1B);  // STATUS next
    count(1);
    wd_write(wd, 0x18, 0x20);
    CHECK_EQ(read_byte(), 0x00);  // GOOD
    CHECK_EQ(status(), 0x1F);     // MESSAGE IN next
    count(1);
    wd_write(wd, 0x18, 0x20);
    CHECK_EQ(read_byte(), 0x00);  // COMMAND COMPLETE
    CHECK_EQ(status(), 0x20);
    wd_write(wd, 0x18, 0x03);
    CHECK_EQ(status(), 0x85);  // target released the bus
    std::filesystem::remove(path);
}
