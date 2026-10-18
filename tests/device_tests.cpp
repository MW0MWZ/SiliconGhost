// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Tests for the generic peripheral chips (PIT, RTC, EEPROM, SCC).
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "core/scheduler.h"
#include "devices/ds1386.h"
#include "devices/eeprom93cxx.h"
#include "devices/kbc8042.h"
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

// A count written while a mode 2 counter runs takes over when the current
// period ends.
TEST(pit_mode2_new_count_at_end_of_period) {
    Clock clk;
    Pit8254 pit(clk.sched, 100.0e6, {1.0e6, 1.0e6, 1.0e6});
    std::vector<u64> falls;
    pit.set_output_callback([&](unsigned c, bool level) {
        if (c == 0 && !level) falls.push_back(clk.now);
    });
    pit.write(3, 0x34);  // counter 0, LSB+MSB, mode 2
    pit.write(0, 10);
    pit.write(0, 0);
    clk.advance_to(100 * 4);
    pit.write(0, 20);  // during the first period
    pit.write(0, 0);
    clk.advance_to(100 * 60);
    CHECK(falls.size() >= 3);
    if (falls.size() >= 3) {
        CHECK_EQ(falls[0], 100ull * 10);           // the first period keeps its 10 ticks
        CHECK_EQ(falls[1] - falls[0], 100ull * 20);  // then 20
        CHECK_EQ(falls[2] - falls[1], 100ull * 20);
    }
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

// Only writes that change the battery-backed RAM count as changes to save.
TEST(ds1386_counts_nvram_changes) {
    Ds1386 rtc;
    rtc.set_time_source([] { return 1.0e9; });
    rtc.power_on();
    const u64 start = rtc.nvram_changes();
    rtc.write(0x01, 0x30);  // clock register
    rtc.write(Ds1386::kRegCommand, 0x80);
    CHECK_EQ(rtc.nvram_changes(), start);
    rtc.write(0x100, 0x5A);
    CHECK_EQ(rtc.nvram_changes(), start + 1);
    rtc.write(0x100, 0x5A);  // same value
    CHECK_EQ(rtc.nvram_changes(), start + 1);
    rtc.write(0x100, 0x00);
    CHECK_EQ(rtc.nvram_changes(), start + 2);
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

// With PRE high the instructions address the protect register; PRCLEAR is
// not an erase of word 127.
TEST(eeprom_protect_register) {
    Eeprom93Cxx ee;
    auto cmd = [&](u32 bits, bool pre) {
        ee.set_lines(true, false, false, pre);
        for (int i = 10; i >= 0; --i) {
            const bool b = (bits >> i) & 1;
            ee.set_lines(true, false, b, pre);
            ee.set_lines(true, true, b, pre);
        }
    };
    auto end = [&] { ee.set_lines(false, false, false); };
    auto write = [&](unsigned addr, u16 v) {
        ee_cmd(ee, (0b101u << 8) | addr, 11);
        ee_cmd(ee, v, 16);
        end();
    };
    ee_cmd(ee, (0b100u << 8) | 0xC0, 11);  // WEN
    end();
    write(0x7F, 0x1234);
    write(0x20, 0x5555);
    cmd((0b100u << 8) | 0xC0, true);  // PREN
    end();
    cmd((0b111u << 8) | 0xFF, true);  // PRCLEAR
    end();
    CHECK_EQ(ee_read(ee, 0x7F), 0x1234u);
    cmd((0b100u << 8) | 0xC0, true);  // PREN, then PRWRITE 0x20: protect 0x20 and up
    end();
    cmd((0b101u << 8) | 0x20, true);
    end();
    write(0x20, 0xAAAA);
    write(0x1F, 0xAAAA);
    CHECK_EQ(ee_read(ee, 0x20), 0x5555u);
    CHECK_EQ(ee_read(ee, 0x1F), 0xAAAAu);
    cmd((0b110u << 8), true);  // PRREAD
    CHECK(!ee.data_out());
    u32 v = 0;
    for (int i = 0; i < 8; ++i) {
        ee.set_lines(true, false, false, true);
        ee.set_lines(true, true, false, true);
        v = (v << 1) | (ee.data_out() ? 1 : 0);
    }
    end();
    CHECK_EQ(v, 0x20u);
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
    wd.write_data(0x80 | 0x08 | 7);  // 20 MHz input clock, advanced features, own ID 7
    wd.write_address(0x18);
    wd.write_data(0x00);      // RESET
    CHECK(wd.read_aux_status() & 0x10);
    clk.advance_to(100000);
    CHECK(irq);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x01);  // reset with advanced features
    CHECK(!irq);
    wd.write_address(0x02);
    wd.write_data(0x20);     // selection timeout period (RESET cleared it): 32 x 80 / 20 = 128 ms
    wd.write_address(0x15);
    wd.write_data(3);        // destination ID
    wd.write_address(0x18);
    wd.write_data(0x07);     // SELECT without ATN
    CHECK(wd.read_aux_status() & 0x20);
    clk.advance_to(100000 + 150000 * 120);
    CHECK(!irq);
    clk.advance_to(100000 + 150000 * 130);
    CHECK(irq);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x42);  // selection timeout
    wd.write_address(0x18);
    wd.write_data(0x04);  // DISCONNECT while not connected: no interrupt
    clk.advance_to(100000 + 150000 * 140);
    CHECK(!irq);
}

// Quick selection timeouts: an absent target's selection times out within a
// millisecond, whatever the period programmed.
TEST(wd33c93_quick_selection_timeout) {
    Clock clk;
    Wd33c93 wd(clk.sched, 150000.0);
    wd.set_quick_selection_timeout(true);
    bool irq = false;
    wd.set_irq_callback([&](bool a) { irq = a; });
    wd.write_address(0x00);
    wd.write_data(0x80 | 0x08 | 7);  // 20 MHz input clock
    wd.write_address(0x02);
    wd.write_data(0x20);  // 128 ms
    wd.write_address(0x15);
    wd.write_data(3);
    wd.write_address(0x18);
    wd.write_data(0x07);  // SELECT without ATN
    clk.advance_to(150000 / 2);
    CHECK(!irq);
    clk.advance_to(150000 * 2);
    CHECK(irq);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x42);  // selection timeout
}

namespace {
std::string make_image(unsigned blocks) {
    static unsigned serial = 0;  // a name per image, so a test can have several
    const std::string path =
        (std::filesystem::temp_directory_path() / ("sg_scsi_test" + std::to_string(serial++) + ".img")).string();
    std::ofstream f(path, std::ios::binary);
    for (unsigned b = 0; b < blocks; ++b) {
        std::vector<char> block(512, static_cast<char>(b));
        f.write(block.data(), 512);
    }
    return path;
}
// A disk image for one test, deleted when it goes out of scope: declared
// before the devices that open it, it outlives them (an open file cannot
// be deleted on every host).
struct TempImage {
    explicit TempImage(unsigned blocks) : path(make_image(blocks)) {}
    ~TempImage() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
    TempImage(const TempImage&) = delete;
    TempImage& operator=(const TempImage&) = delete;
    std::string path;
};
void wd_write(Wd33c93& wd, u8 reg, u8 v) {
    wd.write_address(reg);
    wd.write_data(v);
}
}  // namespace

TEST(scsi_cdrom_inquiry_capacity_read) {
    const TempImage image(16);
    const std::string& path = image.path;
    ScsiDisk cd(ScsiDisk::Kind::CdRom, path);
    CHECK(cd.ok());
    const u8 inquiry[6] = {0x12, 0, 0, 0, 36, 0};
    auto r = cd.command(inquiry, 6);
    CHECK_EQ(r.data_in.size(), 36u);
    CHECK_EQ(r.data_in[0], 0x05);  // CD-ROM
    const u8 capacity[10] = {0x25};
    r = cd.command(capacity, 10);
    CHECK_EQ(r.data_in[3], 3);     // last LBA: 2048-byte blocks at power-on
    CHECK_EQ(r.data_in[6], 0x08);
    const u8 read10[10] = {0x28, 0, 0, 0, 0, 1, 0, 0, 2, 0};
    r = cd.command(read10, 10);
    CHECK_EQ(r.data_in.size(), 4096u);
    CHECK_EQ(r.data_in[0], 4);      // 512-byte block 4 starts 2048-byte block 1
    CHECK_EQ(r.data_in[2648], 9);   // 2048 + 600 bytes in: 512-byte block 9
    const u8 write10[10] = {0x2A, 0, 0, 0, 0, 0, 0, 0, 1, 0};
    r = cd.command(write10, 10);
    CHECK_EQ(r.status, 0x02);  // write-protected medium
}

// MODE SENSE has only the pages the drive does; READ TOC gives block
// numbers or minutes, seconds and frames.
TEST(scsi_mode_sense_pages_and_toc) {
    const TempImage image(16);
    ScsiDisk cd(ScsiDisk::Kind::CdRom, image.path);
    const u8 geometry[6] = {0x1A, 0x08, 0x04, 0, 255, 0};
    auto r = cd.command(geometry, 6);
    CHECK_EQ(r.status, 0x02);  // no geometry page on a CD-ROM
    const u8 sense[6] = {0x03, 0, 0, 0, 18, 0};
    r = cd.command(sense, 6);
    CHECK_EQ(r.data_in[2], 0x05);
    CHECK_EQ(r.data_in[12], 0x24);
    const u8 params[6] = {0x1A, 0x08, 0x0D, 0, 255, 0};
    r = cd.command(params, 6);
    CHECK_EQ(r.status, 0x00);
    CHECK_EQ(r.data_in.size(), 12u);  // header, page 0D
    CHECK_EQ(r.data_in[4], 0x0D);
    const u8 toc_msf[10] = {0x43, 0x02, 0, 0, 0, 0, 0, 0, 20, 0};
    r = cd.command(toc_msf, 10);
    CHECK_EQ(r.data_in.size(), 20u);
    CHECK_EQ(r.data_in[1], 18u);
    CHECK_EQ(r.data_in[10], 2u);  // track 1 at 00:02:00
    CHECK_EQ(r.data_in[14], 0xAAu);
    CHECK_EQ(r.data_in[18], 2u);  // lead-out after four 2048-byte blocks: 00:02:04
    CHECK_EQ(r.data_in[19], 4u);
    const u8 toc_leadout[10] = {0x43, 0, 0, 0, 0, 0, 0xAA, 0, 20, 0};
    r = cd.command(toc_leadout, 10);
    CHECK_EQ(r.data_in.size(), 12u);
    CHECK_EQ(r.data_in[11], 4u);
}

// MODE SELECT's block descriptor switches the CD-ROM between 2048-byte
// blocks (its power-on size) and 512 (as the PROM and IRIX select); a bus
// reset restores 2048 and reports UNIT ATTENTION. The disk stays at 512.
TEST(scsi_mode_select_sets_the_cdrom_block_size) {
    const TempImage image(16);
    ScsiDisk cd(ScsiDisk::Kind::CdRom, image.path);
    const u8 select6[6] = {0x15, 0x10, 0, 0, 12, 0};
    std::vector<u8> params = {0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0x02, 0x00};  // header, descriptor: 512
    auto r = cd.command(select6, 6);
    CHECK_EQ(r.data_out_length, 12u);
    CHECK_EQ(cd.finish_data_out(select6, params), 0x00);
    const u8 capacity[10] = {0x25};
    r = cd.command(capacity, 10);
    CHECK_EQ(r.data_in[3], 15);    // last LBA of sixteen 512-byte blocks
    CHECK_EQ(r.data_in[6], 0x02);
    const u8 read10[10] = {0x28, 0, 0, 0, 0, 3, 0, 0, 1, 0};
    r = cd.command(read10, 10);
    CHECK_EQ(r.data_in.size(), 512u);
    CHECK_EQ(r.data_in[0], 3);
    const u8 sense6[6] = {0x1A, 0, 0, 0, 12, 0};
    r = cd.command(sense6, 6);
    CHECK_EQ(r.data_in[10], 0x02);  // the descriptor reports it
    cd.bus_reset();
    r = cd.command(capacity, 10);
    CHECK_EQ(r.status, 0x02);  // UNIT ATTENTION first
    const u8 sense[6] = {0x03, 0, 0, 0, 18, 0};
    r = cd.command(sense, 6);
    CHECK_EQ(r.data_in[2], 0x06);
    CHECK_EQ(r.data_in[12], 0x29);  // power on or reset
    r = cd.command(capacity, 10);
    CHECK_EQ(r.data_in[6], 0x08);

    const TempImage disk_image(16);
    ScsiDisk disk(ScsiDisk::Kind::Disk, disk_image.path);
    params[10] = 0x08;
    disk.command(select6, 6);
    CHECK_EQ(disk.finish_data_out(select6, params), 0x02);  // 2048 not supported
    CHECK_EQ(disk.block_size(), 512u);
}

TEST(scsi_cdrom_eject_changes_disc_with_unit_attention) {
    const TempImage first(16), second(32);
    ScsiDisk cd(ScsiDisk::Kind::CdRom, first.path);
    int ejects = 0;
    cd.set_eject_callback([&] {
        ++ejects;
        CHECK(cd.change_medium(second.path));
    });
    const u8 eject[6] = {0x1B, 0, 0, 0, 0x02, 0};  // LoEj, stop
    auto r = cd.command(eject, 6);
    CHECK_EQ(r.status, 0x00);
    CHECK_EQ(ejects, 1);
    const u8 tur[6] = {0x00};
    r = cd.command(tur, 6);
    CHECK_EQ(r.status, 0x02);  // check condition: medium may have changed
    const u8 sense[6] = {0x03, 0, 0, 0, 18, 0};
    r = cd.command(sense, 6);
    CHECK_EQ(r.data_in[2], 0x06);
    CHECK_EQ(r.data_in[12], 0x28);
    r = cd.command(tur, 6);
    CHECK_EQ(r.status, 0x00);
    const u8 capacity[10] = {0x25};
    r = cd.command(capacity, 10);
    CHECK_EQ(r.data_in[3], 7);  // the new disc: 32 x 512 bytes in 2048-byte blocks
    // Ejected with nothing put back: not ready, medium not present.
    cd.set_eject_callback({});
    cd.command(eject, 6);
    r = cd.command(tur, 6);
    CHECK_EQ(r.status, 0x02);
    r = cd.command(sense, 6);
    CHECK_EQ(r.data_in[2], 0x02);
    CHECK_EQ(r.data_in[12], 0x3A);
}

TEST(wd33c93_select_and_transfer_dma) {
    const TempImage image(8);
    const std::string& path = image.path;
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
}

// Fast transfers: a 32 KB read completes in well under the 6.5 ms that
// 5 MB/s would take, while the command still takes its usual time.
TEST(wd33c93_fast_transfers_take_next_to_no_time) {
    for (const bool fast : {false, true}) {
        const TempImage image(128);
        ScsiDisk disk(ScsiDisk::Kind::Disk, image.path);
        Clock clk;
        Wd33c93 wd(clk.sched, 150000.0);
        wd.set_fast_transfers(fast);
        bool irq = false;
        u32 moved = 0;
        wd.set_irq_callback([&](bool a) { irq = a; });
        wd.set_dma([&](const u8*, u32 n) { moved += n; return n; }, [&](u8*, u32) { return 0u; });
        wd.attach(1, &disk);
        wd_write(wd, 0x01, 0x80);  // control: DMA mode
        wd_write(wd, 0x15, 1);
        const u8 cdb[10] = {0x28, 0, 0, 0, 0, 0, 0, 0, 64, 0};  // READ(10): 64 blocks
        for (unsigned i = 0; i < 10; ++i) wd_write(wd, static_cast<u8>(0x03 + i), cdb[i]);
        wd_write(wd, 0x12, 0);
        wd_write(wd, 0x13, 0x80);
        wd_write(wd, 0x14, 0x00);  // transfer count 32768
        wd_write(wd, 0x18, 0x09);  // SELECT-AND-TRANSFER
        clk.advance_to(15000);     // 0.1 ms: not even the command is done
        CHECK(!irq);
        clk.advance_to(75000);  // 0.5 ms
        CHECK_EQ(irq, fast);
        clk.advance_to(1500000);  // 10 ms
        CHECK(irq);
        CHECK_EQ(moved, 32768u);
    }
}

TEST(wd33c93_transfer_count_exhausted_pauses_and_resumes) {
    const TempImage image(8);
    const std::string& path = image.path;
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
}

TEST(wd33c93_select_with_atn_reports_selection_then_message_out) {
    const TempImage image(1);
    const std::string& path = image.path;
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
}

TEST(wd33c93_sync_negotiation_then_command) {
    const TempImage image(4);
    const std::string& path = image.path;
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
}

TEST(wd33c93_message_in_whole_message_holds_ack_on_last_byte) {
    const TempImage image(1);
    const std::string& path = image.path;
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
}

TEST(wd33c93_manual_phases_command_status_message) {
    const TempImage image(2);
    const std::string& path = image.path;
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
}

TEST(ds1386_year_counted_from_a_base) {
    Ds1386 rtc;
    rtc.set_year_base(1940);  // IRIX on the IP22
    double host = 1700000000;
    rtc.set_time_source([&] { return host; });
    rtc.write(Ds1386::kRegCommand, 0x00);
    rtc.write(0xA, 0x83);  // 1940 + 83 = 2023
    rtc.write(0x9, 0x11);
    rtc.write(0x8, 0x14);
    rtc.write(0x4, 0x12);
    rtc.write(0x2, 0x00);
    rtc.write(0x1, 0x00);
    rtc.write(Ds1386::kRegCommand, 0x80);
    host += 86400;
    CHECK_EQ(rtc.read(0x8), 0x15);
    CHECK_EQ(rtc.read(0x6), 4);     // 2023-11-15 was a Wednesday (1 = Sunday)
    CHECK_EQ(rtc.read(0xA), 0x83);  // still counted from 1940
}

TEST(ds1386_power_on_discards_saved_clock_registers) {
    Ds1386 rtc;
    rtc.set_year_base(1940);
    double host = 1700000000;  // 2023-11-14 22:13:20 UTC
    rtc.set_time_source([&] { return host; });
    rtc.memory()[0xA] = 0x56;  // a saved image from another time (1996)
    rtc.memory()[Ds1386::kRegCommand] = 0x00;
    rtc.power_on();
    rtc.write(Ds1386::kRegCommand, 0x00);  // freeze and resume as the PROM does
    rtc.write(Ds1386::kRegCommand, 0x80);
    CHECK_EQ(rtc.read(0xA), 0x83);  // 2023
    CHECK_EQ(rtc.read(0x4), 0x22);
}

namespace {
// Sends a byte to the mouse through the controller and returns its replies.
std::vector<u8> to_mouse(Kbc8042& k, u8 value) {
    k.write_command(0xD4);
    k.write_data(value);
    std::vector<u8> out;
    while (k.read_status() & 0x01) {
        CHECK(k.read_status() & 0x20);  // from the auxiliary device
        out.push_back(k.read_data());
    }
    return out;
}
}  // namespace

// With the command byte's translate bit set (as OpenBSD sets it), set 2
// codes from the keyboard reach the host as set 1, breaks as bit 7; with it
// clear (IRIX), they pass unchanged.
TEST(kbc_translates_set2_to_set1_when_asked) {
    Kbc8042 kbc;
    kbc.write_command(0x60);
    kbc.write_data(0x44);   // translate, no interrupts
    kbc.key_input(0x2D);    // R
    kbc.key_input(0xF0);
    kbc.key_input(0x2D);
    kbc.key_input(0xE0);    // right Ctrl
    kbc.key_input(0x14);
    CHECK_EQ(kbc.read_data(), 0x13);
    CHECK_EQ(kbc.read_data(), 0x93);
    CHECK_EQ(kbc.read_data(), 0xE0);
    CHECK_EQ(kbc.read_data(), 0x1D);
    kbc.write_data(0xF2);  // identify: the ID's second byte is translated too
    CHECK_EQ(kbc.read_data(), 0xFA);
    CHECK_EQ(kbc.read_data(), 0xAB);
    CHECK_EQ(kbc.read_data(), 0x41);
    kbc.write_command(0x60);
    kbc.write_data(0x00);
    kbc.key_input(0x2D);
    kbc.key_input(0xF0);
    CHECK_EQ(kbc.read_data(), 0x2D);
    CHECK_EQ(kbc.read_data(), 0xF0);
}

TEST(kbc_mouse_reports_movement_only_when_enabled) {
    Kbc8042 k;
    bool irq = false;
    k.set_irq_callback([&](bool a) { irq = a; });
    k.write_command(0x60);
    k.write_data(0x47);  // keyboard and mouse interrupts on
    CHECK(to_mouse(k, 0xFF) == (std::vector<u8>{0xFA, 0xAA, 0x00}));
    k.mouse_motion(5, 5, 0);
    CHECK(!(k.read_status() & 0x01));  // reporting is off after reset
    CHECK(to_mouse(k, 0xF4) == std::vector<u8>{0xFA});
    k.mouse_motion(3, -2, 1);  // left button, right and down
    CHECK(irq);
    CHECK_EQ(k.read_data(), 0x08u | 0x01u | 0x20u);
    CHECK_EQ(k.read_data(), 3u);
    CHECK_EQ(k.read_data(), 0xFEu);
    CHECK(!irq);
    CHECK(to_mouse(k, 0xE9) == (std::vector<u8>{0xFA, 0x24, 0x02, 0x64}));  // enabled, left held
    CHECK(to_mouse(k, 0xF5) == std::vector<u8>{0xFA});
    k.mouse_motion(1, 1, 0);
    CHECK(!(k.read_status() & 0x01));
}

TEST(kbc_mouse_splits_large_movements_into_packets) {
    Kbc8042 k;
    to_mouse(k, 0xF4);
    k.mouse_motion(-300, 600, 4);
    std::vector<u8> b;
    while (k.read_status() & 0x01) b.push_back(k.read_data());
    CHECK_EQ(b.size(), 9u);
    int x = 0, y = 0;
    for (std::size_t i = 0; i + 2 < b.size(); i += 3) {
        CHECK_EQ(b[i] & 0x0Fu, 0x08u | 0x04u);
        x += (b[i] & 0x10) ? static_cast<int>(b[i + 1]) - 256 : b[i + 1];
        y += (b[i] & 0x20) ? static_cast<int>(b[i + 2]) - 256 : b[i + 2];
    }
    CHECK_EQ(x, -300);
    CHECK_EQ(y, 600);
}

TEST(wd33c93_reset_line_release_interrupts_with_reset_status) {
    Clock clk;
    Wd33c93 wd(clk.sched, 150000.0);
    bool irq = false;
    wd.set_irq_callback([&](bool a) { irq = a; });
    wd.reset();  // held in reset
    CHECK(!irq);
    wd.reset_released();
    CHECK(irq);
    CHECK_EQ(wd.read_aux_status(), 0x80);  // INT only
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x00);  // reset
    CHECK(!irq);
}

// A single-byte TRANSFER INFO (SBT) moves exactly one byte, offered
// through DBR, and leaves the transfer count alone.
TEST(wd33c93_single_byte_transfer_waits_for_its_byte) {
    const TempImage image(1);
    const std::string& path = image.path;
    ScsiDisk disk(ScsiDisk::Kind::Disk, path);
    Clock clk;
    Wd33c93 wd(clk.sched, 150000.0);
    bool irq = false;
    wd.set_irq_callback([&](bool a) { irq = a; });
    wd.attach(1, &disk);
    wd_write(wd, 0x15, 1);
    wd_write(wd, 0x18, 0x06);  // SELECT WITH ATN
    clk.advance_to(150000);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x11);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x8E);  // MESSAGE OUT requested
    wd_write(wd, 0x12, 0);
    wd_write(wd, 0x13, 0);
    wd_write(wd, 0x14, 0);
    wd_write(wd, 0x18, 0xA0);  // TRANSFER INFO, single byte
    CHECK(wd.read_aux_status() & 0x01);  // DBR: the chip wants the byte
    clk.advance_to(300000);
    CHECK(!irq);  // still waiting for it
    wd_write(wd, 0x19, 0xC0);  // IDENTIFY
    clk.advance_to(450000);
    CHECK(irq);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x1A);  // transfer done, target now in COMMAND phase
    wd.write_address(0x14);
    CHECK_EQ(wd.read_data(), 0u);  // count untouched
}

// With DMA, a data transfer waits while the host's DMA channel is not
// running and goes on when it starts.
TEST(wd33c93_dma_transfer_waits_for_the_host_channel) {
    const TempImage image(8);
    const std::string& path = image.path;
    ScsiDisk disk(ScsiDisk::Kind::Disk, path);
    Clock clk;
    Wd33c93 wd(clk.sched, 150000.0);
    bool irq = false, running = false;
    std::vector<u8> memory;
    wd.set_irq_callback([&](bool a) { irq = a; });
    wd.set_dma(
        [&](const u8* d, u32 n) {
            if (!running) return 0u;
            memory.insert(memory.end(), d, d + n);
            return n;
        },
        [&](u8*, u32) { return 0u; });
    wd.attach(1, &disk);
    wd_write(wd, 0x01, 0x80);  // DMA mode
    wd_write(wd, 0x15, 1);
    const u8 cdb[10] = {0x28, 0, 0, 0, 0, 3, 0, 0, 1, 0};
    for (unsigned i = 0; i < 10; ++i) wd_write(wd, static_cast<u8>(0x03 + i), cdb[i]);
    wd_write(wd, 0x12, 0);
    wd_write(wd, 0x13, 0x02);
    wd_write(wd, 0x14, 0x00);
    wd_write(wd, 0x18, 0x09);  // SELECT-AND-TRANSFER
    clk.advance_to(1500000);
    CHECK(!irq);
    CHECK(memory.empty());
    CHECK(wd.read_aux_status() & 0x20);  // busy
    running = true;
    wd.dma_kick();
    clk.advance_to(3000000);
    CHECK(irq);
    CHECK_EQ(memory.size(), 512u);
    if (!memory.empty()) CHECK_EQ(memory[0], 3);
    wd.write_address(0x17);
    CHECK_EQ(wd.read_data(), 0x16);
}

// Set 3 per-key modes: FB/FC/FD take a list of keys ended by the next
// command, F7-FA set every key; all keys start typematic with breaks.
TEST(kbc_set3_key_modes) {
    Kbc8042 kbc;
    CHECK(kbc.repeats(0x1C) && kbc.sends_break(0x1C));
    kbc.write_data(0xFC);  // make/break only for...
    CHECK_EQ(kbc.read_data(), 0xFA);
    kbc.write_data(0x14);  // Caps Lock
    CHECK_EQ(kbc.read_data(), 0xFA);
    kbc.write_data(0x76);  // Num Lock
    CHECK_EQ(kbc.read_data(), 0xFA);
    kbc.write_data(0xED);  // the next command ends the list
    CHECK_EQ(kbc.read_data(), 0xFA);
    kbc.write_data(0x00);
    CHECK_EQ(kbc.read_data(), 0xFA);
    CHECK(!kbc.repeats(0x14) && kbc.sends_break(0x14));
    CHECK(!kbc.repeats(0x76));
    CHECK(kbc.repeats(0x1C));
    kbc.write_data(0xF8);  // every key make/break (as X sets it)
    CHECK_EQ(kbc.read_data(), 0xFA);
    CHECK(!kbc.repeats(0x1C) && kbc.sends_break(0x1C));
    kbc.write_data(0xF6);  // defaults
    CHECK_EQ(kbc.read_data(), 0xFA);
    CHECK(kbc.repeats(0x14));
    kbc.write_command(0xD1);  // write output port: its byte is taken, not sent to the keyboard
    kbc.write_data(0xDF);
    CHECK(!(kbc.read_status() & 1));
}
