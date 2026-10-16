// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "app/proms.h"
#include "app/session.h"
#include "app/settings.h"
#include "app/terminal.h"
#include "devices/scsi_disk.h"
#include "test.h"

// String equality, printing both sides when they differ.
#define CHECK_STR(a, b)                                                                                 \
    do {                                                                                                \
        const std::string va_ = (a), vb_ = (b);                                                         \
        if (va_ != vb_) {                                                                               \
            std::fprintf(stderr, "  %s:%d: %s == %s failed (\"%s\" vs \"%s\")\n", __FILE__, __LINE__, #a, #b, \
                         va_.c_str(), vb_.c_str());                                                     \
            ++::sg::test::failures();                                                                   \
        }                                                                                               \
    } while (0)

using namespace sg;
using sg::app::Terminal;

namespace {

std::string row(const Terminal& t, int r) {
    std::string s;
    for (int c = 0; c < Terminal::kCols; ++c) s += t.at(r, c).ch;
    s.erase(s.find_last_not_of(' ') + 1);
    return s;
}

std::string temp_path(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }

}  // namespace

TEST(terminal_text_wraps_and_scrolls_into_history) {
    Terminal t(100);
    t.feed("hello\r\nworld");
    CHECK_STR(row(t, 0), std::string("hello"));
    CHECK_STR(row(t, 1), std::string("world"));
    CHECK_EQ(t.cursor_row(), 1);
    CHECK_EQ(t.cursor_col(), 5);
    // 80 columns fill the line; the wrap happens only with the next character.
    t.feed("\r\n" + std::string(80, 'x'));
    CHECK_EQ(t.cursor_row(), 2);
    t.feed("y");
    CHECK_EQ(t.cursor_row(), 3);
    CHECK_STR(row(t, 3), std::string("y"));
    for (int i = 0; i < 30; ++i) t.feed("\r\nline " + std::to_string(i));
    CHECK_STR(row(t, Terminal::kRows - 1), std::string("line 29"));
    CHECK_EQ(t.scrollback().size(), 10u);  // 34 lines on a 24-line screen
    std::string first;
    for (const auto& cell : t.scrollback().front()) first += cell.ch;
    CHECK(first.rfind("hello", 0) == 0);
}

TEST(terminal_cursor_addressing_and_erasing) {
    Terminal t;
    t.feed("abcdefgh\x1B[1;3H");  // row 1, column 3
    CHECK_EQ(t.cursor_col(), 2);
    t.feed("\x1B[K");  // erase to end of line
    CHECK_STR(row(t, 0), std::string("ab"));
    t.feed("\x1B[5;10HX\x1B[2A\x1B[3DY");
    CHECK_EQ(t.at(4, 9).ch, 'X');
    CHECK_EQ(t.at(2, 7).ch, 'Y');
    t.feed("\x1B[2J");
    CHECK_STR(row(t, 4), std::string());
    t.feed("\x1B[H12345\x1B[1;2H\x1B[2P");  // delete two characters
    CHECK_STR(row(t, 0), std::string("145"));
    t.feed("\x1B[@Z");  // insert one, then type into it
    CHECK_STR(row(t, 0), std::string("1Z45"));
}

TEST(terminal_scroll_region_and_attributes) {
    Terminal t(50);
    for (int i = 1; i <= 24; ++i) t.feed("\x1B[" + std::to_string(i) + ";1Hrow" + std::to_string(i));
    t.feed("\x1B[5;10r\x1B[10;1H\n");  // scroll lines 5..10 up by one
    CHECK_STR(row(t, 3), std::string("row4"));
    CHECK_STR(row(t, 4), std::string("row6"));
    CHECK_STR(row(t, 9), std::string());
    CHECK_STR(row(t, 10), std::string("row11"));
    CHECK(t.scrollback().empty());  // a region scroll keeps no history
    t.feed("\x1B[r\x1B[1;1H\x1B[1;7mB\x1B[0mN");
    CHECK_EQ(t.at(0, 0).attr, Terminal::kBold | Terminal::kReverse);
    CHECK_EQ(t.at(0, 1).attr, 0);
    t.feed("\x1B(0lqk\x1B(B");  // line drawing
    CHECK_STR(row(t, 0).substr(0, 5), std::string("BN+-+"));
}

TEST(terminal_reports_cursor_position) {
    Terminal t;
    std::string reply;
    t.set_reply([&](const std::string& s) { reply += s; });
    t.feed("\x1B[3;7H\x1B[6n");
    CHECK_STR(reply, std::string("\x1B[3;7R"));
}

TEST(settings_round_trip_and_validation) {
    app::Settings s;
    std::string error;
    CHECK(app::set_option(s, "prom", "roms/x.bin", error));
    CHECK(app::set_option(s, "cdrom", "a.iso", error));
    CHECK(app::set_option(s, "cdrom", "b.iso", error));
    CHECK(app::set_option(s, "cpu", "r4000sc", error));
    CHECK(app::set_option(s, "cpu_mhz", "175", error));
    CHECK(app::set_option(s, "ram_mb", "128", error));
    CHECK(app::set_option(s, "mac", "08:00:69:01:02:03", error));
    CHECK(app::set_option(s, "hostfwd", "tcp:2222:22", error));
    CHECK(app::set_option(s, "timing", "cycle", error));
    CHECK(app::set_option(s, "jit", "0", error));
    CHECK(app::set_option(s, "mouse_auto_capture", "0", error));
    CHECK(!app::set_option(s, "ram_mb", "100", error));  // not a multiple of 16
    CHECK(!app::set_option(s, "cpu", "r10000", error));
    CHECK(!app::set_option(s, "hostfwd", "tcp:0:22", error));
    CHECK(!app::set_option(s, "colour", "blue", error));

    const std::string path = temp_path("sg_settings_test.ini");
    CHECK(app::save_settings(path, s, error));
    app::Settings loaded;
    CHECK(app::load_settings(path, loaded, error));
    CHECK_STR(loaded.prom, std::string("roms/x.bin"));
    CHECK_EQ(loaded.cdroms.size(), 2u);
    CHECK_STR(loaded.cdroms[1], std::string("b.iso"));
    CHECK_STR(loaded.cpu, std::string("r4000sc"));
    CHECK_EQ(loaded.cpu_mhz, 175.0);
    CHECK_EQ(loaded.ram_mb, 128u);
    CHECK(loaded.mac && (*loaded.mac)[5] == 3);
    CHECK_EQ(loaded.forwards.size(), 1u);
    CHECK_EQ(loaded.forwards[0].guest_port, 22);
    CHECK(loaded.timing == Pacer::Mode::Cycle);
    CHECK(!loaded.jit);
    CHECK(!loaded.mouse_auto_capture);
    const auto config = loaded.indy_config();
    CHECK_EQ(config.prid, 0x0430u);
    CHECK_EQ(config.cpu_hz, 175.0e6);
    std::filesystem::remove(path);
}

TEST(settings_platform_limits_processor_clock_and_memory) {
    using ip22::Board;
    // Only the modules each platform shipped with, at the clocks it shipped them.
    CHECK(app::find_cpu_model("r5000sc")->fits(Board::Indy));
    CHECK(!app::find_cpu_model("r5000sc")->fits(Board::Indigo2));
    CHECK(!app::find_cpu_model("r4600pc")->fits(Board::Indigo2));
    CHECK(app::find_cpu_model("r4400sc")->fits(Board::Indigo2));
    CHECK_EQ(app::find_cpu_model("r4400sc")->mhz(Board::Indigo2).back(), 250.0);
    CHECK_EQ(app::find_cpu_model("r4400sc")->mhz(Board::Indy).back(), 200.0);
    CHECK_EQ(app::ram_sizes(Board::Indy, nullptr).back(), 256u);
    CHECK_EQ(app::ram_sizes(Board::Indigo2, nullptr).back(), 384u);

    // Indy R5000SC at 180 MHz with 256 MiB moved to the Indigo2: an R4400SC
    // at its first clock; the memory fits as it is.
    app::Settings s;
    s.cpu = "r5000sc", s.cpu_mhz = 180, s.ram_mb = 256;
    s.machine = "indigo2";
    app::make_consistent(s, false);
    CHECK_STR(s.cpu, std::string("r4400sc"));
    CHECK_EQ(s.cpu_mhz, 150.0);
    CHECK_EQ(s.ram_mb, 256u);
    // Back to the Indy with 384 MiB: the largest size that fits there.
    s.ram_mb = 384, s.cpu_mhz = 250;
    s.machine = "indy";
    app::make_consistent(s, false);
    CHECK_EQ(s.ram_mb, 256u);
    CHECK_EQ(s.cpu_mhz, 250.0);  // kept: the processor did not change
    bool noted = false;
    for (const auto& n : app::configuration_notes(s)) noted |= n.find("250 MHz") != std::string::npos;
    CHECK(noted);
    // A new processor takes a clock it shipped at.
    s.cpu = "r4600sc";
    app::make_consistent(s, true);
    CHECK_EQ(s.cpu_mhz, 133.0);
    s.machine = "indigo2";
    bool ecoff = false;
    for (const auto& n : app::configuration_notes(s)) ecoff |= n.find("ECOFF") != std::string::npos;
    CHECK(ecoff);
}

TEST(settings_check_hardware_refuses_impossible_combinations) {
    std::string error;
    app::Settings s;
    CHECK(app::check_hardware(s, false, error));  // the defaults
    s.machine = "indigo2", s.cpu = "r5000sc";
    CHECK(!app::check_hardware(s, false, error));
    CHECK(error.find("r4400sc, r4000sc") != std::string::npos);
    s.cpu = "r4400sc", s.cpu_mhz = 250, s.ram_mb = 384;
    CHECK(app::check_hardware(s, false, error));
    s.machine = "indy";
    CHECK(!app::check_hardware(s, false, error));  // 250 MHz: not an Indy clock
    CHECK(error.find("150 MHz, 175 MHz, 200 MHz") != std::string::npos);
    CHECK(!app::check_hardware(s, true, error));  // and 384 MiB does not fit
    CHECK(error.find("384 MiB") != std::string::npos);
    s.ram_mb = 256;
    CHECK(app::check_hardware(s, true, error));
}

TEST(proms_are_identified_and_chosen_for_the_machine) {
    namespace fs = std::filesystem;
    const fs::path dir = temp_path("sg_proms_test");
    fs::remove_all(dir);
    fs::create_directories(dir);
    auto image = [&](const char* name, const std::string& version) {
        std::string data(4096, '\0');
        data.replace(1000, version.size(), version);
        std::ofstream(dir / name, std::ios::binary) << data;
    };
    image("a.bin", "SGI Version 5.0 Rev B6 IP24 Sep 28, 1994");
    image("b.bin", "SGI Version 5.3 Rev B10 R4X00/R5000 IP24 Feb 12, 1996");
    image("c.bin", "SGI Version 5.1 Rev B IP22 Sep 16, 1993");
    image("notes.txt", "nothing to see");
    CHECK_EQ(app::scan_proms(dir.string()).size(), 3u);
    const app::PromImage b = app::identify_prom((dir / "b.bin").string());
    CHECK_STR(b.system, std::string("ip24"));
    CHECK_STR(b.version, std::string("5.3 Rev B10"));
    CHECK(b.r5000);

    app::Settings s;
    s.prom_dir = dir.string();
    app::PromImage p;
    std::string error;
    CHECK(app::select_prom(s, p, error));  // Indy: the newest
    CHECK_STR(fs::path(p.path).filename().string(), std::string("b.bin"));
    s.machine = "indigo2";
    CHECK(app::select_prom(s, p, error));
    CHECK_STR(fs::path(p.path).filename().string(), std::string("c.bin"));
    // An Indy with an R5000 and only the 5.0 PROM: refused, saying why.
    fs::remove(dir / "b.bin");
    s.machine = "indy", s.cpu = "r5000sc";
    CHECK(!app::select_prom(s, p, error));
    CHECK(error.find("predates the R5000") != std::string::npos);
    s.cpu = "r4600sc";
    CHECK(app::select_prom(s, p, error));
    // ... which needs a graphics board.
    s.graphics = false;
    CHECK(!app::select_prom(s, p, error));
    CHECK(error.find("without a graphics board") != std::string::npos);
    s.graphics = true;
    // A specific file must suit the machine.
    s.prom = (dir / "c.bin").string();
    CHECK(!app::select_prom(s, p, error));
    CHECK(error.find("Indigo2 (IP22) PROM") != std::string::npos);
    // No PROM at all for the Indigo2.
    s.prom.clear();
    fs::remove(dir / "c.bin");
    s.machine = "indigo2", s.cpu = "r4400sc";
    CHECK(!app::select_prom(s, p, error));
    CHECK(error.find("no Indigo2 (IP22) boot PROM") != std::string::npos);
    // A dump read with its bytes swapped is recognised and put right.
    {
        std::string data(4096, '\0');
        const std::string version = "SGI Version 5.1 Rev B IP22 Sep 16, 1993";
        data.replace(1000, version.size(), version);
        data[0] = 0x0B, data[1] = 0xF0;
        for (std::size_t i = 0; i < data.size(); i += 2) std::swap(data[i], data[i + 1]);
        std::ofstream(dir / "swapped.bin", std::ios::binary) << data;
    }
    CHECK(app::select_prom(s, p, error));
    CHECK(p.byte_swapped);
    CHECK_STR(p.version, std::string("5.1 Rev B"));
    std::vector<u8> contents;
    CHECK(app::read_prom(p, contents, error));
    CHECK_EQ(contents[0], 0x0Bu);
    CHECK_EQ(contents[1], 0xF0u);
    fs::remove_all(dir);
}

TEST(blank_images_are_created_once) {
    const std::string path = temp_path("sg_blank_test.img");
    std::filesystem::remove(path);
    std::string error;
    CHECK(app::create_blank_image(path, 1u << 20, error));
    CHECK_EQ(std::filesystem::file_size(path), u64{1} << 20);
    CHECK(!app::create_blank_image(path, 4096, error));  // never replaces a file
    CHECK_EQ(std::filesystem::file_size(path), u64{1} << 20);
    std::filesystem::remove(path);
}

TEST(cdrom_lock_holds_the_disc_in) {
    const std::string path = temp_path("sg_cd_lock_test.img");
    {
        std::ofstream f(path, std::ios::binary);
        f << std::string(4096, 'x');
    }
    {
        ScsiDisk cd(ScsiDisk::Kind::CdRom, path);
        const u8 prevent[6] = {0x1E, 0, 0, 0, 1, 0};
        CHECK_EQ(cd.command(prevent, 6).status, 0x00);
        CHECK(cd.removal_prevented());
        const u8 eject[6] = {0x1B, 0, 0, 0, 0x02, 0};
        CHECK_EQ(cd.command(eject, 6).status, 0x02);  // check condition: removal prevented
        CHECK(cd.loaded());
        const u8 allow[6] = {0x1E, 0, 0, 0, 0, 0};
        cd.command(allow, 6);
        cd.eject();  // the drive's button
        CHECK(!cd.loaded());
        const u8 tur[6] = {0x00};
        CHECK_EQ(cd.command(tur, 6).status, 0x02);
    }
    std::filesystem::remove(path);
}
