// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

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
