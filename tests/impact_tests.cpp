// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// IMPACT graphics tests, driven through the board's register window as a
// console driver does.
#include <vector>

#include "core/scheduler.h"
#include "machine/ip22/impact.h"
#include "test.h"

using namespace sg;
using namespace sg::ip22;

namespace {

constexpr u32 kFifo = 0x70080, kStatus = 0x70000, kFifoStatus = 0x70004, kFlagSet = 0x70008, kFlagClear = 0x7000C;
constexpr u32 kIntEnSet = 0x50018, kCfifoLow = 0x50024;
// Raster registers
constexpr u32 kIrAlias = 0x045, kXlineStart = 0x040, kXlineEnd = 0x041, kBlockStart = 0x046, kBlockEnd = 0x047;
constexpr u32 kPackedColor = 0x05B, kCharH = 0x070, kFillMode = 0x110, kConfig = 0x112, kXyWin = 0x115;
constexpr u32 kBkgrdRg = 0x140, kBkgrdBa = 0x141, kLsPat = 0x15A, kPp1FillMode = 0x161, kMaskMsbs = 0x162;
constexpr u32 kMaskA = 0x163, kDrbPointers = 0x16D;
// DCB: device window + CRS + width
constexpr u32 dcb(unsigned dev, unsigned crs, unsigned width) { return 0x60000 + dev * 0x400 + crs * 0x80 + (width & 3) * 8; }

struct Rig {
    u64 now = 0;
    Scheduler sched;
    Impact board;
    std::vector<std::pair<unsigned, bool>> irqs;
    explicit Rig(Impact::Variant v = Impact::Variant::High)
        : board((sched.set_clock([this] { return now; }), sched), 150.0e6, v) {
        board.set_irq_callback([this](unsigned line, bool level) { irqs.emplace_back(line, level); });
    }
    void w(u32 off, u32 v) { CHECK(board.write(off, 4, v)); }
    void w8(u32 off, u8 v) { CHECK(board.write(off, 1, v)); }
    void w64(u32 off, u64 v) { CHECK(board.write(off, 8, v)); }
    u32 r(u32 off) {
        u64 v = 0;
        CHECK(board.read(off, 4, v));
        return static_cast<u32>(v);
    }
    // A raster register through the command FIFO, as one doubleword.
    void reg(u32 r_, u32 v, bool go = false) {
        w64(kFifo, (u64{((0x1800u | r_ | (go ? 0x400u : 0)) << 8) | 4} << 32) | v);
    }
    // OpenBSD's console set-up: 24-bit buffer A, Y flipped from the top row.
    void console_setup() {
        w(0x62038, 0x1D000100);  // VC3: cursor off
        reg(0x163, 0xFFFFFF);
        reg(0x164, 0xFFFFFF);
        reg(kMaskMsbs, 0);
        reg(0x156, 0xFFFFFF);
        reg(0x157, 0xFFFFFF);
        reg(kDrbPointers, 0xC8240);
        reg(kConfig, 0xCAC);
        reg(kXyWin, 0x03FF0000);
        w8(0x61C08, 1);          // XMAP PP1SELECT
        w8(0x61C88, 0);          // INDEX
        w(0x61E00, 0x7A4);       // MAIN_MODE[0]: 24-bit
    }
    void fill(int x, int y, int w_, int h, u32 bgr, unsigned op = 3) {
        reg(kPp1FillMode, (op << 26) | 0x6300 | (op != 3 ? 4 : 0));
        reg(kFillMode, 0);
        reg(kPackedColor, bgr);
        reg(kBlockStart, static_cast<u32>(x) << 16 | static_cast<u32>(y));
        reg(kBlockEnd, static_cast<u32>(x + w_ - 1) << 16 | static_cast<u32>(y + h - 1));
        reg(kIrAlias, 0x18, true);
    }
    u32 pixel(int x, int y) { return static_cast<u32>(board.pixel_word(0x240, x, 1023 - y)) & 0xFFFFFF; }
    u32 shown(int x, int y) {
        std::vector<u32> argb;
        unsigned width = 0, height = 0;
        CHECK(board.render_full(argb, width, height));
        return argb[static_cast<std::size_t>(y) * width + x];
    }
};

}  // namespace

TEST(impact_identifies_itself) {
    Rig t;
    CHECK_EQ(t.r(0), 0x00050190u);  // GIO ID: product 0x10, GIO64
    CHECK_EQ(t.r(dcb(9, 0, 1)) >> 24, 0x72u);  // board version, High IMPACT
    CHECK_EQ(t.r(dcb(9, 1, 1)) >> 24, 0x01u);
    Rig s(Impact::Variant::Solid), m(Impact::Variant::Max);
    CHECK_EQ(s.r(dcb(9, 0, 1)) >> 24, 0x70u);
    CHECK_EQ(m.r(dcb(9, 1, 1)) >> 24, 0x02u);  // two geometry engines
    CHECK_EQ(t.r(0x200000), 0u);               // outside the 1 MB window: nothing, no fault
}

TEST(impact_reports_idle_and_ready) {
    Rig t;
    t.w(0x50020, 0x20);
    CHECK_EQ(t.r(kFifoStatus) & 0x7F, 0u);   // the command FIFO is empty
    CHECK_EQ(t.r(kFifoStatus) >> 22, 0x20u);  // the high-water mark
    CHECK((t.r(kStatus) & 3) == 3);           // HQ3 and board idle
    CHECK_EQ(t.r(dcb(4, 4, 1)) >> 24, 0x08u);  // colour map ready for writes
    t.w8(0x61C88, 8);
    CHECK(t.r(dcb(7, 2, 1)) >> 24 != 0);  // XMAP has room for a mode update
    CHECK_EQ(t.r(dcb(7, 7, 0)), 0x00010101u);  // links in sync
}

TEST(impact_block_fill_lands_in_buffer_a_and_is_displayed) {
    Rig t;
    t.console_setup();
    t.fill(10, 20, 4, 3, 0x0000FF);  // red, as 0x00BBGGRR
    CHECK_EQ(t.pixel(10, 20), 0x0000FFu);
    CHECK_EQ(t.pixel(13, 22), 0x0000FFu);
    CHECK_EQ(t.pixel(14, 22), 0u);
    CHECK_EQ(t.pixel(13, 23), 0u);
    CHECK_EQ(t.shown(10, 20), 0xFFFF0000u);  // window row 20 from the top
    CHECK_EQ(t.shown(9, 20), 0xFF000000u);
    // Tiles: x 192 starts the next page of the row.
    t.fill(190, 0, 4, 1, 0x00FF00);
    CHECK_EQ(t.pixel(191, 0), 0x00FF00u);
    CHECK_EQ(t.pixel(192, 0), 0x00FF00u);
    CHECK_EQ(t.shown(193, 0), 0xFF00FF00u);
}

TEST(impact_logic_op_xor_and_plane_masks) {
    Rig t;
    t.console_setup();
    t.fill(0, 0, 2, 1, 0x123456);
    t.fill(0, 0, 1, 1, 0xFFFFFF, 6);  // XOR
    CHECK_EQ(t.pixel(0, 0), 0xEDCBA9u);
    CHECK_EQ(t.pixel(1, 0), 0x123456u);
    t.reg(kMaskA, 0x0000FF);  // only red
    t.fill(1, 0, 1, 1, 0x000000);
    CHECK_EQ(t.pixel(1, 0), 0x123400u);
}

TEST(impact_opaque_char_stipple_draws_glyph_rows) {
    Rig t;
    t.console_setup();
    t.reg(kPp1FillMode, (3u << 26) | 0x6300);
    t.reg(kFillMode, 0x00400018);  // char block, opaque
    t.reg(kPackedColor, 0xFFFFFF);
    t.reg(kBkgrdRg, 0x00F00);       // background red 0xF0 (12-bit field >> 4)
    t.reg(kBkgrdBa, 0);
    t.reg(kBlockStart, 100u << 16 | 50u);
    t.reg(kBlockEnd, 107u << 16 | 51u);  // 8 x 2
    t.reg(kIrAlias, 0x18, true);
    t.reg(kCharH, 0x81u << 24, true);  // row 0: first and last pixel
    t.reg(kCharH, 0x3Cu << 24, true);  // row 1
    CHECK_EQ(t.pixel(100, 50), 0xFFFFFFu);
    CHECK_EQ(t.pixel(101, 50), 0x0000F0u);
    CHECK_EQ(t.pixel(107, 50), 0xFFFFFFu);
    CHECK_EQ(t.pixel(101, 51), 0x0000F0u);
    CHECK_EQ(t.pixel(102, 51), 0xFFFFFFu);
    CHECK_EQ(t.pixel(105, 51), 0xFFFFFFu);
    CHECK_EQ(t.pixel(106, 51), 0x0000F0u);
    CHECK_EQ(t.pixel(100, 52), 0u);  // nothing past the block
}

TEST(impact_x_line_and_stipple) {
    Rig t;
    t.console_setup();
    t.reg(kPp1FillMode, (3u << 26) | 0x6300);
    t.reg(kFillMode, 0);
    t.reg(kPackedColor, 0x00FF00);
    t.reg(kXlineStart, 0u << 16 | 5u);
    t.reg(kXlineEnd, 4u << 16 | 5u);
    t.reg(kIrAlias, 0x15, true);
    for (int x = 0; x <= 4; ++x) CHECK_EQ(t.pixel(x, 5), 0x00FF00u);
    t.reg(kFillMode, 0x20 | (1u << 10));  // stippled, last pixel left out
    t.reg(kLsPat, 0xA0000000u);           // on, off, on, off...
    t.reg(kPackedColor, 0x0000FF);
    t.reg(kXlineStart, 0u << 16 | 6u);
    t.reg(kXlineEnd, 3u << 16 | 6u);
    t.reg(kIrAlias, 0x15, true);
    CHECK_EQ(t.pixel(0, 6), 0x0000FFu);
    CHECK_EQ(t.pixel(1, 6), 0u);
    CHECK_EQ(t.pixel(2, 6), 0x0000FFu);
    CHECK_EQ(t.pixel(3, 6), 0u);  // the last pixel
}

TEST(impact_direct_window_and_execute_alias) {
    Rig t;
    t.console_setup();
    t.w(0x7C000 + 4 * kPp1FillMode, (3u << 26) | 0x6300);
    t.w(0x7C000 + 4 * kFillMode, 0);
    t.w(0x7C000 + 4 * kPackedColor, 0xABCDEF);
    t.w64(0x7C000 + 4 * kBlockStart, (u64{7u << 16 | 8u} << 32) | (7u << 16 | 8u));
    CHECK_EQ(t.r(0x7C000 + 4 * kPackedColor), 0xABCDEFu);
    t.w(0x7D000 + 4 * kIrAlias, 0x18);  // written and executed
    CHECK_EQ(t.pixel(7, 8), 0xABCDEFu);
}

TEST(impact_fifo_parses_pairs_pixel_data_and_done_flag) {
    Rig t;
    t.console_setup();
    t.reg(kPp1FillMode, (3u << 26) | 0x6300);
    t.reg(kFillMode, 0);
    t.reg(kPackedColor, 0x445566);
    // A pixel data header and its doublewords pass by without being taken as commands.
    t.w(kFifo, 0x80000000u | 5);  // 5 bytes: one doubleword
    t.w(kFifo, 0x0018FFFF);
    t.w(kFifo, 0xFFFFFFFF);
    // Both block corners in one command (8 data bytes), executed after the second.
    t.w(kFifo, ((0x1800u | 0x400u | kBlockStart) << 8) | 8);
    t.w(kFifo, 3u << 16 | 4u);
    t.w(kFifo, 3u << 16 | 4u);
    t.reg(kIrAlias, 0x18);
    t.w(kFifo, ((0x1800u | 0x400u | kBlockStart) << 8) | 8);
    t.w(kFifo, 3u << 16 | 4u);
    t.w(kFifo, 3u << 16 | 4u);
    CHECK_EQ(t.pixel(3, 4), 0x445566u);
    // SET_DONE_FLAG raises flag 16, and the graphics interrupt when enabled.
    t.w(kIntEnSet, 1u << 16);
    t.w(kFifo, 0xE04u << 8);
    CHECK(t.r(kFlagSet) & (1u << 16));
    CHECK(!t.irqs.empty() && t.irqs.back() == std::make_pair(1u, true));
    t.w(kFlagClear, 1u << 16);
    CHECK(t.irqs.back() == std::make_pair(1u, false));
    // The FIFO's low-water interrupt holds while the FIFO is below the mark.
    t.w(kCfifoLow, 0x14);
    t.w(kIntEnSet, 1u << 27);
    CHECK(t.irqs.back() == std::make_pair(0u, true));
}

TEST(impact_window_ids_choose_colour_index_mode) {
    Rig t;
    t.console_setup();
    // Colour map entry 0x105 (block 1, index 5) = green.
    t.w(dcb(3, 0, 2), 0x0501u << 16);  // address, low byte first
    t.w(dcb(3, 2, 3), 0x00FF00u << 8);
    t.w(dcb(4, 0, 2), 0x0501u << 16);  // (the address moved on past the entry)
    CHECK_EQ(t.r(dcb(4, 2, 3)) >> 8, 0x00FF00u);
    // XMAP mode for DID 2: CI8 (format 1), colour map block 1.
    t.w8(0x61C88, 8);
    t.w(0x61E00, (1u << 5) | 1u);
    // VC3 DID tables: one line table, DID 0 from x 0, DID 2 from x 16, used by every line.
    const u16 line_table = 0x100, frame_table = 0x200;
    auto vc3_reg = [&](unsigned i, u16 v) { t.w(dcb(8, 0, 3), (i << 24) | (u32{v} << 8)); };
    auto sram = [&](u16 addr, std::initializer_list<u16> words) {
        vc3_reg(7, addr);
        for (u16 v : words) t.w(dcb(8, 3, 2), u32{v} << 16);
    };
    sram(line_table, {0 << 5 | 0, 16 << 5 | 2, 0x7FF << 5});
    std::vector<u16> frame(1024, line_table);
    vc3_reg(7, frame_table);
    for (u16 v : frame) t.w(dcb(8, 3, 2), u32{v} << 16);
    sram(static_cast<u16>(frame_table + 1024), {0xFFFF});
    vc3_reg(4, frame_table);
    // Index 5 in buffer A's colour index field.
    t.reg(kPp1FillMode, (3u << 26) | (5u << 8) | (1u << 14));  // CI8, buffer A
    t.reg(0x05C, 5u << 12);                                    // red iterator = the index
    t.reg(kFillMode, 0);
    t.reg(kBlockStart, 16u << 16 | 0u);
    t.reg(kBlockEnd, 16u << 16 | 0u);
    t.reg(kIrAlias, 0x18, true);
    CHECK_EQ(t.shown(16, 0), 0xFF00FF00u);
    CHECK_EQ(t.shown(15, 0), 0xFF000000u);  // DID 0: 24-bit black
}

TEST(impact_cursor_draws_over_the_picture) {
    Rig t;
    t.console_setup();
    t.w(dcb(3, 0, 2), 0x0100u << 16);  // colour map entry 1 = blue
    t.w(dcb(3, 2, 3), 0x0000FFu << 8);
    auto vc3_reg = [&](unsigned i, u16 v) { t.w(dcb(8, 0, 3), (i << 24) | (u32{v} << 8)); };
    vc3_reg(1, 0x4000);  // glyph
    vc3_reg(7, 0x4000);
    t.w(dcb(8, 3, 2), 0x8000u << 16);  // plane 0, row 0, first pixel
    vc3_reg(2, 31 + 40);
    vc3_reg(3, 31 + 30);
    vc3_reg(0x1D, 3);  // enabled and displayed, 32 x 32
    int x = 0, y = 0;
    CHECK(t.board.cursor_position(x, y));
    CHECK_EQ(x, 40);
    CHECK_EQ(y, 30);
    CHECK_EQ(t.shown(40, 30), 0xFF0000FFu);
    CHECK_EQ(t.shown(41, 30), 0xFF000000u);
}

TEST(impact_max_has_two_engines_on_alternate_rows) {
    Rig t(Impact::Variant::Max);
    t.console_setup();
    t.fill(0, 0, 1, 4, 0x0000FF);
    for (int y = 0; y < 4; ++y) CHECK_EQ(t.pixel(0, y), 0x0000FFu);
    CHECK_EQ(t.shown(0, 3), 0xFFFF0000u);
}

TEST(impact_retrace_interrupt_when_enabled) {
    Rig t;
    t.w(dcb(8, 0, 3), (0x1Eu << 24) | (1u << 8));  // VC3 display control: retrace interrupt
    t.now = static_cast<u64>(150.0e6 / 60) + 1;
    t.sched.run_due(t.now);
    CHECK(!t.irqs.empty() && t.irqs.back() == std::make_pair(2u, true));
    CHECK(t.r(kStatus) & 4);  // vertical blank
}
