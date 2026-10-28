// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// IMPACT graphics tests, driven through the board's register window as a
// console driver does.
#include <array>
#include <cstring>
#include <initializer_list>
#include <vector>

#include "core/endian.h"
#include "core/scheduler.h"
#include "machine/ip22/impact.h"
#include "mem/bus.h"
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
    Rig() : board((sched.set_clock([this] { return now; }), sched), 150.0e6) {
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
    CHECK_EQ(t.r(dcb(9, 0, 1)) >> 24, 0x0Au);  // board version: second raster board, 4 TRAMs on each
    CHECK_EQ(t.r(dcb(9, 1, 1)) >> 24, 0x02u);  // two geometry engines
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

// XMAP BUF_SELECT holds two words, at INDEX 0 and 4: the kernel's retrace
// handler writes both at every swap.
TEST(impact_xmap_buf_select_has_two_words) {
    Rig t;
    t.w(dcb(7, 1, 1), 4u << 24);
    t.w(dcb(7, 3, 0), 0x12345678);
    t.w(dcb(7, 1, 1), 0u << 24);
    t.w(dcb(7, 3, 0), 0x00000005);
    CHECK_EQ(t.r(dcb(7, 3, 0)), 0x5u);
    t.w(dcb(7, 1, 1), 4u << 24);
    CHECK_EQ(t.r(dcb(7, 3, 0)), 0x12345678u);
}

// The kernel's start-up: the RE link sync writes RE_RAC control words and
// waits for the status to show the links in sync; nothing on screen changes.
// The flat panel probe writes 0x55 to the I2C data register and finds no
// panel when it does not read back.
TEST(impact_start_up_link_sync_and_flat_panel_probe) {
    Rig t;
    const u64 v = t.board.display_version();
    t.w(dcb(7, 1, 1), 0u << 24);
    t.w(dcb(7, 7, 0), 0x50214);
    t.w(dcb(7, 1, 1), 28u << 24);
    t.w(dcb(7, 7, 0), 0x653);
    t.w(dcb(7, 1, 1), 0u << 24);
    t.w(dcb(7, 7, 0), 0x414);
    t.w(dcb(7, 7, 0), 0x514);
    t.w(dcb(7, 1, 1), 7u << 24);
    const u32 s = t.r(dcb(7, 7, 0));
    CHECK_EQ(s & 0xF, (s >> 8) & 0xF);
    CHECK_EQ((s >> 16) & 0xF, 1u);
    CHECK_EQ(t.board.display_version(), v);
    t.w(dcb(11, 0, 1), 0x55u << 24);
    t.w(dcb(11, 1, 1), 0x40u << 24);
    CHECK(t.r(dcb(11, 0, 1)) >> 24 != 0x55u);
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
    Rig t;
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

// GE11 diagnostic ports: microcode written three words a line reads back in
// the verifier's order; starting the engine answers revision 1 with flag 17.
TEST(impact_ge_microcode_download_readback_and_start) {
    Rig t;
    t.w(0x50044, 0x200000);  // GE0: microcode line 0
    for (u32 i = 0; i < 4 * 3; ++i) t.w(0x50040, 0x1000 + i);  // lines 0-3
    t.w(0x50044, 0x200000);
    t.w(0x50044, 0x80000004);  // read 4 lines
    CHECK(t.r(0x50008) & (1u << 18));
    const u32 want[] = {0x1000, 0x1002 & 0xFF, 0x1004, 0x1006, 0x1008 & 0xFF, 0x100A};
    for (u32 w_ : want) CHECK_EQ(t.r(0x5022C), w_);
    CHECK(!(t.r(0x50008) & (1u << 18)));
    t.w(0x50044, 0x200001);  // from an odd line: two empty words first
    t.w(0x50044, 0x80000002);
    CHECK_EQ(t.r(0x5022C), 0u);
    CHECK_EQ(t.r(0x5022C), 0u);
    CHECK_EQ(t.r(0x5022C), 0x1003u);
    t.w(0x50044, 0x40038);  // the kernel's other control registers do not start it
    t.w(0x50040, 0xD1);
    CHECK(!(t.r(0x70008) & (1u << 17)));
    t.w(0x50044, 0x40000);  // execution control: start
    t.w(0x50040, 1);
    CHECK(t.r(0x70008) & (1u << 17));
    CHECK_EQ(t.r(0x70010), 0u);
    CHECK_EQ(t.r(0x70014), 1u);
}

// A context switch: the save completes at once (flag 19); the next 63 FIFO
// words are the incoming context's image, not commands (flag 6 after the
// last). A context switched out midway through a command finishes it when
// it is switched back in.
TEST(impact_context_switch_takes_the_image_and_resumes_commands) {
    Rig t;
    t.console_setup();
    auto switch_to = [&](u32 slot, bool first) {
        t.w(0x7000C, (1u << 19) | (1u << 6));
        t.w(0x50050, 0x100);
        CHECK(t.r(0x70008) & (1u << 19));
        for (u32 i = 0; i < 63; ++i) {
            CHECK(!(t.r(0x70008) & (1u << 6)));
            t.w(kFifo, i == 0 ? slot : i == 1 ? (first ? 0x80000000u : 0) : 0xFFFFFFFFu);
        }
        CHECK(t.r(0x70008) & (1u << 6));
    };
    switch_to(0x1000, true);
    // Context A starts a block fill's packed colour: the command word only.
    t.reg(kPp1FillMode, 0x0C006300);
    t.reg(kFillMode, 0);
    t.w(kFifo, ((0x1800u | kPackedColor) << 8) | 4);
    switch_to(0x2000, true);  // B: its words are its own commands
    t.reg(kPackedColor, 0x00FF00);
    switch_to(0x1000, false);  // A again: its colour word arrives now
    t.w(kFifo, 0x0000FF);
    t.reg(kBlockStart, 0);
    t.reg(kBlockEnd, (3u << 16) | 3);
    t.reg(kIrAlias, 0x18, true);
    CHECK_EQ(t.pixel(1, 1), 0x0000FFu);
}

// The DMA engine's and raster interface's registers, set through the
// command FIFO, read back from their context RAM (the kernel's sync).
TEST(impact_dma_and_reif_registers_read_back_from_context_ram) {
    Rig t;
    t.w64(kFifo, (u64{(0x827u << 8) | 4} << 32) | 0x5);   // DMA register 0x27
    t.w64(kFifo, (u64{(0xA06u << 8) | 4} << 32) | 0xC);   // REIF register 6
    CHECK_EQ(t.r(0x5039C), 0x5u);
    CHECK_EQ(t.r(0x50218), 0xCu);
}

// Overlay drawing (draw field 0x4N) lands in the 9-bit overlay planes at
// DRBpointers' page, through the overlay plane mask; clip-ID drawing (0x50)
// in the clip-ID planes, which the CID match then tests; screen masks keep
// drawing inside (or outside) their rectangles.
TEST(impact_overlay_clip_ids_and_screen_masks) {
    Rig t;
    t.console_setup();
    t.reg(kDrbPointers, 0x1C0);  // the overlay's pages
    t.reg(kMaskMsbs, 0xFF);
    t.reg(kPp1FillMode, (3u << 26) | (0x4Fu << 14) | 0x500);
    t.reg(kFillMode, 0);
    t.reg(0x05C, 0x2A << 12);  // red iterator: CI 0x2A
    t.reg(kBlockStart, (10u << 16) | 10);
    t.reg(kBlockEnd, (12u << 16) | 12);
    t.reg(kIrAlias, 0x18, true);
    CHECK_EQ(t.board.overlay_pixel(0x1C0, 11, 1023 - 11), 0x2Au);
    CHECK_EQ(t.board.overlay_pixel(0x1C0, 13, 1023 - 11), 0u);

    // Clip ID 2 over x 0-4, then draw with CIDmatch for ID 2 only.
    t.reg(kDrbPointers, 0xC8240);
    t.reg(0x17B, 0xC00);
    t.reg(kPp1FillMode, (3u << 26) | (0x50u << 14) | 0x500);
    t.reg(0x05C, 2 << 12);
    t.reg(kBlockStart, 0);
    t.reg(kBlockEnd, (4u << 16) | 4);
    t.reg(kIrAlias, 0x18, true);
    t.reg(0x17B, 1u << (4 + 2));
    t.fill(0, 0, 8, 1, 0x00FF00);
    CHECK_EQ(t.pixel(2, 0), 0x00FF00u);
    CHECK_EQ(t.pixel(6, 0), 0u);  // clip ID 0 there: not drawn
    t.reg(0x17B, 0);

    // Screen mask 1 enabled, keep inside: x 20-21, y (framebuffer) 1018-1023.
    t.reg(0x147, (20u << 16) | 21);
    t.reg(0x148, (1018u << 16) | 1023);
    t.reg(0x14F, 0x11);
    t.fill(18, 0, 6, 1, 0x0000FF);
    CHECK_EQ(t.pixel(20, 0), 0x0000FFu);
    CHECK_EQ(t.pixel(21, 0), 0x0000FFu);
    CHECK_EQ(t.pixel(19, 0), 0u);
    CHECK_EQ(t.pixel(22, 0), 0u);
    t.reg(0x14F, 0x01);  // keep outside
    t.fill(18, 1, 6, 1, 0xFF0000);
    CHECK_EQ(t.pixel(19, 1), 0xFF0000u);
    CHECK_EQ(t.pixel(20, 1), 0u);
}

namespace {
// A DMA register through the command FIFO (0x800 + n, one word).
void dma_reg(Rig& t, unsigned n, u32 v) { t.w64(kFifo, (u64{((0x800u + n) << 8) | 4} << 32) | v); }
// Arms a 24-bit transfer of a w x h block at (x, y): block type `type`, host format `xfrmode`.
void arm(Rig& t, int x, int y, int w, int h, unsigned type, u32 xfrmode) {
    t.reg(kPp1FillMode, (3u << 26) | 0x6300);
    t.reg(0x159, xfrmode);
    t.reg(0x153, (static_cast<u32>(h) << 16) | static_cast<u32>(w));
    t.reg(kFillMode, type << 22);
    t.reg(kBlockStart, static_cast<u32>(x) << 16 | static_cast<u32>(y));
    t.reg(kBlockEnd, static_cast<u32>(x + w - 1) << 16 | static_cast<u32>(y + h - 1));
    t.reg(kIrAlias, 0x18, true);
}
}  // namespace

// PIO: a write transfer takes doublewords through char_h:char_l (each line
// in a fresh doubleword); a read gives them back through char_h's execute
// alias (high half) and char_l.
TEST(impact_pio_pixel_transfers) {
    Rig t;
    t.console_setup();
    arm(t, 4, 4, 2, 2, 3, 0x80);  // RGBA bytes, ABGR order
    for (u64 dw : {0xFF0000FFFF00FF00ull, 0xFFFF0000FF808080ull}) t.w64(0x7D1C0, dw);  // char_h:char_l, execute
    CHECK_EQ(t.pixel(4, 4), 0x0000FFu);  // A=FF B=00 G=00 R=FF
    CHECK_EQ(t.pixel(5, 4), 0x00FF00u);
    CHECK_EQ(t.pixel(4, 5), 0xFF0000u);
    CHECK_EQ(t.pixel(5, 5), 0x808080u);
    arm(t, 4, 4, 2, 2, 2, 0x40080);  // read back, DMA order
    CHECK_EQ(t.r(0x7D1C0), 0x000000FFu);
    CHECK_EQ(t.r(0x7C1C4), 0x0000FF00u);
    CHECK_EQ(t.r(0x7D1C0), 0x00FF0000u);
    CHECK_EQ(t.r(0x7C1C4), 0x00808080u);
}

// DMA through a pool's page table: host to board into a write transfer,
// board to host out of a read transfer once both sides have started.
TEST(impact_dma_pixel_transfers) {
    Rig t;
    std::vector<u8> ram(1u << 20);
    Bus bus;
    bus.map_ram(0, ram.size(), ram.data());
    t.board.set_bus(&bus);
    t.console_setup();
    // Pool 1's table at 0x10000: logical page 0 -> frame 0x20 (0x20000), page 1 -> 0x30.
    store_be<u32>(&ram[0x10000], 0x20);
    store_be<u32>(&ram[0x10004], 0x30);
    for (u32 i = 0; i < 3; ++i) store_be<u32>(&ram[0x20000 + 4 * i], 0x00FF0000u >> (8 * i) | 0xFF000000u);  // ABGR words
    dma_reg(t, 0x22, 0);
    dma_reg(t, 0x23, 0x10000);
    dma_reg(t, 0x00, 0x80000000);  // page list: normal mode
    dma_reg(t, 0x04, 0x100);       // stride
    dma_reg(t, 0x05, 0);
    dma_reg(t, 0x06, 0);
    dma_reg(t, 0x07, 1);           // one line
    dma_reg(t, 0x08, 12);          // of 12 bytes
    arm(t, 0, 0, 3, 1, 5, 0x80);
    dma_reg(t, 0x0B, 0x3);         // run, pool 1, host to board
    CHECK_EQ(t.pixel(0, 0), 0xFF0000u);
    CHECK_EQ(t.pixel(1, 0), 0x00FF00u);
    CHECK_EQ(t.pixel(2, 0), 0x0000FFu);
    // Read the three pixels back to logical 0x1000 (frame 0x30).
    dma_reg(t, 0x06, 0x1000);
    arm(t, 0, 0, 3, 1, 4, 0x80);
    dma_reg(t, 0x0B, 0xB);         // run, pool 1, board to host: waits
    CHECK_EQ(load_be<u32>(&ram[0x30000]), 0u);
    t.reg(0x102, 9);               // the raster side starts
    CHECK_EQ(load_be<u32>(&ram[0x30000]) & 0xFFFFFF, 0xFF0000u);
    CHECK_EQ(load_be<u32>(&ram[0x30008]) & 0xFFFFFF, 0x0000FFu);
}

// A page-list entry names a slot of the pool's table, 1 << PAGE_SHIFT
// pages (0x2B + pool); ROW_START is an offset within it, as the kernel's
// TRAM backing store uses with 4 MB slots (shift 10).
TEST(impact_dma_page_list_windows) {
    Rig t;
    std::vector<u8> ram(1u << 20);
    Bus bus;
    bus.map_ram(0, ram.size(), ram.data());
    t.board.set_bus(&bus);
    t.console_setup();
    // Pool 1's table at 0x10000: window 0 page 1 -> frame 0x30, window 1 page 1 -> frame 0x40.
    store_be<u32>(&ram[0x10000 + 4 * 1], 0x30);
    store_be<u32>(&ram[0x10000 + 4 * (1024 + 1)], 0x40);
    for (u32 i = 0; i < 3; ++i) {
        store_be<u32>(&ram[0x30000 + 4 * i], 0xFF000000u);                          // black in window 0
        store_be<u32>(&ram[0x40000 + 4 * i], 0x00FF0000u >> (8 * i) | 0xFF000000u);  // colours in window 1
    }
    dma_reg(t, 0x22, 0);
    dma_reg(t, 0x23, 0x10000);
    dma_reg(t, 0x2C, 10);          // pool 1: 1024 pages to an entry
    dma_reg(t, 0x00, 0x80010000);  // page list: window 1, the last entry
    dma_reg(t, 0x04, 0x100);
    dma_reg(t, 0x05, 0);
    dma_reg(t, 0x06, 0x1000);      // offset 0x1000 within the window
    dma_reg(t, 0x07, 1);
    dma_reg(t, 0x08, 12);
    arm(t, 0, 0, 3, 1, 5, 0x80);
    dma_reg(t, 0x0B, 0x3);
    CHECK_EQ(t.pixel(0, 0), 0xFF0000u);
    CHECK_EQ(t.pixel(1, 0), 0x00FF00u);
    CHECK_EQ(t.pixel(2, 0), 0x0000FFu);
}

// Several entries (libGLcore's pixel tiles, a tile to a slot): each line
// takes LINE_BYTES from ROW_OFFSET in the first tile, STRIDE bytes of each
// middle one and LAST_BYTES of the last, from ROW_START + line * STRIDE.
TEST(impact_dma_page_list_tiles) {
    Rig t;
    std::vector<u8> ram(1u << 20);
    Bus bus;
    bus.map_ram(0, ram.size(), ram.data());
    t.board.set_bus(&bus);
    t.console_setup();
    // Pool 1, four pages to an entry: tiles in slots 1, 3 and 0, rows of
    // two pixels; word k of tile n reads n << 16 | k.
    const u32 frame[] = {0x60, 0x40, 0, 0x50};
    for (u32 slot : {0u, 1u, 3u}) {
        store_be<u32>(&ram[0x10000 + 4 * (slot * 4)], frame[slot]);
        for (u32 k = 0; k < 16; ++k) store_be<u32>(&ram[(frame[slot] << 12) + 4 * k], 0xFF000000u | slot << 16 | k);
    }
    dma_reg(t, 0x22, 0);
    dma_reg(t, 0x23, 0x10000);
    dma_reg(t, 0x2C, 2);
    dma_reg(t, 0x00, 0x00010003);  // page list: slots 1, 3,
    dma_reg(t, 0x01, 0x80000000);  // and 0, the last
    dma_reg(t, 0x04, 8);           // tile rows of 8 bytes
    dma_reg(t, 0x05, 4);           // from the second pixel of the first tile
    dma_reg(t, 0x06, 8);           // from tile row 1
    dma_reg(t, 0x07, 2);           // two lines
    dma_reg(t, 0x08, 4);           // first tile: 4 bytes
    dma_reg(t, 0x09, 4);           // last tile: 4 bytes
    arm(t, 0, 0, 4, 2, 5, 0x80);
    dma_reg(t, 0x0B, 0x3);
    CHECK_EQ(t.pixel(0, 0), 0x10003u);
    CHECK_EQ(t.pixel(1, 0), 0x30002u);
    CHECK_EQ(t.pixel(2, 0), 0x30003u);
    CHECK_EQ(t.pixel(3, 0), 0x00002u);
    CHECK_EQ(t.pixel(0, 1), 0x10005u);
    CHECK_EQ(t.pixel(1, 1), 0x30004u);
    CHECK_EQ(t.pixel(2, 1), 0x30005u);
    CHECK_EQ(t.pixel(3, 1), 0x00004u);
}

namespace {

u32 fbits(float f) {
    u32 w;
    std::memcpy(&w, &f, 4);
    return w;
}

// A GE token through the command FIFO: command word, then its data words.
void tok(Rig& t, u32 cmd, std::initializer_list<u32> data = {}) {
    t.w(kFifo, (cmd << 8) | static_cast<u32>(4 * data.size()));
    for (u32 d : data) t.w(kFifo, d);
}

// A context switch to `slot` whose window has its lower left at framebuffer
// (ox, oy) and is w x h (screen mask 1 its rectangle), drawing 24-bit
// buffer A at page 0x240.
void gl_context(Rig& t, u32 slot, bool first, int ox = 100, int oy = 200, int w = 64, int h = 64) {
    t.w(0x7000C, (1u << 19) | (1u << 6));
    t.w(0x50050, 0x100);
    std::array<u32, 63> img{};
    img[0] = slot;
    img[1] = first ? 0x80000000u : 0;
    img[2] = static_cast<u32>(ox) | static_cast<u32>(oy) << 16;
    img[11] = static_cast<u32>(ox) << 16 | static_cast<u32>(ox + w - 1);  // mask 1 (words 11-12)
    img[12] = static_cast<u32>(oy) << 16 | static_cast<u32>(oy + h - 1);
    img[13] = 0x240;
    img[16] = img[17] = 1;
    for (u32 v : img) t.w(kFifo, v);
    CHECK(t.r(0x70008) & (1u << 6));
}

void gl_rgb24(Rig& t, int w = 64, int h = 64) {
    tok(t, 0x009);                           // INIT_RGB
    tok(t, 0x09A, {0x2200});                 // INIT_FORMAT_VALUES: 24-bit
    tok(t, 0x049, {1, 1, 0});                // DRAW_BUFFER: A
    tok(t, 0x033, {0, 0, static_cast<u32>(w), static_cast<u32>(h)});  // VIEWPORT
}

void vertex(Rig& t, float x, float y, float z = 0) { tok(t, 0x000, {fbits(x), fbits(y), fbits(z), fbits(1)}); }
void colour(Rig& t, float r, float g, float b) { tok(t, 0x002, {fbits(r), fbits(g), fbits(b), fbits(1)}); }

u32 gl_pixel(Rig& t, int x, int y) { return static_cast<u32>(t.board.pixel_word(0x240, 100 + x, 200 + y)) & 0xFFFFFF; }

}  // namespace

// libGLcore's triangle: colour and vertex tokens between BEGIN and END,
// transformed through the (identity) matrices to the viewport, drawn into
// the context's window with the colours interpolated.
TEST(impact_gl_triangle_lands_in_the_window_smooth_shaded) {
    Rig t;
    gl_context(t, 0x1000, true);
    gl_rgb24(t);
    tok(t, 0x01A);  // BEGIN_TRIANGLES
    colour(t, 1, 0, 0), vertex(t, -1, -1);
    colour(t, 0, 1, 0), vertex(t, 1, -1);
    colour(t, 0, 0, 1), vertex(t, -1, 1);
    tok(t, 0x024);  // END_TRIANGLES
    const u32 corner = gl_pixel(t, 0, 0);  // nearly all red at the first corner (R in bits 7:0)
    CHECK((corner & 0xFF) >= 0xF0 && ((corner >> 8) & 0xFF) < 0x10 && (corner >> 16) < 0x10);
    const u32 mid = gl_pixel(t, 20, 20);
    CHECK((mid & 0xFF) > 0x20 && ((mid >> 8) & 0xFF) > 0x20 && ((mid >> 16) & 0xFF) > 0x20);
    CHECK_EQ(gl_pixel(t, 60, 60), 0u);  // outside the triangle
    CHECK_EQ(t.board.pixel_word(0x240, 99, 200) & 0xFFFFFF, 0u);  // left of the window
}

// A triangle far larger than its window: what lies in the window is drawn,
// up to its long edge (x + y = 64 in window pixels), and nothing outside.
TEST(impact_gl_large_triangle_keeps_to_the_window) {
    Rig t;
    gl_context(t, 0x1000, true);
    gl_rgb24(t);
    tok(t, 0x01A);
    colour(t, 1, 1, 1), vertex(t, -3, -3);
    colour(t, 1, 1, 1), vertex(t, 3, -3);
    colour(t, 1, 1, 1), vertex(t, -3, 3);
    tok(t, 0x024);
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 64; ++x) {
            if (x + y + 1 == 64) continue;  // on the edge
            CHECK_EQ(gl_pixel(t, x, y), x + y + 1 < 64 ? 0xFFFFFFu : 0u);
        }
    for (int k = -1; k <= 64; ++k) {
        CHECK_EQ(gl_pixel(t, -1, k), 0u);
        CHECK_EQ(gl_pixel(t, k, -1), 0u);
    }
}

// Depth: with the test on (LESS), the nearer of two overlapping triangles
// wins whatever the order; the clear sets the buffer to its value.
TEST(impact_gl_depth_test_keeps_the_nearer_surface) {
    Rig t;
    gl_context(t, 0x1000, true);
    gl_rgb24(t);
    tok(t, 0x0BC, {0xFFFFFF});  // CLEAR_DEPTH
    tok(t, 0x04A);              // clear the depth buffer
    tok(t, 0x066, {1});         // depth test on
    auto quad = [&](float z, float r, float g) {
        tok(t, 0x01D);  // BEGIN_QUADS
        colour(t, r, g, 0);
        vertex(t, -1, -1, z), vertex(t, 1, -1, z), vertex(t, 1, 1, z), vertex(t, -1, 1, z);
        tok(t, 0x027);
    };
    quad(-0.5f, 1, 0);  // near, red
    quad(0.5f, 0, 1);   // far, green: hidden
    CHECK_EQ(gl_pixel(t, 30, 30), 0x0000FFu);
    tok(t, 0x0BA, {fbits(0), fbits(0), fbits(1), fbits(1)});  // CLEAR_COLOR blue
    tok(t, 0x015);                                            // glClear's colour part
    CHECK_EQ(gl_pixel(t, 30, 30), 0xFF0000u);
    CHECK_EQ(t.board.pixel_word(0x240, 100 + 64, 200) & 0xFFFFFF, 0u);  // the clear keeps to the window
}

// SCHEDULE_SWAP raises command-processor flag 0 (bit 10); a state readback
// (0xA2) answers in GE readback low and raises flag 17.
// glPolygonOffset (0xE7: factor, units, 1.1) with GL_POLYGON_OFFSET_FILL
// (bit 0 of 0xE8): a second quad at the same depth passes LESS once pulled
// nearer, and fails without the offset.
TEST(impact_gl_polygon_offset_pulls_a_coplanar_quad_nearer) {
    Rig t;
    gl_context(t, 0x1000, true);
    gl_rgb24(t);
    tok(t, 0x0BC, {0xFFFFFF});
    tok(t, 0x04A);
    tok(t, 0x066, {1});
    auto quad = [&](float r, float g, float b) {
        tok(t, 0x01D);
        colour(t, r, g, b);
        vertex(t, -1, -1, 0), vertex(t, 1, -1, 0), vertex(t, 1, 1, 0), vertex(t, -1, 1, 0);
        tok(t, 0x027);
    };
    quad(1, 0, 0);
    tok(t, 0x0E7, {fbits(0), fbits(-2), fbits(1.1f)});
    quad(0, 1, 0);  // offset off: hidden
    CHECK_EQ(gl_pixel(t, 30, 30), 0x0000FFu);
    tok(t, 0x0E8, {2});  // lines only
    quad(0, 1, 0);
    CHECK_EQ(gl_pixel(t, 30, 30), 0x0000FFu);
    tok(t, 0x0E8, {1});  // fill
    quad(0, 1, 0);
    CHECK_EQ(gl_pixel(t, 30, 30), 0x00FF00u);
    tok(t, 0x0E7, {fbits(0), fbits(2), fbits(1.1f)});  // pushed away again
    quad(0, 0, 1);
    CHECK_EQ(gl_pixel(t, 30, 30), 0x00FF00u);
}

// Edge flags (0x0F true, 0x10 false): in line and point polygon modes a
// separate triangle's edge is drawn only when the vertex starting it was
// flagged; strips draw every edge.
TEST(impact_gl_edge_flags_hide_polygon_edges) {
    Rig t;
    gl_context(t, 0x1000, true);
    gl_rgb24(t);
    auto lit = [&](int x, int y) {  // anything drawn at (x, y) or next to it
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
                if (gl_pixel(t, x + dx, y + dy)) return true;
        return false;
    };
    auto diagonal = [&] {  // the edge from (48, 16) to (16, 48)
        for (int x = 22; x <= 42; ++x)
            if (lit(x, 64 - x)) return true;
        return false;
    };
    auto triangle = [&](u32 begin, u32 end) {
        tok(t, begin);
        colour(t, 1, 1, 1);
        tok(t, 0x00F), vertex(t, -0.5f, -0.5f);
        tok(t, 0x010), vertex(t, 0.5f, -0.5f);
        tok(t, 0x00F), vertex(t, -0.5f, 0.5f);
        tok(t, end);
    };
    tok(t, 0x0A8, {0x408, 0x1B01});  // POLYGON_MODE: both faces, lines
    triangle(0x01A, 0x024);          // triangles
    CHECK(lit(32, 16));               // from the first vertex
    CHECK(lit(16, 32));               // from the third
    CHECK(!diagonal());               // from the second, unflagged
    triangle(0x01B, 0x025);          // a strip
    CHECK(diagonal());
    tok(t, 0x0BA, {fbits(0), fbits(0), fbits(0), fbits(0)});
    tok(t, 0x015);
    CHECK(!lit(48, 16));
    tok(t, 0x04D, {fbits(3)});
    tok(t, 0x0A8, {0x408, 0x1B00});  // points
    triangle(0x01A, 0x024);
    CHECK(lit(16, 16));
    CHECK(lit(16, 48));
    CHECK(!lit(48, 16));
}

// Antialiased points and lines: coverage scales alpha, so with
// SRC_ALPHA / ONE_MINUS_SRC_ALPHA blending the inside is solid and the
// edges partly drawn; aliased ones stay square and hard.
TEST(impact_gl_smooth_points_and_lines_blend_their_edges) {
    Rig t;
    gl_context(t, 0x1000, true);
    gl_rgb24(t);
    tok(t, 0x03F, {0x302, 0x303});
    tok(t, 0x065, {1});
    tok(t, 0x04D, {fbits(8)});  // point size 8
    tok(t, 0x06C, {1});         // GL_POINT_SMOOTH
    tok(t, 0x016);
    colour(t, 1, 1, 1);
    vertex(t, -0.5f, -0.5f);    // window (16, 16)
    tok(t, 0x020);
    CHECK_EQ(gl_pixel(t, 15, 15), 0xFFFFFFu);
    CHECK_EQ(gl_pixel(t, 12, 12), 0u);  // the square's corner is outside the disc
    const u32 edge = gl_pixel(t, 12, 13) & 0xFF;
    CHECK(edge > 0 && edge < 0xFF);
    tok(t, 0x06C, {0});
    tok(t, 0x016);
    vertex(t, 0.5f, -0.5f);     // window (48, 16): a hard 8 x 8 square
    tok(t, 0x020);
    CHECK_EQ(gl_pixel(t, 44, 12), 0xFFFFFFu);
    CHECK_EQ(gl_pixel(t, 51, 19), 0xFFFFFFu);
    CHECK_EQ(gl_pixel(t, 52, 19), 0u);
    tok(t, 0x069, {1});         // GL_LINE_SMOOTH
    tok(t, 0x045, {fbits(3)});  // width 3
    tok(t, 0x017);
    vertex(t, -1, 0.5f), vertex(t, 1, 0.5f);  // along y = 48
    tok(t, 0x021);
    CHECK_EQ(gl_pixel(t, 30, 48), 0xFFFFFFu);
    CHECK_EQ(gl_pixel(t, 30, 47), 0xFFFFFFu);
    const u32 side = gl_pixel(t, 30, 49) & 0xFF;  // centre 48.0 + 1.5: half of row 49
    CHECK(side > 0x40 && side < 0xC0);
    CHECK_EQ(gl_pixel(t, 30, 50), 0u);
}

TEST(impact_gl_swap_flag_and_state_readback) {
    Rig t;
    gl_context(t, 0x1000, true);
    t.w(kFlagClear, ~0u);
    tok(t, 0x037);
    CHECK(t.r(kFlagSet) & (1u << 10));
    colour(t, 0.25f, 0.5f, 0.75f);
    tok(t, 0x0A2, {5, 1, 1});  // the current colour's green
    CHECK(t.r(kFlagSet) & (1u << 17));
    CHECK_EQ(t.r(0x70014), fbits(0.5f));
    tok(t, 0x0A2, {0x1FC, 1, 1});  // dither on by default (glFinish reads it)
    CHECK_EQ(t.r(0x70014) & 1u, 1u);
}

// Each context keeps its GL state across switches (the GE saves it into the
// context's ERAM slot), and a first load starts from OpenGL's initial state.
TEST(impact_gl_state_follows_its_context) {
    Rig t;
    gl_context(t, 0x1000, true);
    colour(t, 0.25f, 0, 0);
    gl_context(t, 0x3000, true);
    tok(t, 0x0A2, {4, 1, 1});
    CHECK_EQ(t.r(0x70014), fbits(1.0f));  // B's colour: the initial white
    colour(t, 0.75f, 0, 0);
    gl_context(t, 0x1000, false);
    tok(t, 0x0A2, {4, 1, 1});
    CHECK_EQ(t.r(0x70014), fbits(0.25f));
    gl_context(t, 0x3000, false);
    tok(t, 0x0A2, {4, 1, 1});
    CHECK_EQ(t.r(0x70014), fbits(0.75f));
}

// ERAM pass-through: words the kernel writes into a slot (restoring an
// evicted context) are what that slot loads.
TEST(impact_gl_eram_write_restores_a_context_record) {
    Rig t;
    gl_context(t, 0x1000, true);
    colour(t, 0.5f, 0, 0);
    gl_context(t, 0x3000, true);  // A's record is now in its slot
    gl_context(t, 0x1000, false);
    tok(t, 0x0A2, {4, 1, 1});
    CHECK_EQ(t.r(0x70014), fbits(0.5f));
    // An ERAM write of zeros over slot 0x1000 leaves a record that starts
    // afresh when the slot is next loaded.
    gl_context(t, 0x3000, false);
    tok(t, 0x0FD, {16});
    t.w(kFifo, 0x80000000u | 64);  // 64 bytes of pixel data: 16 zero words
    for (unsigned i = 0; i < 16; ++i) t.w(kFifo, 0);
    tok(t, 0x0FB, {0x1000, 16});
    tok(t, 0x0FA);
    gl_context(t, 0x1000, false);
    tok(t, 0x0A2, {4, 1, 1});
    CHECK_EQ(t.r(0x70014), fbits(1.0f));
}

// glBitmap: set bits drawn in the raster colour at the raster position,
// rows bottom first, then the raster position moves.
TEST(impact_gl_bitmap_draws_at_the_raster_position) {
    Rig t;
    gl_context(t, 0x1000, true);
    gl_rgb24(t);
    colour(t, 0, 1, 0);
    tok(t, 0x038, {fbits(-1), fbits(-1), fbits(0), fbits(1)});  // RASTER_POS4F: the window's corner
    tok(t, 0x094, {0, 8, 2, fbits(0), fbits(0), fbits(8), fbits(0), 1});  // 8 x 2, move 8 right, one data word
    t.w(kFifo, 0x80000000u | 2);  // the rows: 11110000, 00001111
    t.w(kFifo, 0xF00F0000u);
    t.w(kFifo, 0);
    CHECK_EQ(gl_pixel(t, 0, 0), 0x00FF00u);
    CHECK_EQ(gl_pixel(t, 4, 0), 0u);
    CHECK_EQ(gl_pixel(t, 4, 1), 0x00FF00u);
    CHECK_EQ(gl_pixel(t, 0, 1), 0u);
    tok(t, 0x0A2, {0x2CD, 1, 1});  // the raster position moved 8 right
    CHECK_EQ(t.r(0x70014), fbits(8 + 49151.5f));
}

// glDrawPixels(GL_RGBA, GL_UNSIGNED_BYTE) as IRIX 6.5's libGLcore sends it:
// the transfer mode in a register list, the GE pixel pipeline (int to float,
// colour matrix, clamp, pipe control: entry points from this release's
// microcode, its ERAM state 0x20 below 6.5.22's), the image rectangle, then
// SEND_PIXELS and the rows.
TEST(impact_gl_draw_pixels_through_the_pipeline) {
    Rig t;
    gl_context(t, 0x1000, true);
    gl_rgb24(t);
    tok(t, 0x038, {fbits(0), fbits(0), fbits(0), fbits(1)});  // raster position: the window's centre (32, 32)
    auto block = [&](u32 token, u32 addr, std::initializer_list<u32> v) {
        std::vector<u32> d = {1, 2, static_cast<u32>(4 * v.size()), addr, 0, 2, static_cast<u32>(v.size())};
        d.insert(d.end(), v);
        t.w(kFifo, (token << 8) | static_cast<u32>(4 * d.size()));
        for (u32 x : d) t.w(kFifo, x);
    };
    tok(t, 0x07F, {1, 2});  // RSS_REG_SET (one pair), the pairs as pixel data
    t.w(kFifo, 0x80000000u | 8);
    t.w(kFifo, 0x159), t.w(kFifo, 0xC00080);  // XFRMODE: RGBA bytes, red first
    block(0x080, 0x13B, {0x1F43, 0, 0x3E5D, 0xCE5, 0x1569, 0x1B4, 0x3525, 0xD47});  // the program
    block(0x0CD, 0xCE5, {0x34, 0x4064, 0x50C, 2, 4, fbits(1.0f / 255)});
    block(0x080, 0x1B4, {6, 4, 0, 0x50C, 0x50C});
    block(0x0CD, 0xD47, {0x50C, 0x1A000, 0x50C, 2, 4, fbits(255), fbits(255), fbits(255), fbits(255), fbits(255), fbits(255),
                         fbits(255), fbits(255), fbits(255), fbits(255), fbits(255), fbits(255)});
    block(0x0CD, 0xD88, {0, 0, 1, 0, 0x10002});  // the rectangle: 2 x 1 from the raster position
    block(0x0CD, 0xD97, {0, 0, 0, 0, 1, 0});     // host elements: unsigned bytes
    tok(t, 0x08D, {2, 0, 0, 1, 0, 1, 0x49C0, 0});  // SEND_PIXELS: one row of two words, drawn
    t.w(kFifo, 0x80000000u | 8);
    t.w(kFifo, 0x11223344u);  // RGBA
    t.w(kFifo, 0xAABBCCFFu);
    CHECK_EQ(gl_pixel(t, 32, 32), 0x332211u);
    CHECK_EQ(gl_pixel(t, 33, 32), 0xCCBBAAu);
}

// A display list: a jump from the FIFO runs segments from host memory
// through the display-list page table; a segment's link (a push of a
// return segment and a jump to a called one; a return) runs after its
// body, and the FIFO resumes when no segment is left.
TEST(impact_display_list_runs_segments_and_returns) {
    Rig t;
    std::vector<u8> ram(1u << 20);
    Bus bus;
    bus.map_ram(0, ram.size(), ram.data());
    t.board.set_bus(&bus);
    gl_context(t, 0x1000, true);
    store_be<u32>(&ram[0x10000], 0x20);  // display-list page 0 -> frame 0x20
    std::size_t at = 0x20000;
    auto put = [&](u32 w) { store_be<u32>(&ram[at], w), at += 4; };
    // Segment A at 0: link (push C at 0x200, jump to B at 0x100), then COLOR4F 0.25.
    put((0x837u << 8) | 8), put(0x200), put(5);
    put((0x80Cu << 8) | 8), put(0x100), put(8);
    put((0x002u << 8) | 16), put(fbits(0.25f)), put(0), put(0), put(fbits(1));
    // Segment B: link (return: to the pushed segment), a COLOR4F 0.5.
    at = 0x20100;
    put((0x839u << 8) | 8), put(0), put(0);
    put((0x002u << 8) | 16), put(fbits(0.5f)), put(fbits(0.75f)), put(0), put(fbits(1));
    // Segment C: green 0.125.
    at = 0x20200;
    put((0x002u << 8) | 16), put(fbits(0.5f)), put(fbits(0.125f)), put(0), put(fbits(1));
    auto dma8 = [&](u32 reg, u32 hi, u32 lo) {
        t.w(kFifo, ((0x800u | reg) << 8) | 8);
        t.w(kFifo, hi);
        t.w(kFifo, lo);
    };
    dma8(0x28, 0, 0x10000);  // the display-list page table
    dma8(0x0C, 0, 11);       // jump to A (11 words)
    tok(t, 0x0A2, {5, 1, 1});
    CHECK_EQ(t.r(0x70014), fbits(0.125f));  // C ran last
    tok(t, 0x0A2, {4, 1, 1});
    CHECK_EQ(t.r(0x70014), fbits(0.5f));
}

// A texture image through the texture loader, then a quad drawn with it:
// texture registers from a register list and the indexed shadow token,
// the destination page and the sub-image from the GE pixel state, the
// texels by SEND_PIXELS with a texture-load routine; nearest sampling
// modulates the (white) fragment colour. Pages are relative to the
// context's TRAM base.
TEST(impact_gl_texture_load_and_textured_quad) {
    Rig t;
    gl_context(t, 0x1000, true);
    gl_rgb24(t);
    auto block = [&](u32 addr, std::initializer_list<u32> v) {
        std::vector<u32> d = {1, 2, static_cast<u32>(4 * v.size()), addr, 0, 2, static_cast<u32>(v.size())};
        d.insert(d.end(), v);
        t.w(kFifo, (0x0CDu << 8) | static_cast<u32>(4 * d.size()));
        for (u32 x : d) t.w(kFifo, x);
    };
    auto triples = [&](std::initializer_list<u32> v) {  // RSS_REG_SHADOW: (register, value, shadow) as pixel data
        tok(t, 0x07C, {1, 4});
        std::vector<u32> d(v);
        if (d.size() & 1) d.push_back(0);
        t.w(kFifo, 0x80000000u | static_cast<u32>(4 * d.size()));
        for (u32 x : d) t.w(kFifo, x);
    };
    // TL_MODE: four 8-bit components; TL_SPEC: 2 x 2; TEXMODE1: on,
    // modulate, RGBA; TEXMODE2: 8-bit, nearest; TXSIZE 2 x 2; TXADDR 0.
    triples({0x1A1, 0x34E26, 0x27E, 0x1A2, 0x110, 0x280, 0x111, 0x819, 0x1F8, 0x180, 0x4, 0x262, 0x181, 0x11, 0x264,
             0x190, 0, 0xBA6});
    tok(t, 0x0EC, {0xBA6, 0xBA7, 0x191, 3});  // TXMIPMAP[0]: page 3
    tok(t, 0x07F, {1, 2});
    t.w(kFifo, 0x80000000u | 8);
    t.w(kFifo, 0x159), t.w(kFifo, 0xC00080);  // XFRMODE: RGBA bytes
    block(0xD88, {0, 0, 2, 2, 0x20002});          // the sub-image: 2 x 2 at (0, 0)
    block(0xD91, {2, 0, 0, 3, 4, 0});             // level 0 at page 3, border page 4
    block(0xCEA, {fbits(1.0f / 255)});            // 8-bit components
    tok(t, 0x08D, {2, 0, 0, 2, 0, 1, 0x5114, 0});  // SEND_PIXELS: two rows of two texels, a texture load
    t.w(kFifo, 0x80000000u | 16);
    for (u32 x : {0xFF0000FFu, 0x00FF00FFu, 0x0000FFFFu, 0xFFFFFFFFu}) t.w(kFifo, x);
    tok(t, 0x00B);  // TEX_ON
    tok(t, 0x01D);  // BEGIN_QUADS
    colour(t, 1, 1, 1);
    auto tc = [&](float s, float u) { tok(t, 0x008, {fbits(s), fbits(u)}); };
    tc(0, 0), vertex(t, -1, -1);
    tc(1, 0), vertex(t, 1, -1);
    tc(1, 1), vertex(t, 1, 1);
    tc(0, 1), vertex(t, -1, 1);
    tok(t, 0x027);
    CHECK_EQ(gl_pixel(t, 10, 10), 0x0000FFu);  // red (R in bits 7:0)
    CHECK_EQ(gl_pixel(t, 50, 10), 0x00FF00u);
    CHECK_EQ(gl_pixel(t, 10, 50), 0xFF0000u);
    CHECK_EQ(gl_pixel(t, 50, 50), 0xFFFFFFu);
    // TEX_BASE (0xEB): the context moved to TRAM page 0x20. The same
    // relative page now holds other texels, and the first ones are still
    // there at base 0.
    tok(t, 0x0EB, {0x20});
    tok(t, 0x08D, {2, 0, 0, 2, 0, 1, 0x5114, 0});
    t.w(kFifo, 0x80000000u | 16);
    for (int k = 0; k < 4; ++k) t.w(kFifo, 0x808080FFu);
    // Notes with no texture base in them leave it where it is.
    for (u32 c : {0x001u, 0x006u, 0x096u, 0x097u, 0x0F2u, 0x102u}) tok(t, c);
    tok(t, 0x08C, {1});
    tok(t, 0x01D);
    colour(t, 1, 1, 1);
    tc(0, 0), vertex(t, -1, -1), tc(1, 0), vertex(t, 1, -1), tc(1, 1), vertex(t, 1, 1), tc(0, 1), vertex(t, -1, 1);
    tok(t, 0x027);
    CHECK_EQ(gl_pixel(t, 10, 10), 0x808080u);
    tok(t, 0x0EB, {0});
    tok(t, 0x01D);
    tc(0, 0), vertex(t, -1, -1), tc(1, 0), vertex(t, 1, -1), tc(1, 1), vertex(t, 1, 1), tc(0, 1), vertex(t, -1, 1);
    tok(t, 0x027);
    CHECK_EQ(gl_pixel(t, 10, 10), 0x0000FFu);
    tok(t, 0x00C);  // TEX_OFF: untextured again
    tok(t, 0x01D);
    colour(t, 0, 1, 0);
    vertex(t, -1, -1), vertex(t, 1, -1), vertex(t, 1, 1), vertex(t, -1, 1);
    tok(t, 0x027);
    CHECK_EQ(gl_pixel(t, 10, 10), 0x00FF00u);
}

// libGLcore's restore of saved TRAM: XFRCONTROL to the texture side (5:
// DMA, texture) arms the loader, and host DMA bytes become texels of the
// level TL_SPEC describes, as one stream (here 2 x 2 in one DMA line), at
// TL_MIPMAP's page past the context's TRAM base. The save reads them back.
TEST(impact_texture_load_by_host_dma) {
    Rig t;
    std::vector<u8> ram(1u << 20);
    Bus bus;
    bus.map_ram(0, ram.size(), ram.data());
    t.board.set_bus(&bus);
    gl_context(t, 0x1000, true);
    gl_rgb24(t);
    tok(t, 0x0EB, {0x10});  // TEX_BASE
    store_be<u32>(&ram[0x10000], 0x20);
    const u32 texels[4] = {0xFF0000FFu, 0x00FF00FFu, 0x0000FFFFu, 0xFFFFFFFFu};
    for (u32 i = 0; i < 4; ++i) store_be<u32>(&ram[0x20000 + 4 * i], texels[i]);
    t.reg(0x1A1, 0x34E26);  // TL_MODE: RGBA, 8-bit
    t.reg(0x1A2, 0x110);    // TL_SPEC: 2 x 2
    t.reg(0x1A4, 3);        // TL_MIPMAP: page 3 (TRAM page 0x13)
    t.reg(0x1A5, 4);
    t.reg(0x1AB, 0);
    t.reg(0x1AC, 0);
    t.reg(0x159, 0xC00080);  // XFRMODE: RGBA bytes
    t.reg(0x102, 5);         // XFRCONTROL: DMA to the texture side
    dma_reg(t, 0x22, 0);
    dma_reg(t, 0x23, 0x10000);
    dma_reg(t, 0x00, 0x80000000);
    dma_reg(t, 0x04, 16);
    dma_reg(t, 0x05, 0);
    dma_reg(t, 0x06, 0);
    dma_reg(t, 0x07, 1);
    dma_reg(t, 0x08, 16);
    dma_reg(t, 0x0B, 0x3);  // run, pool 1, host to board
    t.reg(0x102, 0);
    // Sample it: TEXMODE1 on, modulate, RGBA; TEXMODE2 8-bit nearest; TXSIZE 2 x 2; TXMIPMAP[0] page 3.
    tok(t, 0x07C, {1, 4});
    const u32 regs[] = {0x111, 0x819, 0x1F8, 0x180, 0x4, 0x262, 0x181, 0x11, 0x264, 0x190, 0, 0xBA6};
    t.w(kFifo, 0x80000000u | static_cast<u32>(sizeof regs));
    for (u32 x : regs) t.w(kFifo, x);
    tok(t, 0x0EC, {0xBA6, 0xBA7, 0x191, 3});
    tok(t, 0x00B);
    tok(t, 0x01D);
    colour(t, 1, 1, 1);
    auto tc = [&](float s, float u) { tok(t, 0x008, {fbits(s), fbits(u)}); };
    tc(0, 0), vertex(t, -1, -1), tc(1, 0), vertex(t, 1, -1), tc(1, 1), vertex(t, 1, 1), tc(0, 1), vertex(t, -1, 1);
    tok(t, 0x027);
    CHECK_EQ(gl_pixel(t, 10, 10), 0x0000FFu);
    CHECK_EQ(gl_pixel(t, 50, 10), 0x00FF00u);
    CHECK_EQ(gl_pixel(t, 10, 50), 0xFF0000u);
    CHECK_EQ(gl_pixel(t, 50, 50), 0xFFFFFFu);
    // Save and restore go through a view of whole pages as a level 64
    // texels wide (TL_SPEC / TXSIZE 64 x 8192): restore four texels to page
    // 7, then save them: a raw read of the sampler's pages (READ_TEXTURE_RAW:
    // rows, transfer mode, its raster start word) and the host's DMA read,
    // which runs at once with the texels waiting.
    t.reg(0x1A2, 0xD60);
    t.reg(0x1A4, 7);
    t.reg(0x102, 5);
    dma_reg(t, 0x0B, 0x3);
    t.reg(0x102, 0);
    tok(t, 0x07C, {1, 4});
    t.w(kFifo, 0x80000000u | 24);
    for (u32 x : {0x181u, 0xD6u, 0x264u, 0x190u, 0u, 0xBA6u}) t.w(kFifo, x);  // TXSIZE 64 x 8192, TXADDR 0
    tok(t, 0x0EC, {0xBA6, 0xBA7, 0x191, 7});
    tok(t, 0x0E5, {1, 0xC00080, 0x400D});
    dma_reg(t, 0x06, 0x100);
    dma_reg(t, 0x0B, 0xB);  // run, pool 1, board to host
    for (u32 i = 0; i < 4; ++i) CHECK_EQ(load_be<u32>(&ram[0x20100 + 4 * i]), texels[i]);
    // The kernel's restore (MgrasRestoreTexData): the loader registers by
    // RSS_REG_SET, then 0xD8 with the word count in 9-bit parts (4 words),
    // and the texels by host DMA into the GE; here to page 9.
    const u32 more[4] = {0x11223344u, 0x55667788u, 0x99AABBCCu, 0xDDEEFF00u};
    for (u32 i = 0; i < 4; ++i) store_be<u32>(&ram[0x20200 + 4 * i], more[i]);
    tok(t, 0x07F, {1, 2});
    t.w(kFifo, 0x80000000u | 32);
    for (u32 x : {0x1A1u, 0x34E26u, 0x1A2u, 0xD60u, 0x1A4u, 9u, 0x159u, 0xC00080u}) t.w(kFifo, x);
    tok(t, 0x0E3);
    tok(t, 0x0D8, {0, 0, 4});
    dma_reg(t, 0x06, 0x200);
    dma_reg(t, 0x0B, 0x3);
    tok(t, 0x07C, {1, 4});
    t.w(kFifo, 0x80000000u | 24);
    for (u32 x : {0x190u, 0u, 0xBA6u, 0x191u, 9u, 0xBA7u}) t.w(kFifo, x);  // TXADDR 0, TXMIPMAP[0] page 9
    tok(t, 0x0E5, {1, 0xC00080, 0x400D});
    dma_reg(t, 0x06, 0x300);
    dma_reg(t, 0x0B, 0xB);
    for (u32 i = 0; i < 4; ++i) CHECK_EQ(load_be<u32>(&ram[0x20300 + 4 * i]), more[i]);
}

// TRAM: small levels share a page without overlapping; clamped sampling
// outside a borderless texture reads the border colour; bilinear filtering
// at a texel corner averages its four neighbours; the environment's decal
// mode keeps the fragment's alpha.
TEST(impact_tram_levels_borders_and_filtering) {
    ImpactTram tram;
    GeTexLoad l;
    l.mode = 0x34E26;  // RGBA, 8-bit
    l.page = 2;
    auto load = [&](unsigned lg, u8 v) {
        l.spec = lg << 8 | lg << 4;
        const unsigned n = 1u << lg;
        std::vector<u8> row(4 * n, v);
        for (unsigned r = 0; r < n; ++r) tram.load_line(l, r, row.data(), n);
    };
    load(2, 0x40);  // 4 x 4
    load(1, 0x80);  // 2 x 2, same page
    GeTexture tx;
    tx.mode1 = 0x819;
    tx.mode2 = 0x4 | 1u << 11 | 1u << 12 | 1u << 16;  // 8-bit, clamp S and T, no border
    tx.bcolour_rg = 0xFFF;                            // border red 1
    tx.size = 0x22;
    tx.pages[0] = 2;
    CHECK(std::abs(tram.sample(tx, 0.5f, 0.5f, 0)[0] - 0x40 / 255.0f) < 1e-4f);
    tx.size = 0x11;
    CHECK(std::abs(tram.sample(tx, 0.5f, 0.5f, 0)[0] - 0x80 / 255.0f) < 1e-4f);
    CHECK_EQ(tram.texel(tx, 0, -1, 0)[0], 1.0f);  // outside: the border colour
    CHECK_EQ(tram.texel(tx, 0, -1, 0)[1], 0.0f);
    // A 2 x 2 checker of 0 and 1, sampled bilinearly at its centre.
    l.spec = 0x110;
    const u8 r0[8] = {0, 0, 0, 0, 255, 255, 255, 255}, r1[8] = {255, 255, 255, 255, 0, 0, 0, 0};
    tram.load_line(l, 0, r0, 2);
    tram.load_line(l, 1, r1, 2);
    tx.mode2 = 0x4 | 1u << 7;  // magnification linear, repeat
    CHECK(std::abs(tram.sample(tx, 0.5f, 0.5f, -1)[0] - 0.5f) < 1e-3f);
    // Read back: RGBA bytes, then the raw page view (64-texel rows).
    const auto row = tram.read_line(tx, 0x80, 1, 2);
    CHECK(row.size() == 8 && row[0] == 0xFF && row[3] == 0xFF && row[4] == 0);
    l.spec = 0x6D60;  // a 64 x 8192 view of the pages, as the texture manager restores them
    std::vector<u8> raw(64 * 4);
    for (unsigned k = 0; k < raw.size(); ++k) raw[k] = static_cast<u8>(k);
    l.page = 9;
    tram.load_line(l, 0, raw.data(), 64);
    tx.pages[0] = 9;
    CHECK(tram.read_raw(tx, 0x80, 0, 64) == raw);
    tx.mode1 = 0x819 | 1u << 1;  // decal
    const auto o = ImpactTram::environment(tx, {0.2f, 0.2f, 0.2f, 0.3f}, {1, 0, 0, 1});
    CHECK_EQ(o[0], 1.0f);
    CHECK_EQ(o[3], 0.3f);
}

// SET_DATA: a GE memory word libGLcore sets reads back through the state
// readback (0xA2: address in the low answer).
TEST(impact_gl_set_data_reads_back) {
    Rig t;
    gl_context(t, 0x1000, true);
    gl_rgb24(t);
    tok(t, 0x0ED, {0xB6A, 0x405});
    tok(t, 0x0A2, {0xB6A, 1, 1});
    CHECK_EQ(t.r(0x70014), 0x405u);
}

// Only display-control writes that change what is shown make the picture out
// of date: the X server sets the XMAP's index every frame to poll it.
TEST(impact_display_changes_only_with_shown_state) {
    Rig t;
    u64 v = t.board.display_version();
    t.w(dcb(7, 1, 1), 8u << 24);  // XMAP index
    CHECK_EQ(t.board.display_version(), v);
    t.w(dcb(7, 1, 1), 0u << 24);
    t.w(dcb(7, 4, 1), 0x05u << 24);  // main mode byte 0
    CHECK(t.board.display_version() != v);
    v = t.board.display_version();
    t.w(dcb(7, 1, 1), 0u << 24);
    t.w(dcb(7, 4, 1), 0x05u << 24);  // the same again
    CHECK_EQ(t.board.display_version(), v);
    auto vc3_reg = [&](unsigned i, u16 value) { t.w(dcb(8, 0, 3), (i << 24) | (u32{value} << 8)); };
    vc3_reg(2, 100);  // cursor X
    CHECK(t.board.display_version() != v);
    v = t.board.display_version();
    vc3_reg(2, 100);
    CHECK_EQ(t.board.display_version(), v);
}

TEST(impact_render_converts_only_what_changed) {
    Rig t;
    t.console_setup();
    std::vector<u32> argb, full;
    unsigned width = 0, height = 0, fw = 0, fh = 0;
    auto same = [&] {
        CHECK(t.board.render(argb, width, height));
        CHECK(t.board.render_full(full, fw, fh));
        CHECK_EQ(width, fw);
        CHECK_EQ(height, fh);
        CHECK(argb == full);
    };
    same();
    // Rows not written keep the last picture: poison one and see it stay.
    argb[0] = 0x12345678u;
    t.fill(10, 20, 4, 3, 0x0000FF);
    CHECK(t.board.render(argb, width, height));
    CHECK_EQ(argb[0], 0x12345678u);
    CHECK_EQ(argb[20 * width + 10], 0xFFFF0000u);
    argb[0] = 0xFF000000u;
    same();
    t.fill(1000, 1000, 8, 24, 0x00FF00);  // across a page row boundary, both engines
    t.fill(180, 500, 30, 1, 0xFF0000);    // across a tile boundary
    same();
    // The cursor: its old and new rows.
    t.w(dcb(3, 0, 2), 0x0100u << 16);
    t.w(dcb(3, 2, 3), 0x0000FFu << 8);
    auto vc3_reg = [&](unsigned i, u16 v) { t.w(dcb(8, 0, 3), (i << 24) | (u32{v} << 8)); };
    vc3_reg(1, 0x4000);
    vc3_reg(7, 0x4000);
    t.w(dcb(8, 3, 2), 0x8000u << 16);
    vc3_reg(2, 31 + 40);
    vc3_reg(3, 31 + 30);
    vc3_reg(0x1D, 3);
    same();
    vc3_reg(3, 31 + 300);
    same();
    vc3_reg(2, 31 + 41);
    same();
    t.w(dcb(3, 0, 2), 0x0005u << 16);  // its colours: from entry 4 on (config bits 10:0, times 4)
    t.w(dcb(3, 2, 3), 0x00FF00u << 8);
    t.w(dcb(7, 1, 1), 3u << 24);
    t.w(dcb(7, 2, 1), 1u << 24);
    same();
    vc3_reg(0x1D, 0);
    same();
    // A colour map change redraws everything.
    t.w(0x61E00, 0x7A0);  // main mode 0: colour index
    same();
    t.w(dcb(3, 0, 2), 0x00FFu << 16);
    t.w(dcb(3, 2, 3), 0x00FF00u << 8);
    same();
}
