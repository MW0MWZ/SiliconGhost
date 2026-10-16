// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Newport graphics tests, driven through the REX3 register window.
#include <vector>

#include "core/scheduler.h"
#include "machine/ip22/newport.h"
#include "test.h"

using namespace sg;
using namespace sg::ip22;

namespace {

constexpr u32 kGo = 0x800;
constexpr u32 kDrawMode1 = 0x000, kDrawMode0 = 0x004, kXStart = 0x100, kWrMask = 0x220, kColorI = 0x224;
constexpr u32 kHostRw0 = 0x230, kDcbMode = 0x238, kDcbData0 = 0x240, kXStartI = 0x148, kXYStartI = 0x150;
constexpr u32 kXYEndI = 0x154, kTopScan = 0x1320, kUStatus = 0x133C, kXYWin = 0x1324;

// DCBMODE fields
constexpr u32 width(u32 n) { return n & 3; }
constexpr u32 kCrsInc = 1u << 3;
constexpr u32 crs(u32 c) { return c << 4; }
constexpr u32 slave(u32 a) { return a << 7; }
constexpr u32 kSwapEndian = 1u << 28;

struct Rig {
    u64 now = 0;
    Scheduler sched;
    Newport np;
    // CLIPMODE resets with no CID codes selected: allow them all, as drivers do.
    explicit Rig(Newport::Options o = {}) : np((sched.set_clock([this] { return now; }), sched), 150.0e6, o) { w(0x1328, 0x1E00); }
    void w(u32 off, u32 v) { CHECK(np.write(off, 4, v)); }
    u32 r(u32 off) {
        u64 v = 0;
        CHECK(np.read(off, 4, v));
        return static_cast<u32>(v);
    }
    void vc2_reg(u32 index, u32 value) {
        w(kDcbMode, width(3) | kCrsInc | crs(0) | slave(0));
        w(kDcbData0, (index << 24) | (value << 8));
    }
    void vc2_ram(u32 addr, std::initializer_list<u16> words) {
        vc2_reg(7, addr);
        w(kDcbMode, width(2) | crs(3) | slave(0));
        for (u16 v : words) w(kDcbData0, u32{v} << 16);
    }
    void xmap_mode(u32 index, u32 mode) {
        w(kDcbMode, width(0) | crs(5) | slave(4));
        w(kDcbData0, (index << 24) | mode);
    }
    void cmap_entry(u32 index, u32 rgb) {
        w(kDcbMode, width(2) | kCrsInc | crs(0) | slave(1) | kSwapEndian);
        w(kDcbData0, index << 16);
        w(kDcbMode, width(3) | crs(2) | slave(1));
        w(kDcbData0, rgb << 8);
    }
    // 8-bit colour-index block fill of the inclusive rectangle.
    void fill(int x0, int y0, int x1, int y1, u32 ci) {
        w(kDrawMode1, (3u << 28) | (1u << 3) | 1u | 0x7000u);  // logicop SRC, 8bpp, RGB/CI planes
        w(kWrMask, 0xFF);
        w(kColorI, ci);
        w(kXYStartI, (static_cast<u32>(x0) << 16) | static_cast<u32>(y0));
        w(kXYEndI, (static_cast<u32>(x1) << 16) | static_cast<u32>(y1));
        w(kDrawMode0 | kGo, 2u | (1u << 2) | (1u << 5) | (1u << 8) | (1u << 9));  // DRAW BLOCK DOSETUP STOPX STOPY
    }
};

}  // namespace

TEST(newport_probe_and_status) {
    Rig t;
    t.w(kXStartI, 0x12345678);
    CHECK_EQ(t.r(kXStart), 0x02B3C000u);
    CHECK_EQ(t.r(kUStatus) & 7, 3u);
    u64 v;
    CHECK(!t.np.read(0x2000, 4, v));  // outside the register window: no response
}

TEST(newport_dcb_vc2_ram_and_cmap_revision) {
    Rig t;
    t.vc2_ram(0x100, {0x1234, 0xBEEF});
    t.vc2_reg(7, 0x100);
    t.w(kDcbMode, width(2) | crs(3) | slave(0));
    CHECK_EQ(t.r(kDcbData0) >> 16, 0x1234u);
    CHECK_EQ(t.r(kDcbData0) >> 16, 0xBEEFu);
    // CMAP1 revision carries the monitor sense in bits 7:4.
    t.w(kDcbMode, width(1) | crs(6) | slave(3));
    CHECK_EQ(t.r(kDcbData0) >> 24, 0xA1u);
    u64 b = 0;
    CHECK(t.np.read(kDcbData0 + 3, 1, b));  // byte access sees the same value
    CHECK_EQ(b, 0xA1ull);
}

TEST(newport_block_fill_and_host_read) {
    Rig t;
    t.fill(10, 20, 13, 21, 0x42);
    // Read back four 8-bit pixels of the first row (packed).
    t.w(kDrawMode1, (3u << 28) | (1u << 8) | (1u << 7) | (1u << 3) | 1u | 0x7000u);
    t.w(kXYStartI, (10u << 16) | 20u);
    t.w(kXYEndI, (13u << 16) | 20u);
    t.w(kDrawMode0 | kGo, 1u);
    CHECK_EQ(t.r(kHostRw0), 0x42424242u);
    // The fill stops after the rectangle; the next row is still 0x42, outside is 0.
    t.w(kXYStartI, (12u << 16) | 21u);
    t.w(kXYEndI, (15u << 16) | 21u);
    t.w(kDrawMode0 | kGo, 1u);
    CHECK_EQ(t.r(kHostRw0), 0x42420000u);
}

TEST(newport_scanout_through_vc2_xmap_cmap) {
    Rig t;
    // 4x2 visible screen: frame table at 0x400 -> line sequence at 0 for 2
    // lines of 6 active pixel clocks, the first two of which fill the
    // display pipeline (so the screen starts at frame buffer column 2).
    t.vc2_ram(0x000, {0x8380});        // EOL, 6 pixels visible, no second word
    t.vc2_ram(0x400, {0x000, 2, 0, 0});
    t.vc2_reg(0x1F, 1);                // release soft reset
    t.vc2_reg(0x00, 0x400);            // video entry
    t.vc2_reg(0x10, 0x0006);           // display + video timing
    t.xmap_mode(0, 0x000404);          // 8bpp CI, gamma bypass
    t.cmap_entry(0x42, 0x123456);
    t.w(kTopScan, 0x3FF);              // first displayed line is frame buffer row 0
    t.w(kXYWin, 0x10001000);
    t.fill(0, 0, 1, 1, 0x17);          // the columns hidden by the pipeline
    t.w(kXYWin, 0x10021000);           // as IRIX and the PROM set it
    t.fill(0, 0, 3, 1, 0x42);
    std::vector<u32> px;
    unsigned w = 0, h = 0;
    CHECK(t.np.render(px, w, h));
    CHECK_EQ(w, 4u);
    CHECK_EQ(h, 2u);
    if (px.size() == 8) {
        CHECK_EQ(px[0], 0xFF123456u);
        CHECK_EQ(px[7], 0xFF123456u);
    }
}

TEST(newport_host_pixels_advance_per_go) {
    Rig t;
    t.w(kDrawMode1, (3u << 28) | (1u << 8) | (1u << 3) | 1u | 0x7000u);  // 8bpp host and draw depth
    t.w(kWrMask, 0xFF);
    t.w(kXYStartI, (100u << 16) | 50u);
    t.w(kXYEndI, (103u << 16) | 50u);
    t.w(kDrawMode0, 2u | (1u << 6));  // DRAW SPAN, COLORHOST, one pixel per GO
    for (u32 c : {0x11u, 0x22u, 0x33u}) t.w(kHostRw0 | kGo, c << 24);
    t.w(kDrawMode1, (3u << 28) | (1u << 8) | (1u << 7) | (1u << 3) | 1u | 0x7000u);
    t.w(kXYStartI, (100u << 16) | 50u);
    t.w(kDrawMode0 | kGo, 1u);
    CHECK_EQ(t.r(kHostRw0), 0x11223300u);
}

// IRIX probes for an Indy Presenter adapter on DCB slave 12: with nothing
// there, an acknowledged access leaves the bus busy until it is reset or
// times out.
TEST(newport_dcb_access_to_an_absent_slave_stays_busy_until_reset_or_timeout) {
    Rig t;
    constexpr u32 kBackBusy = 1u << 4, kSyncAck = 1u << 11, kDcbReset = 0x1340;
    t.w(kDcbMode, width(1) | slave(0) | kSyncAck);
    t.w(kDcbData0, 0);  // VC2 answers
    CHECK(!(t.r(kUStatus) & kBackBusy));
    t.w(kDcbMode, 0x0099b601);  // the probe's mode: slave 12, acknowledge awaited
    t.w(kDcbData0, 0);
    CHECK(t.r(kUStatus) & kBackBusy);
    CHECK(t.r(kUStatus) & kBackBusy);
    t.w(kDcbReset, 0);  // also returns DCBMODE to its reset value
    CHECK(!(t.r(kUStatus) & kBackBusy));
    CHECK_EQ(t.r(kDcbMode), 0x780u);
    t.w(kDcbMode, 0x0099b601);
    t.w(kDcbData0, 0);
    CHECK(t.r(kUStatus) & kBackBusy);
    t.now += 150 * 20;  // 20 us at 150 MHz
    CHECK(!(t.r(kUStatus) & kBackBusy));
}

// A text glyph as NetBSD draws it: one block-mode row per ZPATTERN write
// with GO, opaque, so clear bits take the background colour.
TEST(newport_zpattern_rows_draw_opaque_glyph) {
    Rig t;
    constexpr u32 kZPattern = 0x014, kColorBack = 0x018;
    t.w(kDrawMode0, 2u | (1u << 2) | (1u << 8) | (1u << 12) | (1u << 16));  // DRAW BLOCK STOPX ZPATTERN OPAQUE
    t.w(kDrawMode1, 0x30000000u | 0x7000u | 0x100u | 0x80u | 0x8u | 1u);
    t.w(kXYStartI, (8u << 16) | 16u);
    t.w(kXYEndI, 11u << 16);
    t.w(kColorI, 0xFF);
    t.w(kColorBack, 0x11);
    t.w(kWrMask, 0xFF);
    t.w(kZPattern | kGo, 0xA0000000u);
    t.w(kZPattern | kGo, 0x50000000u);
    t.w(kDrawMode1, (3u << 28) | (1u << 8) | (1u << 7) | (1u << 3) | 1u | 0x7000u);
    t.w(kXYStartI, (8u << 16) | 16u);
    t.w(kXYEndI, (11u << 16) | 16u);
    t.w(kDrawMode0 | kGo, 1u);
    CHECK_EQ(t.r(kHostRw0), 0xFF11FF11u);
    t.w(kXYStartI, (8u << 16) | 17u);
    t.w(kXYEndI, (11u << 16) | 17u);
    t.w(kDrawMode0 | kGo, 1u);
    CHECK_EQ(t.r(kHostRw0), 0x11FF11FFu);
}

// Host pixels in block mode: each GO draws one data word's pixels, and the
// word that ends a row moves on to the next row without drawing it.
TEST(newport_host_pixels_fill_a_block_row_by_row) {
    Rig t;
    t.w(kDrawMode1, 0x30000000u | 0x400u | 0x100u | 0x80u | 0x8u | 1u | 0x7000u);
    t.w(kWrMask, 0xFF);
    t.w(kXYStartI, 0);
    t.w(kXYEndI, (7u << 16) | 1u);
    t.w(kDrawMode0, 2u | (1u << 2) | (1u << 6) | (1u << 8) | (1u << 9));
    CHECK(t.np.write(kHostRw0 | kGo, 8, 0x1011121314151617ull));
    CHECK(t.np.write(kHostRw0 | kGo, 8, 0x18191a1b1c1d1e1full));
    t.w(kDrawMode1, 0x30000000u | 0x100u | 0x80u | 0x8u | 1u | 0x7000u);
    t.w(kXYStartI, (0u << 16) | 1u);
    t.w(kXYEndI, (3u << 16) | 1u);
    t.w(kDrawMode0 | kGo, 1u);
    CHECK_EQ(t.r(kHostRw0), 0x18191a1bu);
}

// The display version moves when the picture can change, and not when a
// drawing operation leaves the frame buffer as it was.
TEST(newport_display_version_tracks_visible_changes) {
    Rig t;
    const u64 v0 = t.np.display_version();
    t.fill(0, 0, 3, 3, 0x42);
    const u64 v1 = t.np.display_version();
    CHECK(v1 != v0);
    t.fill(0, 0, 3, 3, 0x42);  // the same pixels again
    CHECK_EQ(t.np.display_version(), v1);
    t.w(kTopScan, 0x3FF);
    const u64 v2 = t.np.display_version();
    CHECK(v2 != v1);
    t.cmap_entry(0x42, 0x123456);
    CHECK(t.np.display_version() != v2);
}

// The incremental, table-driven render() gives exactly what the plain
// conversion (render_full()) gives: random display modes per DID, random
// DID layouts, maps and gamma, drawing in every plane, a moving cursor, and
// changes between pictures.
TEST(newport_render_matches_the_reference_conversion) {
    u32 seed = 12345;
    auto rnd = [&](u32 n) {
        seed = seed * 1664525u + 1013904223u;
        return (seed >> 8) % n;
    };
    for (int round = 0; round < 6; ++round) {
        Rig t;
        constexpr unsigned kW = 60, kH = 40;  // visible size (the pipeline adds 2 clocks)
        t.vc2_ram(0x000, {static_cast<u16>(0x8080 | ((kW + 2) << 7))});
        t.vc2_ram(0x400, {0x000, kH, 0, 0});
        // DID table: a pointer per line, then 0-3 runs per line
        std::vector<u16> ptrs;
        for (unsigned y = 0; y < kH; ++y) {
            const u16 at = static_cast<u16>(0x800 + y * 8);
            ptrs.push_back(at);
            std::vector<u16> recs;
            unsigned x = 0;
            for (unsigned n = rnd(4); n > 0; --n) {
                x += rnd(30);
                recs.push_back(static_cast<u16>((x << 5) | rnd(32)));
            }
            recs.push_back(0xFFE0);
            for (std::size_t k = 0; k < recs.size(); ++k) t.vc2_ram(static_cast<u16>(at + k), {recs[k]});
        }
        for (unsigned y = 0; y < kH; ++y) t.vc2_ram(0x600 + y, {ptrs[y]});
        for (u32 d = 0; d < 32; ++d) t.xmap_mode(d, rnd(0x1000000) & 0x07FFFF);
        for (int i = 0; i < 300; ++i) t.cmap_entry(rnd(0x2000), rnd(0x1000000));
        auto gamma = [&] {
            for (int i = 0; i < 20; ++i) {
                t.w(kDcbMode, width(1) | crs(0) | slave(7));
                t.w(kDcbData0, rnd(256) << 24);
                t.w(kDcbMode, width(3) | crs(1) | slave(7));
                t.w(kDcbData0, rnd(0x1000000) << 8);
            }
        };
        gamma();
        // cursor glyph and position
        for (u16 a = 0; a < 256; ++a) t.vc2_ram(static_cast<u16>(0x3000 + a), {static_cast<u16>(rnd(0x10000))});
        t.vc2_reg(0x01, 0x3000);
        auto move_cursor = [&] {
            t.vc2_reg(0x02, rnd(kW + 40));
            t.vc2_reg(0x03, rnd(kH + 40));
        };
        move_cursor();
        t.vc2_reg(0x05, 0x600);  // DID entry
        t.vc2_reg(0x1F, 1);
        t.vc2_reg(0x00, 0x400);
        // display, timing, DID, cursor; large or crosshair in some rounds
        t.vc2_reg(0x10, 0x0006 | 0x0008 | 0x0010 | 0x0080 | (round % 2 ? 0x0200 : 0) | (round % 3 == 2 ? 0x0100 : 0));
        t.w(kTopScan, 0x3FF);
        t.w(kXYWin, 0x10001000);
        auto retrace = [&] {  // the VC2 takes cursor and DID pointers then
            t.now += 150000000 / 60 + 1;
            t.sched.run_due(t.now);
        };
        retrace();
        auto draw = [&](int n) {
            for (int i = 0; i < n; ++i) {
                static constexpr u32 kPlanes[] = {1, 1, 1, 4, 5, 6};
                const u32 planes = kPlanes[rnd(6)], depth = rnd(4);
                t.w(kDrawMode1, (3u << 28) | (depth << 3) | planes | 0x7000u);
                t.w(kWrMask, 0xFFFFFF);
                t.w(kColorI, rnd(0x1000000));
                const u32 x0 = rnd(kW + 4), y0 = rnd(kH), x1 = x0 + rnd(20), y1 = y0 + rnd(10);
                t.w(kXYStartI, (x0 << 16) | y0);
                t.w(kXYEndI, (x1 << 16) | y1);
                t.w(kDrawMode0 | kGo, 2u | (1u << 2) | (1u << 5) | (1u << 8) | (1u << 9));
            }
        };
        draw(40);
        std::vector<u32> fast, ref;
        unsigned fw = 0, fh = 0, rw = 0, rh = 0;
        auto compare = [&] {
            CHECK(t.np.render(fast, fw, fh));
            CHECK(t.np.render_full(ref, rw, rh));
            CHECK_EQ(fw, rw);
            CHECK_EQ(fh, rh);
            CHECK(fast == ref);
        };
        compare();
        for (int step = 0; step < 12; ++step) {
            switch (rnd(5)) {
                case 0: draw(1 + static_cast<int>(rnd(5))); break;
                case 1:
                    move_cursor();
                    retrace();
                    break;
                case 2: t.cmap_entry(rnd(0x2000), rnd(0x1000000)); break;
                case 3: gamma(); break;
                default: t.w(kTopScan, (0x3FF + rnd(8)) & 0x3FF); break;
            }
            compare();
        }
    }
}

namespace {

// 24-bit RGB pixels on an XL24: one point per GO, and reading one back.
constexpr u32 kRgb24 = (3u << 28) | 0x7000u | (1u << 15) | (3u << 3) | 1u;  // logicop SRC, compare always, RGBMODE, 24bpp, RGB planes
void point24(Rig& t, int x, int y, u32 bgr, u32 dm1 = kRgb24, u32 dm0_extra = 0) {
    t.w(kDrawMode1, dm1);
    t.w(kWrMask, 0xFFFFFF);
    t.w(kColorI, bgr);
    t.w(kXYStartI, (static_cast<u32>(x) << 16) | static_cast<u32>(y));
    t.w(kXYEndI, (static_cast<u32>(x) << 16) | static_cast<u32>(y));
    t.w(kDrawMode0 | kGo, 2u | (1u << 5) | dm0_extra);  // DRAW SPAN DOSETUP: one pixel
}
u32 read24(Rig& t, int x, int y) {
    t.w(kDrawMode1, kRgb24 | (3u << 8));  // 32-bit host pixels
    t.w(kXYStartI, (static_cast<u32>(x) << 16) | static_cast<u32>(y));
    t.w(kXYEndI, (static_cast<u32>(x) << 16) | static_cast<u32>(y));
    t.w(kDrawMode0 | kGo, 1u);
    return t.r(kHostRw0) & 0xFFFFFF;
}
constexpr u32 kBlend = 1u << 18, kBackBlend = 1u << 25;
constexpr u32 factors(u32 s, u32 d) { return (s << 19) | (d << 22); }
constexpr u32 kSa = 4, kMsa = 5;

}  // namespace

// Cb = Cs*Fs + Cd*Fd, factors normalised as (v + msb)/256.
TEST(newport_blend_source_alpha_over_frame_buffer_and_colorback) {
    Rig t(Newport::Options{true, 0xA});
    point24(t, 10, 10, 0x0000FF);  // red
    t.w(0x204, 0x80u << 11);       // COLORALPHA 128
    point24(t, 10, 10, 0xFF0000, kRgb24 | kBlend | factors(kSa, kMsa));  // blue at alpha 128
    CHECK_EQ(read24(t, 10, 10), (128u << 16) | 126u);
    // BACKBLEND: the destination is COLORBACK (green) whatever is stored.
    t.w(0x018, 0x00FF00);
    point24(t, 11, 10, 0xFF0000, kRgb24 | kBlend | kBackBlend | factors(kSa, kMsa));
    CHECK_EQ(read24(t, 11, 10), (128u << 16) | (126u << 8));
    // ONE, ONE adds and clamps.
    point24(t, 12, 10, 0x8080FF);
    point24(t, 12, 10, 0x80C001, kRgb24 | kBlend | factors(1, 1));
    CHECK_EQ(read24(t, 12, 10), 0xFFFFFFu);
}

// Below 24 bits an RGB colour is rounded to the nearest stored value, after
// blending on 8-bit channels; COLORBACK is 8-8-8 in RGB mode.
TEST(newport_rgb_rounding_blend_and_colorback) {
    Rig t(Newport::Options{true, 0xA});
    constexpr u32 kRgb12 = (kRgb24 & ~(3u << 3)) | (2u << 3);
    point24(t, 10, 10, 0x080808, kRgb12);  // half a step rounds up
    CHECK_EQ(read24(t, 10, 10), 0x111111u);
    point24(t, 11, 10, 0x070707, kRgb12);  // less rounds down
    CHECK_EQ(read24(t, 11, 10), 0u);
    t.w(0x204, 0xFFu << 11);  // COLORALPHA 1.0
    point24(t, 12, 10, 0x7F7F7F, kRgb12 | kBlend | factors(kSa, kMsa));  // 0x7F blended, then rounded
    CHECK_EQ(read24(t, 12, 10), 0x888888u);
    t.w(0x018, 0x0000FF);  // COLORBACK red
    t.w(0x014, 0);         // ZPATTERN: background everywhere
    point24(t, 13, 10, 0xFFFFFF, kRgb12, (1u << 12) | (1u << 16));  // ENZPATTERN ZPOPAQUE
    CHECK_EQ(read24(t, 13, 10), 0x00F00Fu);
}

// A slope written as minus zero leaves the iterator where it is.
TEST(newport_minus_zero_slope_keeps_alpha) {
    Rig t(Newport::Options{true, 0xA});
    for (int x = 20; x < 24; ++x) point24(t, x, 5, 0x0000FF);
    t.w(0x204, 0x80u << 11);  // COLORALPHA 128
    t.w(0x214, 0x80000000u);  // SLOPEALPHA -0
    t.w(kDrawMode1, kRgb24 | kBlend | factors(kSa, kMsa));
    t.w(kColorI, 0xFF0000);
    t.w(kXYStartI, (20u << 16) | 5u);
    t.w(kXYEndI, (23u << 16) | 5u);
    t.w(kDrawMode0 | kGo, 2u | (1u << 5) | (1u << 8) | (1u << 18));  // DRAW SPAN DOSETUP STOPX SHADE
    for (int x = 20; x < 24; ++x) CHECK_EQ(read24(t, x, 5), (128u << 16) | 126u);
}

// A_LINE: the two pixels across the exact line, weighted from AWEIGHT and
// blended by coverage.
TEST(newport_antialiased_line_coverage) {
    Rig t(Newport::Options{true, 0xA});
    auto line = [&](double x1, double y1, double x2, double y2) {
        auto fixed = [](double v) { return static_cast<u32>(static_cast<s32>(v * 16)) << 7; };
        t.w(kDrawMode1, kRgb24 | kBlend | factors(kSa, kMsa));
        t.w(kWrMask, 0xFFFFFF);
        t.w(kColorI, 0xFFFFFF);
        t.w(0x130, 0xFEDCBA98);  // AWEIGHT: entry i = i
        t.w(0x134, 0x76543210);
        t.w(0x100, fixed(x1));
        t.w(0x104, fixed(y1));
        t.w(0x108, fixed(x2));
        t.w(0x10C, fixed(y2));
        t.w(kDrawMode0 | kGo, 2u | (4u << 2) | (1u << 5) | (1u << 8));  // DRAW A_LINE DOSETUP STOPX
    };
    line(10, 20.5, 20, 20.5);  // through the row's centres: all there, none above
    CHECK_EQ(read24(t, 15, 20), 0xFFFFFFu);
    CHECK_EQ(read24(t, 15, 21), 0u);
    line(10, 41, 20, 41);  // between two rows: S takes entry 7 (0x77 blends to 0x76), T entry 8
    CHECK_EQ(t.r(0x100) >> 11, 20u);  // XSTART rests at the end point
    CHECK_EQ(read24(t, 15, 40), 0x767676u);
    CHECK_EQ(read24(t, 15, 41), 0x888888u);
    line(30, 61, 30, 51);  // y major, falling: the same pair across the columns
    CHECK_EQ(read24(t, 29, 55), 0x767676u);
    CHECK_EQ(read24(t, 30, 55), 0x888888u);
    line(40.5, 71, 40.5, 61);
    CHECK_EQ(read24(t, 40, 65), 0xFFFFFFu);
    CHECK_EQ(read24(t, 39, 65), 0u);
    CHECK_EQ(read24(t, 41, 65), 0u);
}

// LENGTH32 lines draw 32 pixels a GO, each GO carrying on from the next
// pixel: with XOR every pixel of the line ends up drawn exactly once.
TEST(newport_length32_line_draws_each_pixel_once) {
    Rig t;
    t.w(kDrawMode1, (6u << 28) | (1u << 3) | 1u | 0x7000u);  // logicop XOR, 8bpp, RGB/CI planes
    t.w(kWrMask, 0xFF);
    t.w(kColorI, 1);
    t.w(kXYStartI, (0u << 16) | 5u);
    t.w(kXYEndI, (99u << 16) | 5u);
    const u32 line = 2u | (2u << 2) | (1u << 8) | (1u << 15);  // DRAW I_LINE STOPX LENGTH32
    t.w(kDrawMode0 | kGo, line | (1u << 5));                  // DOSETUP on the first GO
    for (int go = 1; go < 4; ++go) t.w(kDrawMode0 | kGo, line);
    t.w(kDrawMode1, (3u << 28) | (1u << 8) | (1u << 7) | (1u << 3) | 1u | 0x7000u);  // read back
    for (u32 x = 0; x < 104; x += 4) {
        t.w(kXYStartI, (x << 16) | 5u);
        t.w(kXYEndI, ((x + 3) << 16) | 5u);
        t.w(kDrawMode0 | kGo, 1u);
        CHECK_EQ(t.r(kHostRw0), x < 100 ? 0x01010101u : 0u);
    }
}

// The register select stepped by ENCRSINC carries on into the next access
// until DCBMODE is written; the VC2 decodes CRS bits 1:0, and CRS 2 reads the
// low byte a CRS 1 read fetched.
TEST(newport_dcb_crs_increment_persists) {
    Rig t;
    t.vc2_reg(0x05, 0x1234);  // DID entry
    t.w(kDcbMode, width(1) | kCrsInc | crs(0) | slave(0));
    t.w(kDcbData0, 0x05u << 24);  // index 5; CRS now 1
    CHECK_EQ((t.r(kDcbMode) >> 4) & 7, 1u);
    CHECK_EQ(t.r(kDcbData0) >> 24, 0x12u);  // CRS 1: high byte (and the low one latched)
    CHECK_EQ(t.r(kDcbData0) >> 24, 0x34u);  // CRS 2: the latched low byte
    t.w(kDcbMode, width(1) | crs(5) | slave(0));  // CRS 5 = CRS 1 on the VC2
    CHECK_EQ(t.r(kDcbData0) >> 24, 0x12u);
}

// The XMAP9 picks among the layers: a non-zero overlay over the pixel, the
// pixel over the underlay, the underlay behind zero pixels.
TEST(newport_overlay_and_underlay_layers) {
    Rig t(Newport::Options{true});  // the XL24: 8 overlay bits per buffer
    t.vc2_ram(0x000, {0x8380});
    t.vc2_ram(0x400, {0x000, 1, 0, 0});
    t.vc2_reg(0x1F, 1);
    t.vc2_reg(0x00, 0x400);
    t.vc2_reg(0x10, 0x0006);
    t.w(kDcbMode, width(1) | crs(0) | slave(4));
    t.w(kDcbData0, 0x00);  // XMAP config: 24-bit system, pop-ups off
    // Mode 0: 8bpp CI over an 8-bit underlay with AUX_MSB_CMAP 1 (0x100 up).
    t.xmap_mode(0, (1u << 19) | (1u << 16) | 0x000404);
    t.cmap_entry(0x42, 0x123456);   // the pixel
    t.cmap_entry(0x105, 0x00FF00);  // underlay value 5
    t.cmap_entry(0x0, 0x000000);
    t.w(kTopScan, 0x3FF);
    t.w(kXYWin, 0x10021000);
    t.fill(0, 0, 0, 0, 0x42);       // screen column 0: a pixel; 1-3 stay 0
    // Underlay value 5 in every column (overlay planes, 8 bits).
    t.w(kDrawMode1, (3u << 28) | (1u << 3) | 4u | 0x7000u);  // SRC, 8bpp, overlay planes
    t.w(kWrMask, 0xFFFFFF);
    t.w(kColorI, 5);
    t.w(kXYStartI, 0);
    t.w(kXYEndI, 3u << 16);
    t.w(kDrawMode0 | kGo, 2u | (1u << 2) | (1u << 5) | (1u << 8) | (1u << 9));
    std::vector<u32> px;
    unsigned w = 0, h = 0;
    CHECK(t.np.render(px, w, h));
    if (px.size() == 4) {
        CHECK_EQ(px[0], 0xFF123456u);  // the pixel is non-zero: it shows
        CHECK_EQ(px[1], 0xFF00FF00u);  // zero pixel: the underlay
    }
    // As an overlay instead (mode 2), the non-zero overlay covers the pixel.
    t.xmap_mode(0, (1u << 19) | (2u << 16) | 0x000404);
    CHECK(t.np.render(px, w, h));
    if (px.size() == 4) CHECK_EQ(px[0], 0xFF00FF00u);
}

// Without RWPACKED a read returns one pixel, in the leftmost 8-bit field;
// 4-bit pixels use 8-bit fields too.
TEST(newport_host_read_fields) {
    Rig t;
    t.fill(10, 20, 13, 20, 0x42);
    t.w(kDrawMode1, (3u << 28) | (1u << 8) | (1u << 3) | 1u | 0x7000u);  // 8-bit host, not packed
    t.w(kXYStartI, (10u << 16) | 20u);
    t.w(kXYEndI, (13u << 16) | 20u);
    t.w(kDrawMode0 | kGo, 1u);
    CHECK_EQ(t.r(kHostRw0), 0x42000000u);
    t.w(kDrawMode1, (3u << 28) | (0u << 8) | (1u << 7) | (1u << 3) | 1u | 0x7000u);  // 4-bit host, packed
    t.w(kXYStartI, (10u << 16) | 20u);
    t.w(kDrawMode0 | kGo, 1u);
    CHECK_EQ(t.r(kHostRw0), 0x02020202u);  // the low 4 bits of each pixel, a byte each
}

namespace {
u32 pixel8(Rig& t, int x, int y) {
    t.w(kDrawMode1, (3u << 28) | (1u << 8) | (1u << 3) | 1u | 0x7000u);  // 8-bit host, one pixel
    t.w(kXYStartI, (static_cast<u32>(x) << 16) | static_cast<u32>(y));
    t.w(kXYEndI, (static_cast<u32>(x) << 16) | static_cast<u32>(y));
    t.w(kDrawMode0 | kGo, 1u);
    return t.r(kHostRw0) >> 24;
}
void line_mode(Rig& t, u32 ci) {
    t.w(kDrawMode1, (3u << 28) | (1u << 3) | 1u | 0x7000u);
    t.w(kWrMask, 0xFF);
    t.w(kColorI, ci);
}
}  // namespace

// A line drawn a point per GO (after one SETUP) follows the same pixels as
// the line drawn whole: the Bresenham error term carries from GO to GO.
TEST(newport_line_points_follow_the_whole_line) {
    Rig a, b;
    for (Rig* t : {&a, &b}) {
        line_mode(*t, 7);
        t->w(kXYStartI, (3u << 16) | 2u);
        t->w(kXYEndI, (40u << 16) | 13u);
    }
    a.w(kDrawMode0 | kGo, 2u | (2u << 2) | (1u << 5) | (1u << 8) | (1u << 9));  // I_LINE, DOSETUP, whole
    b.w(kDrawMode0 | 0, 2u | (2u << 2));
    b.w(0x030, 0);                                                           // SETUP
    for (int i = 0; i < 38; ++i) b.w(kDrawMode0 | kGo, 2u | (2u << 2));     // a point per GO
    for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 44; ++x) CHECK_EQ(pixel8(a, x, y), pixel8(b, x, y));
    CHECK_EQ(pixel8(a, 40, 13), 7u);
    CHECK_EQ(pixel8(a, 3, 2), 7u);
}

// The line stipple repeats each bit LSREPEAT times and wraps after LSLENGTH
// bits (17 here), most significant first.
TEST(newport_line_stipple_repeat_and_length) {
    Rig t;
    line_mode(t, 9);
    t.w(0x008, (0u << 24) | (2u << 8) | 2u);  // LSMODE: length 17, repeat 2, count 2
    t.w(0x00C, 0xA0000000u);                  // LSPATTERN: 1 0 1 0 0 ...
    t.w(kXYStartI, (0u << 16) | 4u);
    t.w(kXYEndI, (39u << 16) | 4u);
    t.w(kDrawMode0 | kGo, 2u | (2u << 2) | (1u << 5) | (1u << 8) | (1u << 9) | (1u << 13));  // ENLSPATTERN
    const char* expect = "1100110000000000000000000000000000110011";  // 2 x 17 = 34 pixels per cycle
    for (int x = 0; x < 40; ++x) CHECK_EQ(pixel8(t, x, 4), expect[x] == '1' ? 9u : 0u);
}

// XYOFFSET adds XYMOVE to drawn coordinates; YSTRIDE steps block rows by 2.
TEST(newport_xyoffset_and_ystride) {
    Rig t;
    t.w(0x114, (5u << 16) | 3u);  // XYMOVE
    t.w(kDrawMode1, (3u << 28) | 0x7000u | (1u << 3) | 1u);
    t.w(kWrMask, 0xFF);
    t.w(kColorI, 0x33);
    t.w(kXYStartI, (1u << 16) | 1u);
    t.w(kXYEndI, (1u << 16) | 1u);
    t.w(kDrawMode0 | kGo, 2u | (1u << 2) | (1u << 5) | (1u << 8) | (1u << 9) | (1u << 20));  // block, XYOFFSET
    CHECK_EQ(pixel8(t, 6, 4), 0x33u);
    CHECK_EQ(pixel8(t, 1, 1), 0u);
    t.w(kXYStartI, (20u << 16) | 10u);
    t.w(kXYEndI, (20u << 16) | 15u);
    t.w(kDrawMode0 | kGo, 2u | (1u << 2) | (1u << 5) | (1u << 8) | (1u << 9) | (1u << 23));  // block, YSTRIDE
    for (int y = 10; y <= 16; ++y) CHECK_EQ(pixel8(t, 20, y), (y % 2 == 0 && y <= 15) ? 0x33u : 0u);
}

// A colour index with a fraction of a half or more rounds up.
TEST(newport_ci_rounding) {
    Rig t;
    t.w(kDrawMode1, (3u << 28) | (1u << 3) | 1u | 0x7000u);
    t.w(kWrMask, 0xFF);
    t.w(0x200, (5u << 11) | (8u << 7));  // COLORRED index 5.5
    t.w(kXYStartI, (3u << 16) | 3u);
    t.w(kXYEndI, (3u << 16) | 3u);
    t.w(kDrawMode0 | kGo, 2u | (1u << 5));
    CHECK_EQ(pixel8(t, 3, 3), 6u);
}

// A double-buffered logic op takes its destination from the DBLSRC buffer.
TEST(newport_logic_op_destination_from_dblsrc) {
    Rig t(Newport::Options{true, 0xA});
    auto point = [&](u32 dm1, u32 mask, u32 ci) {
        t.w(kDrawMode1, dm1);
        t.w(kWrMask, mask);
        t.w(kColorI, ci);
        t.w(kXYStartI, (4u << 16) | 4u);
        t.w(kXYEndI, (4u << 16) | 4u);
        t.w(kDrawMode0 | kGo, 2u | (1u << 5));
    };
    constexpr u32 k8 = (1u << 3) | 1u | 0x7000u;  // 8bpp, RGB/CI planes, compare always
    point((3u << 28) | k8, 0xFFFF, 0x00);        // both buffers 0
    point((3u << 28) | k8, 0xFF00, 0x0F);        // buffer 1 0x0F
    point((6u << 28) | k8 | (1u << 5), 0xFF, 0xF0);  // XOR into buffer 0 against buffer 1
    CHECK_EQ(pixel8(t, 4, 4), 0xFFu);
}

// CMAP0 reports the board revision in bits 6:4, with the 8-bit flag.
TEST(newport_board_revision) {
    auto revision = [](Newport::Options o) {
        Rig t(o);
        t.w(kDcbMode, width(0) | crs(6) | slave(1));
        return t.r(kDcbData0) >> 24;
    };
    CHECK_EQ(revision({}), 0xA1u);
    CHECK_EQ(revision({true, 0xA}), 0x02u);
    CHECK_EQ(revision({false, 0xA, 6}), 0xE1u);
    CHECK_EQ(revision({true, 0xA, 4}), 0x42u);
}

// The VC2 takes a new cursor position, and DID table, at vertical retrace.
TEST(newport_cursor_and_did_latched_at_retrace) {
    Rig t;
    t.vc2_ram(0x000, {static_cast<u16>(0x8080 | (62u << 7))});  // 60 x 40
    t.vc2_ram(0x400, {0x000, 40, 0, 0});
    for (u16 a = 0; a < 128; ++a) t.vc2_ram(static_cast<u16>(0x3000 + a), {0xFFFF});  // solid glyph
    for (u32 i = 0; i < 0x2000; i += 0x100) t.cmap_entry(i, 0xFFFFFF);  // a white screen; the cursor is black
    t.vc2_reg(0x01, 0x3000);
    t.vc2_reg(0x1F, 1);
    t.vc2_reg(0x00, 0x400);
    t.vc2_reg(0x10, 0x0006 | 0x0008 | 0x0010 | 0x0080);  // display, timing, cursor
    auto retrace = [&] {
        t.now += 150000000 / 60 + 1;
        t.sched.run_due(t.now);
    };
    retrace();
    std::vector<u32> before, after;
    unsigned w = 0, h = 0;
    CHECK(t.np.render(before, w, h));
    t.vc2_reg(0x02, 40);  // CURSX, then CURSY: on screen
    t.vc2_reg(0x03, 40);
    int x = 0, y = 0;
    CHECK(t.np.cursor_position(x, y));
    CHECK_EQ(x, 40 - 29);
    CHECK_EQ(y, 40 - 31);
    after = before;
    CHECK(t.np.render(after, w, h));
    CHECK(after == before);  // not shown before the retrace
    retrace();
    CHECK(t.np.render(after, w, h));
    CHECK(after != before);
}
