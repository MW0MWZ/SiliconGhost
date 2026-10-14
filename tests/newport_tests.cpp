// SPDX-License-Identifier: GPL-2.0-or-later
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
    Rig() : np((sched.set_clock([this] { return now; }), sched), 150.0e6, Newport::Options{}) {}
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
        w(kDrawMode1, (3u << 28) | (1u << 3) | 1u);  // logicop SRC, 8bpp, RGB/CI planes
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
    // Read back four 8-bit pixels of the first row.
    t.w(kDrawMode1, (3u << 28) | (1u << 8) | (1u << 3) | 1u);
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
    t.w(kDrawMode1, (3u << 28) | (1u << 8) | (1u << 3) | 1u);  // 8bpp host and draw depth
    t.w(kWrMask, 0xFF);
    t.w(kXYStartI, (100u << 16) | 50u);
    t.w(kXYEndI, (103u << 16) | 50u);
    t.w(kDrawMode0, 2u | (1u << 6));  // DRAW SPAN, COLORHOST, one pixel per GO
    for (u32 c : {0x11u, 0x22u, 0x33u}) t.w(kHostRw0 | kGo, c << 24);
    t.w(kDrawMode1, (3u << 28) | (1u << 8) | (1u << 3) | 1u);
    t.w(kXYStartI, (100u << 16) | 50u);
    t.w(kDrawMode0 | kGo, 1u);
    CHECK_EQ(t.r(kHostRw0), 0x11223300u);
}
