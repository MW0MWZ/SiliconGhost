// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Newport (XL8/XL24) graphics for the IP22 GFX slot: REX3 raster engine,
// RB2 frame buffer, the display control bus (DCB) with VC2 video timing and
// cursor, XMAP9 pixel-mode mapping, CMAP colour maps and the Bt445 RAMDAC.
//
// REX3 registers occupy 0x1F0F0000-0x1F0F1FFF; writing or reading a register
// through its +0x800 alias also executes the programmed primitive ("GO").
#pragma once

#include <array>
#include <functional>
#include <vector>

#include "core/scheduler.h"
#include "mem/bus.h"

namespace sg::ip22 {

class Newport final : public Device {
public:
    struct Options {
        bool xl24 = false;
        u8 monitor_id = 0xA;  // monitor sense: 0xA = 1280x1024 @ 60 Hz
    };

    // Frame buffer geometry: 1280 visible + 64 off-screen columns, 1024 rows.
    static constexpr int kPitch = 1344;
    static constexpr int kRows = 1024;

    Newport(Scheduler& scheduler, double cpu_hz, const Options& options);
    void reset();

    const char* name() const override { return "newport"; }
    bool read(u32 offset, unsigned size, u64& value) override;
    bool write(u32 offset, unsigned size, u64 value) override;

    // Vertical retrace interrupt (INT3 LOCAL1 bit 7).
    void set_retrace_callback(std::function<void(bool)> cb) { retrace_cb_ = std::move(cb); }

    // Bumped by anything that can change the picture (frame buffer writes,
    // VC2, XMAP, CMAP, RAMDAC, TOPSCAN): a front end can skip render() while
    // it stays the same.
    u64 display_version() const { return display_version_; }
    // Scans out the visible screen as 0xAARRGGBB pixels. Returns false when
    // video timing is not running (no signal); width/height come from the
    // VC2 timing tables. `argb` should hold the previous call's picture: only
    // the lines that changed since then are converted again (all of them if
    // the size differs or the display set-up changed).
    bool render(std::vector<u32>& argb, unsigned& width, unsigned& height);
    // The same picture, converted from scratch the straightforward way
    // (screenshots; render() is checked against it).
    bool render_full(std::vector<u32>& argb, unsigned& width, unsigned& height) const;
    u64 frames() const { return frames_; }
    // Where the VC2 shows the hardware cursor: the top left of its glyph in
    // screen coordinates. False while the cursor is off.
    bool cursor_position(int& x, int& y) const;
    // Pixels the REX3 has drawn (a measure of how busy the board is).
    u64 pixels_drawn() const { return pixels_drawn_; }

private:
    // ---- REX3 ----
    u32 rex3_read(u32 reg);
    void rex3_write(u32 reg, u32 value);
    void execute();
    void do_setup();
    void draw_span_block(bool block);
    void line_setup(bool fractional);
    void advance_line_stipple();
    void draw_line();
    void draw_aline();
    void scr2scr();
    void pixel_read();
    void write_x_start(u32 v) { xstart_ = v & 0x07FFFF80u; }
    void write_y_start(u32 v) { ystart_ = v & 0x07FFFF80u; }
    void write_x_end(u32 v) { xend_ = v & 0x07FFFF80u; }
    void write_y_end(u32 v) { yend_ = v & 0x07FFFF80u; }
    static int int_part(u32 v) { return static_cast<s16>(v >> 11); }
    void set_color_index(u32 v);
    u32 ci_index() const;  // the colour index COLORRED holds

    u32 source_color(int x, int y);
    u32 host_color(u32* alpha = nullptr);
    u32 blend(u32 src, int px, int py) const;
    u32 default_color(u32 colour) const;
    u32 shaded_color(int x, int y) const;
    void iterate_shade();
    void reload_shade();
    void plot_source(int x, int y, u32 colour);
    void plot(int x, int y, u32 colour);
    bool clip(int x, int y, int& px, int& py) const;
    void store(int px, int py, u32 colour);
    u32 fetch(int px, int py) const;  // raw pixel shifted for the current planes/depth/buffer
    unsigned src_shift() const;

    // ---- DCB ----
    // DCBDATA accesses: the bytes of the CPU access, in address order and
    // limited to DCBMODE.DATAWIDTH, are transferred one by one.
    unsigned dcb_width(unsigned size) const;
    void dcb_write(unsigned size, u64 value);
    u64 dcb_read(u32 reg, unsigned size);
    void dcb_begin();
    bool dcb_stalled() const;
    void dcb_write_byte(unsigned addr, unsigned crs, u8 b);
    u8 dcb_read_byte(unsigned addr, unsigned crs);

    void vc2_write(unsigned crs, u8 b);
    u8 vc2_read(unsigned crs);
    void xmap_write(unsigned which, unsigned crs, u8 b);
    u8 xmap_read(unsigned which, unsigned crs);
    void cmap_write(unsigned which, unsigned crs, u8 b);
    u8 cmap_read(unsigned which, unsigned crs);
    void ramdac_write(unsigned crs, u8 b);
    u8 ramdac_read(unsigned crs);

    // ---- Display ----
    bool screen_size(unsigned& width, unsigned& height) const;
    // Scan-out changes: a whole-picture change (timing, DID tables, cursor
    // glyph, maps, gamma) or just the lines the cursor covers.
    void picture_changed() {
        ++display_version_;
        full_redraw_ = true;
    }
    void cursor_moving();
    // Raw pixel value -> final colour for one XMAP mode, gamma included.
    struct ModeTable {
        u32 mode = ~0u;
        unsigned shift = 0;
        u32 mask = 0;
        bool direct = false;  // 24-bit RGB: no table, converted per pixel
        std::vector<u32> lut;
    };
    const ModeTable& mode_table(unsigned did);
    u32 direct_colour(u32 mode, u32 pix) const;
    u32 special_colour(u32 mode, u32 pix, u32 aux, unsigned cursor) const;  // cursor, PUP, overlay, underlay or pixel
    bool aux_layer(u32 mode, bool pixel_nonzero, u32 aux, unsigned cursor, u32& addr) const;
    u32 aux_mask(u32 mode, bool& underlay) const;
    bool pixel_value_nonzero(u32 mode, u32 pix) const;
    u32 pixel_colour(u32 mode, u32 pix) const;
    u8 did_at(const std::vector<std::pair<int, u8>>& records, int x) const;
    unsigned cursor_pixel(int x, int y) const;
    void retrace();
    void set_vrint(bool level);

    Scheduler& scheduler_;
    double cpu_hz_;
    Options opts_;
    Timer retrace_timer_;
    std::function<void(bool)> retrace_cb_;
    u64 frames_ = 0;
    u64 pixels_drawn_ = 0;

    // REX3 state
    u32 drawmode0_ = 0, drawmode1_ = 0;
    u32 lsmode_ = 0, lspattern_ = 0, lspatsave_ = 0, zpattern_ = 0;
    u32 colorback_ = 0, colorvram_ = 0, alpharef_ = 0;
    u32 smask0x_ = 0, smask0y_ = 0;
    std::array<u32, 8> smask_{};  // SMASK1X..SMASK4Y
    u32 xstart_ = 0, ystart_ = 0, xend_ = 0, yend_ = 0;
    int xsave_ = 0, xmove_ = 0, ymove_ = 0;
    u32 bresd_ = 0, bress1_ = 0, bresoctinc1_ = 0, bresrndinc2_ = 0, brese1_ = 0, bress2_ = 0;
    u32 aweight0_ = 0, aweight1_ = 0;
    u32 colorred_ = 0, coloralpha_ = 0, colorgrn_ = 0, colorblue_ = 0;
    s32 curr_red_ = 0, curr_alpha_ = 0, curr_grn_ = 0, curr_blue_ = 0;
    u32 src_alpha_ = 0xFF;  // alpha of the pixel being drawn, for blending
    u32 slopered_ = 0, slopealpha_ = 0, slopegrn_ = 0, slopeblue_ = 0;
    u32 wrmask_ = 0, colori_ = 0, colorx_ = 0;
    u64 hostrw_ = 0;
    bool zfail_next_ = false, zfail_ = false;
    bool fastclear_ = false;  // the primitive running writes COLORVRAM (fast clear)  // STEPZ: the next GO's first pixel fails the Z test
    int host_shift_ = 0;
    u32 dcbmode_ = 0;
    unsigned dcb_crs_ = 0;  // the DCB's register select, stepped by ENCRSINC until DCBMODE is written
    u32 topscan_ = 0, xywin_ = 0, clipmode_ = 0, config_ = 0;
    bool vrint_ = false;
    u64 dcb_busy_until_ = 0;  // an access to an absent slave is waiting for its acknowledge

    u64 display_version_ = 0;
    bool full_redraw_ = true;
    std::vector<u8> row_dirty_ = std::vector<u8>(kRows, 1);  // frame buffer rows written since the last render
    std::vector<std::pair<int, int>> cursor_rows_;          // screen lines [first, end) the cursor left or entered
    std::array<ModeTable, 32> mode_tables_{};                // per DID, rebuilt when its mode or the maps change
    std::vector<u32> rgbci_, cidaux_;

    // DCB slaves
    struct Vc2 {
        std::array<u16, 32> reg{};
        std::vector<u16> ram = std::vector<u16>(0x8000);
        unsigned index = 0;
        u8 data_hi = 0;
        bool ram_hi_pending = true;  // next RAM byte written is the high byte
        u8 ram_lo_latch = 0;
        u8 reg_lo_latch = 0;  // low byte of the register a CRS 1 read fetched
        bool ram_read_hi = true;
    } vc2_;
    struct Xmap {
        u8 config = 0, cursor_msb = 0, pup_msb = 0, mode_addr = 0;
        std::array<u32, 32> mode{};
        std::array<u8, 4> mode_bytes{};
        unsigned mode_count = 0;
    };
    std::array<Xmap, 2> xmap_;
    struct Cmap {
        std::vector<u32> palette = std::vector<u32>(8192);  // 0xRRGGBB
        u16 addr = 0;
        unsigned rgb = 0;  // byte position within an R,G,B triple
        u32 pending = 0;
        u8 command = 0, crs7 = 0;
    };
    std::array<Cmap, 2> cmap_;
    struct Ramdac {
        std::array<std::array<u8, 256>, 3> lut{};
        u8 addr = 0;
        unsigned rgb = 0;
        std::array<u8, 8> regs{};
    } ramdac_;
};

}  // namespace sg::ip22
