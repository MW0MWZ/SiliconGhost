// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// IMPACT graphics (Solid, High and Maximum IMPACT) in the Indigo2's
// graphics slot.
//
// The board decodes a 1 MB window at the slot base:
//   0x00000           GIO ID word
//   0x40000-0x45FFF   HQ3 microcode RAM (24-bit words)
//   0x50000-0x5FFFF   HQ3 privileged registers and command FIFO port
//   0x60000-0x67FFF   display control bus (DCB) devices, 1 KB each
//   0x68000-0x6803F   DCB per-device timing registers
//   0x70000-0x7BFFF   HQ3 user registers: status, flags, command FIFO port
//   0x7C000-0x7CFFF   raster (RSS) registers; at +0x1000 a write also executes
//
// Drawing arrives as command FIFO words: commands from 0x1000 up write
// raster registers (0x400 = execute after the write); lower ones go to the
// HQ3 and the geometry engines. The FIFO is processed as it is written, so it
// never backs up and the board reads as idle between accesses.
//
// The raster engine draws into pixel memory: per raster engine 1024 pages,
// each a 192 x 16 pixel tile of 36-bit words (or 768 x 16 of the 9-bit
// overlay), with buffers named by page pointers. Scan-out goes through the
// VC3's window ID tables, the XMAP's display modes, the colour maps and the
// RAMDAC's gamma.
//
// No SGI documentation of this board is public: register behaviour follows
// what IRIX, the PROM and the BSD and X drivers do with it. Accesses nothing
// here answers are logged (debug level) so they can be traced.
#pragma once

#include <array>
#include <vector>

#include "core/scheduler.h"
#include "machine/ip22/gfx_board.h"

namespace sg::ip22 {

class Impact final : public GfxBoard {
public:
    enum class Variant { Solid, High, Max };
    // The board's three GIO interrupt lines (set_irq_callback's line numbers).
    enum Irq : unsigned { kIrqFifo = 0, kIrqGraphics = 1, kIrqRetrace = 2 };

    Impact(Scheduler& scheduler, double cpu_hz, Variant variant);
    void reset() override;

    const char* name() const override { return "impact"; }
    bool read(u32 offset, unsigned size, u64& value) override;
    bool write(u32 offset, unsigned size, u64 value) override;
    bool read_is_pure(u32 offset, unsigned size) const override;

    u64 display_version() const override { return display_version_; }
    bool render(std::vector<u32>& argb, unsigned& width, unsigned& height) override;
    bool render_full(std::vector<u32>& argb, unsigned& width, unsigned& height) const override;
    u64 frames() const override { return frames_; }
    bool cursor_position(int& x, int& y) const override;
    u64 pixels_drawn() const override { return pixels_drawn_; }

    // GIO product ID: 32-bit ID word, product 0x10, revision 1, GIO64.
    static constexpr u32 kGioId = 0x00050190;
    static constexpr unsigned kPages = 1024;          // per raster engine
    static constexpr unsigned kTileWidth = 192;       // 36-bit pixels per page row
    static constexpr unsigned kOverlayTileWidth = 768;
    static constexpr unsigned kTileRows = 16;

    // Pixel memory access for tests and diagnostics: the 36-bit word at
    // framebuffer (x, y) of the buffer starting at `page`.
    u64 pixel_word(unsigned page, int x, int y) const;
    u32 overlay_pixel(unsigned page, int x, int y) const;

private:
    // ---- host interface (HQ3) ----
    struct FifoPort {
        u32 cmd = 0;          // the command being collected
        unsigned bytes = 0;   // its data byte count
        unsigned need = 0;    // data words it takes
        unsigned skip = 0;    // pixel data words to pass over
        unsigned got = 0;
        std::array<u32, 64> data{};
        bool in_command = false;
    };
    void fifo_push(FifoPort& port, u32 word);
    void run_command(u32 cmd, const u32* data, unsigned words, unsigned bytes);
    u32 status() const;
    void update_irqs();
    void set_line(unsigned line, bool level);
    void unknown(const char* what, u32 offset, u64 value) const;

    u32 priv_read(u32 offset) const;
    void priv_write(u32 offset, u32 value);
    u32 user_read(u32 offset) const;
    void user_write(u32 offset, u32 value);

    // ---- display control bus ----
    u32 dcb_read(u32 offset, unsigned size);
    void dcb_write(u32 offset, unsigned size, u64 value);
    u32 dcb_device_read(unsigned dev, unsigned crs, unsigned width);
    void dcb_device_write(unsigned dev, unsigned crs, unsigned width, u32 data);
    void cmap_write(unsigned which, unsigned crs, unsigned width, u32 data);
    u32 cmap_read(unsigned which, unsigned crs);
    void dac_write(unsigned crs, unsigned width, u32 data);
    u32 dac_read(unsigned crs);
    void xmap_write(unsigned crs, unsigned width, u32 data);
    u32 xmap_read(unsigned crs, unsigned width);
    void vc3_write(unsigned crs, unsigned width, u32 data);
    u32 vc3_read(unsigned crs, unsigned width);
    u32 vc3_scanline() const;
    static u32 be_bytes(const u8* p, unsigned n);
    static void put_be_bytes(u8* p, unsigned n, u32 v);

    // ---- raster engine (RE4 + PP1) ----
    u32 rss_read(u32 reg) const;
    void rss_write(u32 reg, u32 value, bool execute);
    void execute();
    void block();
    void point();
    void xline();
    void char_stipple(u64 bits, unsigned count);
    bool to_framebuffer(int x, int y, int& fx, int& fy) const;
    void plot(int x, int y, u32 colour);  // window coordinates
    u32 draw_colour() const;
    u32 background_colour() const;
    void store_pixel(int fx, int fy, u32 colour);
    u64* word_at(unsigned page, int fx, int fy);
    const u64* word_at(unsigned page, int fx, int fy) const;
    unsigned engines() const { return ((rss_[kDrbSize] >> 6) & 3) + 1; }

    // ---- scan-out ----
    void retrace();
    unsigned did_table_lines(u32 table) const;
    void did_row(u32 table, unsigned row, unsigned width, std::vector<u8>& dids) const;
    void display_size(unsigned& width, unsigned& height) const;
    u32 scan_pixel(unsigned x, unsigned row, unsigned height, u8 did, u8 odid) const;

    Scheduler& scheduler_;
    double cpu_hz_;
    Variant variant_;
    std::array<bool, 3> line_{};

    std::vector<u32> ucode_;              // HQ3 microcode RAM
    std::vector<u32> priv_;               // privileged registers, as last written
    u32 flags_ = 0, flag_enable_ = 0, int_enable_ = 0;
    u32 cfifo_high_ = 0, cfifo_low_ = 0;
    FifoPort user_port_, priv_port_;
    std::array<u32, 0x80> dma_regs_{};
    std::array<u32, 0x10> reif_regs_{};
    u32 formatter_ = 0;

    // DCB
    std::array<u32, 16> dcb_timing_{};
    struct Cmap {
        std::vector<u32> entries = std::vector<u32>(8192);  // 0xRRGGBB
        u32 addr = 0;
        u8 command = 0;
    };
    std::array<Cmap, 2> cmap_;
    struct Dac {
        std::array<std::array<u16, 3>, 256> gamma{};  // 10-bit R, G, B
        u32 addr = 0;
        unsigned phase = 0;
        std::array<u8, 256> regs{};
        u8 mode = 0;
    } dac_;
    struct Xmap {
        u8 pp1_select = 0, index = 0;
        std::array<u8, 16> config{};    // INDEX 0-3: the config word
        std::array<u8, 4> buf_select{};
        std::array<u8, 128> main_mode{};
        std::array<u8, 128> overlay_mode{};
        std::array<u8, 4> dib{};
    } xmap_;
    struct Vc3 {
        u8 index = 0;
        std::array<u16, 32> regs{};
        std::vector<u16> sram = std::vector<u16>(32768);
    } vc3_;
    u8 board_version_[2] = {};
    u8 board_version_written_ = 0;

    // RSS
    static constexpr u32 kIr = 0x013, kXlineStart = 0x040, kXlineEnd = 0x041, kIrAlias = 0x045;
    static constexpr u32 kBlockStart = 0x046, kBlockEnd = 0x047, kPackedColor = 0x05B, kRed = 0x05C;
    static constexpr u32 kCharH = 0x070, kCharL = 0x071, kFillMode = 0x110, kConfig = 0x112, kXyWin = 0x115;
    static constexpr u32 kBkgrdRg = 0x140, kBkgrdBa = 0x141, kWinMode = 0x14F, kLsPat = 0x15A, kStatus = 0x15E;
    static constexpr u32 kPp1FillMode = 0x161, kMaskMsbs = 0x162, kMaskLsbsA = 0x163, kMaskLsbsB = 0x164;
    static constexpr u32 kDrbPointers = 0x16D, kDrbSize = 0x16E, kPixColorR = 0x176;
    std::array<u32, 0x400> rss_{};
    struct Stipple {
        bool armed = false, opaque = false;
        int x0 = 0, x1 = 0, y1 = 0, x = 0, y = 0, dx = 1, dy = 1;
        u32 colour = 0, background = 0;
    } stipple_;
    std::array<std::vector<u64>, 2> pixmem_;  // per raster engine: kPages pages of 36-bit words

    // display
    Timer retrace_timer_, retrace_end_timer_;
    u64 last_retrace_ = 0;
    u64 frames_ = 0;
    u64 display_version_ = 0;
    u64 pixels_drawn_ = 0;
};

}  // namespace sg::ip22
