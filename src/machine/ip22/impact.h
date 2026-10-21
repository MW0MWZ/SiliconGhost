// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// IMPACT graphics in the Indigo2's graphics slot: Maximum IMPACT (two
// geometry engines, two raster engines on alternate scanlines, 4 MB of
// texture memory).
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

#include <algorithm>
#include <array>
#include <deque>
#include <map>
#include <optional>
#include <vector>

#include "core/scheduler.h"
#include "machine/ip22/gfx_board.h"
#include "machine/ip22/impact_ge.h"
#include "machine/ip22/impact_te.h"
#include "mem/bus.h"

namespace sg::ip22 {

class Impact final : public GfxBoard, private ImpactRaster {
public:
    // The board's three GIO interrupt lines (set_irq_callback's line numbers).
    enum Irq : unsigned { kIrqFifo = 0, kIrqGraphics = 1, kIrqRetrace = 2 };

    Impact(Scheduler& scheduler, double cpu_hz);
    void reset() override;
    // The board is a GIO bus master: its DMA engine reads and writes system
    // memory through `bus`.
    void set_bus(Bus* bus) { bus_ = bus; }

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
        unsigned conv = 0;    // data conversion (GE tokens)
        unsigned got = 0;
        std::array<u32, 64> data{};
        bool in_command = false;
        u32 pixel = 0;        // a pixel data header being collected (bit 31 set)
        u32 pixel_words = 0;  // its data words still to come
        u32 last_token = 0;   // the last command, which the pixel data that follows belongs to
        u32 last_words = 0;   // and its data words
    };
    void fifo_push(FifoPort& port, u32 word);
    void run_command(FifoPort& port, u32 cmd, const u32* data, unsigned words, unsigned bytes);
    void ge_command(FifoPort& port, u32 cmd, const u32* data, unsigned words, unsigned bytes);
    void pixel_data(FifoPort& port, const std::vector<u8>& bytes);
    void eram_pass_through(u32 cmd, const u32* data, unsigned words);
    void run_display_list(u32 offset, u32 words);
    u32 status() const;
    u32 flags() const;  // the stored flags and those that follow state
    void update_irqs();
    void set_line(unsigned line, bool level);
    void unknown(const char* what, u32 offset, u64 value) const;

    u32 priv_read(u32 offset) const;
    void priv_write(u32 offset, u32 value);
    u32 user_read(u32 offset) const;
    void user_write(u32 offset, u32 value);

    // ---- host DMA and pixel transfers ----
    void dma_start(u32 word);
    void dma_run(u32 word);
    bool host_read(u64 paddr, u8* out, std::size_t n) const;
    void arm_transfer(int sx, int sy, int ex, int ey, unsigned type);
    void transfer_line_in(const u8* bytes, std::size_t n);  // one line into the armed write transfer
    std::vector<u8> transfer_line_out();                     // the armed read transfer's next line
    void pio_write(u64 dw, unsigned bytes);
    u32 pio_read_hi();
    u32 host_to_pixel(const u8* p) const { return host_to_pixel(p, rss_[kXfrMode]); }
    u32 host_to_pixel(const u8* p, u32 mode) const;
    void pixel_to_host(u32 pixel, std::vector<u8>& out) const;
    u32 read_pixel(int fx, int fy) const;  // the read buffer's pixel as store_pixel takes it
    unsigned transfer_bpp() const { return transfer_bpp(rss_[kXfrMode]); }
    static unsigned transfer_bpp(u32 mode);
    void format_bytes(std::vector<u8>& bytes) const;

    // ---- context switching ----
    void begin_context_switch();
    void load_context();
    void save_record(u32 slot);
    bool load_record(u32 slot);

    // ---- GE11 diagnostic ports ----
    void ge_diag_write(unsigned ge, bool address_port, u32 value);

    // ---- display control bus ----
    u32 dcb_read(u32 offset, unsigned size);
    void dcb_write(u32 offset, unsigned size, u64 value);
    u32 dcb_device_read(unsigned dev, unsigned crs, unsigned width);
    void dcb_device_write(unsigned dev, unsigned crs, unsigned width, u32 data);
    void cmap_write(unsigned which, unsigned crs, unsigned width, u32 data);
    u32 cmap_read(unsigned which, unsigned crs);
    void dac_write(unsigned crs, unsigned width, u32 data);
    u32 dac_read(unsigned crs);
    bool xmap_write(unsigned crs, unsigned width, u32 data);  // true: what is shown may have changed
    u32 xmap_read(unsigned crs, unsigned width);
    bool vc3_write(unsigned crs, unsigned width, u32 data);   // likewise
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
    // Where a pixel write goes: the pixel processors' mode, the buffer
    // pointers and the plane masks (as the registers hold them).
    struct PixelTarget {
        u32 pp1 = 0, drb = 0, lsb_a = 0, lsb_b = 0, msbs = 0;
    };
    PixelTarget rss_target() const;
    void store_pixel_to(const PixelTarget& t, int fx, int fy, u32 colour);
    u32 target_colour(const PixelTarget& t, int fx, int fy) const;

    // ---- GL drawing (the geometry engines' primitives) ----
    void ge_triangle(const GeDraw& d, const GeVertex& a, const GeVertex& b, const GeVertex& c) override;
    void ge_line(const GeDraw& d, const GeVertex& a, const GeVertex& b, float width, bool stipple, u32 pattern, u32 repeat,
                 u32& stipple_pos) override;
    void ge_point(const GeDraw& d, const GeVertex& v, float size) override;
    void ge_fragment(const GeDraw& d, int x, int y, const std::array<float, 4>& colour, float z) override;
    unsigned ge_host_bpp(u32 xfrmode) const override { return transfer_bpp(xfrmode); }
    u32 ge_host_pixel(const u8* p, u32 xfrmode) const override { return host_to_pixel(p, xfrmode); }
    void ge_bracket(bool enter, const GeDraw* d) override;
    void ge_rss_write(u32 reg, u32 value) override { rss_write(reg, value, false); }
    void ge_read_pixels(const GeDraw& d, u32 block_start, u32 block_end, u32 xfrmode, bool pio, u32 drb, bool b_half,
                        unsigned zst) override;
    void ge_clear_colour(const GeDraw& d, u32 colour) override;
    void ge_clear_zst(const GeDraw& d, bool depth, u32 value, u32 mask) override;
    void ge_tex_load(const GeTexLoad& l, unsigned line, const u8* texels, unsigned n) override { tram_.load_line(l, line, texels, n); }
    void ge_tex_read(const GeTexture& t, u32 xfrmode, bool raw, unsigned rows, unsigned w) override;
    void gl_shade(const GeDraw& d, float* rgba, float s, float t, float q, float lambda, float fog) const;
    PixelTarget gl_target(const GeDraw& d) const;
    bool gl_to_framebuffer(const GeDraw& d, int x, int y, int& fx, int& fy) const;
    bool gl_visible(const GeDraw& d, int fx, int fy) const;
    void gl_fragment(const GeDraw& d, int x, int y, const float* rgba, float z);
    void gl_fragment_at(const GeDraw& d, int x, int y, int fx, int fy, const float* rgba, float z);
    bool gl_clear_rect(const GeDraw& d, int& x0, int& x1, int& y0, int& y1) const;
    u32& zst_at(int fx, int fy);
    // Whether a framebuffer pixel may be written: its clip ID matches and it
    // passes every enabled screen mask.
    bool pixel_visible(int fx, int fy) const;
    void store_overlay(const PixelTarget& t, int fx, int fy, u32 value);
    u64* overlay_word(unsigned page, int fx, int fy, unsigned& shift);
    const u64* overlay_word(unsigned page, int fx, int fy, unsigned& shift) const;
    unsigned engines() const { return ((rss_[kDrbSize] >> 6) & 3) + 1; }
    unsigned engine_of(int fy) const { return engines() == 2 && !pixmem_[1].empty() ? fy & 1 : 0; }
    // Pixel memory: a buffer is a run of pages from `page`, each a 192 x 16
    // tile, DRBsize bits 5:2 tiles to a row. With two raster engines each
    // holds every other row. (Here, as they are on every pixel's way.)
    std::size_t word_index(unsigned page, int fx, unsigned row) const {
        const unsigned tiles = std::max(1u, (rss_[kDrbSize] >> 2) & 15);
        const unsigned pg = (page + row / kTileRows * tiles + static_cast<unsigned>(fx) / kTileWidth) & (kPages - 1);
        return std::size_t{pg} * kTileWidth * kTileRows + (row % kTileRows) * kTileWidth + static_cast<unsigned>(fx) % kTileWidth;
    }
    const u64* word_at(unsigned page, int fx, int fy) const {
        const bool two = engines() == 2 && !pixmem_[1].empty();
        const unsigned engine = two ? fy & 1 : 0;
        return &pixmem_[engine][word_index(page, fx, two ? static_cast<unsigned>(fy) >> 1 : static_cast<unsigned>(fy))];
    }
    // The writable form notes the page row written for render().
    u64* word_at(unsigned page, int fx, int fy) {
        const bool two = engines() == 2 && !pixmem_[1].empty();
        const unsigned engine = two ? fy & 1 : 0;
        const std::size_t i = word_index(page, fx, two ? static_cast<unsigned>(fy) >> 1 : static_cast<unsigned>(fy));
        row_written_[engine][i / kTileWidth] = 1;
        return &pixmem_[engine][i];
    }
    void mark_written(unsigned engine, const u64* p) {
        row_written_[engine][static_cast<std::size_t>(p - pixmem_[engine].data()) / kTileWidth] = 1;
    }

    // ---- scan-out ----
    // What a picture's rows depend on besides pixel memory and the state
    // whose change redraws everything.
    struct Shown {
        bool cursor = false, two = false;
        int cursor_x = 0, cursor_y = 0, cursor_size = 0;
        u32 cursor_base = 0;  // its colour map entries
        unsigned tiles = 0, overlay_tiles = 0, width = 0, height = 0;
        bool operator==(const Shown&) const = default;
    };
    Shown shown() const;
    // The screen rows whose row is set (all with none) as 0xAARRGGBB pixels.
    void scan_out(std::vector<u32>& argb, unsigned width, unsigned height, const u8* rows) const;
    void retrace();
    unsigned did_table_lines(u32 table) const;
    void did_row(u32 table, unsigned row, unsigned width, std::vector<u8>& dids) const;
    void display_size(unsigned& width, unsigned& height) const;

    Scheduler& scheduler_;
    double cpu_hz_;
    std::array<bool, 3> line_{};

    std::vector<u32> ucode_;              // HQ3 microcode RAM
    std::vector<u32> priv_;               // privileged registers, as last written
    u32 flags_ = 0, flag_enable_ = 0, int_enable_ = 0;
    u32 cfifo_high_ = 0, cfifo_low_ = 0;
    FifoPort user_port_, priv_port_, dl_port_;  // (display lists run through the third)
    std::array<u32, 0x80> dma_regs_{};
    std::array<u32, 0x10> reif_regs_{};
    u32 formatter_ = 0;
    Bus* bus_ = nullptr;
    u32 dma_read_pending_ = 0;  // a board-to-host start word waiting for the raster side
    // The armed pixel transfer (FILLMODE block types 2-5).
    struct Xfer {
        bool armed = false, read = false, pio = false;
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0, dx = 1, dy = 1;  // the block, window coordinates
        unsigned width = 0, lines = 0, bpp = 1;
        unsigned line = 0;                // next line in or out
        unsigned begin = 0, stride = 0;   // PIO framing: byte offset of a line in its first doubleword, and padding
        unsigned skip = 0;                // PIO write: bytes still to pass over before the line's pixels
        std::vector<u8> buf;              // PIO write: the line collected so far
        std::vector<u8> out;              // PIO read: the stream produced and not yet read
        std::size_t out_pos = 0;
        u32 lo = 0;                       // PIO read: the low half of the doubleword the last high read took
    } xfer_;

    // The geometry engines as their diagnostic ports see them (the engines
    // themselves are modelled at command level): a diagnostic address, and
    // microcode lines of 72 bits as three words, downloaded and read back.
    struct GeDiag {
        u32 addr = 0;
        unsigned word = 0;  // of the line being written
        std::vector<std::array<u32, 3>> ucode = std::vector<std::array<u32, 3>>(kGeUcodeLines);
    };
    static constexpr unsigned kGeUcodeLines = 0x20000;
    static constexpr u32 kGeUcodeBase = 0x200000, kGeExecControl = 0x40000;
    std::array<GeDiag, 2> ge_;
    std::deque<u32> ge_readback_;  // diagnostic readback words not yet read
    std::array<u32, 2> ge_answer_{};  // GE readback high, low

    // Graphics contexts. A switch saves the outgoing context (on the board:
    // into its slot of GE external RAM) and then takes the next 63 FIFO
    // words as the incoming context's image: word 0 its slot, word 1 bit 31
    // its first load, words 2-13 its window, 16-17 the banks it draws into.
    static constexpr unsigned kContextImageWords = 63;
    struct GlWindow {
        u32 origin = 0, mode = 0, pp1_mode = 0, drb = 0;
        std::array<std::array<u32, 2>, 4> masks{};  // screen masks 1-4: x range, y range
        u32 banks[2] = {};
    };
    unsigned ctx_words_ = 0;  // image words still to come
    std::array<u32, kContextImageWords> ctx_image_{};
    std::optional<u32> ctx_slot_;  // the loaded context's slot
    GlWindow gl_window_;

    // The geometry engines' GL state for the loaded context, and their
    // external RAM (64K words each), where a switched-out context's record
    // waits: its GE state, its half-received user command, any GE answer
    // not yet read, and its DMA and raster interface registers. The kernel
    // copies records out to host memory and back through ERAM pass-through
    // commands when it runs out of slots.
    struct ContextRecord {
        ImpactGe::State ge;
        FifoPort user_port;
        u32 ge_flags = 0;
        std::array<u32, 2> ge_answer{};
        std::array<u32, 0x80> dma_regs{};
        std::array<u32, 0x10> reif_regs{};
    };
    static constexpr unsigned kEramWords = 0x10000, kKernelSlotWords = 0x17C7;
    static_assert(sizeof(ContextRecord) <= kKernelSlotWords * 4, "a context record fits the kernel's ERAM slot");
    ImpactGe gl_{*this};
    std::array<std::vector<u32>, 2> eram_{std::vector<u32>(kEramWords), std::vector<u32>(kEramWords)};
    bool nosave_next_ = false;  // the kernel's own context: no GE save at the next switch
    std::optional<std::vector<u32>> eram_in_;  // ERAM write data collected
    std::optional<u32> eram_write_ge0_;        // two GEs: the address GE0's last write went to
    std::optional<std::pair<u32, u32>> eram_read_ge0_;
    std::optional<unsigned> eram_write_ge_;
    std::deque<u8> eram_out_;  // bytes for the board-to-host DMA ahead of the raster side's: ERAM reads, texture reads
    std::vector<u32> pixel_buf_;  // a pixel data command's words collected
    std::vector<u32> gl_pixels_;  // pixel data for the GE, until it has what it needs

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
        std::array<u8, 64> dib{};  // index 0: the scanout page pointers; the rest kept as written
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
    static constexpr u32 kScrMask1X = 0x147, kPp1WinMode = 0x17B;
    static constexpr u32 kXfrControl = 0x102, kXfrSize = 0x153, kXfrMode = 0x159;
    static constexpr u32 kDrbPointers = 0x16D, kDrbSize = 0x16E, kPixColorR = 0x176;
    std::array<u32, 0x400> rss_{};
    struct Stipple {
        bool armed = false, opaque = false;
        int x0 = 0, x1 = 0, y1 = 0, x = 0, y = 0, dx = 1, dy = 1;
        u32 colour = 0, background = 0;
    } stipple_;
    std::array<std::vector<u64>, 2> pixmem_;  // per raster engine: kPages pages of 36-bit words
    // Per raster engine, each page row (kTileWidth words) written since the
    // last render(), so that it converts only the screen rows that show one.
    std::array<std::vector<u8>, 2> row_written_;
    // The two clip-ID planes, 2 bits a framebuffer pixel. Where the board
    // keeps them is not known; nothing reads them back but the CID match.
    static constexpr unsigned kFbSize = 2048;
    std::vector<u8> cid_ = std::vector<u8>(kFbSize * kFbSize);
    ImpactTram tram_;
    std::vector<u32> zst_;  // depth (23:0) and stencil (31:24) a framebuffer pixel, made on first use
    std::optional<std::array<u32, 0x400>> gl_saved_rss_;  // the X server's raster registers during a GL pixel operation
    unsigned gl_read_zst_ = 0;  // a GL read of the depth (1) or stencil (2) buffer is armed

    // display
    Timer retrace_timer_, retrace_end_timer_;
    u64 last_retrace_ = 0;
    u64 frames_ = 0;
    u64 display_version_ = 0;
    // render() converts everything again after a change to the colour maps,
    // RAMDAC, XMAP or VC3 (but for the cursor's place and colours), or to
    // the layout.
    bool full_redraw_ = true;
    Shown shown_;  // what the last render() showed
    u64 pixels_drawn_ = 0;
    u32 unhandled_pp1_ = ~0u;  // the last PP1 fill mode logged as not drawn
};

}  // namespace sg::ip22
