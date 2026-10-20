// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// The IMPACT geometry engines (GE11) at command level.
//
// libGLcore sends OpenGL to the board as command-processor tokens (command
// numbers below 0x200) through the command FIFO; on the board the HQ3 hands
// them to the GE11s, whose microcode keeps the GL state, transforms, lights
// and clips, and sets primitives up in the raster engine. Here the same work
// is done directly: the GL state lives in one plain block per context (kept
// in the GE's external RAM across context switches, where the kernel may copy
// it out and back), and primitives reach the raster side in window
// coordinates through ImpactRaster.
//
// Token data formats follow what libGLcore sends (IRIX 6.5): compare
// functions arrive as indices 0-7 (NEVER .. ALWAYS), blend factors and
// polygon modes as GL enums, logic ops as indices in X11 order, stencil ops
// as indices (KEEP ZERO REPLACE INCR DECR INVERT), the alpha reference as
// ref * 4096, the depth clear value as a 24-bit integer, the colour mask as
// RGBA bits 0-3, the line stipple as (repeat - 1, pattern with its first
// bit in bit 15). State reads (glGet*, glIsEnabled, glFinish) come back
// through state_word() at the GE addresses libGLcore reads.
#pragma once

#include <array>
#include <optional>
#include <vector>

#include "core/types.h"

namespace sg::ip22 {

// A vertex in window coordinates: x, y relative to the window's lower left
// (y up), z in depth buffer units (0 .. 2^24 - 1); colour 0..1 RGBA (or an
// index in red); texture coordinates; fog factor (1 = no fog).
struct GeVertex {
    float x = 0, y = 0, z = 0, w = 1;
    std::array<float, 4> c{};
    std::array<float, 4> t{};
    float fog = 1;
};

// The texture engine's (TE1) registers a primitive is sampled with: TEXMODE1
// (enable, environment, components, texel class), TEXMODE2 (filters, wrap,
// depth, mipmapping), TXSIZE, TXLOD, the border and environment colours,
// and the per-level TRAM pages (TXMIPMAP, TXBORDER). With texturing, fog
// comes after the texture environment, per fragment.
struct GeTexture {
    bool on = false;
    u32 mode1 = 0, mode2 = 0, size = 0, lod = 0;
    u32 env_rg = 0, env_b = 0, bcolour_rg = 0, bcolour_ba = 0;
    std::array<u32, 16> pages{}, border_pages{};
    bool fog = false;
    std::array<float, 3> fog_colour{};
};

// A texture load's destination, as the texture loader registers give it:
// TL_MODE (components, select slot, depth), TL_SPEC (level size), the
// level's TRAM page and border page, and the sub-image's origin (signed:
// -1 for a border).
struct GeTexLoad {
    u32 mode = 0, spec = 0, page = 0, border_page = 0;
    s32 s0 = 0, t0 = 0;
};

// Where and how GL fragments are drawn: the window (from the kernel) and
// the per-fragment state, with the pixel processors' mode and plane masks
// as the GE derives them.
struct GeDraw {
    // window
    u32 origin = 0;    // x | y << 16 (signed halves), framebuffer, y up
    u32 winmode = 0;   // screen-mask enables (WINMODE format)
    std::array<std::array<u32, 2>, 4> masks{};  // screen masks 1-4 (x range, y range), framebuffer
    u32 pp1winmode = 0;
    u32 drb = 0;       // DRBpointers for the buffer drawn
    // pixel processors
    u32 pp1 = 0;       // PP1FILLMODE: pixel type, buffer size, draw field, logic op, dither
    u32 mask_lsb = 0, mask_msb = 0;
    bool ci = false;   // colour index: the index rides in red
    // per-fragment
    bool scissor = false;
    std::array<int, 4> scissor_box{};  // x0, x1, y0, y1, window coordinates, inclusive
    bool alpha_test = false;
    u32 alpha_func = 7;
    float alpha_ref = 0;
    bool stencil = false;
    u32 st_func = 7, st_ref = 0, st_cmask = 0xFF, st_wmask = 0xFF;
    std::array<u32, 3> st_ops{};  // fail, zfail, zpass
    bool depth_test = false, depth_write = true;
    u32 depth_func = 1;
    bool blend = false;
    u32 blend_src = 1, blend_dst = 0, blend_eq = 0;  // GL enums (eq 0: add)
    std::array<float, 4> blend_color{};
    bool poly_stipple = false;
    std::array<u32, 32> stipple{};
    bool overlay = false;  // drawing the overlay planes
    GeTexture tex;
};

// What the raster side does for the GE.
class ImpactRaster {
public:
    virtual ~ImpactRaster() = default;
    virtual void ge_triangle(const GeDraw& d, const GeVertex& a, const GeVertex& b, const GeVertex& c) = 0;
    // A line; `stipple_pos` carries the stipple counter across a strip.
    virtual void ge_line(const GeDraw& d, const GeVertex& a, const GeVertex& b, float width, bool stipple,
                         u32 pattern, u32 repeat, u32& stipple_pos) = 0;
    virtual void ge_point(const GeDraw& d, const GeVertex& v, float size) = 0;
    // One fragment at window (x, y) (bitmaps, images).
    virtual void ge_fragment(const GeDraw& d, int x, int y, const std::array<float, 4>& colour, float z) = 0;
    // A host pixel in the raster engine's transfer format `xfrmode`: its
    // size, and its value (8-8-8-8 red lowest, or an index).
    virtual unsigned ge_host_bpp(u32 xfrmode) const = 0;
    virtual u32 ge_host_pixel(const u8* p, u32 xfrmode) const = 0;
    // A GL pixel operation's raster registers: entering loads the window's
    // state over the X server's (saved), leaving puts them back.
    virtual void ge_bracket(bool enter, const GeDraw* d) = 0;
    virtual void ge_rss_write(u32 reg, u32 value) = 0;
    // glReadPixels: arm a read of the block (window coordinates) from the
    // colour buffer at `drb` (the read field picking a pair's half), or
    // the depth / stencil buffer; the host's DMA or PIO reads follow.
    virtual void ge_read_pixels(const GeDraw& d, u32 block_start, u32 block_end, u32 xfrmode, bool pio, u32 drb, bool b_half,
                                unsigned zst) = 0;
    // A clear of the window cut to the scissor box: colour buffer (`colour`
    // as 8-8-8-8 or an index, through d's plane masks), or depth / stencil.
    virtual void ge_clear_colour(const GeDraw& d, u32 colour) = 0;
    virtual void ge_clear_zst(const GeDraw& d, bool depth, u32 value, u32 mask) = 0;
    // One row of a texture load (row `line` of the sub-image), 4 bytes a
    // texel: its components in order, unused ones ignored.
    virtual void ge_tex_load(const GeTexLoad& l, unsigned line, const u8* texels, unsigned n) = 0;
    // A texture readback: `rows` rows of `w` texels of the texture `t`
    // describes (raw: TRAM pages as saved), for the host's DMA reads.
    virtual void ge_tex_read(const GeTexture& t, u32 xfrmode, bool raw, unsigned rows, unsigned w) = 0;
};

class ImpactGe {
public:
    static constexpr unsigned kLights = 8;
    static constexpr float kZMax = 16777215.0f;

    struct Mat4 {
        std::array<float, 16> m{};  // column major, as OpenGL
    };
    template <unsigned N>
    struct Stack {
        std::array<Mat4, N> m{};
        u32 top = 0;
    };
    struct Light {
        std::array<float, 4> ambient{}, diffuse{}, specular{}, position{};  // position in eye space
        std::array<float, 3> spot_dir{};  // eye space
        float spot_exp = 0, spot_cutoff = 180;
        std::array<float, 3> atten{};  // constant, linear, quadratic
    };
    struct Material {
        std::array<float, 4> ambient{}, diffuse{}, specular{}, emission{};
        float shininess = 0;
        std::array<float, 3> indexes{};
    };
    // The vertex as the pipeline carries it: clip position, eye position,
    // front and back colours, texture coordinates, fog factor.
    struct PVertex {
        std::array<float, 4> h{}, e{}, c{}, cb{}, t{};
        float fog = 1;
    };
    static constexpr unsigned kMaxPoly = 64;

    // All of a context's GE state. Plain data: it is copied in and out of
    // the GE's external RAM word for word.
    struct State {
        u32 inited = 0;
        // window (kernel)
        u32 win_valid = 0, win_origin = 0, win_mode = 0, win_pp1 = 0, win_drb = 0;
        std::array<std::array<u32, 2>, 4> win_masks{};
        // matrices
        u32 matrix_mode = 0x1700;
        Stack<32> mv;
        Stack<4> proj, tex;
        Stack<2> colour;
        // viewport (window coordinates) and depth range
        std::array<float, 4> viewport{0, 0, 1, 1};  // x, y, width, height
        std::array<float, 2> depth_range{0, 1};
        // current attributes
        std::array<float, 4> colour_now{1, 1, 1, 1};
        std::array<float, 3> normal{0, 0, 1};
        std::array<float, 4> texcoord{0, 0, 0, 1};
        float index_now = 1;
        // lighting
        u32 lighting = 0, lights_on = 0, local_viewer = 0, two_side = 0, normalize = 0;
        std::array<Light, kLights> lights{};
        std::array<Material, 2> mat{};
        std::array<float, 4> lmodel_ambient{0.2f, 0.2f, 0.2f, 1};
        u32 cmat_on = 0, cmat_face = 0x408, cmat_param = 0x1602;
        u32 lmc_colour = 0, unlit = 0;  // IRIS GL lmcolor through IGLOO
        // fog
        u32 fog = 0, fog_mode = 0x800;  // GL_EXP
        float fog_start = 0, fog_end = 1, fog_density = 1;
        std::array<float, 4> fog_colour{};
        u32 fog_index = 0;
        // user clip planes, eye space
        u32 clip_on = 0;
        std::array<std::array<float, 4>, 6> clip_planes{};
        // rasterisation
        u32 smooth = 1, cull = 0, cull_mode = 0x405, front_ccw = 1;
        std::array<u32, 2> polygon_mode{0x1B02, 0x1B02};
        float line_width = 1, point_size = 1;
        u32 line_smooth = 0, point_smooth = 0;
        u32 line_stipple = 0, line_repeat = 0, line_pattern = 0xFFFF, stipple_pos = 0;
        u32 poly_stipple = 0;
        std::array<u32, 32> stipple_rows{};
        std::array<float, 2> polygon_offset{};
        // fragments
        u32 scissor_test = 0;
        std::array<int, 4> scissor{0, 1279, 0, 1023};  // as sent: x, width - 1, y, height - 1
        u32 alpha_test = 0, alpha_func = 7, alpha_ref = 0;
        u32 stencil = 0, st_func = 7, st_ref = 0, st_cmask = 0xFF, st_wmask = 0xFF;
        std::array<u32, 3> st_ops{};
        u32 depth_test = 0, depth_func = 1, depth_mask = 1;
        u32 blend = 0, blend_src = 1, blend_dst = 0, blend_eq = 0;
        std::array<float, 4> blend_colour{};
        u32 logic = 0, logic_op = 3, dither = 1;
        u32 colour_mask = 0xF, index_mask = 0xFFF;
        // clears
        std::array<float, 4> clear_colour{};
        u32 clear_depth = 0xFFFFFF, clear_stencil = 0, clear_index = 0;
        // buffers
        u32 ci = 0;                 // colour-index context
        u32 pixel_format = 0;       // INIT_FORMAT_VALUES (PP1 bits 13, 10:8) | kFormatSent
        u32 draw_bits = 1, draw_bits_swapped = 1, draw_words = 0, buffer_count = 0;
        u32 swapped = 0, banks_known = 0;
        u32 read_back = 0;
        // raster position
        std::array<float, 3> raster{};
        std::array<float, 4> raster_colour{1, 1, 1, 1}, raster_tex{0, 0, 0, 1};
        float raster_dist = 0;
        u32 raster_valid = 1;
        // Texturing: on (TEX_ON / TEX_OFF); the GE's shadow of the
        // context's TE registers (kTeRegs order), TXADDR and the per-level
        // tables it indexes (TXMIPMAP, TXBORDER, DETAILSCALE); the texture
        // loader's destination (ERAM 0xDB1: level width, ?, level, page |
        // bank << 8, border page, ?) and the host format's scale (0xD0A).
        // Coordinate generation per S T R Q: on bits, mode (0 object
        // linear, 1 eye linear, 2 sphere map), the planes (eye planes in
        // eye space), and which plane the next plane data is for
        // (coordinate | eye << 2).
        u32 tex_on = 0;
        std::array<u32, 31> te{};
        u32 te_index = 0;
        std::array<std::array<u32, 32>, 3> te_tables{};
        std::array<u32, 6> tl_dest{};
        u32 tl_scale = 0;
        u32 texgen_on = 0;
        std::array<u32, 4> texgen_mode{1, 1, 1, 1};
        std::array<std::array<float, 4>, 4> texgen_obj{}, texgen_eye{};
        u32 texgen_plane = 0;
        // glBitmap: the header waiting for its rows (format, row bits, rows,
        // origin x / y and move x / y as floats, data words); general
        // bitmaps come in batches of rows padded to 64 bits.
        std::array<u32, 8> bitmap{};
        u32 bitmap_pending = 0, bitmap_general = 0, bitmap_row = 0;
        // Pixel operations: the GE's pixel pipeline memory (PIXEL_STATE and
        // INIT_PIXEL_STATE_ERAM writes; the program at 0x13B), the pixel
        // zoom's inverse (0x192), the image rectangle (0xDA8: x0, y0, x1,
        // y1 relative to the raster position, transfer size), the raster
        // engine's transfer mode and factor and the pixel block as the
        // register lists last gave them, and a SEND_PIXELS header waiting
        // for its rows.
        struct PixMem {
            std::array<u32, 0x100> ge{};    // GE memory 0x100-0x1FF
            std::array<u32, 0x200> eram{};  // ERAM 0xC00-0xDFF
            std::array<u32, 0x300 / 32> loaded{};
            u32 prog_len = 0;
            u32 eram_shift = 0;  // how much lower this microcode keeps the ERAM pixel state
        } pix;
        float zoom_inv = 0;
        std::array<u32, 5> tl_rect{};
        u32 xfrmode = 0, xfr_factor = 0;
        std::array<u32, 2> pixel_block{};
        u32 pixel_op = 0, pixel_planes = 0;
        std::array<u32, 8> send_pixels{};
        u32 send_pixels_pending = 0;
        // GE memory words set by SET_DATA (0xB00-0xBFF)
        std::array<u32, 0x100> set_data{};
        std::array<u32, 8> set_data_valid{};
        // query scratch (GET_LIGHTFV / GET_MATERIALFV)
        std::array<u32, 4> scratch{};
        // primitive assembly
        u32 prim = 0xFF, nv = 0, tflip = 0;
        std::array<PVertex, kMaxPoly> vb{};
    };

    explicit ImpactGe(ImpactRaster& raster);
    void reset();

    // One GE token with its data words (already converted to floats where
    // the command word asked). False when the GE does not handle it.
    bool token(u32 cmd, const u32* d, unsigned n);
    // The word a state readback (0xA1 / 0xA2) returns for GE address `addr`.
    std::optional<u32> state_word(u32 addr);

    // Pixel data the GE takes (glBitmap rows): whether it waits for some,
    // and how many words before it can draw.
    bool wants_pixel_data() const { return s_.bitmap_pending || s_.send_pixels_pending; }
    bool wants_image() const { return s_.send_pixels_pending != 0; }
    unsigned pixel_words_needed() const;
    void pixel_data(const u32* words, unsigned n);
    // Pixel data for the command before it: a pixel state block, a colour
    // table, or a raster register list ((register, value) pairs, or
    // (register, value, shadow address) triples).
    void pixel_state_block(const u32* d, unsigned n);
    void colour_table(const u32* d, unsigned n);
    void rss_list(const u32* d, unsigned n, unsigned group);
    // A context switch away: a pixel operation's registers go back.
    void end_bracket();

    // SCHEDULE_SWAP reached the GE.
    void swap_buffers();
    // The bank GL draws into (VALIDATE_BANKS, or a context image).
    void set_draw_bank(u32 bank);
    // The window from the kernel (a context image or KERN_WINDOW).
    void set_window(u32 origin, u32 mode, u32 pp1winmode, const std::array<std::array<u32, 2>, 4>& masks, u32 drb);

    State& state() { return s_; }
    const State& state() const { return s_; }
    void init_state();  // OpenGL's initial values

    static constexpr u32 kFormatSent = 1u << 31;

private:
    void ensure_init() {
        if (!s_.inited) init_state();
    }
    Mat4& top();
    void matrix_mult(const Mat4& m);
    Mat4 mvp() const;
    PVertex transform(const std::array<float, 4>& obj);
    std::array<float, 4> light(const std::array<float, 4>& eye, const std::array<float, 3>& n, bool back) const;
    float fog_factor(const std::array<float, 4>& eye) const;
    GeVertex project(const PVertex& v) const;
    void vertex(const std::array<float, 4>& obj);
    void end_primitive();
    void polygon(PVertex* v, unsigned n, const PVertex& provoking);
    void line(PVertex a, PVertex b, bool restart);
    void point(const PVertex& v);
    unsigned clip_polygon(PVertex* v, unsigned n) const;
    bool clip_line(PVertex& a, PVertex& b) const;
    void light_param(u32 cmd, const u32* d, unsigned n);
    void material(const u32* d, unsigned n);
    void clear_colour_buffer();
    void clear_zst(bool depth);

    // The draw state for what follows.
    const GeDraw& draw();
    std::optional<u32> draw_mask() const;
    u32 draw_pointers() const;
    u32 pp1_base() const;
    bool rgb12_pair() const;
    bool ci_pair() const;
    bool rgb36() const;
    u32 pair_field() const;
    void colour_write_masks(u32& lsb, u32& msb) const;
    std::array<int, 4> scissor_rect() const;

    bool te_write(u32 reg, u32 v);
    u32 te_reg(u32 reg) const;
    bool texturing() const;
    GeTexture tex_state() const;
    std::array<float, 4> texgen(const std::array<float, 4>& obj, const std::array<float, 4>& eye) const;
    void tex_load(const u32* words, unsigned n);

    void bitmap_rows(const u32* words, unsigned n);
    void draw_pixels(const u32* words, unsigned n);
    void get_pixels(bool pio);
    bool pipeline(const std::vector<u8>& bytes, unsigned w, unsigned h, unsigned stride, unsigned out_comps,
                  std::vector<std::vector<double>>& rows) const;
    u32 pix_read(u32 addr) const;
    void pix_write(u32 addr, u32 v);
    bool pix_loaded(u32 addr) const;

    ImpactRaster& raster_;
    State s_;
    GeDraw draw_;
    bool bracket_ = false;
    // The colour tables (glPixelMap, INIT_COLOR_TABLE): the board's, not a
    // context's; libGLcore sends them before each operation using them.
    std::vector<u32> lut_ = std::vector<u32>(0x1000);
};

}  // namespace sg::ip22
