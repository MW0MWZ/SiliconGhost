// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// IMPACT: the raster side of GL drawing. The geometry engines hand over
// triangles, lines, points and clears in window coordinates; here they are
// scan converted and each fragment goes through the per-fragment stages
// (window masks and clip ID, scissor, polygon stipple, alpha test, stencil
// and depth, blending or the logic op, plane masks) into pixel memory. The
// depth and stencil (ZST) buffer is kept as its own plane: where the pixel
// processors place it in RDRAM is not known, and nothing but GL reads it.
#include <algorithm>
#include <cmath>

#include "machine/ip22/impact.h"

namespace sg::ip22 {

namespace {

int half(u32 v) { return static_cast<s16>(v & 0xFFFF); }

bool compare(u32 func, double a, double b) {
    switch (func & 7) {
        case 0: return false;
        case 1: return a < b;
        case 2: return a == b;
        case 3: return a <= b;
        case 4: return a > b;
        case 5: return a != b;
        case 6: return a >= b;
        default: return true;
    }
}

// Rounded half away from zero, as lround() but without the library call
// (adding the half in double is exact for any float).
u8 to8(float c) { return static_cast<u8>(static_cast<int>(static_cast<double>(std::clamp(c, 0.0f, 1.0f) * 255.0f) + 0.5)); }

// A colour as RGBA 8888; a depth in buffer units as the stored 24 bits.
u32 pack(const float* c) { return to8(c[0]) | to8(c[1]) << 8 | to8(c[2]) << 16 | u32{to8(c[3])} << 24; }
u32 depth_value(float z) { return static_cast<u32>(std::clamp(z, 0.0f, ImpactGe::kZMax) + 0.5f) & 0xFFFFFF; }

// The part of framebuffer row fy, cut to [x0, x1], inside every screen mask
// that keeps its inside; false if there is none.
bool mask_span(const GeDraw& d, int fy, int& x0, int& x1) {
    for (unsigned n = 0; n < 4; ++n) {
        if ((d.winmode & (0x11u << n)) != (0x11u << n)) continue;
        const u32 mx = d.masks[n][0], my = d.masks[n][1];
        if (fy < half(my >> 16) || fy > half(my)) return false;
        x0 = std::max(x0, half(mx >> 16)), x1 = std::min(x1, half(mx));
    }
    return x0 <= x1;
}

// What gl_visible() asks of a pixel of one framebuffer row beyond
// mask_span(): the clip-ID match, and the spans that screen masks keeping
// their outside take away.
struct RowTest {
    const u8* cid = nullptr;  // the row's clip IDs, when there is a match
    u32 match = 0;
    int out[4][2] = {};
    unsigned outs = 0;
    bool any() const { return cid || outs; }
    bool operator()(int fx) const {
        if (cid && !((match >> cid[fx]) & 1)) return false;
        for (unsigned i = 0; i < outs; ++i)
            if (fx >= out[i][0] && fx <= out[i][1]) return false;
        return true;
    }
};

RowTest row_test(const GeDraw& d, const u8* cid_row, int fy) {
    RowTest r;
    r.match = (d.pp1winmode >> 4) & 0xF;
    if (r.match) r.cid = cid_row;
    for (unsigned n = 0; n < 4; ++n) {
        if ((d.winmode & (0x11u << n)) != (1u << n)) continue;
        const u32 mx = d.masks[n][0], my = d.masks[n][1];
        if (fy < half(my >> 16) || fy > half(my)) continue;
        r.out[r.outs][0] = half(mx >> 16), r.out[r.outs][1] = half(mx);
        ++r.outs;
    }
    return r;
}

float blend_factor(u32 f, unsigned k, const float* s, const float* d, const std::array<float, 4>& kc) {
    switch (f) {
        case 0: return 0;
        case 1: return 1;
        case 0x300: return s[k];
        case 0x301: return 1 - s[k];
        case 0x302: return s[3];
        case 0x303: return 1 - s[3];
        case 0x304: return d[3];
        case 0x305: return 1 - d[3];
        case 0x306: return d[k];
        case 0x307: return 1 - d[k];
        case 0x308: return k == 3 ? 1.0f : std::min(s[3], 1 - d[3]);
        case 0x8001: return kc[k];
        case 0x8002: return 1 - kc[k];
        case 0x8003: return kc[3];
        case 0x8004: return 1 - kc[3];
        default: return 1;
    }
}

}  // namespace

Impact::PixelTarget Impact::gl_target(const GeDraw& d) const {
    PixelTarget t;
    t.pp1 = d.pp1;
    t.drb = d.drb;
    t.lsb_a = t.lsb_b = d.mask_lsb;
    t.msbs = d.mask_msb;
    return t;
}

// Window coordinates (y up) to the framebuffer.
bool Impact::gl_to_framebuffer(const GeDraw& d, int x, int y, int& fx, int& fy) const {
    fx = half(d.origin) + x;
    fy = half(d.origin >> 16) + y;
    return fx >= 0 && fx < static_cast<int>(kFbSize) && fy >= 0 && fy < static_cast<int>(kFbSize);
}

u32& Impact::zst_at(int fx, int fy) {
    if (zst_.empty()) zst_.assign(std::size_t{kFbSize} * kFbSize, 0);
    return zst_[std::size_t{static_cast<unsigned>(fy)} * kFbSize + static_cast<unsigned>(fx)];
}

// The destination colour (8-8-8-8, or an index) of the buffer `t` draws.
u32 Impact::target_colour(const PixelTarget& t, int fx, int fy) const {
    const unsigned type = (t.pp1 >> 8) & 7, field = (t.pp1 >> 14) & 0x7F;
    const unsigned page_a = t.drb & 0x3FF;
    const bool b = field == 2 || field == 4;
    const unsigned page_b = (t.drb >> 10) & 0x3FF ? (t.drb >> 10) & 0x3FF : page_a;
    const bool same_word = type <= 1 || type >= 4;
    const u64 w = *word_at(b && !same_word ? page_b : page_a, fx, fy);
    auto widen = [](u32 v, unsigned n) { return n >= 8 ? v : n == 1 ? v * 255 : ((v << (8 - n)) | (v >> (2 * n - 8))) & 0xFF; };
    switch (type) {
        case 0:
        case 1: {
            const unsigned n = type == 0 ? 4 : 5;
            const u32 c = static_cast<u32>(b ? w >> (3 * n) : w);
            u32 rgb = 0;
            for (unsigned k = 0; k < 3; ++k) rgb |= widen((c >> (n * k)) & ((1u << n) - 1), n) << (8 * k);
            const unsigned apos = type == 0 ? (b ? 28 : 24) : (b ? 31 : 30), aw = type == 0 ? 4 : 1;
            return rgb | widen(static_cast<u32>(w >> apos) & ((1u << aw) - 1), aw) << 24;
        }
        case 2: return static_cast<u32>(w);
        case 3: return (static_cast<u32>(w) & 0xFFFFFF) | 0xFF000000u;
        default: return static_cast<u32>(b ? w >> 12 : w) & 0xFFF;
    }
}

bool Impact::gl_visible(const GeDraw& d, int fx, int fy) const {
    const u32 match = (d.pp1winmode >> 4) & 0xF;
    if (match && !((match >> cid_[std::size_t{static_cast<unsigned>(fy)} * kFbSize + static_cast<unsigned>(fx)]) & 1)) return false;
    for (unsigned n = 0; n < 4; ++n) {
        if (!(d.winmode & (1u << n))) continue;
        const u32 mx = d.masks[n][0], my = d.masks[n][1];
        const bool inside = fx >= half(mx >> 16) && fx <= half(mx) && fy >= half(my >> 16) && fy <= half(my);
        if (inside != ((d.winmode & (0x10u << n)) != 0)) return false;
    }
    return true;
}

// One fragment at window (x, y): colour RGBA 0..1 (an index in red), depth
// in buffer units.
void Impact::gl_fragment(const GeDraw& d, const StorePlan& plan, int x, int y, const float* c, float z) {
    if (d.scissor && (x < d.scissor_box[0] || x > d.scissor_box[1] || y < d.scissor_box[2] || y > d.scissor_box[3])) return;
    int fx, fy;
    if (!gl_to_framebuffer(d, x, y, fx, fy) || !gl_visible(d, fx, fy)) return;
    gl_fragment_at(d, plan, x, y, fx, fy, c, z);
}

// The rest of a fragment's stages, once it is known to be in the
// framebuffer, the scissor box and the window.
void Impact::gl_fragment_at(const GeDraw& d, const StorePlan& plan, int x, int y, int fx, int fy, const float* c, float z) {
    if (d.poly_stipple && !((d.stipple[static_cast<unsigned>(y) & 31] >> (31 - (static_cast<unsigned>(x) & 31))) & 1)) return;
    if (d.alpha_test && !d.ci && !compare(d.alpha_func, c[3], d.alpha_ref)) return;
    if (d.stencil || d.depth_test) {
        u32& zst = zst_at(fx, fy);
        const u32 s_old = zst >> 24, zold = zst & 0xFFFFFF;
        const u32 zv = depth_value(z);
        const bool spass = !d.stencil || compare(d.st_func, d.st_ref & d.st_cmask, s_old & d.st_cmask);
        const bool zpass = spass && (!d.depth_test || compare(d.depth_func, zv, zold));
        u32 n = zst;
        if (d.stencil) {
            const u32 op = !spass ? d.st_ops[0] : !zpass ? d.st_ops[1] : d.st_ops[2];
            u32 s_new = s_old;
            switch (op) {
                case 1: s_new = 0; break;
                case 2: s_new = d.st_ref; break;
                case 3: s_new = std::min(s_old + 1, 0xFFu); break;
                case 4: s_new = s_old ? s_old - 1 : 0; break;
                case 5: s_new = ~s_old & 0xFF; break;
                default: break;
            }
            const u32 s = (s_old & ~d.st_wmask) | (s_new & d.st_wmask);
            n = (n & 0xFFFFFF) | (s & 0xFF) << 24;
        }
        if (zpass && d.depth_test && d.depth_write) n = (n & 0xFF000000u) | zv;
        zst = n;
        if (!zpass) return;
    }
    u32 colour;
    if (d.ci) {
        colour = static_cast<u32>(std::lround(std::clamp(c[0], 0.0f, 1.0f) * 4095.0f)) & 0xFFF;
    } else if (d.blend) {
        const u32 dst = target_colour(plan.t, fx, fy);
        const float dc[4] = {(dst & 0xFF) / 255.0f, ((dst >> 8) & 0xFF) / 255.0f, ((dst >> 16) & 0xFF) / 255.0f, (dst >> 24) / 255.0f};
        float sc[4];
        for (unsigned k = 0; k < 4; ++k) sc[k] = std::clamp(c[k], 0.0f, 1.0f);
        float o[4];
        for (unsigned k = 0; k < 4; ++k) {
            const float fs = blend_factor(d.blend_src, k, sc, dc, d.blend_color), fd = blend_factor(d.blend_dst, k, sc, dc, d.blend_color);
            switch (d.blend_eq) {
                case 0x8007: o[k] = std::min(sc[k], dc[k]); break;
                case 0x8008: o[k] = std::max(sc[k], dc[k]); break;
                case 0x800A: o[k] = sc[k] * fs - dc[k] * fd; break;
                case 0x800B: o[k] = dc[k] * fd - sc[k] * fs; break;
                default: o[k] = sc[k] * fs + dc[k] * fd; break;
            }
        }
        colour = pack(o);
    } else {
        colour = pack(c);
    }
    store_planned(plan, fx, fy, colour);
}

// A textured fragment's colour: the texel at (s, t) / q (level of detail
// `lambda`) through the texture environment, then fog (factor `fog`).
void Impact::gl_shade(const GeDraw& d, float* c, float s, float t, float q, float lambda, float fog) const {
    const GeTexture& tx = d.tex;
    if (q != 0) s /= q, t /= q;
    const std::array<float, 4> f{c[0], c[1], c[2], c[3]};
    const auto o = ImpactTram::environment(tx, f, tram_.sample(tx, s, t, lambda));
    for (unsigned k = 0; k < 4; ++k) c[k] = o[k];
    if (tx.fog) {
        fog = std::clamp(fog, 0.0f, 1.0f);
        for (unsigned k = 0; k < 3; ++k) c[k] = fog * c[k] + (1 - fog) * tx.fog_colour[k];
    }
}

// The level of detail for texture coordinate derivatives (per pixel, in
// level-0 texels): log2 of the larger footprint.
static float tex_lambda(const GeTexture& t, float dsdx, float dtdx, float dsdy, float dtdy) {
    const float w = static_cast<float>(1u << (t.size & 0xF)), h = static_cast<float>(1u << ((t.size >> 4) & 0xF));
    const float px = std::hypot(dsdx * w, dtdx * h), py = std::hypot(dsdy * w, dtdy * h);
    const float rho = std::max(px, py);
    return rho > 0 && std::isfinite(rho) ? std::log2(rho) : 0;
}

// Triangles: pixel centres inside, edges shared by two triangles drawn once
// (left and bottom edges inclusive); colour and depth interpolated linearly
// in window space, texture coordinates perspective correct (s/w, t/w, q/w
// linear in window space).
void Impact::ge_triangle(const GeDraw& d, const GeVertex& a, const GeVertex& b, const GeVertex& c) {
    const float area = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    if (area == 0 || !std::isfinite(area)) return;
    const GeVertex* v[3] = {&a, &b, &c};
    if (area < 0) std::swap(v[1], v[2]);  // counter-clockwise
    const float A = std::abs(area);
    const int x0 = std::max(-4096, static_cast<int>(std::floor(std::min({a.x, b.x, c.x}))));
    const int x1 = std::min(4096, static_cast<int>(std::ceil(std::max({a.x, b.x, c.x}))));
    const int y0 = std::max(-4096, static_cast<int>(std::floor(std::min({a.y, b.y, c.y}))));
    const int y1 = std::min(4096, static_cast<int>(std::ceil(std::max({a.y, b.y, c.y}))));
    // Edge i runs from v[i] to v[i+1]; inside is to its left.
    float ex[3], ey[3];
    bool incl[3];
    for (unsigned i = 0; i < 3; ++i) {
        const GeVertex& p = *v[i];
        const GeVertex& q = *v[(i + 1) % 3];
        ex[i] = q.x - p.x, ey[i] = q.y - p.y;
        // A left edge runs downwards; a bottom edge runs to the right.
        incl[i] = ey[i] < 0 || (ey[i] == 0 && ex[i] > 0);
    }
    const StorePlan plan = store_plan(gl_target(d));
    float tq[3][3];  // s/w, t/w, q/w a vertex
    for (unsigned i = 0; i < 3; ++i) tq[i][0] = v[i]->t[0] * v[i]->w, tq[i][1] = v[i]->t[1] * v[i]->w, tq[i][2] = v[i]->t[3] * v[i]->w;
    // Only the rows and spans of the framebuffer, the scissor box and the
    // window are visited (what gl_fragment would turn away).
    const int ox = half(d.origin), oy = half(d.origin >> 16), fb = static_cast<int>(kFbSize);
    int ya = std::max(y0, -oy), yb = std::min(y1, fb - 1 - oy);
    if (d.scissor) ya = std::max(ya, d.scissor_box[2]), yb = std::min(yb, d.scissor_box[3]);
    for (int y = ya; y <= yb; ++y) {
        const float py = static_cast<float>(y) + 0.5f;
        int xa = std::max(x0, -ox), xb = std::min(x1, fb - 1 - ox);
        if (d.scissor) xa = std::max(xa, d.scissor_box[0]), xb = std::min(xb, d.scissor_box[1]);
        int fa = xa + ox, fz = xb + ox;
        if (!mask_span(d, y + oy, fa, fz)) continue;
        xa = fa - ox, xb = fz - ox;
        const RowTest visible = row_test(d, &cid_[std::size_t{static_cast<unsigned>(y + oy)} * kFbSize], y + oy);
        const bool checks = visible.any();
        // Each edge's crossing of the row bounds the span, widened by more
        // than the rounding of the edge test below, which still decides.
        for (unsigned i = 0; i < 3 && xa <= xb; ++i) {
            if (ey[i] == 0) continue;
            const GeVertex& p = *v[i];
            const double dy = static_cast<double>(py) - p.y;
            const double cross = p.x + ex[i] * dy / ey[i];
            const double far = std::max(std::abs(xa + 0.5 - p.x), std::abs(xb + 0.5 - p.x));
            const double margin = 1e-6 * (std::abs(ex[i] * dy) + std::abs(ey[i]) * far) / std::abs(ey[i]) + 1;
            if (!(margin < 1e6) || !(std::abs(cross) < 1e6)) continue;
            if (ey[i] > 0) xb = std::min(xb, static_cast<int>(std::floor(cross + margin - 0.5)));
            else xa = std::max(xa, static_cast<int>(std::ceil(cross - margin - 0.5)));
        }
        if (xa > xb) continue;
        // The edge tests and weights for the whole span first, in plain
        // loops the compiler can run several pixels at a time.
        const unsigned n = static_cast<unsigned>(xb - xa + 1);
        if (gl_span_.size() < 4 * std::size_t{n}) gl_span_.resize(4 * std::size_t{n});
        float* const L0 = gl_span_.data();
        float* const L1 = L0 + n;
        float* const L2 = L1 + n;
        float* const IN = L2 + n;  // 1: inside all three edges
        {
            float r[3], vx[3];  // each edge's row term; its start's x
            for (unsigned i = 0; i < 3; ++i) r[i] = ex[i] * (py - v[i]->y), vx[i] = v[i]->x;
            for (unsigned j = 0; j < n; ++j) {
                const float px = static_cast<float>(xa + static_cast<int>(j)) + 0.5f;
                const float e0 = r[0] - ey[0] * (px - vx[0]), e1 = r[1] - ey[1] * (px - vx[1]), e2 = r[2] - ey[2] * (px - vx[2]);
                const bool in = (e0 > 0 || (e0 == 0 && incl[0])) && (e1 > 0 || (e1 == 0 && incl[1])) && (e2 > 0 || (e2 == 0 && incl[2]));
                IN[j] = in ? 1.0f : 0.0f;
                // Edge i's value weights the vertex opposite it, i+2.
                L0[j] = e1 / A, L1[j] = e2 / A, L2[j] = e0 / A;
            }
        }
        for (int x = xa; x <= xb; ++x) {
            const unsigned j = static_cast<unsigned>(x - xa);
            if (IN[j] == 0 || (checks && !visible(x + ox))) continue;
            const float px = static_cast<float>(x) + 0.5f;
            const float l0 = L0[j], l1 = L1[j], l2 = L2[j];
            float col[4];
            for (unsigned k = 0; k < 4; ++k) col[k] = l0 * v[0]->c[k] + l1 * v[1]->c[k] + l2 * v[2]->c[k];
            if (d.tex.on) {
                // The planes of s/w, t/w, q/w at (px, py) and one pixel on.
                auto at = [&](float qx, float qy, float& s, float& t) {
                    float m[3];
                    for (unsigned i = 0; i < 3; ++i) {
                        const GeVertex& p = *v[(i + 1) % 3];
                        const GeVertex& q = *v[(i + 2) % 3];
                        m[i] = ((q.x - p.x) * (qy - p.y) - (q.y - p.y) * (qx - p.x)) / A;
                    }
                    float qs = 0, qt = 0, qq = 0;
                    for (unsigned i = 0; i < 3; ++i) qs += m[i] * tq[i][0], qt += m[i] * tq[i][1], qq += m[i] * tq[i][2];
                    s = qq != 0 ? qs / qq : qs, t = qq != 0 ? qt / qq : qt;
                    return qq;
                };
                float s0, t0, sx, tx, sy, ty;
                at(px, py, s0, t0);
                at(px + 1, py, sx, tx);
                at(px, py + 1, sy, ty);
                const float fog = l0 * v[0]->fog + l1 * v[1]->fog + l2 * v[2]->fog;
                gl_shade(d, col, s0, t0, 1, tex_lambda(d.tex, sx - s0, tx - t0, sy - s0, ty - t0), fog);
            }
            gl_fragment_at(d, plan, x, y, x + ox, y + oy, col, l0 * v[0]->z + l1 * v[1]->z + l2 * v[2]->z);
        }
    }
    ++display_version_;
}

// Lines: along the major axis, the pixels whose centres lie in [start, end)
// in the direction of travel; `width` pixels across the minor axis.
void Impact::ge_line(const GeDraw& d, const GeVertex& a, const GeVertex& b, float width, bool stipple, u32 pattern, u32 repeat,
                     u32& pos) {
    const float dx = b.x - a.x, dy = b.y - a.y;
    const bool xmajor = std::abs(dx) >= std::abs(dy);
    const float da = xmajor ? dx : dy;
    if (da == 0 || !std::isfinite(da)) return;
    const float s0 = xmajor ? a.x : a.y, s1 = xmajor ? b.x : b.y;
    const float m0 = xmajor ? a.y : a.x, dm = (xmajor ? dy : dx) / da;
    const int w = std::max(1, static_cast<int>(std::lround(width)));
    int i0, i1, step;
    if (da > 0) i0 = static_cast<int>(std::ceil(s0 - 0.5f)), i1 = static_cast<int>(std::ceil(s1 - 0.5f)), step = 1;
    else i0 = static_cast<int>(std::floor(s0 - 0.5f)), i1 = static_cast<int>(std::floor(s1 - 0.5f)), step = -1;
    if (step > 0 && i1 - i0 > 8192) return;
    if (step < 0 && i0 - i1 > 8192) return;
    const StorePlan plan = store_plan(gl_target(d));
    for (int i = i0; i != i1; i += step) {
        const float sc = static_cast<float>(i) + 0.5f, t = (sc - s0) / da;
        const unsigned p = pos++;
        if (stipple && !((pattern >> (15 - ((p / (repeat + 1)) & 15))) & 1)) continue;
        const int j = static_cast<int>(std::floor(m0 + (sc - s0) * dm));
        float col[4];
        for (unsigned k = 0; k < 4; ++k) col[k] = a.c[k] + (b.c[k] - a.c[k]) * t;
        const float z = a.z + (b.z - a.z) * t;
        if (d.tex.on) {
            auto st = [&](float u, float& s, float& tt) {
                const float qs = a.t[0] * a.w + (b.t[0] * b.w - a.t[0] * a.w) * u, qt = a.t[1] * a.w + (b.t[1] * b.w - a.t[1] * a.w) * u;
                const float qq = a.t[3] * a.w + (b.t[3] * b.w - a.t[3] * a.w) * u;
                s = qq != 0 ? qs / qq : qs, tt = qq != 0 ? qt / qq : qt;
            };
            float s, tt, sn, tn;
            st(t, s, tt);
            st(t + 1 / std::abs(da), sn, tn);
            gl_shade(d, col, s, tt, 1, tex_lambda(d.tex, sn - s, tn - tt, 0, 0), a.fog + (b.fog - a.fog) * t);
        }
        for (int k = j - (w - 1) / 2; k <= j + w / 2; ++k) {
            if (xmajor) gl_fragment(d, plan, i, k, col, z);
            else gl_fragment(d, plan, k, i, col, z);
        }
    }
    ++display_version_;
}

void Impact::ge_fragment(const GeDraw& d, int x, int y, const std::array<float, 4>& colour, float z) {
    gl_fragment(d, store_plan(gl_target(d)), x, y, colour.data(), z);
    ++display_version_;
}

// A GL pixel operation's registers: the X server's are kept aside and the
// window's loaded (origin, Y up, screen masks, clip-ID match, the buffer
// and pixel mode); they come back when it ends.
void Impact::ge_bracket(bool enter, const GeDraw* d) {
    if (!enter) {
        if (gl_saved_rss_) rss_ = *gl_saved_rss_;
        gl_saved_rss_.reset();
        gl_read_zst_ = 0;
        return;
    }
    if (!gl_saved_rss_) gl_saved_rss_ = rss_;
    if (!d) return;
    rss_[kXyWin] = d->origin;
    rss_[kConfig] &= ~8u;
    for (unsigned n = 0; n < 4; ++n) rss_[kScrMask1X + 2 * n] = d->masks[n][0], rss_[kScrMask1X + 2 * n + 1] = d->masks[n][1];
    rss_[kWinMode] = d->winmode;
    rss_[kPp1WinMode] = d->pp1winmode;
    rss_[kDrbPointers] = d->drb;
    rss_[kPp1FillMode] = d->pp1;
}

// glReadPixels: a read block (DMA or PIO) over the pixel block in the
// buffer the GE names; the host's transfer reads it out.
void Impact::ge_read_pixels(const GeDraw& d, u32 block_start, u32 block_end, u32 xfrmode, bool pio, u32 drb, bool b_half,
                            unsigned zst) {
    (void)d;
    rss_[kDrbPointers] = drb;
    rss_[kPp1FillMode] = (rss_[kPp1FillMode] & ~(0x1Fu << 21)) | (b_half ? 1u << 21 : 0);
    rss_[kXfrMode] = xfrmode;
    rss_[kFillMode] = pio ? 2u << 22 : 4u << 22;
    rss_[kBlockStart] = block_start;
    rss_[kBlockEnd] = block_end;
    gl_read_zst_ = zst;
    block();
}

// A texture readback: its rows go to the host's DMA reads.
void Impact::ge_tex_read(const GeTexture& t, u32 xfrmode, bool raw, unsigned rows, unsigned w) {
    if (!w || w > 8192 || rows > 8192) return;
    for (unsigned r = 0; r < rows; ++r) {
        const std::vector<u8> l = raw ? tram_.read_raw(t, xfrmode, r, w) : tram_.read_line(t, xfrmode, r, w);
        eram_out_.insert(eram_out_.end(), l.begin(), l.end());
    }
}

// Points: a square of the point's size (rounded) about its position.
void Impact::ge_point(const GeDraw& d, const GeVertex& v, float size) {
    const int n = std::max(1, static_cast<int>(std::lround(size)));
    auto start = [&](float c) { return n & 1 ? static_cast<int>(std::floor(c)) - (n - 1) / 2 : static_cast<int>(std::floor(c + 0.5f)) - n / 2; };
    const int x0 = start(v.x), y0 = start(v.y);
    std::array<float, 4> c = v.c;
    if (d.tex.on) gl_shade(d, c.data(), v.t[0], v.t[1], v.t[3], 0, v.fog);
    const StorePlan plan = store_plan(gl_target(d));
    for (int y = y0; y < y0 + n; ++y)
        for (int x = x0; x < x0 + n; ++x) gl_fragment(d, plan, x, y, c.data(), v.z);
    ++display_version_;
}

// A clear covers the window's first screen mask (its rectangle in the
// framebuffer), cut to the scissor box.
bool Impact::gl_clear_rect(const GeDraw& d, int& x0, int& x1, int& y0, int& y1) const {
    const int ox = half(d.origin), oy = half(d.origin >> 16);
    x0 = half(d.masks[0][0] >> 16), x1 = half(d.masks[0][0]);
    y0 = half(d.masks[0][1] >> 16), y1 = half(d.masks[0][1]);
    if (d.scissor) {
        x0 = std::max(x0, ox + d.scissor_box[0]), x1 = std::min(x1, ox + d.scissor_box[1]);
        y0 = std::max(y0, oy + d.scissor_box[2]), y1 = std::min(y1, oy + d.scissor_box[3]);
    }
    x0 = std::max(x0, 0), y0 = std::max(y0, 0);
    x1 = std::min(x1, static_cast<int>(kFbSize) - 1), y1 = std::min(y1, static_cast<int>(kFbSize) - 1);
    return x0 <= x1 && y0 <= y1;
}

void Impact::ge_clear_colour(const GeDraw& d, u32 colour) {
    int x0, x1, y0, y1;
    if (!gl_clear_rect(d, x0, x1, y0, y1)) return;
    const StorePlan plan = store_plan(gl_target(d));
    for (int fy = y0; fy <= y1; ++fy) {
        int a = x0, b = x1;
        if (!mask_span(d, fy, a, b)) continue;
        const RowTest visible = row_test(d, &cid_[std::size_t{static_cast<unsigned>(fy)} * kFbSize], fy);
        const bool checks = visible.any();
        for (int fx = a; fx <= b; ++fx)
            if (!checks || visible(fx)) store_planned(plan, fx, fy, colour);
    }
    ++display_version_;
}

void Impact::ge_clear_zst(const GeDraw& d, bool depth, u32 value, u32 mask) {
    int x0, x1, y0, y1;
    if (!gl_clear_rect(d, x0, x1, y0, y1)) return;
    const u32 m = depth ? mask & 0xFFFFFF : (mask & 0xFF) << 24;
    const u32 v = depth ? value & 0xFFFFFF : (value & 0xFF) << 24;
    for (int fy = y0; fy <= y1; ++fy) {
        int a = x0, b = x1;
        if (!mask_span(d, fy, a, b)) continue;
        const RowTest visible = row_test(d, &cid_[std::size_t{static_cast<unsigned>(fy)} * kFbSize], fy);
        u32* row = &zst_at(a, fy);  // a row is contiguous
        if (!visible.any()) {
            for (int fx = a; fx <= b; ++fx, ++row) *row = (*row & ~m) | (v & m);
            continue;
        }
        for (int fx = a; fx <= b; ++fx, ++row)
            if (visible(fx)) *row = (*row & ~m) | (v & m);
    }
}

}  // namespace sg::ip22
