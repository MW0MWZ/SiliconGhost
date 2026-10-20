// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "machine/ip22/impact_te.h"

#include <algorithm>
#include <cmath>

namespace sg::ip22 {

namespace {

// TEXMODE2
constexpr u32 kMagLinear = 1u << 7, kMinLinear = 1u << 8, kMipmap = 1u << 5, kMipLinear = 1u << 6;
constexpr u32 kClampS = 1u << 11, kClampT = 1u << 12;
// GL_CLAMP clamps the coordinates to [0, 1] as well (linear filtering at an
// edge is then half border); without it, clamp to border.
constexpr u32 kGlClamp = 1u << 14;
constexpr u32 kNoBorder = 1u << 16;  // clamped fetches outside read the border colour
constexpr u32 kMmEnable = 1u << 19;

// A component's depth in nibbles (TL_MODE bits 6:5, TEXMODE2 bits 3:2).
unsigned depth_nibbles(u32 field) {
    switch (field & 3) {
        case 0: return 1;
        case 2: return 3;
        default: return 2;
    }
}

// Nibbles a texel cell takes: four components, rounded up to a power of two.
unsigned cell_nibbles(unsigned d) { return d == 1 ? 4 : d == 2 ? 8 : 16; }

// Nibble offset in its cell of component `c` of an `nc`-component texture
// in select slot `sel`: one-component textures have four slots, two
// two-component ones; wider textures fill the cell.
unsigned comp_offset(unsigned sel, unsigned nc, unsigned c, unsigned d) {
    const unsigned slot = nc == 1 ? sel & 3 : nc == 2 ? (sel & 1) * 2 : 0;
    return std::min(slot + c, 3u) * d;
}

// Texel offset of a small level (larger side `n` <= 16) in its shared page.
std::size_t small_offset(unsigned n) {
    std::size_t off = 0;
    for (unsigned k = 16; k > n; k /= 2) off += std::size_t{k} * k;
    return off;
}

std::size_t small_border_offset(unsigned n) {
    std::size_t off = 0;
    for (unsigned k = 16; k > n; k /= 2) off += 4 * k + 4;
    return off;
}

std::size_t texel_addr(u32 page, unsigned w, unsigned h, unsigned s, unsigned t, unsigned tn) {
    const unsigned side = std::max(w, h);
    const std::size_t off = side <= 16 ? small_offset(side) : 0;
    return (page * ImpactTram::kPageNibbles + (off + std::size_t{t} * w + s) * tn) % ImpactTram::kNibbles;
}

// Border texel (s, t), s in -1..w, t in -1..h, in the upper half of the
// border page: bottom row, top row (w + 2 each), left column, right column.
std::size_t border_addr(u32 page, unsigned w, unsigned h, long s, long t, unsigned tn) {
    const long wi = w, hi = h;
    const long idx = t < 0 ? s + 1 : t >= hi ? wi + 2 + s + 1 : s < 0 ? 2 * (wi + 2) + t : 2 * (wi + 2) + hi + t;
    const unsigned side = std::max(w, h);
    const std::size_t off = side <= 16 ? small_border_offset(side) : 0;
    return (page * ImpactTram::kPageNibbles + ImpactTram::kPageNibbles / 2 + (off + static_cast<std::size_t>(idx)) * tn) %
           ImpactTram::kNibbles;
}

float field12(u32 v) { return static_cast<float>(v & 0xFFF) / 4095.0f; }

}  // namespace

void ImpactTram::set_nibble(std::size_t a, u32 v) {
    u8& b = tram_[a / 2];
    b = a & 1 ? static_cast<u8>((b & 0x0F) | (v & 0xF) << 4) : static_cast<u8>((b & 0xF0) | (v & 0xF));
}

void ImpactTram::load_line(const GeTexLoad& l, unsigned line, const u8* texels, unsigned n) {
    if (tram_.empty()) tram_.assign(kBytes, 0);
    const unsigned nc = ((l.mode >> 1) & 3) + 1, sel = (l.mode >> 3) & 3, d = depth_nibbles(l.mode >> 5);
    const unsigned w = 1u << ((l.spec >> 4) & 0xF), h = 1u << ((l.spec >> 8) & 0xF);
    const long t = static_cast<long>(l.t0) + line, wi = w, hi = h;
    if (t < -1 || t > hi) return;  // a border row at most
    const unsigned tn = cell_nibbles(d);
    for (unsigned k = 0; k < n; ++k) {
        const long s = static_cast<long>(l.s0) + k;
        if (s < -1 || s > wi) continue;
        const bool inside = s >= 0 && s < wi && t >= 0 && t < hi;
        const std::size_t a = inside ? texel_addr(l.page, w, h, static_cast<unsigned>(s), static_cast<unsigned>(t), tn)
                                     : border_addr(l.border_page, w, h, s, t, tn);
        for (unsigned c = 0; c < nc; ++c) {
            const u32 v = texels[4 * k + c];
            const u32 stored = d == 1 ? v >> 4 : d == 3 ? (v << 4 | v >> 4) : v;
            const std::size_t at = a + comp_offset(sel, nc, c, d);
            for (unsigned i = 0; i < d; ++i) set_nibble((at + i) % kNibbles, stored >> (4 * i));
        }
    }
}

std::array<float, 4> ImpactTram::texel(const GeTexture& t, unsigned level, long i, long j) const {
    const unsigned ls = t.size & 0xF, lt = (t.size >> 4) & 0xF;
    const unsigned w = 1u << (ls > level ? ls - level : 0), h = 1u << (lt > level ? lt - level : 0);
    auto axis = [](long v, unsigned n, bool clamp) {
        const long m = n;
        return clamp ? std::clamp(v, -1L, m) : ((v % m) + m) % m;
    };
    const long s = axis(i, w, t.mode2 & kClampS), tt = axis(j, h, t.mode2 & kClampT);
    const unsigned m1 = t.mode1, nc = ((m1 >> 3) & 3) + 1, sel = (m1 >> 7) & 3, d = depth_nibbles(t.mode2 >> 2);
    const unsigned tn = cell_nibbles(d);
    std::size_t a;
    if (s < 0 || tt < 0 || s >= static_cast<long>(w) || tt >= static_cast<long>(h)) {
        if (t.mode2 & kNoBorder) {
            const float r = field12(t.bcolour_rg), g = field12(t.bcolour_rg >> 12), b = field12(t.bcolour_ba),
                        al = field12(t.bcolour_ba >> 12);
            switch ((m1 >> 9) & 7) {
                case 1: return {al, 0, 0, 0};
                case 2: return {r, al, 0, 0};
                case 3: return {r, 0, 0, 0};
                default: return {r, g, b, al};
            }
        }
        a = border_addr(t.border_pages[level & 15], w, h, s, tt, tn);
    } else {
        a = texel_addr(t.pages[level & 15], w, h, static_cast<unsigned>(s), static_cast<unsigned>(tt), tn);
    }
    std::array<float, 4> c{};
    const float full = static_cast<float>((1u << (4 * d)) - 1);
    for (unsigned k = 0; k < nc; ++k) {
        const std::size_t at = a + comp_offset(sel, nc, k, d);
        u32 v = 0;
        for (unsigned n = 0; n < d; ++n) v |= nibble((at + n) % kNibbles) << (4 * n);
        c[k] = static_cast<float>(v) / full;
    }
    return c;
}

std::array<float, 4> ImpactTram::level(const GeTexture& t, unsigned lv, float s, float tc, bool linear) const {
    const unsigned ls = t.size & 0xF, lt = (t.size >> 4) & 0xF;
    const unsigned lw = ls > lv ? ls - lv : 0, lh = lt > lv ? lt - lv : 0;
    const float w = static_cast<float>(1u << lw), h = static_cast<float>(1u << lh);
    const bool gl_clamp = t.mode2 & kGlClamp, cs = t.mode2 & kClampS, ct = t.mode2 & kClampT;
    if (gl_clamp && cs) s = std::clamp(s, 0.0f, 1.0f);
    if (gl_clamp && ct) tc = std::clamp(tc, 0.0f, 1.0f);
    // Far outside coordinates: keep the texel index arithmetic in range.
    auto bound = [](float v) { return std::clamp(v, -1.0e6f, 1.0e6f); };
    const float u = bound(s * w), v = bound(tc * h);
    if (!linear) {
        long i = static_cast<long>(std::floor(u)), j = static_cast<long>(std::floor(v));
        if (gl_clamp && cs) i = std::min(i, static_cast<long>(w) - 1);
        if (gl_clamp && ct) j = std::min(j, static_cast<long>(h) - 1);
        return texel(t, lv, i, j);
    }
    const float uu = u - 0.5f, vv = v - 0.5f;
    const float fi = std::floor(uu), fj = std::floor(vv), a = uu - fi, b = vv - fj;
    const long i = static_cast<long>(fi), j = static_cast<long>(fj);
    const auto t00 = texel(t, lv, i, j), t10 = texel(t, lv, i + 1, j), t01 = texel(t, lv, i, j + 1), t11 = texel(t, lv, i + 1, j + 1);
    std::array<float, 4> c{};
    for (unsigned k = 0; k < 4; ++k)
        c[k] = t00[k] * (1 - a) * (1 - b) + t10[k] * a * (1 - b) + t01[k] * (1 - a) * b + t11[k] * a * b;
    return c;
}

std::array<float, 4> ImpactTram::sample(const GeTexture& t, float s, float tc, float lambda) const {
    const u32 m = t.mode2;
    if (!std::isfinite(s) || !std::isfinite(tc)) s = tc = 0;
    if (!(lambda > 0)) return level(t, 0, s, tc, m & kMagLinear);
    const bool linear = m & kMinLinear;
    const bool mipmaps = (m & kMmEnable) && (m & kMipmap);
    const int top = mipmaps ? static_cast<int>(std::min(t.lod & 0xF, 15u)) : 0;
    if (!top) return level(t, 0, s, tc, linear);
    lambda = std::min(lambda, 32.0f);
    if (!(m & kMipLinear)) {
        const int l = std::clamp(static_cast<int>(std::ceil(lambda + 0.5f)) - 1, 0, top);
        return level(t, static_cast<unsigned>(l), s, tc, linear);
    }
    const int l0 = std::clamp(static_cast<int>(std::floor(lambda)), 0, top), l1 = std::min(l0 + 1, top);
    const float f = std::clamp(lambda - static_cast<float>(l0), 0.0f, 1.0f);
    const auto a = level(t, static_cast<unsigned>(l0), s, tc, linear), b = level(t, static_cast<unsigned>(l1), s, tc, linear);
    std::array<float, 4> c{};
    for (unsigned k = 0; k < 4; ++k) c[k] = a[k] + (b[k] - a[k]) * f;
    return c;
}

// TEXMODE1 bits 2:1 pick the environment (modulate, decal, blend,
// alpha-only modulate; GL_REPLACE comes as modulate), 4:3 the components,
// 11:9 the texel class (4 RGB(A), 2 luminance(-alpha), 1 alpha, 3
// intensity). The environment colour: TXENV_RG red 11:0, green 23:12;
// TXENV_B blue 11:0, alpha 23:12.
std::array<float, 4> ImpactTram::environment(const GeTexture& t, const std::array<float, 4>& f, const std::array<float, 4>& tex) {
    const u32 m1 = t.mode1, nc = ((m1 >> 3) & 3) + 1, cls = (m1 >> 9) & 7;
    bool has_c = true, has_a;
    std::array<float, 3> ct{tex[0], tex[0], tex[0]};
    float at = 1;
    switch (cls) {
        case 1: has_c = false, has_a = true, at = tex[0]; break;
        case 2: has_a = nc == 2, at = tex[1]; break;
        case 3: has_a = true, at = tex[0]; break;
        default:
            if (nc >= 3) ct = {tex[0], tex[1], tex[2]}, has_a = nc == 4, at = tex[3];
            else has_a = nc == 2, at = tex[1];
            break;
    }
    const std::array<float, 3> cc{field12(t.env_rg), field12(t.env_rg >> 12), field12(t.env_b)};
    const float ac = field12(t.env_b >> 12);
    std::array<float, 4> out = f;
    switch ((m1 >> 1) & 3) {
        case 1:  // decal: RGB replaces; RGBA blends by the texel's alpha
            if (has_c) {
                const float a = has_a ? at : 1;
                for (unsigned k = 0; k < 3; ++k) out[k] = f[k] * (1 - a) + ct[k] * a;
            }
            break;
        case 2:  // blend toward the environment colour
            if (has_c)
                for (unsigned k = 0; k < 3; ++k) out[k] = f[k] * (1 - ct[k]) + cc[k] * ct[k];
            if (has_a) out[3] = cls == 3 ? f[3] * (1 - at) + ac * at : f[3] * at;
            break;
        case 3:  // alpha-only modulate
            if (has_a) out[3] = f[3] * at;
            break;
        default:  // modulate
            if (has_c)
                for (unsigned k = 0; k < 3; ++k) out[k] = f[k] * ct[k];
            if (has_a) out[3] = f[3] * at;
            break;
    }
    return out;
}

unsigned ImpactTram::read_texel_bytes(u32 xfrmode) {
    const u32 f = (xfrmode >> 4) & 0xF, n = f == 9 ? 1 : f == 0xA ? 2 : f == 7 ? 3 : 4;
    return n * ((xfrmode & 0xF) == 1 ? 2 : 1);
}

namespace {

// A 12-bit component as the transfer carries it: 16 bits by repeating it,
// or its top byte.
void put_component(std::vector<u8>& out, u32 v, bool wide) {
    if (wide) {
        const u32 x = (v << 4 | v >> 8) & 0xFFFF;
        out.push_back(static_cast<u8>(x >> 8));
        out.push_back(static_cast<u8>(x));
    } else {
        out.push_back(static_cast<u8>(v >> 4));
    }
}

}  // namespace

std::vector<u8> ImpactTram::read_line(const GeTexture& t, u32 xfrmode, unsigned line, unsigned w) const {
    const u32 f = (xfrmode >> 4) & 0xF, n = f == 9 ? 1 : f == 0xA ? 2 : f == 7 ? 3 : 4;
    const bool wide = (xfrmode & 0xF) == 1;
    const u32 cls = (t.mode1 >> 9) & 7, nc = ((t.mode1 >> 3) & 3) + 1;
    std::vector<u8> out;
    out.reserve(std::size_t{w} * read_texel_bytes(xfrmode));
    for (unsigned k = 0; k < w; ++k) {
        const auto tf = texel(t, 0, k, line);
        std::array<u32, 4> c{};
        for (unsigned i = 0; i < 4; ++i) c[i] = static_cast<u32>(std::lround(tf[i] * 4095.0f));
        std::array<u32, 4> rgba;
        switch (cls) {
            case 1: rgba = {0, 0, 0, c[0]}; break;
            case 2: rgba = {c[0], 0, 0, nc == 2 ? c[1] : 0xFFF}; break;
            case 3: rgba = {c[0], 0, 0, 0xFFF}; break;
            default: rgba = {c[0], c[1], c[2], nc == 4 ? c[3] : 0xFFF}; break;
        }
        // Two 8-bit components travel as one 16-bit unit, alpha first.
        const std::array<u32, 4> comps = n == 4 ? rgba : n == 2 && !wide ? std::array<u32, 4>{c[1], c[0], 0, 0} : c;
        for (unsigned i = 0; i < n; ++i) put_component(out, comps[i], wide);
    }
    return out;
}

std::vector<u8> ImpactTram::read_raw(const GeTexture& t, u32 xfrmode, unsigned line, unsigned w) const {
    const unsigned nc = ((t.mode1 >> 3) & 3) + 1, sel = (t.mode1 >> 7) & 3, d = depth_nibbles(t.mode2 >> 2);
    const bool wide = (xfrmode & 0xF) == 1;
    const unsigned n = read_texel_bytes(xfrmode) / (wide ? 2 : 1), tn = cell_nibbles(d);
    std::vector<u8> out;
    out.reserve(std::size_t{w} * n * 2);
    for (unsigned s = 0; s < w; ++s) {
        const std::size_t a = texel_addr(t.pages[0], w, 1u << 13, s, line, tn);
        for (unsigned c = 0; c < n; ++c) {
            u32 v = 0;
            if (c < nc) {
                const std::size_t at = a + comp_offset(sel, nc, c, d);
                for (unsigned i = 0; i < d; ++i) v |= nibble((at + i) % kNibbles) << (4 * i);
            }
            const u32 v16 = d == 1 ? v * 0x1111 : d == 2 ? v * 0x101 : (v << 4 | v >> 8);
            if (wide) out.push_back(static_cast<u8>(v16 >> 8)), out.push_back(static_cast<u8>(v16));
            else out.push_back(static_cast<u8>(v16 >> 8));
        }
    }
    return out;
}

}  // namespace sg::ip22
