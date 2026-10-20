// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// IMPACT's texture engine (TE1) and its texture memory (TRAM).
//
// TRAM has no address/data port: texels arrive through the raster engine's
// transfer path, placed by the texture loader registers (page, level size,
// sub-image origin, components and depth). The driver hands out 16 KB
// pages; a level starts at its page, and levels of 16x16 texels and smaller
// share one page, packed largest first. A texel cell is four components of
// the texture's depth (4, 8 or 12 bits), rounded up to a power of two, so a
// page holds what it holds on the board whatever the component count;
// textures of one or two components can share cells (select slots). A
// level's border texels go to the upper half of its border page: the
// bottom row, the top row, then the left and right columns.
//
// Sampling takes texture coordinates (1.0 across the texture) and a level
// of detail, wraps or clamps them, filters within a level (nearest or
// bilinear) and between mipmap levels, and the texture environment then
// combines the texel with the fragment colour.
#pragma once

#include <array>
#include <vector>

#include "core/types.h"
#include "machine/ip22/impact_ge.h"

namespace sg::ip22 {

class ImpactTram {
public:
    static constexpr std::size_t kBytes = 4u << 20;
    static constexpr std::size_t kNibbles = kBytes * 2;
    static constexpr std::size_t kPageNibbles = 16384 * 2;

    // One row of a texture load, 4 bytes a texel.
    void load_line(const GeTexLoad& l, unsigned line, const u8* texels, unsigned n);
    // The filtered texel at (s, t) for level of detail `lambda` (log2 of
    // level-0 texels per pixel), components 0..1 in the texture's order.
    std::array<float, 4> sample(const GeTexture& t, float s, float tc, float lambda) const;
    // The texture environment: fragment colour `f` with texel `tex`.
    static std::array<float, 4> environment(const GeTexture& t, const std::array<float, 4>& f, const std::array<float, 4>& tex);

    // Texel components of the level-`level` texel (i, j), wrapped or
    // clamped (outside the level: the border texel, or the border colour).
    std::array<float, 4> texel(const GeTexture& t, unsigned level, long i, long j) const;

    // Texture readback, row `line`, `w` texels in transfer mode `xfrmode`'s
    // layout: of the level the sampler is aimed at (glGetTexImage: the
    // texture's own components, or RGBA with the class's components where
    // GL puts them), or raw (the texture manager's save of TRAM pages: a
    // view of the pages as a level `w` texels wide, each texel its stored
    // components). 8- or 16-bit components.
    std::vector<u8> read_line(const GeTexture& t, u32 xfrmode, unsigned line, unsigned w) const;
    std::vector<u8> read_raw(const GeTexture& t, u32 xfrmode, unsigned line, unsigned w) const;
    // Bytes a texel takes in a readback (format 9 one component, 0xA two,
    // 7 RGB, else RGBA; type 1 16-bit components).
    static unsigned read_texel_bytes(u32 xfrmode);

    bool empty() const { return tram_.empty(); }

private:
    u32 nibble(std::size_t a) const { return a / 2 < tram_.size() ? (tram_[a / 2] >> (4 * (a & 1))) & 0xF : 0; }
    void set_nibble(std::size_t a, u32 v);
    std::array<float, 4> level(const GeTexture& t, unsigned level, float s, float tc, bool linear) const;

    std::vector<u8> tram_;  // made on first load
};

}  // namespace sg::ip22
