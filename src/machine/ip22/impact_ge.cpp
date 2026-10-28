// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "machine/ip22/impact_ge.h"

#include "core/endian.h"
#include "machine/ip22/impact_te.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>

namespace sg::ip22 {

namespace {

// Tokens (command numbers below 0x200).
enum : u32 {
    kVertex = 0x000, kColour = 0x002, kNormal = 0x007, kTexCoord = 0x008, kInitRgb = 0x009, kInitCi = 0x00A,
    kTexOn = 0x00B, kTexOff = 0x00C, kLightingOn = 0x00D, kLightingOff = 0x00E, kEdgeFlagOn = 0x00F,
    kEdgeFlagOff = 0x010, kMaterial = 0x011,
    kLightAmbient = 0x012, kLightDiffuse = 0x013, kLightSpecular = 0x014, kClear = 0x015,
    kBeginPoints = 0x016, kBeginPolygon = 0x01F, kEndPoints = 0x020, kEndPolygon = 0x029,
    kMatrixMode = 0x02A, kLoadMatrix = 0x02B, kLoadIdentity = 0x02C, kMultMatrix = 0x02D, kPopMatrix = 0x02E,
    kPushMatrix = 0x02F, kRotate = 0x030, kScale = 0x031, kTranslate = 0x032, kViewport = 0x033, kFrustum = 0x034,
    kOrtho = 0x035, kFlush = 0x036, kRasterPos = 0x038, kScissor = 0x039, kStencilMask = 0x03A, kColourMask = 0x03B,
    kDepthMask = 0x03C, kIndexMask = 0x03D, kAlphaFunc = 0x03E, kBlendFunc = 0x03F, kLogicOp = 0x040,
    kStencilFunc = 0x041, kStencilOp = 0x042, kDepthFunc = 0x043, kReadBuffer = 0x044, kLineWidth = 0x045,
    kLineStipple = 0x046, kShadeModel = 0x047, kDepthRange = 0x048, kDrawBuffer = 0x049, kClearDepthBuffer = 0x04A,
    kClearStencilBuffer = 0x04C, kPointSize = 0x04D, kLoadPolygonStipple = 0x04E, kLightPosition = 0x04F,
    kLightQuadratic = 0x055, kFrontFace = 0x056, kLightModelAmbient = 0x057, kLightModelLocalViewer = 0x058,
    kLightModelTwoSide = 0x059, kBlendEquation = 0x060, kAlphaTest = 0x064, kBlend = 0x065, kDepthTest = 0x066,
    kDither = 0x067, kFog = 0x068, kLineSmooth = 0x069, kLineStippleOn = 0x06A, kLogicOpOn = 0x06B,
    kPointSmooth = 0x06C, kPolygonStippleOn = 0x06D, kScissorTest = 0x06E, kStencilTest = 0x06F, kLightOn = 0x071,
    kCullOn = 0x072, kCullOff = 0x073, kCullFace = 0x074, kEnableCap = 0x075, kDisableCap = 0x076,
    kValidateBanks = 0x098, kInitFormatValues = 0x09A, kLightOff = 0x0A0, kNormalize = 0x0A3, kClipPlane = 0x0A5,
    kClipPlaneOn = 0x0A6, kClipPlaneOff = 0x0A7, kPolygonMode = 0x0A8, kFogColour = 0x0B4, kFogStart = 0x0B5,
    kFogEnd = 0x0B6, kFogDensity = 0x0B7, kFogIndex = 0x0B8, kFogMode = 0x0B9, kClearColour = 0x0BA,
    kClearDepth = 0x0BC, kClearIndex = 0x0BD, kClearStencil = 0x0BE, kColourMaterialFront = 0x0BF,
    kColourMaterialBack = 0x0C0, kColourMaterialBoth = 0x0C1, kColourMaterialOn = 0x0C2, kColourMaterialOff = 0x0C3,
    kUpdateRasterPos = 0x0D6, kLoadRasterPosInfo = 0x0D7, kBlendColour = 0x0D9, kSwapTmesh = 0x0DC,
    kLmcColourOn = 0x0DD, kLmcColourOff = 0x0DE, kIrisNormal = 0x0DF, kPolygonOffset = 0x0E7, kPolygonOffsetModes = 0x0E8,
    kGetMaterial = 0x0EE, kGetLight = 0x0EF, kBitmapRows = 0x090, kGeneralBitmap = 0x091, kBitmapMove = 0x092,
    kBitmap = 0x094, kRssRegShadow = 0x07C, kRssRegSet = 0x07F, kPixelState = 0x080, kSendPixels = 0x08D,
    kInitColourTable = 0x089, kTlNoTable = 0x09B, kTlSubloadEnd = 0x09C, kWriteDmaGeSetup = 0x09D,
    kInitPixelStateEram = 0x0CD, kSaveRss = 0x0D2, kRestoreRss = 0x0D3, kGetPixels = 0x0DB, kSaveTexmode = 0x0E9,
    kRestoreTexmode = 0x0EA, kRssRegShadowIdx = 0x0EC, kTexGeniv = 0x05A, kTexGenPlaneHeader = 0x05B,
    kTexGenPlaneData = 0x05C, kReadTexture = 0x0F0, kReadTextureRaw = 0x0E5,
    kDisable = 0x05F, kSetData = 0x0ED, kTexBase = 0x0EB, kRenderModeNote = 0x08C, kSyncForRead = 0x0F2,
    kTramRestore = 0x0D8, kTramRestorePrep = 0x0E3, kReadBegin = 0x096, kReadEnd = 0x097, kIglooNote = 0x102,
    kTramSaveEnd = 0x0DA, kPad = 0x001, kMakeCurrentRead = 0x006, kInitTables = 0x082,
};

// GE microcode routines are named by the source file they lie in (the
// microcode's line table). Entry points differ between microcode builds
// and IRIX releases, but each stays within its file's range: these are the
// files' starts in IRIX 6.5's mgras_ge.u (the double-buffered builds run a
// few words later, 6.5.22's entries a few words on).
enum Module : u32 {
    kModNone, kModClamp, kModExpand, kModShiftAdd, kModMask, kModLut, kModScaleBias, kModIntToFloat, kModBitmapToFloat,
    kModBitmapToInt, kModPipeControl, kModZoom, kModUnpack, kModCompress, kModColMatrix, kModXZoom, kModCopyPixels, kModTexLoad,
};
constexpr struct {
    u32 start;
    Module mod;
} kUcodeFiles[] = {
    {0x152F, kModNone},        {0x1569, kModClamp},         {0x161D, kModNone},          {0x1654, kModExpand},
    {0x1864, kModShiftAdd},    {0x1A3D, kModMask},          {0x1AC3, kModLut},           {0x1E3D, kModScaleBias},
    {0x1F43, kModIntToFloat},  {0x1FD9, kModNone},          {0x24D6, kModBitmapToFloat}, {0x259F, kModBitmapToInt},
    {0x2689, kModNone},        {0x3398, kModPipeControl},   {0x365B, kModZoom},          {0x386C, kModUnpack},
    {0x3992, kModCompress},    {0x3AD3, kModNone},          {0x3E5D, kModColMatrix},     {0x3EED, kModXZoom},
    {0x419C, kModNone},        {0x4500, kModCopyPixels},    {0x4AAC, kModNone},          {0x4EBA, kModTexLoad},
    {0x5282, kModNone},
};

Module module_at(u32 entry) {
    Module m = kModNone;
    for (const auto& f : kUcodeFiles)
        if (entry >= f.start) m = f.mod;
    return m;
}

// TE1 registers. The GE keeps the context's (kTeRegs; TXADDR and the
// per-level tables it indexes apart) and samples with them.
enum : u32 {
    kTexMode1 = 0x111, kTxEnvRg = 0x142, kTxEnvB = 0x143, kTexMode2 = 0x180, kTxSize = 0x181, kTxLod = 0x183,
    kTxBColourRg = 0x184, kTxBColourBa = 0x185, kTxAddr = 0x190, kTxMipmap = 0x191, kTxBorder = 0x193,
    kDetailScale = 0x194, kTlMode = 0x1A1, kTlSpec = 0x1A2, kTlMipmap = 0x1A4, kTlBorder = 0x1A5,
    kTlSLeft = 0x1AB, kTlTBottom = 0x1AC,
};
constexpr u32 kTeRegs[31] = {
    0x111, 0x142, 0x143, 0x180, 0x181, 0x182, 0x183, 0x184, 0x185, 0x186, 0x187, 0x188, 0x189, 0x18A, 0x18B, 0x18C,
    0x192, 0x195, 0x1A0, 0x1A1, 0x1A2, 0x1A3, 0x1A4, 0x1A5, 0x1A6, 0x1A7, 0x1A8, 0x1A9, 0x1AA, 0x1AB, 0x1AC,
};
constexpr u32 kTeTables[3] = {kTxMipmap, kTxBorder, kDetailScale};

constexpr u32 kPixProgram = 0x13B, kF2_23 = 0x4B000000;
// ERAM pixel state (IRIX 6.5.22's addresses; IRIX 6.5's are 0x20 lower,
// told apart by where the image rectangle's block lands).
constexpr u32 kPixRect = 0xDA8, kPixTexDest = 0xDB1, kPixElementType = 0xDBB, kPixScale = 0xD0A;

enum : u32 { kPoints = 0, kLines, kLineStrip, kLineLoop, kTriangles, kTriStrip, kTriFan, kQuads, kQuadStrip, kPolygon, kNone = 0xFF };

constexpr u32 kGlLogicOp = 0x0BF1;  // the blend equation that turns GL_BLEND into the logic op
// PP1 fill mode for 24-bit RGB drawing into buffer A: pixel type 2, 36-bit
// buffer, draw field 1, logic op copy (enabled).
constexpr u32 kPp1Rgb24A = 0x0C006204;
constexpr u32 kPp1Dither = 1u << 3;

float f32(u32 w) {
    float f;
    std::memcpy(&f, &w, 4);
    return f;
}

u32 bits(float f) {
    u32 w;
    std::memcpy(&w, &f, 4);
    return w;
}

// Up to four float arguments; missing ones are GL's defaults (0, 0, 0, 1).
std::array<float, 4> args4(const u32* d, unsigned n) {
    std::array<float, 4> v{0, 0, 0, 1};
    for (unsigned i = 0; i < 4 && i < n; ++i) v[i] = f32(d[i]);
    return v;
}

// `count` arguments of a *D token: doubles (high word first) when there
// are two words each, floats otherwise.
void argsd(const u32* d, unsigned n, float* out, unsigned count) {
    for (unsigned i = 0; i < count; ++i) {
        if (n >= 2 * count) {
            const u64 w = (u64{d[2 * i]} << 32) | d[2 * i + 1];
            double v;
            std::memcpy(&v, &w, 8);
            out[i] = static_cast<float>(v);
        } else {
            out[i] = i < n ? f32(d[i]) : 0.0f;
        }
    }
}

using Mat4 = ImpactGe::Mat4;

Mat4 identity() {
    Mat4 m;
    m.m[0] = m.m[5] = m.m[10] = m.m[15] = 1;
    return m;
}

Mat4 mul(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (unsigned c = 0; c < 4; ++c)
        for (unsigned row = 0; row < 4; ++row) {
            float s = 0;
            for (unsigned k = 0; k < 4; ++k) s += a.m[k * 4 + row] * b.m[c * 4 + k];
            r.m[c * 4 + row] = s;
        }
    return r;
}

std::array<float, 4> xform(const Mat4& m, const std::array<float, 4>& v) {
    std::array<float, 4> r{};
    for (unsigned row = 0; row < 4; ++row)
        r[row] = m.m[row] * v[0] + m.m[4 + row] * v[1] + m.m[8 + row] * v[2] + m.m[12 + row] * v[3];
    return r;
}

bool invert(const Mat4& a, Mat4& out) {
    const auto& m = a.m;
    std::array<float, 16> inv{};
    inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
    inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
    inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
    inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
    inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
    inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
    inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
    inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];
    const float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
    if (det == 0) return false;
    for (unsigned i = 0; i < 16; ++i) out.m[i] = inv[i] / det;
    return true;
}

Mat4 rotate(float angle, float x, float y, float z) {
    Mat4 m = identity();
    const float len = std::sqrt(x * x + y * y + z * z);
    if (len == 0) return m;
    x /= len, y /= len, z /= len;
    const float a = angle * 3.14159265358979f / 180, c = std::cos(a), s = std::sin(a), t = 1 - c;
    m.m[0] = x * x * t + c, m.m[4] = x * y * t - z * s, m.m[8] = x * z * t + y * s;
    m.m[1] = y * x * t + z * s, m.m[5] = y * y * t + c, m.m[9] = y * z * t - x * s;
    m.m[2] = x * z * t - y * s, m.m[6] = y * z * t + x * s, m.m[10] = z * z * t + c;
    return m;
}

float dot3(const std::array<float, 3>& a, const std::array<float, 3>& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

std::array<float, 3> normalised(std::array<float, 3> v) {
    const float l = std::sqrt(dot3(v, v));
    if (l > 1e-20f) v = {v[0] / l, v[1] / l, v[2] / l};
    return v;
}

// A plane times the inverse of `m`: object space to eye space.
std::array<float, 4> plane_to_eye(const std::array<float, 4>& p, const Mat4& m) {
    Mat4 inv;
    if (!invert(m, inv)) inv = identity();
    std::array<float, 4> r{};
    for (unsigned c = 0; c < 4; ++c) r[c] = p[0] * inv.m[c * 4] + p[1] * inv.m[c * 4 + 1] + p[2] * inv.m[c * 4 + 2] + p[3] * inv.m[c * 4 + 3];
    return r;
}

ImpactGe::PVertex lerp(const ImpactGe::PVertex& a, const ImpactGe::PVertex& b, float t) {
    ImpactGe::PVertex v;
    for (unsigned k = 0; k < 4; ++k) {
        v.h[k] = a.h[k] + (b.h[k] - a.h[k]) * t;
        v.e[k] = a.e[k] + (b.e[k] - a.e[k]) * t;
        v.c[k] = a.c[k] + (b.c[k] - a.c[k]) * t;
        v.cb[k] = a.cb[k] + (b.cb[k] - a.cb[k]) * t;
        v.t[k] = a.t[k] + (b.t[k] - a.t[k]) * t;
    }
    v.fog = a.fog + (b.fog - a.fog) * t;
    return v;
}

}  // namespace

ImpactGe::ImpactGe(ImpactRaster& raster) : raster_(raster) { reset(); }

void ImpactGe::reset() { s_ = State{}; }

void ImpactGe::init_state() {
    State n{};
    // The window stays: the kernel gives it with the context, not GL.
    n.win_valid = s_.win_valid, n.win_origin = s_.win_origin, n.win_mode = s_.win_mode, n.win_pp1 = s_.win_pp1;
    n.win_drb = s_.win_drb, n.win_masks = s_.win_masks;
    n.swapped = s_.swapped, n.banks_known = s_.banks_known;
    s_ = n;
    s_.inited = 1;
    for (Mat4& m : s_.mv.m) m = identity();
    for (Mat4& m : s_.proj.m) m = identity();
    for (Mat4& m : s_.tex.m) m = identity();
    for (Mat4& m : s_.colour.m) m = identity();
    for (unsigned i = 0; i < kLights; ++i) {
        Light& l = s_.lights[i];
        const float on = i == 0 ? 1.0f : 0.0f;
        l.ambient = {0, 0, 0, 1};
        l.diffuse = {on, on, on, 1};
        l.specular = {on, on, on, 1};
        l.position = {0, 0, 1, 0};
        l.spot_dir = {0, 0, -1};
        l.spot_cutoff = 180;
        l.atten = {1, 0, 0};
    }
    s_.texgen_obj[0] = s_.texgen_eye[0] = {1, 0, 0, 0};
    s_.texgen_obj[1] = s_.texgen_eye[1] = {0, 1, 0, 0};
    for (Material& m : s_.mat) {
        m.ambient = {0.2f, 0.2f, 0.2f, 1};
        m.diffuse = {0.8f, 0.8f, 0.8f, 1};
        m.specular = {0, 0, 0, 1};
        m.emission = {0, 0, 0, 1};
        m.indexes = {0, 1, 1};
    }
}

ImpactGe::Mat4& ImpactGe::top() {
    switch (s_.matrix_mode) {
        case 0x1701: return s_.proj.m[s_.proj.top];
        case 0x1702: return s_.tex.m[s_.tex.top];
        case 0x1800: return s_.colour.m[s_.colour.top];
        default: return s_.mv.m[s_.mv.top];
    }
}

void ImpactGe::matrix_mult(const Mat4& m) {
    Mat4& t = top();
    t = mul(t, m);
}

ImpactGe::Mat4 ImpactGe::mvp() const { return mul(s_.proj.m[s_.proj.top], s_.mv.m[s_.mv.top]); }

// ---- state readback -------------------------------------------------------

std::optional<u32> ImpactGe::state_word(u32 addr) {
    ensure_init();
    const State& s = s_;
    auto on = [](u32 v) { return v ? 1u : 0u; };
    const Mat4& mv = s.mv.m[s.mv.top];
    const Mat4& proj = s.proj.m[s.proj.top];
    const Mat4& tex = s.tex.m[s.tex.top];
    if (addr == 4 && s.ci) return bits(s.index_now);
    if (addr >= 4 && addr <= 7) return bits(s.colour_now[addr - 4]);
    if (addr >= 8 && addr <= 10) return bits(s.normal[addr - 8]);
    if (addr >= 11 && addr <= 14) return bits(s.texcoord[addr - 11]);
    if (addr >= 0x3A && addr <= 0x49) {
        const Mat4& m = s.matrix_mode == 0x1701 ? proj : s.matrix_mode == 0x1702 ? tex : mv;
        return bits(m.m[addr - 0x3A]);
    }
    if (addr >= 0x4A && addr <= 0x59) return bits(proj.m[addr - 0x4A]);
    if (addr >= 0x6A && addr <= 0x79) return bits(tex.m[addr - 0x6A]);
    if (addr >= 0x1B9 && addr <= 0x1C8) return bits(s.colour.m[s.colour.top].m[addr - 0x1B9]);
    if (addr >= 0x2C5 && addr <= 0x2C8) return bits(s.raster_colour[addr - 0x2C5]);
    if (addr >= 0x2C9 && addr <= 0x2CC) return bits(s.raster_tex[addr - 0x2C9]);
    if (addr >= 0x32E && addr <= 0x331) return s.scratch[addr - 0x32E];
    if (addr >= 0x4D2 && addr <= 0x4D5) return bits(s.lmodel_ambient[addr - 0x4D2]);
    if (addr >= 0xAFB && addr <= 0xB00) return on((s.clip_on >> (addr - 0xAFB)) & 1) << 31;
    if (addr >= 0xB01 && addr <= 0xB18) return bits(s.clip_planes[(addr - 0xB01) / 4][(addr - 0xB01) % 4]);
    if (addr >= 0xB46 && addr <= 0xB49) return bits(s.fog_colour[addr - 0xB46]);
    if (addr >= 0xB7E && addr <= 0xB81) return bits(s.clear_colour[addr - 0xB7E]);
    if (addr >= 0xB98 && addr <= 0xB9B) {
        const auto r = scissor_rect();
        const int v[4] = {r[0], r[2], r[1], r[3]};
        return static_cast<u32>(v[addr - 0xB98]);
    }
    switch (addr) {
        case 3: return 0;  // read before texture loads and pixel operations: a pipeline round trip
        case 0x29: return s.matrix_mode - 0x1701;
        case 0x83: return 0x899 + s.mv.top * 16;
        case 0x896: return 0xA99 + s.proj.top * 16;
        case 0x897: return 0xAB9 + s.tex.top * 16;
        // Enables in PP1 fill-mode style: polygon stipple bit 0, line stipple bit 5, fog bit 17.
        case 0x1FA: return on(s.poly_stipple) | on(s.line_stipple) << 5 | on(s.fog) << 17;
        // Dither bit 0, stencil test 4, alpha test 6, the logic op in 29:26.
        case 0x1FC: return on(s.dither) | on(s.stencil) << 4 | on(s.alpha_test) << 6 | (s.logic_op & 0xF) << 26;
        case 0x200: return (static_cast<u32>(scissor_rect()[1]) << 16) | (static_cast<u32>(scissor_rect()[0]) & 0xFFFF);
        case 0x202: return (static_cast<u32>(scissor_rect()[3]) << 16) | (static_cast<u32>(scissor_rect()[2]) & 0xFFFF);
        case 0x23A: return (s.st_func & 7) | (s.st_ops[0] & 7) << 3 | (s.st_ops[1] & 7) << 6 | (s.st_ops[2] & 7) << 9 | (s.st_ref & 0xFF) << 12;
        case 0x23C: return (s.st_cmask & 0xFF) | (s.st_wmask & 0xFF) << 8;
        case 0x23E: return (s.depth_func & 7) | (s.depth_mask ? 0xFFFFFFu << 4 : 0);
        case 0x240: return (s.alpha_func & 7) | (s.alpha_ref & 0xFFF) << 3;
        case 0x29F: return s.ci ? s.index_mask : s.colour_mask;
        case 0x2A0: return 0;  // the error word
        case 0x2BF: return bits(s.raster_dist);
        case 0x2C4: return bits(1.0f);
        // The raster position, biased as the raster engine takes positions.
        case 0x2CD: return bits(s.raster[0] + 49151.5f);
        case 0x2CE: return bits(s.raster[1] + 49151.5f);
        case 0x2CF: return bits(s.raster[2] / kZMax);
        case 0x2D0: return s.raster_valid ? 7 : 0;
        case 0x4DF: return s.cmat_param;
        case 0x4E0: return s.cmat_face;
        case 0xB42: return s.fog_mode;
        case 0xB43: return bits(s.fog_density);
        case 0xB44: return bits(s.fog_start);
        case 0xB45: return bits(s.fog_end);
        case 0xB4A: return s.fog_index;
        case 0xB86: return s.clear_depth & 0xFFFFFF;
        case 0xB87: return bits(static_cast<float>(s.clear_index));
        case 0xB88: return s.clear_stencil;
        case 0xB8B: return s.polygon_mode[0];
        case 0xB8C: return s.polygon_mode[1];
        case 0xB8D: return s.smooth ? 0x1D01 : 0x1D00;
        case 0xB8E: return s.local_viewer;
        case 0xB8F: return s.two_side;
        case 0xB91: return static_cast<u32>(static_cast<int>(s.viewport[0]));
        case 0xB92: return static_cast<u32>(static_cast<int>(s.viewport[1]));
        case 0xB93: return static_cast<u32>(static_cast<int>(s.viewport[2]));
        case 0xB94: return static_cast<u32>(static_cast<int>(s.viewport[3]));
        case 0xB95: return bits(s.depth_range[0]);
        case 0xB96: return bits(s.depth_range[1]);
        case 0xB9C: return s.scissor_test;
        case 0xBA4: return s.line_smooth;
        case 0xBB1: return bits(s.point_size);
        case 0xBB2: return bits(s.line_width);
        case 0xBB3: return s.cull_mode;
        case 0xBB4: return s.front_ccw ? 0x901 : 0x900;
        case 0xBBA: return s.line_pattern;
        case 0xBBB: return s.line_repeat;
        case 0xBBE: return s.blend;
        case 0xBBF: return s.blend_src;
        case 0xBC0: return s.blend_dst;
        case 0xBC1: return s.blend_eq ? s.blend_eq : 0x8006;
        case 0xC16: return bits(s.polygon_offset[0]);
        case 0xC18: return bits(s.polygon_offset[1]);
        default: break;
    }
    // Words libGLcore set itself (SET_DATA).
    if (addr >= 0xB00 && addr < 0xC00 && (s.set_data_valid[(addr - 0xB00) / 32] >> (addr % 32) & 1)) return s.set_data[addr - 0xB00];
    return std::nullopt;
}

// ---- tokens ---------------------------------------------------------------

void ImpactGe::light_param(u32 cmd, const u32* d, unsigned n) {
    if (!n || d[0] >= kLights) return;
    Light& l = s_.lights[d[0]];
    const auto v = args4(d + 1, n - 1);
    const Mat4& mv = s_.mv.m[s_.mv.top];
    switch (cmd) {
        case kLightAmbient: l.ambient = v; break;
        case kLightDiffuse: l.diffuse = v; break;
        case kLightSpecular: l.specular = v; break;
        case kLightPosition: l.position = xform(mv, {v[0], v[1], v[2], n >= 5 ? v[3] : 1.0f}); break;
        case kLightPosition + 1: {  // spot direction
            const auto e = xform(mv, {v[0], v[1], v[2], 0});
            l.spot_dir = {e[0], e[1], e[2]};
            break;
        }
        case kLightPosition + 2: l.spot_exp = v[0]; break;
        case kLightPosition + 3: l.spot_cutoff = v[0]; break;
        case kLightPosition + 4: l.atten[0] = v[0]; break;
        case kLightPosition + 5: l.atten[1] = v[0]; break;
        case kLightQuadratic: l.atten[2] = v[0]; break;
        default: break;
    }
}

void ImpactGe::material(const u32* d, unsigned n) {
    if (n < 3) return;
    const u32 face = d[0], pname = d[1];
    const auto v = args4(d + 2, n - 2);
    for (unsigned f = 0; f < 2; ++f) {
        if ((face == 0x404 && f == 1) || (face == 0x405 && f == 0)) continue;
        Material& m = s_.mat[f];
        switch (pname) {
            case 0x1200: m.ambient = v; break;
            case 0x1201: m.diffuse = v; break;
            case 0x1202: m.specular = v; break;
            case 0x1600: m.emission = v; break;
            case 0x1601: m.shininess = v[0]; break;
            case 0x1602: m.ambient = m.diffuse = v; break;
            case 0x1603: m.indexes = {v[0], v[1], v[2]}; break;
            default: break;
        }
    }
}

bool ImpactGe::token(u32 cmd, const u32* d, unsigned n) {
    ensure_init();
    State& s = s_;
    const u32 w0 = n ? d[0] : 0;
    const u32 on = w0 != 0;
    // glColorMaterial: the current colour sets the tracked material parameter.
    auto colour_material = [&] {
        if (!s.cmat_on) return;
        const u32 data[6] = {s.cmat_face, s.cmat_param, bits(s.colour_now[0]), bits(s.colour_now[1]), bits(s.colour_now[2]), bits(s.colour_now[3])};
        material(data, 6);
    };
    switch (cmd) {
        case kMatrixMode: s.matrix_mode = n ? w0 : 0x1700; break;
        case kLoadIdentity: top() = identity(); break;
        case kPushMatrix:
            switch (s.matrix_mode) {
                case 0x1701: if (s.proj.top + 1 < 4) s.proj.m[s.proj.top + 1] = s.proj.m[s.proj.top], ++s.proj.top; break;
                case 0x1702: if (s.tex.top + 1 < 4) s.tex.m[s.tex.top + 1] = s.tex.m[s.tex.top], ++s.tex.top; break;
                case 0x1800: if (s.colour.top + 1 < 2) s.colour.m[s.colour.top + 1] = s.colour.m[s.colour.top], ++s.colour.top; break;
                default: if (s.mv.top + 1 < 32) s.mv.m[s.mv.top + 1] = s.mv.m[s.mv.top], ++s.mv.top; break;
            }
            break;
        case kPopMatrix:
            switch (s.matrix_mode) {
                case 0x1701: if (s.proj.top) --s.proj.top; break;
                case 0x1702: if (s.tex.top) --s.tex.top; break;
                case 0x1800: if (s.colour.top) --s.colour.top; break;
                default: if (s.mv.top) --s.mv.top; break;
            }
            break;
        case kLoadMatrix:
        case kMultMatrix: {
            Mat4 m;
            argsd(d, n, m.m.data(), 16);
            if (cmd == kLoadMatrix) top() = m;
            else matrix_mult(m);
            break;
        }
        case kRotate: {
            float a[4];
            argsd(d, n, a, 4);
            matrix_mult(rotate(a[0], a[1], a[2], a[3]));
            break;
        }
        case kTranslate:
        case kScale: {
            float a[3];
            argsd(d, n, a, 3);
            Mat4 m = identity();
            if (cmd == kTranslate) m.m[12] = a[0], m.m[13] = a[1], m.m[14] = a[2];
            else m.m[0] = a[0], m.m[5] = a[1], m.m[10] = a[2];
            matrix_mult(m);
            break;
        }
        case kOrtho:
        case kFrustum: {
            float a[6];
            argsd(d, n, a, 6);
            const float l = a[0], r = a[1], b = a[2], t = a[3], nr = a[4], f = a[5];
            Mat4 m;
            if (r == l || t == b || f == nr) break;
            if (cmd == kOrtho) {
                m = identity();
                m.m[0] = 2 / (r - l), m.m[5] = 2 / (t - b), m.m[10] = -2 / (f - nr);
                m.m[12] = -(r + l) / (r - l), m.m[13] = -(t + b) / (t - b), m.m[14] = -(f + nr) / (f - nr);
            } else {
                m.m[0] = 2 * nr / (r - l), m.m[5] = 2 * nr / (t - b);
                m.m[8] = (r + l) / (r - l), m.m[9] = (t + b) / (t - b), m.m[10] = -(f + nr) / (f - nr), m.m[11] = -1;
                m.m[14] = -2 * f * nr / (f - nr);
            }
            matrix_mult(m);
            break;
        }
        case kViewport:
            if (n < 4) return false;
            for (unsigned i = 0; i < 4; ++i) s.viewport[i] = static_cast<float>(static_cast<s32>(d[i]));
            break;
        case kDepthRange: {
            float a[2];
            argsd(d, n, a, 2);
            s.depth_range = {std::clamp(a[0], 0.0f, 1.0f), std::clamp(a[1], 0.0f, 1.0f)};
            break;
        }
        case kShadeModel: s.smooth = w0 != 0x1D00; break;
        case kColour:
            s.colour_now = args4(d, n);
            if (s.ci) {
                // An index: red carries it to the raster side.
                s.index_now = s.colour_now[0];
                s.colour_now = {s.index_now / 4095.0f, 0, 0, 1};
            }
            // The raster colour follows the current colour (IRIS GL sets a
            // colour between bitmaps without a new raster position).
            s.raster_colour = s.colour_now;
            colour_material();
            if (s.lmc_colour && !s.cmat_on) s.unlit = 1;
            break;
        case kNormal:
        case kIrisNormal: {
            const auto a = args4(d, n);
            s.normal = {a[0], a[1], a[2]};
            s.unlit = 0;
            break;
        }
        case kTexCoord: s.texcoord = args4(d, n); break;
        case kLmcColourOn: s.lmc_colour = 1; break;
        case kLmcColourOff: s.lmc_colour = 0, s.unlit = 0; break;
        case kSwapTmesh: std::swap(s.vb[0], s.vb[1]), s.tflip ^= 1; break;
        case kLightingOn: s.lighting = 1; break;
        case kLightingOff: s.lighting = 0; break;
        case kMaterial: material(d, n); break;
        case kLightAmbient:
        case kLightDiffuse:
        case kLightSpecular: light_param(cmd, d, n); break;
        case kLightModelAmbient: s.lmodel_ambient = args4(d, n); break;
        case kLightModelLocalViewer: s.local_viewer = on; break;
        case kLightModelTwoSide: s.two_side = on; break;
        case kLightOn:
        case kLightOff:
            if (w0 >= kLights) return false;
            if (cmd == kLightOn) s.lights_on |= 1u << w0;
            else s.lights_on &= ~(1u << w0);
            break;
        case kColourMaterialFront:
        case kColourMaterialBack:
        case kColourMaterialBoth:
            s.cmat_face = cmd == kColourMaterialFront ? 0x404 : cmd == kColourMaterialBack ? 0x405 : 0x408;
            s.cmat_param = w0;
            colour_material();
            break;
        case kColourMaterialOn: s.cmat_on = 1, colour_material(); break;
        case kColourMaterialOff: s.cmat_on = 0; break;
        case kNormalize: s.normalize = on; break;
        case kClipPlane:
            if (n < 5 || w0 - 0x3000 >= 6) return false;
            s.clip_planes[w0 - 0x3000] = plane_to_eye({f32(d[1]), f32(d[2]), f32(d[3]), f32(d[4])}, s.mv.m[s.mv.top]);
            break;
        case kClipPlaneOn:
        case kClipPlaneOff:
            if (w0 - 0x3000 >= 6) return false;
            if (cmd == kClipPlaneOn) s.clip_on |= 1u << (w0 - 0x3000);
            else s.clip_on &= ~(1u << (w0 - 0x3000));
            break;
        case kFog: s.fog = on; break;
        case kFogMode: s.fog_mode = w0; break;
        case kFogColour: s.fog_colour = args4(d, n); break;
        case kFogIndex: s.fog_index = w0; break;
        case kFogStart: s.fog_start = f32(w0); break;
        case kFogEnd: s.fog_end = f32(w0); break;
        case kFogDensity: s.fog_density = f32(w0); break;
        case kLineSmooth: s.line_smooth = on; break;
        case kPointSmooth: s.point_smooth = on; break;
        case kPolygonOffset:
            if (n < 2) return false;
            s.polygon_offset = {f32(d[0]), f32(d[1]), n >= 3 ? f32(d[2]) : 1.1f};
            break;
        case kPolygonOffsetModes:
            // glEnable(GL_POLYGON_OFFSET_FILL / LINE / POINT) set bits 0 / 1 / 2
            // of the word libGLcore keeps, sent whole each time.
            s.polygon_offset_on = w0 & 7;
            break;
        case kGetLight:
        case kGetMaterial: {
            if (n < 2) return false;
            const u32 pname = d[1];
            std::array<float, 4> v{};
            if (cmd == kGetLight) {
                if (w0 >= kLights) return false;
                const Light& l = s.lights[w0];
                switch (pname) {
                    case 0x1200: v = l.ambient; break;
                    case 0x1201: v = l.diffuse; break;
                    case 0x1202: v = l.specular; break;
                    case 0x1203: v = l.position; break;
                    case 0x1204: v = {l.spot_dir[0], l.spot_dir[1], l.spot_dir[2], 0}; break;
                    case 0x1205: v[0] = l.spot_exp; break;
                    case 0x1206: v[0] = l.spot_cutoff; break;
                    case 0x1207: v[0] = l.atten[0]; break;
                    case 0x1208: v[0] = l.atten[1]; break;
                    case 0x1209: v[0] = l.atten[2]; break;
                    default: break;
                }
            } else {
                const Material& m = s.mat[w0 == 0x405 ? 1 : 0];
                switch (pname) {
                    case 0x1200: v = m.ambient; break;
                    case 0x1201: v = m.diffuse; break;
                    case 0x1202: v = m.specular; break;
                    case 0x1600: v = m.emission; break;
                    case 0x1601: v[0] = m.shininess; break;
                    case 0x1603: v = {m.indexes[0], m.indexes[1], m.indexes[2], 0}; break;
                    default: break;
                }
            }
            for (unsigned i = 0; i < 4; ++i) s.scratch[i] = bits(v[i]);
            break;
        }
        case kClearColour:
            // IRIS GL clear() through IGLOO sends the current colour's GE
            // addresses (4-7) rather than floats.
            if (n >= 4 && std::all_of(d, d + 4, [](u32 w) { return w < 0x100; })) {
                for (unsigned k = 0; k < 4; ++k) s.clear_colour[k] = d[k] >= 4 && d[k] <= 7 ? s.colour_now[d[k] - 4] : 0.0f;
            } else {
                s.clear_colour = args4(d, n);
            }
            break;
        case kClearDepth: s.clear_depth = w0 & 0xFFFFFF; break;
        case kClearStencil: s.clear_stencil = w0 & 0xFF; break;
        case kClearIndex: s.clear_index = (w0 >> 16) == 0 ? w0 : static_cast<u32>(f32(w0)); break;
        case kClear: clear_colour_buffer(); break;
        case kClearDepthBuffer: clear_zst(true); break;
        case kClearStencilBuffer: clear_zst(false); break;
        case kDepthTest: s.depth_test = on; break;
        case kDepthFunc: s.depth_func = w0 & 7; break;
        case kDepthMask: s.depth_mask = on; break;
        case kAlphaTest: s.alpha_test = on; break;
        case kAlphaFunc:
            if (n < 2) return false;
            s.alpha_func = w0 & 7, s.alpha_ref = std::min<u32>(d[1], 0xFFF);
            break;
        case kBlend: s.blend = on; break;
        case kBlendFunc:
            if (n < 2) return false;
            s.blend_src = w0, s.blend_dst = d[1];
            break;
        case kBlendEquation: s.blend_eq = w0; break;
        case kBlendColour:
            if (n < 4) return false;
            for (unsigned k = 0; k < 4; ++k) s.blend_colour[k] = std::clamp(f32(d[k]), 0.0f, 1.0f);
            break;
        case kLogicOpOn: s.logic = on; break;
        case kLogicOp: s.logic_op = w0 & 0xF; break;
        case kStencilTest: s.stencil = on; break;
        case kStencilFunc:
            if (n < 3) return false;
            s.st_func = w0 & 7, s.st_ref = d[1] & 0xFF, s.st_cmask = d[2] & 0xFF;
            break;
        case kStencilOp:
            if (n < 3) return false;
            s.st_ops = {w0 & 7, d[1] & 7, d[2] & 7};
            break;
        case kStencilMask: s.st_wmask = w0 & 0xFF; break;
        case kColourMask: s.colour_mask = w0 & 0xF; break;
        case kIndexMask: s.index_mask = w0 & 0xFFF; break;
        case kDither: s.dither = on; break;
        case kScissorTest: s.scissor_test = on; break;
        case kScissor:
            if (n < 4) return false;
            for (unsigned i = 0; i < 4; ++i) s.scissor[i] = static_cast<s32>(d[i]);
            break;
        case kCullOn: s.cull = 1; break;
        case kCullOff: s.cull = 0; break;
        case kCullFace: s.cull_mode = n ? w0 : 0x405; break;
        case kFrontFace: s.front_ccw = w0 != 0x900; break;
        case kEdgeFlagOn: s.edge_flag = 1; break;  // glEdgeFlag(GL_TRUE), and a new context's default
        case kEdgeFlagOff: s.edge_flag = 0; break;
        case kPolygonMode:
            if (n < 2) return false;
            if (w0 == 0x404) s.polygon_mode[0] = d[1];
            else if (w0 == 0x405) s.polygon_mode[1] = d[1];
            else s.polygon_mode = {d[1], d[1]};
            break;
        case kLineWidth: s.line_width = std::max(1.0f, f32(w0)); break;
        case kPointSize: s.point_size = std::max(1.0f, f32(w0)); break;
        case kLineStippleOn: s.line_stipple = on; break;
        case kLineStipple:
            if (n < 2) return false;
            s.line_repeat = w0 & 0xFF, s.line_pattern = d[1] & 0xFFFF;
            break;
        case kPolygonStippleOn: s.poly_stipple = on; break;
        case kLoadPolygonStipple:
            if (n < 32) return false;
            std::copy(d, d + 32, s.stipple_rows.begin());
            break;
        case kFlush: break;
        case kValidateBanks: set_draw_bank(w0); break;
        case kDrawBuffer:
            s.draw_bits = w0;
            s.draw_words = n >= 3;
            s.draw_bits_swapped = (n >= 2 ? d[1] : 1) & 0x7F;
            s.buffer_count = (n >= 3 ? d[2] : 0) & 1;
            break;
        case kReadBuffer: s.read_back = n >= 3 && d[2] == 0x405; break;
        case kInitFormatValues: s.pixel_format = kFormatSent | (w0 & 0x2700); break;
        case kInitRgb: s.ci = 0; break;
        case kInitCi: s.ci = 1; break;
        case kTexOn: s.tex_on = 1; break;
        case kTexOff: s.tex_on = 0; break;
        case kEnableCap:
        case kDisableCap:
            // GL_TEXTURE_GEN_S .. Q; other capabilities are libGLcore's own.
            if (w0 - 0xC60 < 4) {
                const u32 bit = 1u << (w0 - 0xC60);
                s.texgen_on = cmd == kEnableCap ? s.texgen_on | bit : s.texgen_on & ~bit;
            }
            break;
        case kTexGeniv:
            // [coordinate 0x2000 + i, GL_TEXTURE_GEN_MODE, mode]
            if (n >= 3 && d[0] - 0x2000 < 4 && d[1] == 0x2500)
                s.texgen_mode[d[0] - 0x2000] = d[2] == 0x2401 ? 0 : d[2] == 0x2402 ? 2 : 1;
            break;
        case kTexGenPlaneHeader:
            // [coordinate, GL_OBJECT_PLANE / GL_EYE_PLANE]; the plane follows
            if (n >= 2 && d[0] - 0x2000 < 4) s.texgen_plane = (d[0] - 0x2000) | (d[1] == 0x2502 ? 4u : 0u);
            break;
        case kTexGenPlaneData: {
            if (n < 4) return false;
            const auto p = args4(d, n);
            const unsigned i = s.texgen_plane & 3;
            // Eye planes are kept in eye space: times the inverse of the
            // modelview current now.
            if (s.texgen_plane & 4) s.texgen_eye[i] = plane_to_eye(p, s.mv.m[s.mv.top]);
            else s.texgen_obj[i] = p;
            break;
        }
        case kRasterPos: {
            const auto obj = args4(d, n);
            const PVertex v = transform(obj);
            const bool inside = std::abs(v.h[0]) <= v.h[3] && std::abs(v.h[1]) <= v.h[3] && std::abs(v.h[2]) <= v.h[3] && v.h[3] > 0;
            const GeVertex w = project(v);
            s.raster = {w.x, w.y, w.z};
            s.raster_colour = v.c;
            s.raster_valid = inside;
            const auto e = v.e;
            s.raster_dist = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
            s.raster_tex = xform(s.tex.m[s.tex.top], s.texcoord);
            break;
        }
        case kUpdateRasterPos: {
            const auto a = args4(d, n);
            s.raster[0] += a[0], s.raster[1] += a[1];
            break;
        }
        case kBitmap:
            if (n < 8) return false;
            std::copy(d, d + 8, s.bitmap.begin());
            s.bitmap_general = 0, s.bitmap_row = 0, s.bitmap_pending = 1;
            if (!s.bitmap[7]) bitmap_rows(nullptr, 0);
            break;
        case kPixelState:
        case kInitPixelStateEram:
            if (n >= 7) pixel_state_block(d, n);  // (with no data, the block follows as pixel data)
            break;
        case kRssRegSet:
        case kRssRegShadow:
        case kInitColourTable: break;  // their data follows as pixel data
        case kSaveRss:
            s.pixel_op = 1;
            s.pixel_planes = n > 3 ? d[3] : 0;
            break;
        case kRestoreRss:
            end_bracket();
            s.pixel_op = 0, s.pixel_planes = 0;
            break;
        case kGetPixels: get_pixels(w0 & 2); break;
        case kSendPixels:
            if (n < 8) return false;
            std::copy(d, d + 8, s.send_pixels.begin());
            s.send_pixels_pending = 1;
            break;
        case kWriteDmaGeSetup:
            // An image (or texels) by host DMA in one transfer: the image
            // rectangle's transfer size gives its rows and width.
            if (n >= 4 && module_at(d[3]) == kModTexLoad) {
                // A TRAM restore: rows of saved texels, as the transfer
                // mode lays them out, over the image rectangle.
                const u32 w = s.tl_rect[2], h = s.tl_rect[3];
                s.send_pixels = {(w * ImpactTram::read_texel_bytes(s.xfrmode) + 3) / 4, 0, 0, h, 0, 1, d[3], 0};
                s.send_pixels_pending = 1;
            } else if (n >= 4 && module_at(d[3]) == kModCopyPixels) {
                const u32 xs = s.tl_rect[4], w = xs & 0xFFFF, h = xs >> 16;
                const u32 words = (w * raster_.ge_host_bpp(s.xfrmode) + 3) / 4;
                s.send_pixels = {words, 0, 0, h, 0, 1, d[3], 0};
                s.send_pixels_pending = 1;
            }
            break;
        case kTlNoTable:
        case kTlSubloadEnd:
        case kSaveTexmode:
        case kRestoreTexmode: break;  // pixel operations are drawn untextured here anyway
        case kSetData:
            // [GE address, value]: a word of GE memory libGLcore keeps there
            // (seen: 0xB69 / 0xB6A, the read and draw buffers' GL enums),
            // read back with the state.
            if (n < 2) return false;
            if (d[0] >= 0xB00 && d[0] < 0xC00) {
                s.set_data[d[0] - 0xB00] = d[1];
                s.set_data_valid[(d[0] - 0xB00) / 32] |= 1u << (d[0] % 32);
            }
            break;
        case kTramRestore:
            // The kernel's restore of a context's TRAM pages (MgrasRestoreTexData,
            // its __glMgr_WriteDMAGESetup): the word count of the texels the
            // host DMA brings next, in three 9-bit parts, high first. The
            // texture loader registers came before, through RSS_REG_SET.
            if (n < 3) return false;
            s.tram_restore_words = (d[0] & 0x1FF) << 18 | (d[1] & 0x1FF) << 9 | (d[2] & 0x1FF);
            break;
        case kReadBegin:
        case kReadEnd:
            // No data: libGLcore brackets its pixel reads with them
            // (__glMgrim_ReadPixels, __glMgrSlowCopyPixels, around
            // __glMgrDoReadPixels).
        case kIglooNote:
            // No data: the IRIS GL interface (__glMgrim_IglooInterfaceSGIX)
            // sends it as IRIS GL programs start drawing.
        case kTramSaveEnd:      // no data: the kernel's TRAM save ends with it, before RESTORE_RSS
        case kTramRestorePrep:  // no data, just before 0xD8
        case kSyncForRead:
            // No data: libGLcore's host-DMA read set-up (histogram, minmax,
            // reads by DMA) sends it, then polls for the GE to drain before
            // it programs the DMA engine itself. Commands here finish as
            // they come, so there is nothing to wait for.
        case kRenderModeNote:
            // glRenderMode sends it (a byte of its state, after token 6) on
            // the way into or out of feedback / selection, and so does a
            // new context's set-up; drawing here does not depend on it.
        case kMakeCurrentRead:  // no data: before 0x8C, as above
        case kPad:
            // No data: the kernel's ERAM reads (MgrasEramRead) send their
            // pass-through commands a doubleword at a time, the last one
            // filled out with it.
        case kInitTables:
            // A new context's set-up (libGLcore, after its 0x8C): five
            // lookup tables for the GE microcode, each as pixel data that
            // follows (exponentials for fog and lighting, and constant
            // tables). The fog and lighting here are computed directly.
            break;
        case kTexBase:
            // The kernel's texture manager places each context in TRAM
            // (MgrasPlaceBaseAddr: a run of 16 KB pages out of 256) and
            // sends the first page here (_MgrasWriteTexBase); libGLcore's
            // pages are relative to it. 0 for contexts without textures.
            s.tex_base = w0 & 0xFF;
            break;
        case kDisable:
            // Sent with a zero word around texture register lists; no
            // effect on drawing has been found.
            break;
        case kReadTexture:
            // glGetTexImage: [width, height, s and t offsets, transfer mode,
            // the raster interface start word]: the level the sampler is
            // aimed at, row by row.
            if (n < 5) return false;
            raster_.ge_tex_read(tex_state(), d[4], false, d[1], d[0]);
            break;
        case kReadTextureRaw:
            // The texture manager's save of TRAM pages: [rows, transfer
            // mode, start word], rows as wide as the bound level.
            if (n < 2) return false;
            raster_.ge_tex_read(tex_state(), d[1], true, d[0], 1u << (te_reg(kTxSize) & 0xF));
            break;
        case kRssRegShadowIdx:
            // One register through an indexed shadow table: [index shadow,
            // table shadow, register, value].
            if (n >= 4) te_write(d[2] & 0x3FF, d[3]);
            break;
        case kGeneralBitmap:
            if (n < 5) return false;
            s.bitmap = {d[0], d[1], d[2], d[3], d[4], 0, 0, 0};
            s.bitmap_general = 1, s.bitmap_row = 0, s.bitmap_pending = 0;
            break;
        case kBitmapRows:
            if (n < 2 || !s.bitmap_general) return false;
            s.bitmap[7] = d[1];
            s.bitmap_pending = 1;
            break;
        case kBitmapMove:
            if (n < 2) return false;
            if (s.raster_valid) s.raster[0] += f32(d[0]), s.raster[1] += f32(d[1]);
            s.bitmap_general = 0, s.bitmap_pending = 0;
            break;
        case kLoadRasterPosInfo:
            s.colour_now = s.raster_colour;
            if (s.ci) s.index_now = s.colour_now[0] * 4095.0f;
            break;
        default:
            if (cmd >= kBeginPoints && cmd <= kBeginPolygon) {
                s.prim = cmd - kBeginPoints;
                s.nv = 0;
                break;
            }
            if (cmd >= kEndPoints && cmd <= kEndPolygon) {
                end_primitive();
                s.prim = kNone;
                break;
            }
            if (cmd == kVertex) {
                vertex(args4(d, n));
                break;
            }
            if (cmd >= kLightPosition && cmd <= kLightQuadratic) {
                light_param(cmd, d, n);
                break;
            }
            return false;
    }
    return true;
}

// ---- vertices ---------------------------------------------------------------

// OpenGL's lighting equation at eye position `eye` with object normal `n`
// (front face, or back with the normal reversed and the back material).
std::array<float, 4> ImpactGe::light(const std::array<float, 4>& eye, const std::array<float, 3>& n0, bool back) const {
    const State& s = s_;
    // The normal to eye space: times the inverse transpose of the modelview.
    const Mat4& inv = mv_inverse();
    std::array<float, 3> n{n0[0] * inv.m[0] + n0[1] * inv.m[1] + n0[2] * inv.m[2], n0[0] * inv.m[4] + n0[1] * inv.m[5] + n0[2] * inv.m[6],
                           n0[0] * inv.m[8] + n0[1] * inv.m[9] + n0[2] * inv.m[10]};
    if (s.normalize) n = normalised(n);
    if (back) n = {-n[0], -n[1], -n[2]};
    const Material& m = s.mat[back ? 1 : 0];
    std::array<float, 3> c{};
    for (unsigned k = 0; k < 3; ++k) c[k] = m.emission[k] + m.ambient[k] * s.lmodel_ambient[k];
    const std::array<float, 3> p{eye[3] != 0 ? eye[0] / eye[3] : eye[0], eye[3] != 0 ? eye[1] / eye[3] : eye[1], eye[3] != 0 ? eye[2] / eye[3] : eye[2]};
    for (unsigned i = 0; i < kLights; ++i) {
        if (!(s.lights_on & (1u << i))) continue;
        const Light& l = s.lights[i];
        std::array<float, 3> L;
        float att = 1;
        if (l.position[3] == 0) {
            L = normalised({l.position[0], l.position[1], l.position[2]});
        } else {
            const float w = l.position[3];
            L = {l.position[0] / w - p[0], l.position[1] / w - p[1], l.position[2] / w - p[2]};
            const float dist = std::sqrt(dot3(L, L));
            L = normalised(L);
            const float den = l.atten[0] + l.atten[1] * dist + l.atten[2] * dist * dist;
            att = den > 0 ? 1 / den : 1;
            if (l.spot_cutoff != 180) {
                const float sd = -dot3(L, normalised(l.spot_dir));
                const float cut = std::cos(l.spot_cutoff * 3.14159265358979f / 180);
                att *= sd < cut ? 0.0f : std::pow(std::max(sd, 0.0f), l.spot_exp);
            }
        }
        if (att == 0) continue;
        const float nl = dot3(n, L);
        std::array<float, 3> term{};
        for (unsigned k = 0; k < 3; ++k) term[k] = m.ambient[k] * l.ambient[k];
        if (nl > 0) {
            for (unsigned k = 0; k < 3; ++k) term[k] += nl * m.diffuse[k] * l.diffuse[k];
            std::array<float, 3> h;
            if (s.local_viewer) {
                const auto v = normalised({-p[0], -p[1], -p[2]});
                h = {L[0] + v[0], L[1] + v[1], L[2] + v[2]};
            } else {
                h = {L[0], L[1], L[2] + 1};
            }
            h = normalised(h);
            const float nh = dot3(n, h);
            if (nh > 0) {
                const float sp = m.shininess > 0 ? std::pow(nh, m.shininess) : 1.0f;
                for (unsigned k = 0; k < 3; ++k) term[k] += sp * m.specular[k] * l.specular[k];
            }
        }
        for (unsigned k = 0; k < 3; ++k) c[k] += att * term[k];
    }
    return {std::clamp(c[0], 0.0f, 1.0f), std::clamp(c[1], 0.0f, 1.0f), std::clamp(c[2], 0.0f, 1.0f), std::clamp(m.diffuse[3], 0.0f, 1.0f)};
}

float ImpactGe::fog_factor(const std::array<float, 4>& eye) const {
    const float z = std::abs(eye[3] != 0 ? eye[2] / eye[3] : eye[2]);
    float f;
    switch (s_.fog_mode) {
        case 0x2601: f = s_.fog_end != s_.fog_start ? (s_.fog_end - z) / (s_.fog_end - s_.fog_start) : 1.0f; break;  // linear
        case 0x801: f = std::exp(-(s_.fog_density * z) * (s_.fog_density * z)); break;                                  // exp2
        default: f = std::exp(-s_.fog_density * z); break;
    }
    return std::clamp(f, 0.0f, 1.0f);
}

ImpactGe::PVertex ImpactGe::transform(const std::array<float, 4>& obj) {
    State& s = s_;
    PVertex v;
    v.edge = s.edge_flag;
    v.e = xform(s.mv.m[s.mv.top], obj);
    v.h = xform(s.proj.m[s.proj.top], v.e);
    v.c = v.cb = s.colour_now;
    if (s.lighting && !s.unlit && !s.ci) {
        v.c = light(v.e, s.normal, false);
        v.cb = s.two_side ? light(v.e, s.normal, true) : v.c;
    }
    if (s.fog && texturing() && !s.ci) {
        v.fog = fog_factor(v.e);  // applied after the texture environment
    } else if (s.fog) {
        const float f = fog_factor(v.e);
        if (s.ci) {
            v.c[0] = v.cb[0] = v.c[0] + (1 - f) * static_cast<float>(s.fog_index) / 4095.0f;
        } else {
            for (unsigned k = 0; k < 3; ++k) {
                v.c[k] = f * v.c[k] + (1 - f) * s.fog_colour[k];
                v.cb[k] = f * v.cb[k] + (1 - f) * s.fog_colour[k];
            }
        }
    }
    v.t = xform(s.tex.m[s.tex.top], s.texgen_on ? texgen(obj, v.e) : s.texcoord);
    return v;
}

// Clip coordinates to window coordinates (relative to the window, y up).
GeVertex ImpactGe::project(const PVertex& v) const {
    const State& s = s_;
    GeVertex w;
    const float iw = v.h[3] != 0 ? 1 / v.h[3] : 1;
    const float nx = v.h[0] * iw, ny = v.h[1] * iw, nz = v.h[2] * iw;
    w.x = s.viewport[0] + (nx + 1) * s.viewport[2] / 2;
    w.y = s.viewport[1] + (ny + 1) * s.viewport[3] / 2;
    const float n = s.depth_range[0], f = s.depth_range[1];
    w.z = std::clamp((nz * (f - n) + (f + n)) / 2, 0.0f, 1.0f) * kZMax;
    w.w = iw;
    w.c = v.c;
    w.t = v.t;
    w.fog = v.fog;
    return w;
}

void ImpactGe::vertex(const std::array<float, 4>& obj) {
    State& s = s_;
    if (s.prim == kNone) return;
    const PVertex v = transform(obj);
    const unsigned n = s.nv++;
    PVertex poly[kMaxPoly];
    // Edge flags count in separate triangles, quads and polygons only.
    const auto strip = [&](unsigned m) {
        for (unsigned k = 0; k < m; ++k) poly[k].edge = 1;
        polygon(poly, m, v);
    };
    switch (s.prim) {
        case kPoints: point(v); break;
        case kLines:
            s.vb[n % 2] = v;
            if (n % 2 == 1) line(s.vb[0], v, true);
            break;
        case kLineStrip:
        case kLineLoop:
            if (n == 0) s.vb[0] = v;
            else line(s.vb[1 + (n - 1) % 2], v, n == 1);
            s.vb[1 + n % 2] = v;
            break;
        case kTriangles:
            s.vb[n % 3] = v;
            if (n % 3 == 2) {
                std::copy(s.vb.begin(), s.vb.begin() + 3, poly);
                polygon(poly, 3, v);
            }
            break;
        case kTriStrip:
            // Every other triangle reversed to keep the strip's winding; the
            // newest vertex provokes.
            if (n == 0) s.tflip = 0;
            if (n >= 2) {
                poly[0] = s.tflip ? s.vb[1] : s.vb[0];
                poly[1] = s.tflip ? s.vb[0] : s.vb[1];
                poly[2] = v;
                s.tflip ^= 1;
                strip(3);
            }
            s.vb[0] = s.vb[1];
            s.vb[1] = v;
            break;
        case kTriFan:
            if (n == 0) {
                s.vb[0] = v;
                break;
            }
            if (n >= 2) {
                poly[0] = s.vb[0], poly[1] = s.vb[1], poly[2] = v;
                strip(3);
            }
            s.vb[1] = v;
            break;
        case kQuads:
            s.vb[n % 4] = v;
            if (n % 4 == 3) {
                std::copy(s.vb.begin(), s.vb.begin() + 4, poly);
                polygon(poly, 4, v);
            }
            break;
        case kQuadStrip:
            s.vb[n % 4] = v;
            if (n >= 3 && n % 2 == 1) {
                poly[0] = s.vb[(n - 3) % 4], poly[1] = s.vb[(n - 2) % 4], poly[2] = s.vb[n % 4], poly[3] = s.vb[(n - 1) % 4];
                strip(4);
            }
            break;
        case kPolygon:
            if (n < kMaxPoly - 12) s.vb[n] = v;
            break;
        default: break;
    }
}

void ImpactGe::end_primitive() {
    State& s = s_;
    const unsigned n = s.nv;
    if (s.prim == kLineLoop && n >= 2) {
        line(s.vb[1 + (n - 1) % 2], s.vb[0], false);
    } else if (s.prim == kPolygon && n >= 3) {
        const unsigned m = std::min(n, kMaxPoly - 12);
        PVertex poly[kMaxPoly];
        std::copy(s.vb.begin(), s.vb.begin() + m, poly);
        const PVertex prov = poly[0];
        polygon(poly, m, prov);
    }
}

// Sutherland-Hodgman against the view volume and the user planes.
unsigned ImpactGe::clip_polygon(PVertex* v, unsigned n) const {
    PVertex tmp[kMaxPoly];
    for (unsigned plane = 0; plane < 12 && n; ++plane) {
        if (plane >= 6 && !(s_.clip_on & (1u << (plane - 6)))) continue;
        auto dist = [&](const PVertex& p) {
            switch (plane) {
                case 0: return p.h[3] + p.h[0];
                case 1: return p.h[3] - p.h[0];
                case 2: return p.h[3] + p.h[1];
                case 3: return p.h[3] - p.h[1];
                case 4: return p.h[3] + p.h[2];
                case 5: return p.h[3] - p.h[2];
                default: {
                    const auto& q = s_.clip_planes[plane - 6];
                    return q[0] * p.e[0] + q[1] * p.e[1] + q[2] * p.e[2] + q[3] * p.e[3];
                }
            }
        };
        bool all_in = true;
        for (unsigned i = 0; i < n && all_in; ++i) all_in = dist(v[i]) >= 0;
        if (all_in) continue;
        unsigned m = 0;
        for (unsigned i = 0; i < n; ++i) {
            const PVertex& a = v[i];
            const PVertex& b = v[(i + 1) % n];
            const float da = dist(a), db = dist(b);
            if (da >= 0 && m < kMaxPoly) tmp[m++] = a;
            if ((da >= 0) != (db >= 0) && m < kMaxPoly) {
                // Leaving, the new edge runs along the clip boundary: a
                // boundary edge. Entering, it is the rest of a's edge.
                tmp[m] = lerp(a, b, da / (da - db));
                tmp[m++].edge = da >= 0 ? 1 : a.edge;
            }
        }
        std::copy(tmp, tmp + m, v);
        n = m;
    }
    return n;
}

bool ImpactGe::clip_line(PVertex& a, PVertex& b) const {
    float t0 = 0, t1 = 1;
    for (unsigned plane = 0; plane < 12; ++plane) {
        if (plane >= 6 && !(s_.clip_on & (1u << (plane - 6)))) continue;
        auto dist = [&](const PVertex& p) {
            switch (plane) {
                case 0: return p.h[3] + p.h[0];
                case 1: return p.h[3] - p.h[0];
                case 2: return p.h[3] + p.h[1];
                case 3: return p.h[3] - p.h[1];
                case 4: return p.h[3] + p.h[2];
                case 5: return p.h[3] - p.h[2];
                default: {
                    const auto& q = s_.clip_planes[plane - 6];
                    return q[0] * p.e[0] + q[1] * p.e[1] + q[2] * p.e[2] + q[3] * p.e[3];
                }
            }
        };
        const float da = dist(a), db = dist(b);
        if (da < 0 && db < 0) return false;
        if (da < 0) t0 = std::max(t0, da / (da - db));
        else if (db < 0) t1 = std::min(t1, da / (da - db));
    }
    if (t0 > t1) return false;
    const PVertex oa = a, ob = b;
    if (t0 > 0) a = lerp(oa, ob, t0);
    if (t1 < 1) b = lerp(oa, ob, t1);
    return true;
}

// A convex polygon: clipped, culled, then drawn in its polygon mode (a fan
// of triangles, its edges, or its vertices). `prov` gives the flat colour.
void ImpactGe::polygon(PVertex* v, unsigned n, const PVertex& provoking) {
    State& s = s_;
    n = clip_polygon(v, n);
    if (n < 3) return;
    GeVertex w[kMaxPoly];
    for (unsigned i = 0; i < n; ++i) {
        if (v[i].h[3] <= 0) return;
        w[i] = project(v[i]);
    }
    float area = 0;
    for (unsigned i = 0; i < n; ++i) area += w[i].x * w[(i + 1) % n].y - w[(i + 1) % n].x * w[i].y;
    const bool front = (area > 0) == (s.front_ccw != 0);
    if (s.cull) {
        const bool culled = s.cull_mode == 0x404 ? front : s.cull_mode == 0x408 ? true : !front;
        if (culled || area == 0) return;
    }
    const bool back_colours = s.lighting && s.two_side && !front;
    if (back_colours)
        for (unsigned i = 0; i < n; ++i) w[i].c = v[i].cb;
    if (!s.smooth) {
        const auto c = back_colours ? provoking.cb : provoking.c;
        for (unsigned i = 0; i < n; ++i) w[i].c = c;
    }
    const u32 mode = s.polygon_mode[front ? 0 : 1];
    if (s.polygon_offset_on & (mode == 0x1B00 ? 4u : mode == 0x1B01 ? 2u : 1u)) offset_depth(w, n);
    const GeDraw& d = draw();
    // Points and lines only where a vertex starts a boundary edge.
    if (mode == 0x1B00) {
        for (unsigned i = 0; i < n; ++i)
            if (v[i].edge) raster_.ge_point(d, w[i], s.point_size);
        return;
    }
    if (mode == 0x1B01) {
        u32 pos = 0;
        for (unsigned i = 0; i < n; ++i)
            if (v[i].edge) raster_.ge_line(d, w[i], w[(i + 1) % n], s.line_width, s.line_stipple, s.line_pattern, s.line_repeat, pos);
        return;
    }
    for (unsigned k = 1; k + 1 < n; ++k) raster_.ge_triangle(d, w[0], w[k], w[k + 1]);
}

// glPolygonOffset: the polygon's depths move by factor x its largest depth
// slope (window units a pixel) plus units x the smallest resolvable step. The
// GE's own arithmetic is not known: this is OpenGL 1.1's rule, with the third
// word libGLcore sends taken as that step in depth-buffer units (1.1), and
// glPolygonOffsetEXT's bias (third word 1.0) as a fraction of the depth range.
void ImpactGe::offset_depth(GeVertex* w, unsigned n) const {
    const auto& po = s_.polygon_offset;
    float m = 0;
    for (unsigned k = 1; k + 1 < n && m == 0; ++k) {
        const float ux = w[k].x - w[0].x, uy = w[k].y - w[0].y, uz = w[k].z - w[0].z;
        const float vx = w[k + 1].x - w[0].x, vy = w[k + 1].y - w[0].y, vz = w[k + 1].z - w[0].z;
        const float a = ux * vy - uy * vx;
        if (a == 0) continue;
        m = std::max(std::abs((uz * vy - uy * vz) / a), std::abs((ux * vz - uz * vx) / a));
    }
    const float off = po[0] * m + (po[2] == 1.0f ? po[1] * kZMax : po[1] * po[2]);
    for (unsigned i = 0; i < n; ++i) w[i].z = std::clamp(w[i].z + off, 0.0f, kZMax);
}

void ImpactGe::line(PVertex a, PVertex b, bool restart) {
    State& s = s_;
    if (restart) s.stipple_pos = 0;
    if (!clip_line(a, b)) return;
    if (a.h[3] <= 0 || b.h[3] <= 0) return;
    GeVertex wa = project(a);
    const GeVertex wb = project(b);
    if (!s.smooth) wa.c = wb.c;  // flat lines take the segment's second vertex colour
    raster_.ge_line(draw(), wa, wb, s.line_width, s.line_stipple, s.line_pattern, s.line_repeat, s.stipple_pos);
}

void ImpactGe::point(const PVertex& v) {
    for (unsigned k = 0; k < 3; ++k)
        if (std::abs(v.h[k]) > v.h[3]) return;
    if (v.h[3] <= 0) return;
    raster_.ge_point(draw(), project(v), s_.point_size);
}

// Words a waiting bitmap or image needs: a bitmap's data words; an
// image's words a row (header words 0 + 1) times its rows (3 + 3 * 4).
unsigned ImpactGe::pixel_words_needed() const {
    if (s_.tram_restore_words) return std::min<u32>(s_.tram_restore_words, 1u << 22);
    if (!s_.send_pixels_pending) return s_.bitmap_pending ? s_.bitmap[7] : 0;
    const auto& h = s_.send_pixels;
    return std::min<u32>((h[0] + h[1]) * (h[3] + 3 * h[4]), 1u << 22);
}

void ImpactGe::pixel_data(const u32* words, unsigned n) {
    if (s_.tram_restore_words) {
        s_.tram_restore_words = 0;
        tram_restore(words, n);
        return;
    }
    if (s_.send_pixels_pending) {
        s_.send_pixels_pending = 0;
        if (module_at(s_.send_pixels[6]) == kModTexLoad) tex_load(words, n);
        else draw_pixels(words, n);
        return;
    }
    bitmap_rows(words, n);
}

// glBitmap's rows, bottom row first, at the raster position less the
// bitmap's origin: set bits drawn in the raster colour at the raster
// depth, through the fragment stages; then the raster position moves.
// Format word bit 17 is GL_UNPACK_LSB_FIRST.
void ImpactGe::bitmap_rows(const u32* words, unsigned n) {
    State& s = s_;
    s.bitmap_pending = 0;
    const auto& h = s.bitmap;
    const unsigned bits_per_row = h[1];
    const unsigned stride = s.bitmap_general ? (bits_per_row + 63) / 64 * 8 : (bits_per_row + 7) / 8;
    if (stride) {
        const unsigned rows = s.bitmap_general ? std::min(n * 4 / stride, h[2] > s.bitmap_row ? h[2] - s.bitmap_row : 0) : h[2];
        if (s.raster_valid) {
            const int x0 = static_cast<int>(std::floor(s.raster[0] - f32(h[3])));
            const int y0 = static_cast<int>(std::floor(s.raster[1] - f32(h[4]))) + static_cast<int>(s.bitmap_row);
            const bool lsb = h[0] & (1u << 17);
            const GeDraw& d = draw();
            for (unsigned r = 0; r < rows; ++r)
                for (unsigned c = 0; c < bits_per_row; ++c) {
                    const unsigned i = r * stride + c / 8;
                    if (i / 4 >= n) break;
                    const u32 byte = (words[i / 4] >> (24 - 8 * (i % 4))) & 0xFF;
                    if ((lsb ? byte >> (c % 8) : byte >> (7 - c % 8)) & 1)
                        raster_.ge_fragment(d, x0 + static_cast<int>(c), y0 + static_cast<int>(r), s.raster_colour, s.raster[2]);
                }
        }
        if (s.bitmap_general) {
            s.bitmap_row += rows;
            return;
        }
    }
    if (!s.bitmap_general && s.raster_valid) s.raster[0] += f32(h[5]), s.raster[1] += f32(h[6]);
}

// ---- pixel operations -------------------------------------------------------

u32 ImpactGe::pix_read(u32 addr) const {
    const auto& p = s_.pix;
    if (addr >= 0x100 && addr < 0x200) return p.ge[addr - 0x100];
    if (addr >= 0xC00 && addr < 0xE00) return p.eram[addr - 0xC00];
    return 0;
}

bool ImpactGe::pix_loaded(u32 addr) const {
    const unsigned i = addr >= 0x100 && addr < 0x200 ? addr - 0x100 : addr >= 0xC00 && addr < 0xE00 ? 0x100 + addr - 0xC00 : ~0u;
    return i != ~0u && ((s_.pix.loaded[i / 32] >> (i % 32)) & 1);
}

void ImpactGe::pix_write(u32 addr, u32 v) {
    auto& p = s_.pix;
    unsigned i;
    if (addr >= 0x100 && addr < 0x200) p.ge[addr - 0x100] = v, i = addr - 0x100;
    else if (addr >= 0xC00 && addr < 0xE00) p.eram[addr - 0xC00] = v, i = 0x100 + addr - 0xC00;
    else return;
    p.loaded[i / 32] |= 1u << (i % 32);
}

// A pixel state write: [?, ?, bytes, address, 0, 2, count, values].
void ImpactGe::pixel_state_block(const u32* d, unsigned n) {
    ensure_init();
    if (n < 7) return;
    const u32 addr = d[3], count = std::min(d[6], n - 7);
    if (addr == kPixProgram) s_.pix.prog_len = count;
    if (count == 5 && (addr == kPixRect || addr == kPixRect - 0x20)) s_.pix.eram_shift = kPixRect - addr;
    const u32 rect = kPixRect - s_.pix.eram_shift, dest = kPixTexDest - s_.pix.eram_shift;
    for (u32 k = 0; k < count; ++k) {
        const u32 a = addr + k, v = d[7 + k];
        pix_write(a, v);
        if (a == 0x192) s_.zoom_inv = f32(v);
        else if (a >= rect && a < rect + 5) s_.tl_rect[a - rect] = v;
        else if (a >= dest && a < dest + 6) s_.tl_dest[a - dest] = v;
        else if (a == kPixScale - s_.pix.eram_shift) s_.tl_scale = v;
    }
}

// INIT_COLOR_TABLE: [?, bytes, table, ?, offset, count, entries], two
// 16-bit entries a word.
void ImpactGe::colour_table(const u32* d, unsigned n) {
    if (n < 6) return;
    const u32 off = d[4], count = std::min(d[5], n - 6);
    for (u32 k = 0; k < count && off + k < lut_.size(); ++k) lut_[off + k] = d[6 + k];
}

// libGLcore's raster register lists. Inside a pixel operation (SAVE_RSS ..
// RESTORE_RSS) they are written, over the window's state; outside one they
// are the GE's shadow of the context's raster state, kept for its pixel
// paths.
void ImpactGe::rss_list(const u32* d, unsigned n, unsigned group) {
    ensure_init();
    State& s = s_;
    for (unsigned i = 0; i + group <= n; i += group) {
        const u32 reg = d[i] & 0x3FF, v = d[i + 1];
        // Texture registers belong to the context in any case: the GE
        // keeps them and samples with them.
        if (te_write(reg, v)) continue;
        switch (reg) {
            case 0x159: s.xfrmode = v; break;
            case 0x155: s.xfr_factor = v; break;
            case 0x046: s.pixel_block[0] = v; break;
            case 0x047: s.pixel_block[1] = v; break;
            default: break;
        }
        if (s.pixel_op) {
            if (!bracket_) raster_.ge_bracket(true, &draw()), bracket_ = true;
            raster_.ge_rss_write(reg, v);
        }
    }
}

void ImpactGe::end_bracket() {
    if (bracket_) raster_.ge_bracket(false, nullptr);
    bracket_ = false;
}

// glReadPixels: a read of the pixel block from the read buffer (front or
// back after the swaps), or from the depth buffer (transfer format 2, type
// 3) or stencil (SAVE_RSS's planes). GET_PIXELS' word bit 1 asks for a PIO
// read.
void ImpactGe::get_pixels(bool pio) {
    State& s = s_;
    s.pixel_op = 2;
    const u32 p = s.pixel_planes;
    const bool stencil = (p & 0x1C00000) == 0x1800000 || (p & 0x1C0000) == 0x180000;
    const bool depth = !stencil && ((s.xfrmode >> 4) & 0xF) == 2 && (s.xfrmode & 0xF) == 3;
    const bool back = (s.read_back != 0) != (s.swapped != 0);
    const u32 w = s.win_drb, a = w & 0x3FF, b = (w >> 10) & 0x3FF;
    const bool pairs = rgb12_pair() || ci_pair();
    const u32 drb = pairs ? w : (w & ~0x3FFu) | (back && b ? b : a);
    if (!bracket_) raster_.ge_bracket(true, &draw()), bracket_ = true;
    raster_.ge_read_pixels(draw(), s.pixel_block[0], s.pixel_block[1], s.xfrmode, pio, drb, pairs && back,
                           stencil ? 2 : depth ? 1 : 0);
}

// The GE pixel pipeline (libGLcore's program at 0x13B: (module entry, state
// address) pairs, module k's state in pair k + 1). Each row of host
// elements goes through the modules to the raster engine's components.
bool ImpactGe::pipeline(const std::vector<u8>& bytes, unsigned w, unsigned h, unsigned stride, unsigned out_comps,
                        std::vector<std::vector<double>>& rows) const {
    const unsigned len = std::min<u32>(s_.pix.prog_len, 32) & ~1u;
    std::vector<std::pair<Module, u32>> mods;
    for (unsigned k = 0; 2 * k < len; ++k)
        mods.emplace_back(module_at(pix_read(kPixProgram + 2 * k)), 2 * k + 3 < len ? pix_read(kPixProgram + 2 * k + 3) : 0);
    if (mods.empty()) return false;
    const Module first = mods[0].first;
    const u32 element_type = kPixElementType - s_.pix.eram_shift;
    // The host elements: ERAM 0xDBB types them (0 signed and 1 unsigned
    // bytes, 2 and 3 shorts, 4 32-bit); a pipeline starting with none of
    // the converters takes floats.
    enum class Elem { Bit, S8, U8, S16, U16, S32, U32, F32 } elem;
    switch (pix_loaded(element_type) ? pix_read(element_type) : 4) {
        case 0: elem = Elem::S8; break;
        case 1: elem = Elem::U8; break;
        case 2: elem = Elem::S16; break;
        case 3: elem = Elem::U16; break;
        default:
            elem = first == kModShiftAdd ? Elem::U32 : first == kModIntToFloat || first == kModUnpack || first == kModMask ? Elem::S32 : Elem::F32;
            break;
    }
    unsigned comps = out_comps;
    auto find = [&](Module e) { return std::find_if(mods.begin(), mods.end(), [&](const auto& m) { return m.first == e; }); };
    if (first == kModBitmapToFloat || first == kModBitmapToInt) {
        elem = Elem::Bit, comps = 1;
    } else if (find(kModUnpack) != mods.end()) {
        comps = 1;
    } else if (auto e = find(kModExpand); e != mods.end()) {
        const u32 m = pix_read(e->second + 5);
        comps = m == 5 ? 2 : m == 6 ? 3 : 1;
    } else if (find(kModCompress) != mods.end()) {
        comps = 4;
    }
    unsigned esize = elem == Elem::Bit ? 0 : elem == Elem::S8 || elem == Elem::U8 ? 1 : elem == Elem::S16 || elem == Elem::U16 ? 2 : 4;
    // Integer elements narrower than the type word says (a colour-index or
    // stencil image's type reads 4 for bytes): the largest whose row fits.
    if ((elem == Elem::S32 || elem == Elem::U32) && w && comps * 4 * w > stride) {
        const bool sg = elem == Elem::S32;
        if (comps * 2 * w <= stride) elem = sg ? Elem::S16 : Elem::U16, esize = 2;
        else if (comps * w <= stride) elem = sg && first != kModMask ? Elem::S8 : Elem::U8, esize = 1;
    }
    const unsigned row_in = esize ? w * comps * esize : (w * comps + 7) / 8;
    if (!row_in || bytes.size() < std::size_t{stride} * (h - 1) + row_in) return false;
    auto get = [&](const u8* p, unsigned i) -> double {
        switch (elem) {
            case Elem::Bit: return (p[i / 8] >> (7 - i % 8)) & 1;
            case Elem::S8: return static_cast<s8>(p[i]);
            case Elem::U8: return p[i];
            case Elem::S16: return static_cast<s16>((p[2 * i] << 8) | p[2 * i + 1]);
            case Elem::U16: return (p[2 * i] << 8) | p[2 * i + 1];
            default: {
                const u32 v = (u32{p[4 * i]} << 24) | (p[4 * i + 1] << 16) | (p[4 * i + 2] << 8) | p[4 * i + 3];
                return elem == Elem::S32 ? static_cast<double>(static_cast<s32>(v)) : elem == Elem::U32 ? static_cast<double>(v) : f32(v);
            }
        }
    };
    rows.assign(h, {});
    for (unsigned r = 0; r < h; ++r) {
        const u8* in = bytes.data() + std::size_t{r} * stride;
        std::vector<double> v(std::size_t{w} * comps);
        for (std::size_t i = 0; i < v.size(); ++i) v[i] = get(in, static_cast<unsigned>(i));
        unsigned per = comps;
        for (const auto& [e, st] : mods) {
            switch (e) {
                case kModShiftAdd: {
                    const s32 sh = static_cast<s32>(pix_read(st + 5));
                    const double add = static_cast<s32>(pix_read(st + 6));
                    for (double& x : v) {
                        const s64 i = static_cast<s64>(x);
                        x = static_cast<double>(sh >= 0 ? i << std::min(sh, 32) : i >> std::min(-sh, 63)) + add;
                    }
                    break;
                }
                case kModIntToFloat: {
                    const double sc = f32(pix_read(st + 5));
                    for (double& x : v) x *= sc;
                    break;
                }
                case kModUnpack: {
                    unsigned fields = 0;
                    for (unsigned k = 0; k < 4; ++k) fields += pix_read(st + 10 + k) != 0;
                    std::vector<double> o;
                    o.reserve(v.size() * fields);
                    for (double x : v) {
                        const u32 u = static_cast<u32>(static_cast<s64>(x));
                        for (unsigned k = 0; k < fields; ++k)
                            o.push_back(static_cast<double>((u & pix_read(st + 10 + k)) >> (pix_read(st + 14 + k) & 31)) * f32(pix_read(st + 6 + k)));
                    }
                    v.swap(o);
                    per = fields;
                    break;
                }
                case kModScaleBias:
                    for (std::size_t i = 0; i < v.size(); ++i) {
                        const unsigned k = i % 12;
                        v[i] = v[i] * f32(pix_read(st + 6 + 2 * k)) + f32(pix_read(st + 5 + 2 * k));
                    }
                    break;
                case kModExpand: {
                    const u32 m = pix_read(st + 5);
                    auto fill = [&](unsigned k) { return static_cast<double>(f32(pix_read(st + 7 + k))); };
                    std::vector<double> o;
                    o.reserve(std::size_t{w} * 4);
                    if (m <= 3) {
                        const unsigned p0 = 3 - m;  // the present component; the missing ones follow it
                        for (double x : v) {
                            double px[4];
                            px[p0] = x;
                            for (unsigned j = 0; j < 3; ++j) px[(p0 + 1 + j) % 4] = fill(j);
                            o.insert(o.end(), px, px + 4);
                        }
                    } else if (m == 4) {
                        for (double x : v) o.insert(o.end(), {x, x, x, fill(0)});
                    } else if (m == 7) {
                        for (double x : v) o.insert(o.end(), {x, x, x, x});
                    } else if (m == 5) {
                        for (std::size_t i = 0; i < v.size(); i += 2) o.insert(o.end(), {v[i], v[i], v[i], i + 1 < v.size() ? v[i + 1] : 1.0});
                    } else {
                        for (std::size_t i = 0; i < v.size(); i += 3)
                            o.insert(o.end(), {v[i], i + 1 < v.size() ? v[i + 1] : 0.0, i + 2 < v.size() ? v[i + 2] : 0.0, fill(0)});
                    }
                    v.swap(o);
                    per = 4;
                    break;
                }
                case kModLut: {
                    // Table offsets run up to the 2^23 word, one per pair of
                    // components (high half of a word the first).
                    unsigned nofs = 0;
                    while (nofs < 16 && pix_read(st + 7 + nofs) != kF2_23) ++nofs;
                    const unsigned cycle = nofs > 0 && nofs < 16 ? 2 * nofs : std::max<u32>(pix_read(st + 5), 2);
                    for (std::size_t i = 0; i < v.size(); ++i) {
                        const unsigned pos = i % cycle;
                        const u32 off = pix_read(st + 7 + pos / 2);
                        const std::size_t idx = off + static_cast<std::size_t>(std::max(0.0, std::round(v[i])));
                        const u32 wd = idx < lut_.size() ? lut_[idx] : 0;
                        v[i] = pos % 2 == 0 ? wd >> 16 : wd & 0xFFFF;
                    }
                    break;
                }
                case kModClamp:
                    for (std::size_t i = 0; i < v.size(); ++i) v[i] = std::clamp(v[i], 0.0, 1.0) * f32(pix_read(st + 5 + i % 12));
                    break;
                case kModCompress: {
                    static const std::vector<unsigned> keep[] = {{3}, {0}, {0, 3}, {0, 1, 2, 3}, {0, 1, 2}};
                    const u32 m = pix_read(st + 5);
                    const auto& k = keep[m <= 2 ? m : m == 4 ? 4 : 3];
                    std::vector<double> o;
                    for (std::size_t i = 0; i + per <= v.size(); i += per)
                        for (unsigned c : k) o.push_back(c < per ? v[i + c] : 0.0);
                    v.swap(o);
                    per = static_cast<unsigned>(k.size());
                    break;
                }
                case kModMask:
                    if (pix_loaded(st + 5)) {
                        const u32 m = pix_read(st + 5);
                        for (double& x : v) x = static_cast<double>(static_cast<u32>(static_cast<s64>(x)) & m);
                    }
                    break;
                default: break;  // colour matrix (none), pipe control, zoom: as they are
            }
        }
        rows[r].swap(v);
    }
    return true;
}

// The pixel pipeline's x-zoom replication: its XZOOM module's index table
// (from state word 7) gives each output pixel's source offset, so the
// first source pixel's run is the replication. 1 without the module.
unsigned ImpactGe::xzoom_rep() const {
    const unsigned len = std::min<u32>(s_.pix.prog_len, 32) & ~1u;
    for (unsigned k = 0; 2 * k + 3 < len; ++k) {
        if (module_at(pix_read(kPixProgram + 2 * k)) != kModXZoom) continue;
        const u32 st = pix_read(kPixProgram + 2 * k + 3);
        unsigned n = 0;
        while (n < 64 && pix_read(st + 7 + n) == 0) ++n;
        return std::clamp(n, 1u, 64u);
    }
    return 1;
}

// glDrawPixels: the rows (bottom first) through the GE's pipeline into the
// raster engine's transfer format, drawn at the raster position (plus the
// rectangle's offset within the image, for an image sent in pieces) with
// the pixel zoom, each pixel a fragment at the raster depth.
void ImpactGe::draw_pixels(const u32* words, unsigned n) {
    State& s = s_;
    if (!s.raster_valid) return;
    unsigned wpr = s.send_pixels[0] + s.send_pixels[1];
    if (!wpr || n < wpr) return;
    const unsigned hdr_rows = s.send_pixels[3] + 3 * s.send_pixels[4];
    unsigned rows = hdr_rows ? std::min(hdr_rows, n / wpr) : n / wpr;
    const u32 mode = s.xfrmode, ty = mode & 0xF, fmt = (mode >> 4) & 0xF;
    const unsigned bpp = std::max(1u, raster_.ge_host_bpp(mode));
    std::vector<u8> bytes(std::size_t{n} * 4);
    for (unsigned i = 0; i < n; ++i)
        for (unsigned b = 0; b < 4; ++b) bytes[4 * i + b] = static_cast<u8>(words[i] >> (24 - 8 * b));
    const u32 xs = s.tl_rect[4];
    // The raster engine's pixels, `out_wpr` words a row.
    std::vector<u8> px;
    unsigned src_w = 0, out_wpr = 0;
    const unsigned out_comps = fmt == 7 ? 3 : fmt == 8 ? 4 : fmt == 10 ? 2 : 1;
    std::vector<std::vector<double>> prow;
    unsigned w = xs & 0xFFFF, h = xs >> 16;
    // Zoomed: 0xDA8's size is the GE's output (rows repeated, columns
    // replicated `rep` times); the source is the transport rows, of the
    // columns over the replication.
    const unsigned rep = xzoom_rep();
    const bool zoomed = rep > 1 || (mode & (3u << 19)) || (s.zoom_inv > 0 && s.zoom_inv != 1);
    if (zoomed) w /= rep, h = rows;
    if (s.pix.prog_len >= 2 && pix_read(kPixProgram) && w && h &&
        pipeline(bytes, w, h, rows == h ? wpr * 4 : static_cast<unsigned>(bytes.size() / h), out_comps, prow)) {
        const bool index = fmt == 0 || fmt == 1;
        const double wrap = ty == 0 ? 0xFF : ty == 1 ? 0xFFFF : 4294967295.0;
        out_wpr = (w * bpp + 3) / 4;
        px.assign(std::size_t{out_wpr} * 4 * h, 0);
        for (unsigned r = 0; r < h; ++r) {
            std::size_t at = std::size_t{r} * out_wpr * 4;
            for (double v : prow[r]) {
                if (index) v = static_cast<double>(static_cast<s64>(std::round(v)) & static_cast<s64>(wrap));
                const double rv = std::round(v);
                auto put = [&](u64 x, unsigned nb) {
                    for (unsigned b = 0; b < nb && at < px.size(); ++b) px[at++] = static_cast<u8>(x >> (8 * (nb - 1 - b)));
                };
                switch (ty) {
                    case 0: put(static_cast<u64>(std::clamp(rv, 0.0, 255.0)), 1); break;
                    case 4: put(static_cast<u8>(static_cast<s8>(std::clamp(rv, -128.0, 127.0))), 1); break;
                    case 1: put(static_cast<u64>(std::clamp(rv, 0.0, 65535.0)), 2); break;
                    case 5: put(static_cast<u16>(static_cast<s16>(std::clamp(rv, -32768.0, 32767.0))), 2); break;
                    case 3: put(static_cast<u64>(std::clamp(rv, 0.0, 4294967295.0)), 4); break;
                    case 7: put(static_cast<u32>(static_cast<s32>(std::clamp(rv, -2147483648.0, 2147483647.0))), 4); break;
                    default: put(static_cast<u64>(std::clamp(rv, 0.0, 255.0)), 1); break;
                }
            }
        }
        src_w = w, rows = h;
    } else {
        px = std::move(bytes);
        out_wpr = wpr;
        // An image whose transport rows are not its rows (narrow tiles
        // flattened into one): 0xDA8 gives its shape.
        if (w && h && h != rows && w * bpp % 4 == 0 && std::size_t{w} * h * bpp == std::size_t{wpr} * rows * 4) out_wpr = w * bpp / 4, rows = h;
        src_w = out_wpr * 4 / bpp;
        if (w && w < src_w && (w * bpp + 3) / 4 == out_wpr) src_w = w;
    }
    if (!src_w || !rows) return;
    // GE pixels are RGBA: bytes in ABGR order (RGBorder clear) go to the
    // raster engine alpha first.
    const bool rev = bpp == 4 && (mode & 0xFF) == 0x80 && !(mode & (1u << 22));
    const GeDraw& d = draw();
    // Where each destination pixel takes its source. Zoomed images cover
    // 0xDA8's rectangle (x0, y0, x1, y1 from the raster position, signed)
    // as the board draws them (IRIS's trace, pixtest): rows by 1 / zoom
    // (0x192), columns by the GE's replication and then the raster
    // engine's XFRFACTOR (8.8): XFRMODE bit 19 zooms by it, bit 20
    // decimates by it; each sampled at the destination pixel's centre.
    // Otherwise pixel i takes floor(i / zoom) both ways.
    int x0, y0, out_w, out_h, step_x = 1, step_y = 1;
    std::function<unsigned(unsigned)> src_col, src_row;
    if (zoomed && xs) {
        const s32 r0 = static_cast<s32>(s.tl_rect[0]), r1 = static_cast<s32>(s.tl_rect[1]);
        const s32 r2 = static_cast<s32>(s.tl_rect[2]), r3 = static_cast<s32>(s.tl_rect[3]);
        out_w = std::abs(r2 - r0) + 1, out_h = std::abs(r3 - r1) + 1;
        step_x = r2 < r0 ? -1 : 1, step_y = r3 < r1 ? -1 : 1;
        x0 = static_cast<int>(std::floor(s.raster[0] + static_cast<float>(r0)));
        y0 = static_cast<int>(std::floor(s.raster[1] + static_cast<float>(r1)));
        const unsigned f = std::max(1u, s.xfr_factor & 0xFFFF);
        src_col = [=](unsigned j) {
            const unsigned g = (mode & (1u << 19)) ? (2 * j + 1) * 256 / (2 * f) : (mode & (1u << 20)) ? (2 * j + 1) * f / 512 : j;
            return std::min(src_w - 1, g / rep);
        };
        const double inv = s.zoom_inv > 0 ? s.zoom_inv : 1.0;
        src_row = [=](unsigned j) { return std::min(rows - 1, static_cast<unsigned>((j + 0.5) * inv)); };
    } else {
        const float zoom = s.zoom_inv > 0 ? std::clamp(1.0f / s.zoom_inv, 1.0f / 64, 64.0f) : 1.0f;
        const bool rect = xs && (xs >> 16) == rows && (xs & 0xFFFF) == src_w;
        const float ox = rect ? static_cast<float>(static_cast<s32>(s.tl_rect[0])) * zoom : 0;
        const float oy = rect ? static_cast<float>(static_cast<s32>(s.tl_rect[1])) * zoom : 0;
        if (rect && static_cast<s32>(s.tl_rect[2]) < static_cast<s32>(s.tl_rect[0])) step_x = -1;
        if (rect && static_cast<s32>(s.tl_rect[3]) < static_cast<s32>(s.tl_rect[1])) step_y = -1;
        x0 = static_cast<int>(std::floor(s.raster[0] + ox)), y0 = static_cast<int>(std::floor(s.raster[1] + oy));
        out_w = std::max(1, static_cast<int>(std::lround(src_w * zoom))), out_h = std::max(1, static_cast<int>(std::lround(rows * zoom)));
        src_col = [=](unsigned i) { return std::min(src_w - 1, static_cast<unsigned>(i / zoom)); };
        src_row = [=](unsigned j) { return std::min(rows - 1, static_cast<unsigned>(j / zoom)); };
    }
    // Depth (transfer format 2, 32-bit) and stencil images (SAVE_RSS's
    // planes, as for reads) go into the depth / stencil buffer as they are.
    const u32 planes = s.pixel_planes;
    const bool stencil = (planes & 0x1C00000) == 0x1800000 || (planes & 0x1C0000) == 0x180000;
    const bool depth = !stencil && fmt == 2 && ty == 3;
    std::array<float, 4> c{};
    for (int j = 0; j < out_h; ++j) {
        const unsigned sr = src_row(static_cast<unsigned>(j));
        const int y = y0 + step_y * j;
        for (int i = 0; i < out_w; ++i) {
            const unsigned sc = src_col(static_cast<unsigned>(i));
            const int x = x0 + step_x * i;
            const std::size_t at = std::size_t{sr} * out_wpr * 4 + std::size_t{sc} * bpp;
            if (at + bpp > px.size()) continue;
            u8 p[16];
            std::copy(px.begin() + static_cast<std::ptrdiff_t>(at), px.begin() + static_cast<std::ptrdiff_t>(at + bpp), p);
            if (stencil || depth) {
                const u32 raw = bpp >= 4 ? load_be<u32>(p) : p[bpp - 1];
                raster_.ge_zst_put(d, x, y, depth, depth ? raw >> 8 : raw & 0xFF);
                continue;
            }
            if (rev) std::reverse(p, p + 4);
            const u32 v = raster_.ge_host_pixel(p, mode);
            if (s.ci) c = {static_cast<float>(v & 0xFFF) / 4095.0f, 0, 0, 1};
            else c = {(v & 0xFF) / 255.0f, ((v >> 8) & 0xFF) / 255.0f, ((v >> 16) & 0xFF) / 255.0f, (v >> 24) / 255.0f};
            if (!s.ci && fmt == 7) c[3] = 1;
            raster_.ge_fragment(d, x, y, c, s.raster[2]);
        }
    }
}

// ---- texturing ------------------------------------------------------------------

// A TE register from the driver (register lists, RSS_REG_SHADOW_IDX): into
// the context's shadow. False for other registers.
bool ImpactGe::te_write(u32 reg, u32 v) {
    State& s = s_;
    if (reg == kTxAddr) {
        s.te_index = v;
        return true;
    }
    for (unsigned i = 0; i < 31; ++i)
        if (kTeRegs[i] == reg) {
            s.te[i] = v;
            return true;
        }
    for (unsigned k = 0; k < 3; ++k)
        if (kTeTables[k] == reg) {
            s.te_tables[k][s.te_index % 32] = v;
            ++s.te_index;
            return true;
        }
    return false;
}

u32 ImpactGe::te_reg(u32 reg) const {
    for (unsigned i = 0; i < 31; ++i)
        if (kTeRegs[i] == reg) return s_.te[i];
    return 0;
}

// Texturing is on for the primitives that follow: the GE has it on and the
// context's TEXMODE1 enables the TE.
bool ImpactGe::texturing() const { return s_.tex_on && (s_.te[0] & 1); }

// The inverse of the current modelview (identity when it has none), kept
// until the modelview changes: lighting needs it for every vertex.
const ImpactGe::Mat4& ImpactGe::mv_inverse() const {
    const Mat4& m = s_.mv.m[s_.mv.top];
    if (!inv_valid_ || inv_of_.m != m.m) {
        if (!invert(m, inv_)) inv_ = identity();
        inv_of_ = m;
        inv_valid_ = true;
    }
    return inv_;
}

// The texture registers as the TE samples with them.
GeTexture ImpactGe::tex_state() const {
    const State& s = s_;
    GeTexture t;
    t.on = true;
    t.mode1 = te_reg(kTexMode1), t.mode2 = te_reg(kTexMode2), t.size = te_reg(kTxSize), t.lod = te_reg(kTxLod);
    t.env_rg = te_reg(kTxEnvRg), t.env_b = te_reg(kTxEnvB);
    t.bcolour_rg = te_reg(kTxBColourRg), t.bcolour_ba = te_reg(kTxBColourBa);
    for (unsigned k = 0; k < 16; ++k)
        t.pages[k] = (s.te_tables[0][k] + s.tex_base) & 0xFF, t.border_pages[k] = (s.te_tables[1][k] + s.tex_base) & 0xFF;
    t.fog = s.fog != 0;
    t.fog_colour = {s.fog_colour[0], s.fog_colour[1], s.fog_colour[2]};
    return t;
}

// The vertex's texture coordinates before the texture matrix: the current
// ones, with the generated ones where generation is on.
std::array<float, 4> ImpactGe::texgen(const std::array<float, 4>& obj, const std::array<float, 4>& eye) const {
    const State& s = s_;
    std::array<float, 4> tc = s.texcoord;
    auto dot4 = [](const std::array<float, 4>& a, const std::array<float, 4>& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]; };
    std::array<float, 2> sphere{};
    if (s.texgen_on & 3 && (s.texgen_mode[0] == 2 || s.texgen_mode[1] == 2)) {
        // The reflection of the eye-space view vector about the normal.
        const Mat4& inv = mv_inverse();
        const auto& n0 = s.normal;
        std::array<float, 3> nn = normalised({n0[0] * inv.m[0] + n0[1] * inv.m[1] + n0[2] * inv.m[2],
                                              n0[0] * inv.m[4] + n0[1] * inv.m[5] + n0[2] * inv.m[6],
                                              n0[0] * inv.m[8] + n0[1] * inv.m[9] + n0[2] * inv.m[10]});
        const float ew = eye[3] != 0 ? eye[3] : 1;
        const std::array<float, 3> u = normalised({eye[0] / ew, eye[1] / ew, eye[2] / ew});
        const float nu = dot3(nn, u);
        const std::array<float, 3> r{u[0] - 2 * nn[0] * nu, u[1] - 2 * nn[1] * nu, u[2] - 2 * nn[2] * nu};
        const float m = 2 * std::sqrt(r[0] * r[0] + r[1] * r[1] + (r[2] + 1) * (r[2] + 1));
        if (m > 0) sphere = {r[0] / m + 0.5f, r[1] / m + 0.5f};
    }
    for (unsigned i = 0; i < 4; ++i) {
        if (!(s.texgen_on & (1u << i))) continue;
        switch (s.texgen_mode[i]) {
            case 0: tc[i] = dot4(s.texgen_obj[i], obj); break;
            case 2:
                if (i < 2) tc[i] = sphere[i];
                break;
            default: tc[i] = dot4(s.texgen_eye[i], eye); break;
        }
    }
    return tc;
}

// A texture image (SEND_PIXELS with a texture-load routine): the texels
// to the texture loader. The GE pixel state gives the sub-image (the image
// rectangle: s and t origin, signed, -1 for a border; width, height) and
// the TRAM page (0xDB1); the context's TL_MODE and TL_SPEC the level's
// size, components and depth. libGLcore's loads come through the GE pixel
// pipeline, which hands over the internal format's components. A TRAM
// restore (no destination level: the loader registers are the driver's
// own) is copied as it comes, its texels the transfer mode's size.
void ImpactGe::tex_load(const u32* words, unsigned n) {
    State& s = s_;
    const s32 s0 = static_cast<s32>(s.tl_rect[0]), t0 = static_cast<s32>(s.tl_rect[1]);
    const unsigned w = s.tl_rect[2], h = s.tl_rect[3];
    if (!w || !h || w > 4096 || h > 4096) return;
    const bool raw = s.tl_dest[0] == 0;
    GeTexLoad l;
    l.mode = te_reg(kTlMode);
    l.spec = te_reg(kTlSpec);
    l.page = ((raw ? te_reg(kTlMipmap) : s.tl_dest[3]) + s.tex_base) & 0xFF;
    l.border_page = ((raw ? te_reg(kTlBorder) : s.tl_dest[4]) + s.tex_base) & 0xFF;
    l.s0 = s0, l.t0 = t0;
    std::vector<u8> bytes(std::size_t{n} * 4);
    for (unsigned i = 0; i < n; ++i)
        for (unsigned b = 0; b < 4; ++b) bytes[4 * i + b] = static_cast<u8>(words[i] >> (24 - 8 * b));
    // A row is SEND_PIXELS words 0 + 1 when it sends a row per texel row,
    // else the rows are packed.
    const unsigned wpr = (s.send_pixels[0] + s.send_pixels[1]) * 4;
    const unsigned hdr_rows = s.send_pixels[3] + 3 * s.send_pixels[4];
    const bool row_per_line = hdr_rows == h && wpr >= w;
    std::vector<u8> line(std::size_t{w} * 4);
    const u32 fmt = (s.xfrmode >> 4) & 0xF, ty = s.xfrmode & 0xF;
    std::vector<std::vector<double>> prow;
    const unsigned out_comps = fmt == 7 ? 3 : fmt == 8 ? 4 : fmt == 10 ? 2 : 1;
    if (!raw && s.pix.prog_len >= 2 && pix_read(kPixProgram) &&
        pipeline(bytes, w, h, row_per_line ? wpr : static_cast<unsigned>(bytes.size() / h), out_comps, prow)) {
        for (unsigned r = 0; r < h; ++r) {
            std::fill(line.begin(), line.end(), u8{0});
            const auto& v = prow[r];
            for (unsigned k = 0; k < w; ++k)
                for (unsigned c = 0; c < out_comps && c < 4 && std::size_t{k} * out_comps + c < v.size(); ++c) {
                    // The component's top byte as the transfer type packs it.
                    const double x = std::round(v[std::size_t{k} * out_comps + c]);
                    line[4 * k + c] = ty == 1 ? static_cast<u8>(static_cast<u32>(std::clamp(x, 0.0, 65535.0)) >> 8)
                                              : static_cast<u8>(std::clamp(x, 0.0, 255.0));
                }
            raster_.ge_tex_load(l, r, line.data(), w);
        }
        return;
    }
    // Host texels: 8- or 16-bit components (the pixel format's scale says
    // which) or floats, into the internal format's (luminance replicated,
    // alpha picked out, missing alpha 1).
    const unsigned pitch = row_per_line ? wpr : static_cast<unsigned>(bytes.size() / h);
    const unsigned bpt = raw ? ImpactTram::read_texel_bytes(s.xfrmode) : std::max(1u, pitch / w);
    const float scale = f32(s.tl_scale);
    const bool fl = !raw && scale >= 0.5f;
    const unsigned comp = raw ? 1 : fl ? 4 : scale > 0 && scale < 1.0f / 1000 ? (scale < 1.0f / 100000 ? 4 : 2) : 1;
    const unsigned n_ext = std::clamp(bpt / comp, 1u, 4u);
    const unsigned nc = ((l.mode >> 1) & 3) + 1;
    const bool alpha_class = !raw && ((te_reg(kTexMode1) >> 9) & 7) == 1;
    for (unsigned r = 0; r < h; ++r) {
        for (unsigned k = 0; k < w; ++k) {
            const std::size_t i = std::size_t{r} * pitch + std::size_t{k} * bpt;
            auto src = [&](unsigned c) -> u8 {
                const std::size_t at = i + std::size_t{c} * comp;
                if (fl) {
                    if (at + 4 > bytes.size()) return 0;
                    const u32 b = (u32{bytes[at]} << 24) | (bytes[at + 1] << 16) | (bytes[at + 2] << 8) | bytes[at + 3];
                    return static_cast<u8>(std::lround(std::clamp(f32(b), 0.0f, 1.0f) * 255.0f));
                }
                return at < bytes.size() ? bytes[at] : 0;
            };
            const u8 lum = src(0), alpha = n_ext == 2 || n_ext == 4 ? src(n_ext - 1) : 0xFF;
            std::array<u8, 4> px;
            if (raw || n_ext == nc) px = {src(0), src(1), src(2), src(3)};
            else if (n_ext == 1 && nc == 1) px = {lum, 0, 0, 0};
            else if (alpha_class && nc == 1) px = {alpha, 0, 0, 0};
            else if (n_ext <= 2 && nc == 4) px = {lum, lum, lum, alpha};
            else if (n_ext <= 2 && nc == 3) px = {lum, lum, lum, 0};
            else if (nc == 2) px = {lum, alpha, 0, 0};
            else if (n_ext == 3 && nc == 4) px = {src(0), src(1), src(2), 0xFF};
            else px = {src(0), src(1), src(2), src(3)};
            std::copy(px.begin(), px.end(), line.begin() + 4 * k);
        }
        raster_.ge_tex_load(l, r, line.data(), w);
    }
}

// A kernel TRAM restore's texels: one stream through the loader's view of
// the pages (TL_SPEC, as saved: a level 64 texels wide), texel n at column
// n % width, row n / width from TL_S_LEFT / TL_T_BOTTOM, each texel the
// transfer mode's size and copied as it comes, at TL_MIPMAP's page past the
// context's TRAM base.
void ImpactGe::tram_restore(const u32* words, unsigned n) {
    const State& s = s_;
    GeTexLoad l;
    l.mode = te_reg(kTlMode);
    l.spec = te_reg(kTlSpec);
    l.page = (te_reg(kTlMipmap) + s.tex_base) & 0xFF;
    l.border_page = (te_reg(kTlBorder) + s.tex_base) & 0xFF;
    l.t0 = static_cast<s32>(te_reg(kTlTBottom));
    const s32 s0 = static_cast<s32>(te_reg(kTlSLeft));
    const unsigned w = 1u << ((l.spec >> 4) & 0xF), bpt = std::max(1u, ImpactTram::read_texel_bytes(s.xfrmode));
    const std::size_t texels = std::size_t{n} * 4 / bpt;
    std::vector<u8> row(std::size_t{w} * 4);
    auto byte = [&](std::size_t i) { return static_cast<u8>(words[i / 4] >> (24 - 8 * (i % 4))); };
    for (std::size_t k = 0; k < texels;) {
        const unsigned col = static_cast<unsigned>(k % w), line = static_cast<unsigned>(k / w);
        const unsigned m = static_cast<unsigned>(std::min<std::size_t>(w - col, texels - k));
        std::fill(row.begin(), row.end(), u8{0});
        for (unsigned j = 0; j < m; ++j)
            for (unsigned b = 0; b < std::min(bpt, 4u); ++b) row[4 * j + b] = byte((k + j) * bpt + b);
        l.s0 = s0 + static_cast<s32>(col);
        raster_.ge_tex_load(l, line, row.data(), m);
        k += m;
    }
}

// ---- buffers ------------------------------------------------------------------

// SCHEDULE_SWAP: front and back trade places for what follows, unless the
// kernel names the bank drawn (VALIDATE_BANKS).
void ImpactGe::swap_buffers() {
    ensure_init();
    if (!s_.banks_known) s_.swapped ^= 1;
}

// The bank GL draws into (the window's displayed bank xor 1): 1 is B.
void ImpactGe::set_draw_bank(u32 bank) {
    s_.swapped = (bank & 1) == 0;
    s_.banks_known = 1;
}

void ImpactGe::set_window(u32 origin, u32 mode, u32 pp1winmode, const std::array<std::array<u32, 2>, 4>& masks, u32 drb) {
    s_.win_valid = 1;
    s_.win_origin = origin;
    s_.win_mode = mode;
    s_.win_pp1 = pp1winmode;
    s_.win_masks = masks;
    s_.win_drb = drb;
}

// DRAW_BUFFER's words: the buffers drawn before an odd number of swaps,
// then after. 1 is A; B is 2 in 12-bit visuals and 4 in 24-bit ones; 0x4N
// the overlay; 0 nothing.
std::optional<u32> ImpactGe::draw_mask() const {
    if (!s_.draw_words) return std::nullopt;
    return s_.swapped ? s_.draw_bits_swapped : s_.draw_bits & 0x7F;
}

bool ImpactGe::rgb12_pair() const {
    const u32 f = s_.pixel_format & 0x2700;
    const auto m = draw_mask();
    return !s_.ci && (s_.pixel_format & kFormatSent) && ((f >> 8) & 7) <= 1 && !(f & 0x2000) && !(m && (*m & 0x70) == 0x40);
}

bool ImpactGe::ci_pair() const {
    const auto m = draw_mask();
    return s_.ci && !(m && (*m & 0x70) == 0x40);
}

bool ImpactGe::rgb36() const {
    const auto m = draw_mask();
    return !s_.ci && (s_.pixel_format & kFormatSent) && (s_.pixel_format & 0x2700) == 0x2300 && !(m && (*m & 0x70) == 0x40);
}

// A pair context's draw field: 1 A, 2 B, 3 both.
u32 ImpactGe::pair_field() const {
    if (const auto m = draw_mask()) return *m & 3;
    const bool back = s_.draw_bits & 0xC;
    return back != (s_.swapped != 0) ? 2 : 1;
}

u32 ImpactGe::draw_pointers() const {
    const u32 drb = s_.win_drb;
    if (rgb12_pair() || ci_pair()) return drb;
    const u32 a = drb & 0x3FF, b = (drb >> 10) & 0x3FF ? (drb >> 10) & 0x3FF : a;
    u32 page;
    if (const auto m = draw_mask()) {
        if ((*m & 0x70) == 0x40 || ((*m & 1) && (*m & 0xE))) return drb;
        page = (*m & 0xE) ? b : a;
    } else {
        const bool back = s_.draw_bits & 0xC;
        page = back != (s_.swapped != 0) ? b : a;
    }
    return (drb & ~0x3FFu) | page;
}

u32 ImpactGe::pp1_base() const {
    const u32 fmt = s_.pixel_format & 0x2700;
    const u32 ci_fmt = (s_.pixel_format & kFormatSent) && ((fmt >> 8) & 7) >= 4 ? fmt : 0x600;
    if (rgb12_pair())
        return (kPp1Rgb24A & ~((0x7Fu << 14) | 0x2700)) | fmt | pair_field() << 14 | (s_.dither ? kPp1Dither : 0);
    if (ci_pair()) return (kPp1Rgb24A & ~((0x7Fu << 14) | 0x2700)) | ci_fmt | pair_field() << 14;
    u32 pp1 = s_.ci ? (kPp1Rgb24A & ~0x2700u) | ci_fmt : rgb36() ? (kPp1Rgb24A & ~0x2700u) | 0x2300 : kPp1Rgb24A;
    if (const auto m = draw_mask()) {
        if ((*m & 0x70) == 0x40) {
            pp1 = (pp1 & ~((0x7Fu << 14) | (1u << 11))) | *m << 14 | s_.buffer_count << 11;
            return (pp1 & ~0x2700u) | fmt;
        }
        if ((*m & 1) && (*m & 0xE)) return (pp1 & ~(0x7Fu << 14)) | 3u << 14;
    }
    return pp1;
}

void ImpactGe::colour_write_masks(u32& lsb, u32& msb) const {
    const State& s = s_;
    lsb = msb = 0;
    if (const auto m = draw_mask()) {
        if (*m == 0) return;
        if ((*m & 0x70) == 0x40) {
            msb = s.index_mask & 0xFF;
            return;
        }
    }
    const u32 cm = s.colour_mask;
    if (rgb12_pair()) {
        // Per buffer: its colour components and alpha (4:4:4 A 0x0F000FFF
        // B 0xF0FFF000, 5:5:5 A 0x40007FFF B 0xBFFF8000).
        const bool p555 = ((s.pixel_format >> 8) & 7) == 1;
        const u32 nb = p555 ? 5 : 4;
        u32 c = 0;
        for (unsigned k = 0; k < 3; ++k)
            if ((cm >> k) & 1) c |= ((1u << nb) - 1) << (nb * k);
        auto half = [&](bool b) {
            const u32 apos = p555 ? (b ? 31 : 30) : (b ? 28 : 24), aw = p555 ? 1 : 4;
            return (b ? c << (3 * nb) : c) | (((cm >> 3) & 1) ? ((1u << aw) - 1) << apos : 0);
        };
        const u32 f = pair_field();
        lsb = ((f & 1) ? half(false) : 0) | ((f & 2) ? half(true) : 0);
        return;
    }
    if (s.ci) {
        const u32 m = s.index_mask & 0xFFF, f = ci_pair() ? pair_field() : 1;
        lsb = ((f & 1) ? m : 0) | ((f & 2) ? m << 12 : 0);
        return;
    }
    if (rgb36()) {
        u64 m = 0;
        for (unsigned k = 0; k < 3; ++k)
            if ((cm >> k) & 1) m |= (u64{0xFF} << (8 * k)) | (u64{0xF} << (24 + 4 * k));
        lsb = static_cast<u32>(m), msb = static_cast<u32>(m >> 32);
        return;
    }
    lsb = ((cm & 1) ? 0xFFu : 0) | ((cm & 2) ? 0xFF00u : 0) | ((cm & 4) ? 0xFF0000u : 0) | ((cm & 8) ? 0xFF000000u : 0);
}

// The scissor box, inclusive window coordinates [x0, x1, y0, y1]. The token
// carries [x, width - 1, y, height - 1].
std::array<int, 4> ImpactGe::scissor_rect() const {
    const auto& c = s_.scissor;
    return {c[0], c[0] + c[1], c[2], c[2] + c[3]};
}

const GeDraw& ImpactGe::draw() {
    const State& s = s_;
    GeDraw& d = draw_;
    d.origin = s.win_origin;
    d.winmode = s.win_mode;
    d.masks = s.win_masks;
    d.pp1winmode = s.win_pp1;
    d.drb = draw_pointers();
    const bool blend_logic = s.blend && s.blend_eq == kGlLogicOp;
    const u32 op = s.logic || blend_logic ? s.logic_op : 3;
    d.pp1 = (pp1_base() & ~(0xFu << 26)) | op << 26;
    colour_write_masks(d.mask_lsb, d.mask_msb);
    d.ci = s.ci;
    const auto m = draw_mask();
    d.overlay = m && (*m & 0x70) == 0x40;
    d.scissor = s.scissor_test;
    d.scissor_box = scissor_rect();
    d.alpha_test = s.alpha_test;
    d.alpha_func = s.alpha_func;
    d.alpha_ref = static_cast<float>(s.alpha_ref) / 4096.0f;
    d.stencil = s.stencil;
    d.st_func = s.st_func, d.st_ref = s.st_ref, d.st_cmask = s.st_cmask, d.st_wmask = s.st_wmask, d.st_ops = s.st_ops;
    d.depth_test = s.depth_test;
    d.depth_func = s.depth_func;
    d.depth_write = s.depth_mask;
    d.blend = s.blend && !blend_logic && op == 3;  // a logic op other than copy replaces blending
    d.blend_src = s.blend_src, d.blend_dst = s.blend_dst, d.blend_eq = s.blend_eq;
    d.blend_color = s.blend_colour;
    d.poly_stipple = s.poly_stipple;
    d.stipple = s.stipple_rows;
    d.line_smooth = s.line_smooth, d.point_smooth = s.point_smooth;
    d.tex = GeTexture{};
    if (texturing() && !s.ci) d.tex = tex_state();
    return d;
}

void ImpactGe::clear_colour_buffer() {
    GeDraw d = draw();
    d.alpha_test = d.stencil = d.depth_test = d.blend = d.poly_stipple = false;
    if (s_.ci) {
        raster_.ge_clear_colour(d, s_.clear_index & 0xFFF);
        return;
    }
    auto q = [](float c) { return static_cast<u32>(std::lround(std::clamp(c, 0.0f, 1.0f) * 255.0f)); };
    const auto& c = s_.clear_colour;
    raster_.ge_clear_colour(d, q(c[0]) | q(c[1]) << 8 | q(c[2]) << 16 | q(c[3]) << 24);
}

void ImpactGe::clear_zst(bool depth) {
    GeDraw d = draw();
    d.alpha_test = d.stencil = d.depth_test = d.blend = d.poly_stipple = false;
    if (depth) {
        if (!s_.depth_mask) return;
        raster_.ge_clear_zst(d, true, s_.clear_depth, 0xFFFFFF);
    } else {
        raster_.ge_clear_zst(d, false, s_.clear_stencil, s_.st_wmask);
    }
}

}  // namespace sg::ip22
