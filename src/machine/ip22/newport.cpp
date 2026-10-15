// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// References: SGI REX3, VC2, XMAP9, CMAP and RB2 datasheets; Linux
// include/video/newport.h and newport_con.c; NetBSD sys/arch/sgimips/gio/
// newport.c; X.org xf86-video-newport.
#include "machine/ip22/newport.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "core/endian.h"
#include "core/log.h"

namespace sg::ip22 {

namespace {

// REX3 register offsets (without the GO alias bit).
enum Rex3 : u32 {
    kDrawMode1 = 0x000, kDrawMode0 = 0x004, kLsMode = 0x008, kLsPattern = 0x00C, kLsPatSave = 0x010,
    kZPattern = 0x014, kColorBack = 0x018, kColorVram = 0x01C, kAlphaRef = 0x020, kStall0 = 0x024,
    kSmask0X = 0x028, kSmask0Y = 0x02C, kSetup = 0x030, kStepZ = 0x034, kLsRestore = 0x038, kLsSave = 0x03C,
    kXStart = 0x100, kYStart = 0x104, kXEnd = 0x108, kYEnd = 0x10C, kXSave = 0x110, kXYMove = 0x114,
    kBresD = 0x118, kBresS1 = 0x11C, kBresOctInc1 = 0x120, kBresRndInc2 = 0x124, kBresE1 = 0x128,
    kBresS2 = 0x12C, kAWeight0 = 0x130, kAWeight1 = 0x134, kXStartF = 0x138, kYStartF = 0x13C,
    kXEndF = 0x140, kYEndF = 0x144, kXStartI = 0x148, kXEndF1 = 0x14C, kXYStartI = 0x150, kXYEndI = 0x154,
    kXStartEndI = 0x158,
    kColorRed = 0x200, kColorAlpha = 0x204, kColorGrn = 0x208, kColorBlue = 0x20C, kSlopeRed = 0x210,
    kSlopeAlpha = 0x214, kSlopeGrn = 0x218, kSlopeBlue = 0x21C, kWrMask = 0x220, kColorI = 0x224,
    kColorX = 0x228, kSlopeRed1 = 0x22C, kHostRw0 = 0x230, kHostRw1 = 0x234, kDcbMode = 0x238,
    kDcbData0 = 0x240, kDcbData1 = 0x244,
    kSmask1X = 0x1300, kTopScan = 0x1320, kXYWin = 0x1324, kClipMode = 0x1328, kStall1 = 0x132C,
    kConfig = 0x1330, kStatus = 0x1338, kUStatus = 0x133C, kDcbReset = 0x1340,
};

// DRAWMODE0
constexpr u32 kDm0DoSetup = 1u << 5, kDm0ColorHost = 1u << 6, kDm0StopOnX = 1u << 8, kDm0StopOnY = 1u << 9;
constexpr u32 kDm0SkipFirst = 1u << 10, kDm0SkipLast = 1u << 11, kDm0EnZPattern = 1u << 12;
constexpr u32 kDm0EnLsPattern = 1u << 13, kDm0Length32 = 1u << 15, kDm0ZpOpaque = 1u << 16;
constexpr u32 kDm0LsOpaque = 1u << 17, kDm0Shade = 1u << 18, kDm0LrOnly = 1u << 19, kDm0CiClamp = 1u << 21;
constexpr u32 kDm0AlphaHost = 1u << 7;
// DRAWMODE1
constexpr u32 kDm1RwPacked = 1u << 7, kDm1RwDouble = 1u << 10, kDm1SwapEndian = 1u << 11;
constexpr u32 kDm1RgbMode = 1u << 15, kDm1Dither = 1u << 16, kDm1FastClear = 1u << 17, kDm1Blend = 1u << 18;
constexpr u32 kDm1BackBlend = 1u << 25, kDm1BlendAlpha = 1u << 27;
constexpr unsigned kDm1SFactorShift = 19, kDm1DFactorShift = 22;

constexpr u32 kStatusVersion = 3;
constexpr u32 kStatusBackBusy = 1u << 4;  // the display control bus is busy
constexpr u32 kStatusVrint = 1u << 5;

// Planes (DRAWMODE1[2:0])
enum Planes : unsigned { kPlanesNone = 0, kPlanesRgb = 1, kPlanesRgba = 2, kPlanesOlay = 4, kPlanesPup = 5, kPlanesCid = 6 };

// VC2 registers
enum Vc2Reg : unsigned { kVc2VideoEntry = 0x00, kVc2CursorEntry = 0x01, kVc2CursorX = 0x02, kVc2CursorY = 0x03,
                         kVc2CurCursorX = 0x04, kVc2DidEntry = 0x05, kVc2RamAddr = 0x07, kVc2LineCounter = 0x0B,
                         kVc2Control = 0x10, kVc2Config = 0x1F };
constexpr u16 kDcrVintr = 1u << 0, kDcrDisplay = 1u << 1, kDcrVTiming = 1u << 2, kDcrDid = 1u << 3;
constexpr u16 kDcrCursorFunc = 1u << 4, kDcrCursorVisible = 1u << 7, kDcrCrosshair = 1u << 8, kDcrLargeCursor = 1u << 9;

constexpr u32 kDefaultDrawMode1 = 0x3002F001;
constexpr u32 kDefaultDcbMode = 0x00000780;
constexpr u32 kDefaultXYWin = 0x10001000;
constexpr u32 kDefaultConfig = 0x000230C4;

constexpr int kBayer[4][4] = {{0, 12, 3, 15}, {8, 4, 11, 7}, {2, 14, 1, 13}, {10, 6, 9, 5}};

s32 sext(u32 v, unsigned bits) { return static_cast<s32>(v << (32 - bits)) >> (32 - bits); }

// Slopes are written sign-magnitude (sign in bit 31) and held as two's
// complement. Minus zero is zero: IRIX writes it for flat alpha.
u32 sign_magnitude(u32 data, u32 magnitude_mask, u32 result_sign) {
    const u32 magnitude = data & magnitude_mask;
    if ((data & 0x80000000u) && magnitude) return ((result_sign - magnitude) & (result_sign | magnitude_mask)) | result_sign;
    return magnitude;
}

// Logical BGR formats <-> 8-bit channels.
void unpack(u32 v, unsigned depth, u32& r, u32& g, u32& b) {
    switch (depth) {
        case 0:
            r = 0xFF * (v & 1);
            g = (0xAA * ((v >> 2) & 1)) | (0x55 * ((v >> 1) & 1));
            b = 0xFF * ((v >> 3) & 1);
            break;
        case 1:
            r = (0x92 * ((v >> 2) & 1)) | (0x49 * ((v >> 1) & 1)) | (0x24 * (v & 1));
            g = (0x92 * ((v >> 5) & 1)) | (0x49 * ((v >> 4) & 1)) | (0x24 * ((v >> 3) & 1));
            b = (0xAA * ((v >> 7) & 1)) | (0x55 * ((v >> 6) & 1));
            break;
        case 2:
            r = 0x11 * (v & 0xF);
            g = 0x11 * ((v >> 4) & 0xF);
            b = 0x11 * ((v >> 8) & 0xF);
            break;
        default:
            r = v & 0xFF;
            g = (v >> 8) & 0xFF;
            b = (v >> 16) & 0xFF;
            break;
    }
}

u32 pack(u32 r, u32 g, u32 b, unsigned depth) {
    switch (depth) {
        case 0: return ((b >> 7) << 3) | ((g & 0xC0) >> 5) | (r >> 7);
        case 1: return (b & 0xC0) | ((g & 0xE0) >> 2) | ((r & 0xE0) >> 5);
        case 2: return ((b & 0xF0) << 4) | (g & 0xF0) | ((r & 0xF0) >> 4);
        default: return (b << 16) | (g << 8) | r;
    }
}

u32 depth_mask(unsigned depth) {
    static constexpr u32 kMasks[4] = {0xF, 0xFF, 0xFFF, 0xFFFFFF};
    return kMasks[depth & 3];
}

}  // namespace

Newport::Newport(Scheduler& scheduler, double cpu_hz, const Options& options)
    : scheduler_(scheduler),
      cpu_hz_(cpu_hz),
      opts_(options),
      retrace_timer_(scheduler, [this] { retrace(); }),
      rgbci_(static_cast<std::size_t>(kPitch) * kRows),
      cidaux_(static_cast<std::size_t>(kPitch) * kRows) {
    reset();
}

void Newport::reset() {
    picture_changed();
    drawmode0_ = 0;
    drawmode1_ = kDefaultDrawMode1;
    lsmode_ = lspattern_ = lspatsave_ = zpattern_ = colorback_ = colorvram_ = alpharef_ = 0;
    smask0x_ = smask0y_ = 0;
    smask_.fill(0);
    xstart_ = ystart_ = xend_ = yend_ = 0;
    xsave_ = xmove_ = ymove_ = 0;
    bresd_ = bress1_ = bresoctinc1_ = bresrndinc2_ = brese1_ = bress2_ = aweight0_ = aweight1_ = 0;
    colorred_ = coloralpha_ = colorgrn_ = colorblue_ = 0;
    curr_red_ = curr_alpha_ = curr_grn_ = curr_blue_ = 0;
    slopered_ = slopealpha_ = slopegrn_ = slopeblue_ = 0;
    wrmask_ = colori_ = colorx_ = 0;
    hostrw_ = 0;
    host_shift_ = 0;
    dcbmode_ = kDefaultDcbMode;
    topscan_ = 0;
    xywin_ = kDefaultXYWin;
    clipmode_ = 0;
    config_ = kDefaultConfig;
    vrint_ = false;
    dcb_busy_until_ = 0;
    std::fill(rgbci_.begin(), rgbci_.end(), 0);
    std::fill(cidaux_.begin(), cidaux_.end(), 0);

    vc2_ = Vc2{};
    for (auto& x : xmap_) x = Xmap{};
    for (auto& c : cmap_) c = Cmap{};
    ramdac_ = Ramdac{};
    for (unsigned i = 0; i < 256; ++i)
        for (auto& ch : ramdac_.lut) ch[i] = static_cast<u8>(i);

    retrace_timer_.schedule_in(static_cast<u64>(cpu_hz_ / 60));
}

// ---------------------------------------------------------------------------
// Bus interface
// ---------------------------------------------------------------------------

namespace {
// Maps an offset to its register and whether it is the GO alias; false if undecoded.
bool decode(u32 offset, u32& reg, bool& go) {
    if (offset >= 0x2000) return false;
    go = (offset & 0x800) != 0;
    reg = offset & ~0x800u;
    return true;
}
}  // namespace

bool Newport::read(u32 offset, unsigned size, u64& value) {
    u32 reg;
    bool go;
    if (!decode(offset, reg, go)) return false;  // outside the REX3 window: no response
    const u32 word = reg & ~3u;
    if (word == kDcbData0 || word == kDcbData1) {
        value = dcb_read(reg, size);
        if (go) execute();
        return true;
    }
    if (size == 8) {
        value = (u64{rex3_read(word)} << 32) | rex3_read(word + 4);
    } else {
        const u32 v = rex3_read(word);
        value = size == 4 ? v : (v >> ((4 - size - (reg & 3)) * 8)) & ((1u << (size * 8)) - 1);
    }
    if (go) execute();
    return true;
}

bool Newport::write(u32 offset, unsigned size, u64 value) {
    u32 reg;
    bool go;
    if (!decode(offset, reg, go)) return false;
    const u32 word = reg & (size == 8 ? ~7u : ~3u);  // a doubleword is a register pair
    if (word == kDcbData0 || word == kDcbData1) {
        dcb_write(size, value);
    } else if (size == 8) {
        if (word == kHostRw0) {
            hostrw_ = value;
            host_shift_ = -1;  // recomputed on use
        } else {
            rex3_write(word, static_cast<u32>(value >> 32));
            rex3_write(word + 4, static_cast<u32>(value));
        }
    } else if (size == 4) {
        rex3_write(word, static_cast<u32>(value));
    } else {
        const unsigned shift = (4 - size - (reg & 3)) * 8;
        const u32 mask = ((1u << (size * 8)) - 1) << shift;
        const u32 old = word == kStatus ? 0 : rex3_read(word);
        rex3_write(word, (old & ~mask) | ((static_cast<u32>(value) << shift) & mask));
    }
    if (go) execute();
    return true;
}

u32 Newport::rex3_read(u32 reg) {
    if (reg >= kSmask1X && reg < kSmask1X + 0x20) return smask_[(reg - kSmask1X) >> 2];
    switch (reg) {
        case kDrawMode1: return drawmode1_;
        case kDrawMode0: return drawmode0_;
        case kLsMode: return lsmode_;
        case kLsPattern: return lspattern_;
        case kLsPatSave: return lspatsave_;
        case kZPattern: return zpattern_;
        case kColorBack: return colorback_;
        case kColorVram: return colorvram_;
        case kAlphaRef: return alpharef_;
        case kSmask0X: return smask0x_;
        case kSmask0Y: return smask0y_;
        case kXStart: return xstart_;
        case kYStart: return ystart_;
        case kXEnd: return xend_;
        case kYEnd: return yend_;
        case kXSave: return static_cast<u16>(xsave_);
        case kXYMove: return (static_cast<u32>(static_cast<u16>(xmove_)) << 16) | static_cast<u16>(ymove_);
        case kBresD: return bresd_;
        case kBresS1: return bress1_;
        case kBresOctInc1: return bresoctinc1_;
        case kBresRndInc2: return bresrndinc2_;
        case kBresE1: return brese1_;
        case kBresS2: return bress2_;
        case kAWeight0: return aweight0_;
        case kAWeight1: return aweight1_;
        case kXStartF: return xstart_ & 0x007FFF80u;
        case kYStartF: return ystart_ & 0x007FFF80u;
        case kXEndF:
        case kXEndF1: return xend_ & 0x007FFF80u;
        case kYEndF: return yend_ & 0x007FFF80u;
        case kXStartI: return static_cast<u16>(int_part(xstart_));
        case kXYStartI:
            return (static_cast<u32>(static_cast<u16>(int_part(xstart_))) << 16) | static_cast<u16>(int_part(ystart_));
        case kXYEndI:
            return (static_cast<u32>(static_cast<u16>(int_part(xend_))) << 16) | static_cast<u16>(int_part(yend_));
        case kXStartEndI:
            return (static_cast<u32>(static_cast<u16>(int_part(xstart_))) << 16) | static_cast<u16>(int_part(xend_));
        case kColorRed: return colorred_;
        case kColorAlpha: return coloralpha_;
        case kColorGrn: return colorgrn_;
        case kColorBlue: return colorblue_;
        case kSlopeRed:
        case kSlopeRed1: return slopered_;
        case kSlopeAlpha: return slopealpha_;
        case kSlopeGrn: return slopegrn_;
        case kSlopeBlue: return slopeblue_;
        case kWrMask: return wrmask_;
        case kColorI: return colori_;
        case kColorX: return colorx_;
        case kHostRw0: return static_cast<u32>(hostrw_ >> 32);
        case kHostRw1: return static_cast<u32>(hostrw_);
        case kDcbMode: return dcbmode_;
        case kTopScan: return topscan_;
        case kXYWin: return xywin_;
        case kClipMode: return clipmode_;
        case kConfig: return config_;
        case kStatus: {
            const u32 v = kStatusVersion | (vrint_ ? kStatusVrint : 0) | (dcb_stalled() ? kStatusBackBusy : 0);
            set_vrint(false);  // reading STATUS acknowledges the retrace interrupt
            return v;
        }
        case kUStatus: return kStatusVersion | (vrint_ ? kStatusVrint : 0) | (dcb_stalled() ? kStatusBackBusy : 0);
        default: return 0;
    }
}

void Newport::set_color_index(u32 v) {
    colori_ = v;
    if (drawmode1_ & kDm1RgbMode) {
        colorred_ = (v & 0xFF) << 11;
        colorgrn_ = (v & 0xFF00) << 3;
        colorblue_ = (v & 0xFF0000) >> 5;
        reload_shade();
    }
}

void Newport::rex3_write(u32 reg, u32 v) {
    if (reg >= kSmask1X && reg < kSmask1X + 0x20) {
        smask_[(reg - kSmask1X) >> 2] = v;
        return;
    }
    const unsigned depth = (drawmode1_ >> 3) & 3;
    switch (reg) {
        case kDrawMode1: drawmode1_ = v; return;
        case kDrawMode0: drawmode0_ = v & 0xFFFFFF; return;
        case kLsMode: lsmode_ = v & 0x0FFFFFFF; return;
        case kLsPattern: lspattern_ = v; return;
        case kLsPatSave: lspatsave_ = v; return;
        case kZPattern: zpattern_ = v; return;
        case kColorBack: colorback_ = v; return;
        case kColorVram: colorvram_ = v; return;
        case kAlphaRef: alpharef_ = v & 0xFF; return;
        case kSmask0X: smask0x_ = v; return;
        case kSmask0Y: smask0y_ = v; return;
        case kSetup: do_setup(); return;
        case kLsRestore: lspattern_ = lspatsave_; return;
        case kLsSave: lspatsave_ = lspattern_; return;
        case kStall0:
        case kStepZ:
        case kStall1: return;
        case kXStart:
        case kXStartF:
            write_x_start(reg == kXStartF ? (v & 0x007FFF80u) : v);
            xsave_ = int_part(xstart_);
            return;
        case kYStart:
        case kYStartF: write_y_start(reg == kYStartF ? (v & 0x007FFF80u) : v); return;
        case kXEnd:
        case kXEndF:
        case kXEndF1: write_x_end(reg == kXEnd ? v : (v & 0x007FFF80u)); return;
        case kYEnd:
        case kYEndF: write_y_end(reg == kYEnd ? v : (v & 0x007FFF80u)); return;
        case kXSave: xsave_ = static_cast<s16>(v); return;
        case kXYMove:
            xmove_ = static_cast<s16>(v >> 16);
            ymove_ = static_cast<s16>(v);
            return;
        case kBresD: bresd_ = v & 0x7FFFFFF; return;
        case kBresS1: bress1_ = v & 0x1FFFF; return;
        case kBresOctInc1: bresoctinc1_ = v & 0x070FFFFF; return;
        case kBresRndInc2: bresrndinc2_ = v & 0xFF1FFFFF; return;
        case kBresE1: brese1_ = v & 0xFFFF; return;
        case kBresS2: bress2_ = v & 0x3FFFFFF; return;
        case kAWeight0: aweight0_ = v; return;
        case kAWeight1: aweight1_ = v; return;
        case kXStartI:
            write_x_start(static_cast<u32>(static_cast<s32>(static_cast<s16>(v)) << 11));
            xsave_ = int_part(xstart_);
            return;
        case kXYStartI:
            write_x_start(static_cast<u32>(static_cast<s32>(static_cast<s16>(v >> 16)) << 11));
            write_y_start(static_cast<u32>(static_cast<s32>(static_cast<s16>(v)) << 11));
            xsave_ = int_part(xstart_);
            return;
        case kXYEndI:
            write_x_end(static_cast<u32>(static_cast<s32>(static_cast<s16>(v >> 16)) << 11));
            write_y_end(static_cast<u32>(static_cast<s32>(static_cast<s16>(v)) << 11));
            return;
        case kXStartEndI:
            write_x_start(static_cast<u32>(static_cast<s32>(static_cast<s16>(v >> 16)) << 11));
            write_x_end(static_cast<u32>(static_cast<s32>(static_cast<s16>(v)) << 11));
            xsave_ = int_part(xstart_);
            return;
        case kColorRed:
            colorred_ = v & 0xFFFFFF;
            curr_red_ = static_cast<s32>(colorred_);
            if (!(drawmode1_ & kDm1RgbMode)) {
                static constexpr unsigned kCiShift[4] = {11, 11, 9, 11};
                colori_ = (v >> kCiShift[depth]) & depth_mask(depth);
            }
            return;
        case kColorAlpha:
            coloralpha_ = v & 0xFFFFF;
            curr_alpha_ = static_cast<s32>(coloralpha_);
            return;
        case kColorGrn:
            colorgrn_ = v & 0xFFFFF;
            curr_grn_ = static_cast<s32>(colorgrn_);
            return;
        case kColorBlue:
            colorblue_ = v & 0xFFFFF;
            curr_blue_ = static_cast<s32>(colorblue_);
            return;
        case kSlopeRed:
        case kSlopeRed1: slopered_ = sign_magnitude(v, 0x7FFFFF, 0x800000); return;
        case kSlopeAlpha: slopealpha_ = sign_magnitude(v, 0x7FFFF, 0x80000); return;
        case kSlopeGrn: slopegrn_ = sign_magnitude(v, 0x7FFFF, 0x80000); return;
        case kSlopeBlue: slopeblue_ = sign_magnitude(v, 0x7FFFF, 0x80000); return;
        case kWrMask: wrmask_ = v & 0xFFFFFF; return;
        case kColorI: set_color_index(v); return;
        case kColorX:
            colorx_ = v & 0x7FFFFF;
            colorred_ = colorx_;
            curr_red_ = static_cast<s32>(colorred_);
            return;
        case kHostRw0:
            hostrw_ = (hostrw_ & 0xFFFFFFFFull) | (u64{v} << 32);
            host_shift_ = -1;
            return;
        case kHostRw1:
            hostrw_ = (hostrw_ & 0xFFFFFFFF00000000ull) | v;
            host_shift_ = -1;
            return;
        case kDcbMode: dcbmode_ = v & 0x1FFFFFFF; return;
        case kTopScan:
            topscan_ = v & 0x3FF;
            picture_changed();
            return;
        case kXYWin: xywin_ = v; return;
        case kClipMode: clipmode_ = v & 0x1FFF; return;
        case kConfig: config_ = v & 0x1FFFFF; return;
        case kDcbReset:
            dcb_busy_until_ = 0;
            vc2_.ram_hi_pending = vc2_.ram_read_hi = true;
            for (auto& x : xmap_) x.mode_count = 0;
            for (auto& c : cmap_) c.rgb = 0;
            ramdac_.rgb = 0;
            return;
        default: return;
    }
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

void Newport::execute() {
    if (drawmode0_ & kDm0DoSetup) do_setup();
    switch (drawmode0_ & 3) {
        case 0: return;  // NOOP
        case 1: pixel_read(); return;
        case 2:
            switch ((drawmode0_ >> 2) & 7) {
                case 0: draw_span_block(false); return;
                case 1: draw_span_block(true); return;
                case 2:
                case 3: draw_line(); return;
                case 4: draw_aline(); return;
                default: SG_LOG(Debug, "newport", "invalid address mode %u", (drawmode0_ >> 2) & 7); return;
            }
        case 3: scr2scr(); return;
    }
}

void Newport::do_setup() {
    const s32 x1 = sext(xstart_ >> 7, 20), y1 = sext(ystart_ >> 7, 20);
    const s32 x2 = sext(xend_ >> 7, 20), y2 = sext(yend_ >> 7, 20);
    const s32 dx = std::abs(x1 - x2), dy = std::abs(y1 - y2);
    const u32 octant = (y1 > y2 ? 1u : 0u) | (x1 > x2 ? 2u : 0u) | (dx > dy ? 4u : 0u);
    bresoctinc1_ = (bresoctinc1_ & ~0x07000000u) | (octant << 24);
}

void Newport::reload_shade() {
    curr_red_ = static_cast<s32>(colorred_);
    curr_alpha_ = static_cast<s32>(coloralpha_);
    curr_grn_ = static_cast<s32>(colorgrn_);
    curr_blue_ = static_cast<s32>(colorblue_);
}

void Newport::iterate_shade() {
    curr_red_ += sext(slopered_, 24);
    curr_alpha_ += sext(slopealpha_, 20);
    curr_grn_ += sext(slopegrn_, 20);
    curr_blue_ += sext(slopeblue_, 20);
    if (drawmode0_ & kDm0CiClamp) {
        if (drawmode1_ & kDm1RgbMode) {
            for (s32* c : {&curr_red_, &curr_alpha_, &curr_grn_, &curr_blue_}) *c = std::clamp(*c, 0, 0xFF << 11);
        } else {
            const s32 max = static_cast<s32>(depth_mask((drawmode1_ >> 3) & 3)) << 11;
            curr_red_ = std::clamp(curr_red_, 0, max);
        }
    }
}

u32 Newport::default_color(u32 src) const {
    const unsigned depth = (drawmode1_ >> 3) & 3;
    u32 c = (drawmode1_ & kDm1FastClear) ? colorvram_ : src;
    switch (depth) {
        case 0: return (c & 0xF) * 0x11111111u;
        case 1: return (c & 0xFF) * 0x01010101u;
        case 2:
            if ((drawmode1_ & kDm1RgbMode) && (drawmode1_ & kDm1FastClear))
                c = ((colorvram_ & 0xF00000) >> 12) | ((colorvram_ & 0xF000) >> 8) | ((colorvram_ & 0xF0) >> 4);
            c &= 0xFFF;
            return c | (c << 12);
        default: return c & 0xFFFFFF;
    }
}

u32 Newport::shaded_color(int x, int y) const {
    const unsigned depth = (drawmode1_ >> 3) & 3;
    if (!(drawmode1_ & kDm1RgbMode)) return default_color(static_cast<u32>(curr_red_ >> 11));
    auto channel = [](s32 v) -> u32 {
        const u32 c = (static_cast<u32>(v) >> 11) & 0x1FF;
        if (c >= 0x180 || v < 0) return 0;
        return c > 0xFF ? 0xFF : c;
    };
    const u32 r = channel(curr_red_), g = channel(curr_grn_), b = channel(curr_blue_);
    if ((drawmode1_ & kDm1Dither) && depth < 3) {
        const int t = kBayer[x & 3][y & 3];
        auto dither = [t](u32 s, u32 max) {
            u32 d = s >> 4;
            if ((s & 15) > static_cast<u32>(t)) ++d;
            return std::min(d, max);
        };
        switch (depth) {
            case 0:
                return (dither((b >> 3) - (b >> 4), 1) << 3) | (dither((g >> 2) - (g >> 4), 3) << 1) |
                       dither((r >> 3) - (r >> 4), 1);
            case 1:
                return (dither((b >> 2) - (b >> 4), 3) << 6) | (dither((g >> 1) - (g >> 4), 7) << 3) |
                       dither((r >> 1) - (r >> 4), 7);
            default:
                return (dither(b - (b >> 4), 15) << 8) | (dither(g - (g >> 4), 15) << 4) | dither(r - (r >> 4), 15);
        }
    }
    return pack(r, g, b, depth);
}

u32 Newport::host_color(u32* alpha) {
    static constexpr int kWidths[4] = {8, 8, 16, 32};
    static constexpr u32 kMasks[4] = {0xF, 0xFF, 0xFFF, 0xFFFFFFFF};
    const unsigned hostdepth = (drawmode1_ >> 8) & 3;
    const int width = kWidths[hostdepth];
    if (host_shift_ < 0) host_shift_ = 64 - width;
    u32 colour = static_cast<u32>(hostrw_ >> host_shift_) & kMasks[hostdepth];
    if (alpha) *alpha = hostdepth == 3 ? colour >> 24 : 0xFF;  // only 32-bit host pixels carry alpha (ABGR)
    if (drawmode1_ & kDm1RwPacked) {
        if (((drawmode1_ & kDm1RwDouble) && host_shift_ > 0) || host_shift_ > 32) host_shift_ -= width;
        else host_shift_ = 64 - width;
    }
    const unsigned depth = (drawmode1_ >> 3) & 3;
    const unsigned planes = drawmode1_ & 7;
    if ((planes == kPlanesRgb || planes == kPlanesRgba) && (drawmode1_ & kDm1RgbMode) && hostdepth != depth) {
        u32 r, g, b;
        unpack(colour, hostdepth, r, g, b);
        colour = pack(r, g, b, depth);
    }
    if (drawmode1_ & kDm1SwapEndian) colour = byteswap(colour);
    return colour;
}

// The pixel's colour, and its alpha for blending (src_alpha_): from the
// alpha iterator, or from the host data with ALPHAHOST (which may also be
// set without COLORHOST, to blend iterated colours by host alphas).
u32 Newport::source_color(int x, int y) {
    if (drawmode1_ & kDm1FastClear) return default_color(colorvram_);
    u32 host_alpha = 0xFF, colour;
    if (drawmode0_ & kDm0ColorHost) colour = host_color(&host_alpha);
    else if ((drawmode0_ & kDm0Shade) || (drawmode1_ & kDm1RgbMode)) colour = shaded_color(x, y);
    else colour = default_color(colori_);
    if (drawmode1_ & kDm1Blend) {
        if (drawmode0_ & kDm0AlphaHost) {
            if (!(drawmode0_ & kDm0ColorHost)) host_color(&host_alpha);
            src_alpha_ = host_alpha;
        } else {
            const u32 a = (static_cast<u32>(curr_alpha_) >> 11) & 0x1FF;
            src_alpha_ = (a >= 0x180 || curr_alpha_ < 0) ? 0 : std::min<u32>(a, 0xFF);
        }
    }
    return colour;
}

// Cb = Cs * Fs + Cd * Fd for each channel, the destination being the frame
// buffer or COLORBACK; factors are normalised as (v + msb) / 256, so 0xFF is
// 1.0. Blending works on RGB pixels; the result replaces the logic op.
u32 Newport::blend(u32 src, int px, int py) const {
    const unsigned depth = (drawmode1_ >> 3) & 3;
    u32 sr, sg, sb, dr, dg, db;
    unpack(src & depth_mask(depth), depth, sr, sg, sb);
    unpack((drawmode1_ & kDm1BackBlend) ? colorback_ & depth_mask(depth) : fetch(px, py), depth, dr, dg, db);
    auto norm = [](u32 v) { return v + (v >> 7); };
    const u32 sa = norm(src_alpha_);
    const unsigned sf = (drawmode1_ >> kDm1SFactorShift) & 7, df = (drawmode1_ >> kDm1DFactorShift) & 7;
    auto channel = [&](u32 s, u32 d) {
        static constexpr u32 kOne = 256;
        u32 fs = 0, fd = 0;
        switch (sf) {
            case 1: fs = kOne; break;
            case 2: fs = norm(d); break;
            case 3: fs = kOne - norm(d); break;
            case 4: fs = sa; break;
            case 5: fs = kOne - sa; break;
            default: break;
        }
        switch (df) {
            case 1: fd = kOne; break;
            case 2: fd = norm(s); break;
            case 3: fd = kOne - norm(s); break;
            case 4: fd = sa; break;
            case 5: fd = kOne - sa; break;
            default: break;
        }
        return std::min<u32>(0xFF, (s * fs + d * fd) >> 8);
    };
    return pack(channel(sr, dr), channel(sg, dg), channel(sb, db), depth);
}

bool Newport::clip(int x, int y, int& px, int& py) const {
    if (clipmode_ & 1) {
        if (x < static_cast<s16>(smask0x_ >> 16) || x > static_cast<s16>(smask0x_) ||
            y < static_cast<s16>(smask0y_ >> 16) || y > static_cast<s16>(smask0y_))
            return false;
    }
    px = x + static_cast<s16>(xywin_ >> 16) - 0x1000;
    py = y + static_cast<s16>(xywin_) - 0x1000;
    if (clipmode_ & 0x1E) {
        bool inside = false;
        for (unsigned m = 0; m < 4 && !inside; ++m) {
            if (!(clipmode_ & (2u << m))) continue;
            const u32 mx = smask_[2 * m], my = smask_[2 * m + 1];
            inside = px >= static_cast<s16>(mx >> 16) - 0x1000 && px <= static_cast<s16>(mx) - 0x1000 &&
                     py >= static_cast<s16>(my >> 16) - 0x1000 && py <= static_cast<s16>(my) - 0x1000;
        }
        if (!inside) return false;
    }
    return px >= 0 && px < kPitch && py >= 0 && py < kRows;
}

unsigned Newport::src_shift() const {
    const unsigned planes = drawmode1_ & 7, depth = (drawmode1_ >> 3) & 3, buf = (drawmode1_ >> 5) & 1;
    switch (planes) {
        case kPlanesRgb:
        case kPlanesRgba: {
            static constexpr unsigned kShift[4][2] = {{0, 4}, {0, 8}, {0, 12}, {0, 0}};
            return kShift[depth][buf];
        }
        case kPlanesOlay: return buf ? 16 : 8;
        case kPlanesPup: return buf ? 6 : 2;
        case kPlanesCid: return buf ? 4 : 0;
        default: return 0;
    }
}

u32 Newport::fetch(int px, int py) const {
    const unsigned planes = drawmode1_ & 7, depth = (drawmode1_ >> 3) & 3;
    const std::size_t idx = static_cast<std::size_t>(py) * kPitch + static_cast<std::size_t>(px);
    if (planes == kPlanesRgb || planes == kPlanesRgba) return (rgbci_[idx] >> src_shift()) & depth_mask(depth);
    if (planes == kPlanesOlay) return (cidaux_[idx] >> src_shift()) & 0xFF;
    return (cidaux_[idx] >> src_shift()) & 0x3;
}

void Newport::store(int px, int py, u32 colour) {
    ++pixels_drawn_;
    const unsigned planes = drawmode1_ & 7, depth = (drawmode1_ >> 3) & 3;
    if (planes == kPlanesNone || planes == 3 || planes == 7) return;
    if ((drawmode1_ & (kDm1Blend | kDm1FastClear | kDm1RgbMode)) == (kDm1Blend | kDm1RgbMode) &&
        (planes == kPlanesRgb || planes == kPlanesRgba))
        colour = blend(colour, px, py);
    // Replicate the value into every buffer lane, then move aux values into place.
    u32 v;
    switch (depth) {
        case 0: v = (colour & 0xF) * 0x11111111u; break;
        case 1: v = (colour & 0xFF) * 0x01010101u; break;
        case 2: v = (colour & 0xFFF) | ((colour & 0xFFF) << 12); break;
        default: v = colour & 0xFFFFFF; break;
    }
    if (planes == kPlanesOlay) v <<= 8;
    else if (planes == kPlanesPup) v <<= 2;

    u32& dst = (planes == kPlanesRgb || planes == kPlanesRgba ? rgbci_ : cidaux_)
        [static_cast<std::size_t>(py) * kPitch + static_cast<std::size_t>(px)];
    if (!(drawmode1_ & (kDm1FastClear | kDm1Blend))) {
        const u32 s = v, d = dst;
        switch (drawmode1_ >> 28) {
            case 0: v = 0; break;
            case 1: v = s & d; break;
            case 2: v = s & ~d; break;
            case 3: v = s; break;
            case 4: v = ~s & d; break;
            case 5: v = d; break;
            case 6: v = s ^ d; break;
            case 7: v = s | d; break;
            case 8: v = ~(s | d); break;
            case 9: v = ~(s ^ d); break;
            case 10: v = ~d; break;
            case 11: v = s | ~d; break;
            case 12: v = ~s; break;
            case 13: v = ~s | d; break;
            case 14: v = ~(s & d); break;
            default: v = ~0u; break;
        }
    }
    const u32 mask = wrmask_ & (opts_.xl24 ? 0xFFFFFFu : 0xFFu);
    const u32 updated = (dst & ~mask) | (v & mask);
    if (updated != dst) {
        dst = updated;
        ++display_version_;
        row_dirty_[static_cast<std::size_t>(py)] = 1;
    }
}

// CID checking is not applied: IRIX 6.5 clears the CID planes to one code
// for the whole screen, then draws visible GL windows with CIDMATCH 0 (no
// code selected) and clips them with the screen masks, so on the real
// board such writes cannot be blocked as the REX3 specification reads.
void Newport::plot(int x, int y, u32 colour) {
    int px, py;
    if (clip(x, y, px, py)) store(px, py, colour);
}

void Newport::draw_span_block(bool block) {
    const u32 octant = (bresoctinc1_ >> 24) & 7;
    const int dx = (octant & 2) ? -1 : 1, dy = (octant & 1) ? -1 : 1;
    int sx = int_part(xstart_), sy = int_part(ystart_);
    const int end_x = int_part(xend_) + dx, end_y = int_part(yend_) + dy;
    bool stop_x = drawmode0_ & kDm0StopOnX;
    const bool stop_y = block && (drawmode0_ & kDm0StopOnY);
    const u32 pattern = (drawmode0_ & kDm0EnZPattern)    ? zpattern_
                        : (drawmode0_ & kDm0EnLsPattern) ? lspattern_
                                                         : ~0u;
    const bool opaque = drawmode0_ & (kDm0ZpOpaque | kDm0LsOpaque);
    const bool fastclear = drawmode1_ & kDm1FastClear;

    int prim_limit = 0x10000;
    if (drawmode0_ & kDm0Length32) prim_limit = 32;
    if ((drawmode0_ & kDm0ColorHost) && (drawmode1_ & kDm1RwPacked)) {
        static constexpr int kPerWord[4][2] = {{4, 8}, {4, 8}, {2, 4}, {1, 2}};
        prim_limit = std::min(prim_limit, kPerWord[(drawmode1_ >> 8) & 3][(drawmode1_ & kDm1RwDouble) ? 1 : 0]);
        stop_x = true;
    }

    for (int rows = 0; rows < kRows * 4; ++rows) {
        unsigned bit = 31;
        int drawn = 0;
        const bool lr_abort = (drawmode0_ & kDm0LrOnly) && dx < 0;
        if (!lr_abort) {
            do {
                if ((drawmode0_ & kDm0Shade) && !fastclear) iterate_shade();
                if ((pattern >> bit) & 1) plot(sx, sy, source_color(sx, sy));
                else if (opaque) plot(sx, sy, default_color(colorback_));
                bit = (bit - 1) & 31;
                sx += dx;
                ++drawn;
            } while (sx != end_x && drawn < prim_limit && stop_x && drawn < 0x10000);
        }
        const bool row_done = sx == end_x || lr_abort;
        if (row_done) {
            reload_shade();
            if (block) {
                sx = xsave_;
                sy += dy;
            }
        }
        // Host-supplied pixels come a data word per GO: a finished row
        // waits for the next word rather than repeating this one.
        if (!block || !row_done || !stop_y || sy == end_y || (drawmode0_ & kDm0ColorHost)) break;
    }
    write_x_start(static_cast<u32>(sx) << 11);
    if (block) write_y_start(static_cast<u32>(sy) << 11);
}

// Antialiased line (A_LINE): at each step along the major axis, the two
// pixels either side of the exact line (subpixel endpoints) are drawn, each
// with a coverage from the 16-entry AWEIGHT table: the far pixel's indexed by
// the line's fractional distance into it, the near pixel's by the rest. In
// RGB the coverage is the source alpha (software blends with SA, 1-SA); in
// colour index it replaces the index's low four bits. The order of the
// table's 4-bit entries in AWEIGHT0/1 is not in the specification: entry 0
// is taken from the low bits of AWEIGHT0 (UNVERIFIED).
void Newport::draw_aline() {
    const u32 octant = (bresoctinc1_ >> 24) & 7;
    const bool xmajor = octant & 4;
    // 16.4 fixed point endpoints
    const double x1 = sext(xstart_ >> 7, 20) / 16.0, y1 = sext(ystart_ >> 7, 20) / 16.0;
    const double x2 = sext(xend_ >> 7, 20) / 16.0, y2 = sext(yend_ >> 7, 20) / 16.0;
    const double m1 = xmajor ? x1 : y1, m2 = xmajor ? x2 : y2, n1 = xmajor ? y1 : x1, n2 = xmajor ? y2 : x2;
    const int step = m2 >= m1 ? 1 : -1;
    const double slope = m2 != m1 ? (n2 - n1) / (m2 - m1) : 0.0;
    const int first = static_cast<int>(std::lround(m1)), last = static_cast<int>(std::lround(m2));
    auto weight = [this](double c) {
        const unsigned i = std::min(15u, static_cast<unsigned>(c * 16));
        return ((i < 8 ? aweight0_ : aweight1_) >> (4 * (i & 7))) & 0xF;
    };
    const bool rgb = drawmode1_ & kDm1RgbMode;
    auto draw = [&](int m, int n, unsigned coverage) {
        if (!coverage) return;
        const int px = xmajor ? m : n, py = xmajor ? n : m;
        u32 colour = source_color(px, py);
        if (rgb) src_alpha_ = coverage * 0x11;
        else colour = (colour & ~0xFu) | coverage;
        plot(px, py, colour);
    };
    const bool one = !(drawmode0_ & (kDm0StopOnX | kDm0StopOnY));  // one step per GO
    for (int m = first;; m += step) {
        const bool skip = (m == first && (drawmode0_ & kDm0SkipFirst)) || (m == last && (drawmode0_ & kDm0SkipLast));
        if (drawmode0_ & kDm0Shade) iterate_shade();
        if (!skip) {
            const double n = n1 + (m - m1) * slope;
            const double lower = std::floor(n), f = n - lower;
            draw(m, static_cast<int>(lower), weight(1.0 - f));
            draw(m, static_cast<int>(lower) + 1, weight(f));
        }
        if (one || m == last) {
            // Carry on from the next step (one point per GO) or rest at the end.
            const double nm = one && m != last ? m + step : m2, nn = one && m != last ? n1 + (nm - m1) * slope : n2;
            const auto fixed = [](double v) { return static_cast<u32>(static_cast<s32>(std::lround(v * 16))) << 7; };
            write_x_start(fixed(xmajor ? nm : nn));
            write_y_start(fixed(xmajor ? nn : nm));
            break;
        }
    }
    if (!one) reload_shade();
}

void Newport::draw_line() {
    // Integer Bresenham between the start and end points. Fractional (F_LINE)
    // lines use the integer endpoints for now.
    const u32 octant = (bresoctinc1_ >> 24) & 7;
    const int x1 = int_part(xstart_), y1 = int_part(ystart_), x2 = int_part(xend_), y2 = int_part(yend_);
    const bool xmajor = octant & 4;
    const int sx_step = (octant & 2) ? -1 : 1, sy_step = (octant & 1) ? -1 : 1;
    const int dx = std::abs(x2 - x1), dy = std::abs(y2 - y1);
    const int major = xmajor ? dx : dy, minor = xmajor ? dy : dx;
    int d = 2 * minor - major;
    int x = x1, y = y1;
    auto step = [&] {
        if (d >= 0) {
            if (xmajor) y += sy_step;
            else x += sx_step;
            d -= 2 * major;
        }
        if (xmajor) x += sx_step;
        else y += sy_step;
        d += 2 * minor;
    };
    const u32 pattern = (drawmode0_ & kDm0EnLsPattern) ? lspattern_ : ~0u;
    const bool opaque = drawmode0_ & kDm0LsOpaque;
    unsigned bit = 31;
    auto draw = [&](int px, int py) {
        if (drawmode0_ & kDm0Shade) iterate_shade();
        if ((pattern >> bit) & 1) plot(px, py, source_color(px, py));
        else if (opaque) plot(px, py, default_color(colorback_));
        bit = (bit - 1) & 31;
    };

    if (!(drawmode0_ & (kDm0StopOnX | kDm0StopOnY))) {  // one point per GO
        if (!(drawmode0_ & kDm0SkipFirst)) draw(x, y);
        step();
    } else {
        int n = major;
        if ((drawmode0_ & kDm0Length32) && n > 32) n = 32;
        for (int i = 0; i <= n; ++i) {
            const bool skip = (i == 0 && (drawmode0_ & kDm0SkipFirst)) || (i == n && (drawmode0_ & kDm0SkipLast));
            if (!skip) draw(x, y);
            if (i < n) step();
        }
        reload_shade();
    }
    write_x_start(static_cast<u32>(x) << 11);
    write_y_start(static_cast<u32>(y) << 11);
}

void Newport::scr2scr() {
    const unsigned adrmode = (drawmode0_ >> 2) & 7;
    if (adrmode > 1) return;
    const u32 octant = (bresoctinc1_ >> 24) & 7;
    const int dx = (octant & 2) ? -1 : 1, dy = (octant & 1) ? -1 : 1;
    int sx = int_part(xstart_), sy = int_part(ystart_);
    const int end_x = int_part(xend_) + dx, end_y = int_part(yend_) + dy;
    const bool stop_x = drawmode0_ & kDm0StopOnX, stop_y = (adrmode == 1) && (drawmode0_ & kDm0StopOnY);
    const int xwin = static_cast<s16>(xywin_ >> 16) - 0x1000, ywin = static_cast<s16>(xywin_) - 0x1000;

    for (int rows = 0; rows < kRows * 4; ++rows) {
        int n = 0;
        do {
            const int px = sx + xwin, py = sy + ywin;
            if (px >= 0 && px < kPitch && py >= 0 && py < kRows) plot(sx + xmove_, sy + ymove_, fetch(px, py));
            sx += dx;
        } while (sx != end_x && stop_x && ++n < 0x10000);
        const bool row_done = sx == end_x;
        if (row_done) {
            sx = xsave_;
            sy += dy;
        }
        if (!row_done || !stop_y || sy == end_y) break;
    }
    write_x_start(static_cast<u32>(sx) << 11);
    write_y_start(static_cast<u32>(sy) << 11);
}

void Newport::pixel_read() {
    static constexpr int kPerWord[4][2] = {{8, 16}, {4, 8}, {2, 4}, {1, 2}};
    static constexpr int kStart[4] = {60, 56, 48, 32};
    static constexpr int kStep[4] = {4, 8, 16, 32};
    const unsigned hostdepth = (drawmode1_ >> 8) & 3, depth = (drawmode1_ >> 3) & 3, planes = drawmode1_ & 7;
    int sx = int_part(xstart_), sy = int_part(ystart_);
    const int ex = int_part(xend_), ey = int_part(yend_);
    const int width = std::min(ex - sx + 1, kPerWord[hostdepth][(drawmode1_ & kDm1RwDouble) ? 1 : 0]);
    const int xwin = static_cast<s16>(xywin_ >> 16) - 0x1000, ywin = static_cast<s16>(xywin_) - 0x1000;
    u64 result = 0;
    int shift = kStart[hostdepth];
    for (int i = 0; i < std::max(width, 1); ++i) {
        const int px = sx + xwin, py = sy + ywin;
        u32 v = (px >= 0 && px < kPitch && py >= 0 && py < kRows) ? fetch(px, py) : 0;
        if ((planes == kPlanesRgb || planes == kPlanesRgba) && (drawmode1_ & kDm1RgbMode) && hostdepth != depth) {
            u32 r, g, b;
            unpack(v, depth, r, g, b);
            v = pack(r, g, b, hostdepth);
        }
        const u64 mask = hostdepth == 3 ? 0xFFFFFFFFull : (u64{1} << kStep[hostdepth]) - 1;
        result |= (u64{v} & mask) << shift;
        shift -= kStep[hostdepth];
        if (++sx > ex) {
            sy += ey < sy ? -1 : 1;
            sx = xsave_;
        }
    }
    hostrw_ = result;
    write_x_start(static_cast<u32>(sx) << 11);
    write_y_start(static_cast<u32>(sy) << 11);
}

// ---------------------------------------------------------------------------
// Display control bus
// ---------------------------------------------------------------------------

void Newport::dcb_begin() {
    for (auto& x : xmap_) x.mode_count = 0;
    // Only slaves 0-7 exist on the board. An access to any other that waits
    // for an acknowledge keeps the bus busy until it times out or the bus is
    // reset. IRIX probes for an Indy Presenter adapter this way (a few
    // back-to-back status reads); the PROM simply waits for the bus.
    constexpr u32 kDcbSyncAck = 1u << 13, kDcbAsyncAck = 1u << 14;
    constexpr double kAckTimeout = 10e-6;
    if (((dcbmode_ >> 7) & 0xF) >= 8 && (dcbmode_ & (kDcbSyncAck | kDcbAsyncAck)))
        dcb_busy_until_ = scheduler_.now() + static_cast<u64>(cpu_hz_ * kAckTimeout);
}

bool Newport::dcb_stalled() const { return scheduler_.now() < dcb_busy_until_; }

void Newport::dcb_write(unsigned size, u64 value) {
    dcb_begin();
    // The bytes of the CPU access in address order (most significant first).
    u8 bytes[8];
    unsigned n = 0;
    if (size == 8) {
        for (int i = 7; i >= 0; --i) bytes[n++] = static_cast<u8>(value >> (8 * i));
    } else {
        for (int i = static_cast<int>(size) - 1; i >= 0; --i) bytes[n++] = static_cast<u8>(value >> (8 * i));
    }
    unsigned width = dcbmode_ & 3;
    if (width == 0) width = 4;
    n = std::min(n, width);
    if (dcbmode_ & (1u << 28)) std::reverse(bytes, bytes + n);  // SWAPENDIAN
    const unsigned addr = (dcbmode_ >> 7) & 0xF;
    unsigned crs = (dcbmode_ >> 4) & 7;
    for (unsigned i = 0; i < n; ++i) {
        dcb_write_byte(addr, crs, bytes[i]);
        if (dcbmode_ & 8) crs = (crs + 1) & 7;
    }
}

u64 Newport::dcb_read(u32 reg, unsigned size) {
    dcb_begin();
    unsigned width = dcbmode_ & 3;
    if (width == 0) width = 4;
    const unsigned addr = (dcbmode_ >> 7) & 0xF;
    unsigned crs = (dcbmode_ >> 4) & 7;
    u8 bytes[4] = {0, 0, 0, 0};
    for (unsigned i = 0; i < width; ++i) {
        bytes[i] = dcb_read_byte(addr, crs);
        if (dcbmode_ & 8) crs = (crs + 1) & 7;
    }
    if (dcbmode_ & (1u << 28)) std::reverse(bytes, bytes + width);
    // Data is left-justified in the word and replicated into the lower lanes
    // for 1- and 2-byte transfers, so narrow and full-word loads both see it.
    u32 v = (u32{bytes[0]} << 24) | (u32{bytes[1]} << 16) | (u32{bytes[2]} << 8) | bytes[3];
    if (width == 1) v = bytes[0] * 0x01010101u;
    else if (width == 2) v = ((u32{bytes[0]} << 8) | bytes[1]) * 0x00010001u;
    if (size == 8) return u64{v} << 32;
    if (size == 4) return v;
    return (v >> ((4 - size - (reg & 3)) * 8)) & ((1u << (size * 8)) - 1);
}

void Newport::dcb_write_byte(unsigned addr, unsigned crs, u8 b) {
    switch (addr) {
        case 0: vc2_write(crs, b); break;
        case 1:
            cmap_write(0, crs, b);
            cmap_write(1, crs, b);
            break;
        case 2: cmap_write(0, crs, b); break;
        case 3: cmap_write(1, crs, b); break;
        case 4:
            xmap_write(0, crs, b);
            xmap_write(1, crs, b);
            break;
        case 5: xmap_write(0, crs, b); break;
        case 6: xmap_write(1, crs, b); break;
        case 7: ramdac_write(crs, b); break;
        default: break;  // no device responds
    }
}

u8 Newport::dcb_read_byte(unsigned addr, unsigned crs) {
    switch (addr) {
        case 0: return vc2_read(crs);
        case 1:
        case 2: return cmap_read(0, crs);
        case 3: return cmap_read(1, crs);
        case 4:
        case 5: return xmap_read(0, crs);
        case 6: return xmap_read(1, crs);
        case 7: return ramdac_read(crs);
        default: return 0;
    }
}

void Newport::vc2_write(unsigned crs, u8 b) {
    switch (crs) {
        case 0: vc2_.index = b & 0x1F; break;
        case 1: vc2_.data_hi = b; break;
        case 2: {
            const u16 v = static_cast<u16>((vc2_.data_hi << 8) | b);
            // A cursor move only changes the lines it leaves and enters.
            const bool cursor = vc2_.index == kVc2CursorX || vc2_.index == kVc2CursorY || vc2_.index == kVc2CurCursorX;
            if (cursor) cursor_moving();
            vc2_.reg[vc2_.index] = v;
            if (vc2_.index == kVc2RamAddr) {
                vc2_.reg[kVc2RamAddr] = v & 0x7FFF;
                vc2_.ram_hi_pending = vc2_.ram_read_hi = true;
            } else if (vc2_.index == kVc2CursorY) {
                vc2_.reg[kVc2CurCursorX] = vc2_.reg[kVc2CursorX];
            }
            if (cursor) cursor_moving();
            else if (vc2_.index != kVc2RamAddr) picture_changed();
            break;
        }
        case 3:
            if (vc2_.ram_hi_pending) {
                vc2_.data_hi = b;
                vc2_.ram_hi_pending = false;
            } else {
                u16& addr = vc2_.reg[kVc2RamAddr];
                vc2_.ram[addr & 0x7FFF] = static_cast<u16>((vc2_.data_hi << 8) | b);
                addr = (addr + 1) & 0x7FFF;
                vc2_.ram_hi_pending = true;
                picture_changed();  // timing, DID tables or the cursor glyph
            }
            break;
        default: break;
    }
}

u8 Newport::vc2_read(unsigned crs) {
    auto reg = [this](unsigned i) -> u16 {
        if (i == kVc2LineCounter) {  // beam position, derived from time within the frame
            const u64 frame = static_cast<u64>(cpu_hz_ / 60);
            return static_cast<u16>((scheduler_.now() % frame) * 1066 / frame);
        }
        return vc2_.reg[i];
    };
    switch (crs) {
        case 0: return static_cast<u8>(vc2_.index);
        case 1: return static_cast<u8>(reg(vc2_.index) >> 8);
        case 2: return static_cast<u8>(reg(vc2_.index));
        case 3: {
            u16& addr = vc2_.reg[kVc2RamAddr];
            const u16 w = vc2_.ram[addr & 0x7FFF];
            if (vc2_.ram_read_hi) {
                vc2_.ram_lo_latch = static_cast<u8>(w);
                vc2_.ram_read_hi = false;
                return static_cast<u8>(w >> 8);
            }
            addr = (addr + 1) & 0x7FFF;
            vc2_.ram_read_hi = true;
            return vc2_.ram_lo_latch;
        }
        default: return 0;
    }
}

void Newport::xmap_write(unsigned which, unsigned crs, u8 b) {
    picture_changed();
    Xmap& x = xmap_[which];
    switch (crs) {
        case 0: x.config = b; break;
        case 3: x.cursor_msb = b; break;
        case 4: x.pup_msb = b; break;
        case 5:  // mode table: index byte then 24-bit mode, most significant first
            x.mode_bytes[x.mode_count++] = b;
            if (x.mode_count == 4) {
                x.mode[x.mode_bytes[0] & 31] = (u32{x.mode_bytes[1]} << 16) | (u32{x.mode_bytes[2]} << 8) | x.mode_bytes[3];
                x.mode_count = 0;
            }
            break;
        case 7: x.mode_addr = b; break;
        default: break;
    }
}

u8 Newport::xmap_read(unsigned which, unsigned crs) {
    const Xmap& x = xmap_[which];
    switch (crs) {
        case 0: return static_cast<u8>(x.config | (opts_.xl24 ? 0 : 0x04));
        case 1: return opts_.xl24 ? 3 : 1;
        case 2: return 0x02;  // FIFO empty: all entries available
        case 3: return x.cursor_msb;
        case 4: return x.pup_msb;
        case 5: return static_cast<u8>(x.mode[(x.mode_addr >> 2) & 31] >> (8 * (x.mode_addr & 3)));
        case 7: return x.mode_addr;
        default: return 0;
    }
}

void Newport::cmap_write(unsigned which, unsigned crs, u8 b) {
    picture_changed();
    Cmap& c = cmap_[which];
    switch (crs) {
        case 0:
            c.addr = static_cast<u16>((c.addr & 0x1F00) | b);
            c.rgb = 0;
            break;
        case 1:
            c.addr = static_cast<u16>((c.addr & 0xFF) | ((b & 0x1F) << 8));
            c.rgb = 0;
            break;
        case 2:
            c.pending = (c.pending << 8) | b;
            if (++c.rgb == 3) {
                c.palette[c.addr & 0x1FFF] = c.pending & 0xFFFFFF;
                c.addr = (c.addr + 1) & 0x1FFF;
                c.rgb = 0;
                c.pending = 0;
            }
            break;
        case 3: c.command = b; break;
        case 7: c.crs7 = b; break;
        default: break;
    }
}

u8 Newport::cmap_read(unsigned which, unsigned crs) {
    Cmap& c = cmap_[which];
    switch (crs) {
        case 0: return static_cast<u8>(c.addr);
        case 1: return static_cast<u8>(c.addr >> 8);
        case 2: {
            const u32 e = c.palette[c.addr & 0x1FFF];
            const u8 v = static_cast<u8>(e >> (16 - 8 * c.rgb));
            if (++c.rgb == 3) {
                c.rgb = 0;
                c.addr = (c.addr + 1) & 0x1FFF;
            }
            return v;
        }
        case 3: return c.command;
        case 4: return 0x18;  // write FIFO empty
        case 6:
            // CMAP0: board revision and 8-bit flag; CMAP1: monitor sense.
            if (which == 0) return opts_.xl24 ? 0x02 : 0xA1;
            return static_cast<u8>((opts_.monitor_id << 4) | (opts_.xl24 ? 0x02 : 0x01));
        case 7: return c.crs7;
        default: return 0;
    }
}

void Newport::ramdac_write(unsigned crs, u8 b) {
    picture_changed();
    switch (crs) {
        case 0:
            ramdac_.addr = b;
            ramdac_.rgb = 0;
            break;
        case 1:
            ramdac_.lut[ramdac_.rgb][ramdac_.addr] = b;
            if (++ramdac_.rgb == 3) {
                ramdac_.rgb = 0;
                ++ramdac_.addr;
            }
            break;
        default: ramdac_.regs[crs] = b; break;
    }
}

u8 Newport::ramdac_read(unsigned crs) {
    switch (crs) {
        case 0: return ramdac_.addr;
        case 2: return 0xA0;  // revision A
        default: return ramdac_.regs[crs];
    }
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

void Newport::set_vrint(bool level) {
    if (vrint_ == level) return;
    vrint_ = level;
    if (retrace_cb_) retrace_cb_(level);
}

void Newport::retrace() {
    ++frames_;
    if (vc2_.reg[kVc2Control] & kDcrVintr) set_vrint(true);
    retrace_timer_.schedule_in(static_cast<u64>(cpu_hz_ / 60));
}

bool Newport::screen_size(unsigned& width, unsigned& height) const {
    width = height = 0;
    unsigned vep = vc2_.reg[kVc2VideoEntry];
    for (int entries = 0; entries < 1024; ++entries) {
        const unsigned ptr0 = vc2_.ram[vep++ & 0x7FFF];
        const unsigned lines = vc2_.ram[vep++ & 0x7FFF];
        if (lines == 0) break;
        unsigned ptr = ptr0, cols = 0;
        u16 w;
        int guard = 0;
        do {
            w = vc2_.ram[ptr++ & 0x7FFF];
            if (!(w & 1)) cols += (w >> 7) & 0xFE;
            if (!(w & 0x80)) w = vc2_.ram[ptr++ & 0x7FFF];
        } while (!(w & 0x8000) && ++guard < 256);
        if (cols) {
            width = std::max(width, cols);
            height += lines;
        }
    }
    return width > 0 && height > 0;
}

u8 Newport::did_at(const std::vector<std::pair<int, u8>>& records, int x) const {
    u8 did = 0;
    for (const auto& [rx, d] : records) {
        if (rx > x) break;
        did = d;
    }
    return did;
}

unsigned Newport::cursor_pixel(int x, int y) const {
    const u16 dcr = vc2_.reg[kVc2Control];
    if (!(dcr & kDcrCursorFunc) || !(dcr & kDcrCursorVisible) || (dcr & kDcrCrosshair)) return 0;
    const bool large = dcr & kDcrLargeCursor;
    const int size = large ? 64 : 32;
    const int col = x - (static_cast<int>(vc2_.reg[kVc2CurCursorX]) - (size - 1));
    const int row = y - (static_cast<int>(vc2_.reg[kVc2CursorY]) - (size - 1));
    if (col < 0 || col >= size || row < 0 || row >= size) return 0;
    const unsigned entry = vc2_.reg[kVc2CursorEntry];
    const unsigned bit = 15 - (col & 15);
    if (large) return (vc2_.ram[(entry + row * 4 + (col >> 4)) & 0x7FFF] >> bit) & 1;
    const unsigned word = entry + row * 2 + (col >> 4);
    return ((vc2_.ram[word & 0x7FFF] >> bit) & 1) | (((vc2_.ram[(word + 64) & 0x7FFF] >> bit) & 1) << 1);
}

void Newport::cursor_moving() {
    ++display_version_;
    const int size = (vc2_.reg[kVc2Control] & kDcrLargeCursor) ? 64 : 32;
    const int first = static_cast<int>(vc2_.reg[kVc2CursorY]) - (size - 1);
    if (cursor_rows_.size() >= 64) {  // a lot of movement since the last picture
        full_redraw_ = true;
        cursor_rows_.clear();
    }
    cursor_rows_.emplace_back(first, first + size);
}

const Newport::ModeTable& Newport::mode_table(unsigned did) {
    const u32 mode = xmap_[0].mode[did & 31];
    ModeTable& t = mode_tables_[did & 31];
    if (t.mode == mode) return t;
    t.mode = mode;
    const std::vector<u32>& pal = cmap_[0].palette;
    const unsigned pix_mode = (mode >> 8) & 3, pix_size = (mode >> 10) & 3, buf = mode & 1;
    auto finish = [&](u32 rgb) {
        if (!(mode & 4))  // gamma correction in the RAMDAC unless bypassed
            rgb = (u32{ramdac_.lut[0][(rgb >> 16) & 0xFF]} << 16) | (u32{ramdac_.lut[1][(rgb >> 8) & 0xFF]} << 8) |
                  ramdac_.lut[2][rgb & 0xFF];
        return 0xFF000000u | rgb;
    };
    t.direct = pix_mode != 0 && pix_size == 3;
    if (t.direct) {
        t.lut.clear();
        return t;
    }
    if (pix_mode == 0) {  // colour index
        const u32 ci_msb = (mode & 0xF8) << 5;
        u32 base;
        switch (pix_size) {
            case 0: t.shift = buf ? 4 : 0, t.mask = 0xF, base = ci_msb; break;
            case 2: t.shift = buf ? 12 : 0, t.mask = 0xFFF, base = ci_msb & 0x1000; break;
            default: t.shift = 0, t.mask = 0xFF, base = ci_msb; break;
        }
        t.lut.resize(t.mask + 1);
        for (u32 v = 0; v <= t.mask; ++v) t.lut[v] = finish(pal[(base | v) & 0x1FFF]);
    } else {  // RGB through CMAP map 0..2
        t.shift = pix_size == 0 ? (buf ? 4 : 0) : pix_size == 2 ? (buf ? 12 : 0) : 0;
        t.mask = depth_mask(pix_size);
        t.lut.resize(t.mask + 1);
        const u32 map = 0x1C00 + pix_mode * 0x100;
        for (u32 v = 0; v <= t.mask; ++v) {
            u32 r, g, b;
            unpack(v, pix_size, r, g, b);
            t.lut[v] = finish((pal[map + r] & 0xFF0000) | (pal[map + g] & 0xFF00) | (pal[map + b] & 0xFF));
        }
    }
    return t;
}

u32 Newport::direct_colour(u32 mode, u32 pix) const {
    const std::vector<u32>& pal = cmap_[0].palette;
    u32 r, g, b;
    unpack(pix & 0xFFFFFF, 3, r, g, b);
    const u32 map = 0x1C00 + ((mode >> 8) & 3) * 0x100;
    u32 rgb = (pal[map + r] & 0xFF0000) | (pal[map + g] & 0xFF00) | (pal[map + b] & 0xFF);
    if (!(mode & 4))
        rgb = (u32{ramdac_.lut[0][(rgb >> 16) & 0xFF]} << 16) | (u32{ramdac_.lut[1][(rgb >> 8) & 0xFF]} << 8) |
              ramdac_.lut[2][rgb & 0xFF];
    return 0xFF000000u | rgb;
}

u32 Newport::special_colour(u32 mode, u32 pix, u32 aux, unsigned cursor) const {
    const Xmap& xm = xmap_[0];
    const std::vector<u32>& pal = cmap_[0].palette;
    u32 rgb;
    if (cursor) rgb = pal[((xm.cursor_msb << 5) | cursor) & 0x1FFF];
    else if (const unsigned pup = (aux >> 2) & 3) rgb = pal[((xm.pup_msb << 5) | pup) & 0x1FFF];
    else if (((mode >> 16) & 7) == 2 && ((aux >> 8) & 0xFF)) rgb = pal[((mode >> 11) & 0x1F00) | ((aux >> 8) & 0xFF)];
    else return direct_colour(mode, pix);  // not reached for table modes
    if (!(mode & 4))
        rgb = (u32{ramdac_.lut[0][(rgb >> 16) & 0xFF]} << 16) | (u32{ramdac_.lut[1][(rgb >> 8) & 0xFF]} << 8) |
              ramdac_.lut[2][rgb & 0xFF];
    return 0xFF000000u | rgb;
}

// As render_full(), but converting only the lines that may have changed,
// with a colour table per display mode instead of decoding every pixel.
bool Newport::render(std::vector<u32>& argb, unsigned& width, unsigned& height) {
    const u16 dcr = vc2_.reg[kVc2Control];
    if (!(vc2_.reg[kVc2Config] & 1) || !(dcr & kDcrVTiming) || !screen_size(width, height)) return false;
    constexpr unsigned kPipelineDelay = 2;  // see render_full()
    if (width <= kPipelineDelay) return false;
    width = std::min(width - kPipelineDelay, 2048u);
    height = std::min(height, 2048u);
    const std::size_t total = static_cast<std::size_t>(width) * height;
    if (!(dcr & kDcrDisplay)) {  // blanked
        argb.assign(total, 0xFF000000u);
        full_redraw_ = true;
        return true;
    }
    const bool full = full_redraw_ || argb.size() != total;
    if (full) {
        argb.resize(total);
        for (ModeTable& t : mode_tables_) t.mode = ~0u;  // the maps may have changed
    }

    // The cursor's rows and columns (scan-out coordinates).
    const bool cursor_on = (dcr & kDcrCursorFunc) && (dcr & kDcrCursorVisible) && !(dcr & kDcrCrosshair);
    const int cursor_size = (dcr & kDcrLargeCursor) ? 64 : 32;
    const int cursor_x0 = static_cast<int>(vc2_.reg[kVc2CurCursorX]) - (cursor_size - 1);
    const int cursor_y0 = static_cast<int>(vc2_.reg[kVc2CursorY]) - (cursor_size - 1);

    std::vector<std::pair<int, u8>> records;
    for (unsigned y = 0; y < height; ++y) {
        const unsigned row = (topscan_ + 1 + y) & 0x3FF;
        bool needed = full || row_dirty_[row];
        for (std::size_t i = 0; !needed && i < cursor_rows_.size(); ++i)
            needed = static_cast<int>(y) >= cursor_rows_[i].first && static_cast<int>(y) < cursor_rows_[i].second;
        if (!needed) continue;
        records.clear();
        if (dcr & kDcrDid) {
            unsigned p = vc2_.ram[(vc2_.reg[kVc2DidEntry] + y) & 0x7FFF];
            for (int i = 0; i < 1024; ++i) {
                const u16 r = vc2_.ram[p++ & 0x7FFF];
                const int rx = r >> 5;
                if (rx == 0x7FF) break;
                records.emplace_back(rx, static_cast<u8>(r & 31));
            }
        }
        const bool cursor_row = cursor_on && static_cast<int>(y) >= cursor_y0 && static_cast<int>(y) < cursor_y0 + cursor_size;
        const u32* const pix_row = &rgbci_[static_cast<std::size_t>(row) * kPitch];
        const u32* const aux_row = &cidaux_[static_cast<std::size_t>(row) * kPitch];
        u32* const out = &argb[static_cast<std::size_t>(y) * width];
        std::size_t rec = 0;
        u8 did = 0;
        unsigned x = kPipelineDelay;  // scan-out position
        const unsigned end = width + kPipelineDelay;
        while (x < end) {
            while (rec < records.size() && records[rec].first <= static_cast<int>(x)) did = records[rec++].second;
            // A run of one display mode: up to the next DID change.
            unsigned stop = end;
            if (rec < records.size()) stop = std::min<unsigned>(stop, static_cast<unsigned>(std::max(records[rec].first, 0)));
            if (stop <= x) stop = x + 1;
            const ModeTable& t = mode_table(did);
            const u32 mode = t.mode;
            const u32 special = ((mode >> 16) & 7) == 2 ? 0xFF0Cu : 0x0Cu;  // aux bits that override: PUP, overlay
            for (; x < stop; ++x) {
                const bool inside = x < static_cast<unsigned>(kPitch);
                const u32 pix = inside ? pix_row[x] : 0;
                const u32 aux = inside ? aux_row[x] : 0;
                const unsigned cp = cursor_row && static_cast<int>(x) >= cursor_x0 && static_cast<int>(x) < cursor_x0 + cursor_size
                                        ? cursor_pixel(static_cast<int>(x), static_cast<int>(y))
                                        : 0;
                u32& o = out[x - kPipelineDelay];
                if (cp || (aux & special)) o = special_colour(mode, pix, aux, cp);
                else if (t.direct) o = direct_colour(mode, pix);
                else o = t.lut[(pix >> t.shift) & t.mask];
            }
        }
    }
    std::fill(row_dirty_.begin(), row_dirty_.end(), u8{0});
    cursor_rows_.clear();
    full_redraw_ = false;
    return true;
}

bool Newport::render_full(std::vector<u32>& argb, unsigned& width, unsigned& height) const {
    const u16 dcr = vc2_.reg[kVc2Control];
    if (!(vc2_.reg[kVc2Config] & 1) || !(dcr & kDcrVTiming) || !screen_size(width, height)) return false;
    // The first pixel clocks of each active line fill the display pipeline;
    // software allows for it by placing the screen that far into the
    // framebuffer (XYWIN x = 0x1002), and the timing tables count them.
    constexpr unsigned kPipelineDelay = 2;
    if (width <= kPipelineDelay) return false;
    width = std::min(width - kPipelineDelay, 2048u);
    height = std::min(height, 2048u);
    argb.assign(static_cast<std::size_t>(width) * height, 0xFF000000u);
    if (!(dcr & kDcrDisplay)) return true;  // blanked

    const Xmap& xm = xmap_[0];
    const std::vector<u32>& pal = cmap_[0].palette;
    std::vector<std::pair<int, u8>> records;
    for (unsigned y = 0; y < height; ++y) {
        records.clear();
        if (dcr & kDcrDid) {
            unsigned p = vc2_.ram[(vc2_.reg[kVc2DidEntry] + y) & 0x7FFF];
            for (int i = 0; i < 1024; ++i) {
                const u16 r = vc2_.ram[p++ & 0x7FFF];
                const int rx = r >> 5;
                if (rx == 0x7FF) break;
                records.emplace_back(rx, static_cast<u8>(r & 31));
            }
        }
        const unsigned row = (topscan_ + 1 + y) & 0x3FF;
        u32* out = &argb[static_cast<std::size_t>(y) * width];
        std::size_t rec = 0;
        u8 did = 0;
        for (unsigned ox = 0; ox < width; ++ox) {
            const unsigned x = ox + kPipelineDelay;  // scan-out position
            while (rec < records.size() && records[rec].first <= static_cast<int>(x)) did = records[rec++].second;
            const u32 mode = xm.mode[did];
            const std::size_t idx = static_cast<std::size_t>(row) * kPitch + x;
            const u32 pix = x < static_cast<unsigned>(kPitch) && row < static_cast<unsigned>(kRows) ? rgbci_[idx] : 0;
            const u32 aux = x < static_cast<unsigned>(kPitch) && row < static_cast<unsigned>(kRows) ? cidaux_[idx] : 0;
            u32 rgb;
            if (const unsigned cp = cursor_pixel(static_cast<int>(x), static_cast<int>(y))) {
                rgb = pal[((xm.cursor_msb << 5) | cp) & 0x1FFF];
            } else if (const unsigned pup = (aux >> 2) & 3) {
                rgb = pal[((xm.pup_msb << 5) | pup) & 0x1FFF];
            } else if (((mode >> 16) & 7) == 2 && ((aux >> 8) & 0xFF)) {  // 8-bit overlay
                rgb = pal[((mode >> 11) & 0x1F00) | ((aux >> 8) & 0xFF)];
            } else {
                const unsigned pix_mode = (mode >> 8) & 3, pix_size = (mode >> 10) & 3, buf = mode & 1;
                if (pix_mode == 0) {  // colour index
                    const u32 ci_msb = (mode & 0xF8) << 5;
                    u32 ci;
                    switch (pix_size) {
                        case 0: ci = ci_msb | ((pix >> (buf ? 4 : 0)) & 0xF); break;
                        case 2: ci = (ci_msb & 0x1000) | ((pix >> (buf ? 12 : 0)) & 0xFFF); break;
                        default: ci = ci_msb | (pix & 0xFF); break;
                    }
                    rgb = pal[ci & 0x1FFF];
                } else {  // RGB through CMAP map 0..2
                    u32 v = pix;
                    if (pix_size == 0) v >>= buf ? 4 : 0;
                    else if (pix_size == 2) v >>= buf ? 12 : 0;
                    u32 r, g, b;
                    unpack(v & depth_mask(pix_size), pix_size, r, g, b);
                    const u32 base = 0x1C00 + pix_mode * 0x100;
                    rgb = (pal[base + r] & 0xFF0000) | (pal[base + g] & 0xFF00) | (pal[base + b] & 0xFF);
                }
            }
            if (!(mode & 4)) {  // gamma correction in the RAMDAC unless bypassed
                rgb = (u32{ramdac_.lut[0][(rgb >> 16) & 0xFF]} << 16) | (u32{ramdac_.lut[1][(rgb >> 8) & 0xFF]} << 8) |
                      ramdac_.lut[2][rgb & 0xFF];
            }
            out[ox] = 0xFF000000u | rgb;
        }
    }
    return true;
}

}  // namespace sg::ip22
