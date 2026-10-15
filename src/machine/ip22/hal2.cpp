// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "machine/ip22/hal2.h"

#include <algorithm>

#include <cmath>

namespace sg::ip22 {

namespace {

// Control window (PBUS PIO channel 0), word registers
constexpr u32 kIsr = 0x10, kRev = 0x20, kIar = 0x30, kIdr0 = 0x40, kIdr3 = 0x70;
constexpr u32 kControlWindowEnd = 0x400;

// ISR bits: the reset lines are written; TSTATUS (indirect access busy) and
// USTATUS read as idle.
constexpr u32 kIsrWritable = 0x1C;  // QUAD_MODE, GLOBAL_RESET_N, CODEC_RESET_N

// IAR: type/number/parameter select a register; bit 7 reads it back, the
// low two bits pick which 16-bit half a read returns.
constexpr u32 kIarRead = 0x80;
constexpr u32 kIarRegMask = 0xFF0C;

// Indirect registers
constexpr u32 kAesRxC = 0x1204, kAesTxC = 0x1304, kDacC1 = 0x1404, kAdcC1 = 0x1504, kDacC2 = 0x1408, kAdcC2 = 0x1508;
// ADC_C2: left/right input gain (4 bits each); bits 9 and 8 select mic rather than line
constexpr u32 kC2LeftGainShift = 4, kC2RightGainShift = 0, kC2GainMask = 0xF;
constexpr u32 kC2LeftMic = 0x200, kC2RightMic = 0x100;
// The host's recording input arrives at line level, already amplified; the
// mic preamp's gain would boost it a second time, so a mic side is taken down
// by this much first.
constexpr double kMicTrimDb = 9.0;
// DAC_C2: mute and right/left output attenuation (5 bits each)
constexpr u32 kC2Mute = 0x400, kC2RightAttShift = 18, kC2LeftAttShift = 23, kC2AttMask = 0x1F;
// Volume window (PBUS PIO channel 2): 8-bit output levels, 0 silent. The PROM
// sets both to its `volume` variable before the chime.
constexpr u32 kVolRight = 0x800, kVolLeft = 0x804;
constexpr u32 kBresC1 = 0x2104, kBresC2 = 0x2108;  // + 0x100 per generator (1-3)
constexpr u32 kDmaPortEn = 0x9104, kDmaEnd = 0x9108;
// DMA_PORT_EN / DMA_END bits per port
constexpr u32 kPortBit[Hal2::kNumPorts] = {0x08, 0x10, 0x04, 0x02};
// C1 fields
constexpr u32 kC1DmaMask = 0x7, kC1ClkShift = 3, kC1ClkMask = 3, kC1DataShift = 8, kC1DataMask = 3;

}  // namespace

void Hal2::reset() {
    isr_ = 0;
    iar_ = 0;
    idr_.fill(0);
    regs_.clear();
    other_.clear();
    if (changed_) changed_();
}

u32 Hal2::indirect(u32 reg) const {
    const auto it = regs_.find(reg);
    return it == regs_.end() ? 0 : it->second;
}

void Hal2::indirect_access(u32 iar) {
    const u32 reg = iar & kIarRegMask;
    if (iar & kIarRead) {
        // A 32-bit register reads back low half in IDR0, high half in IDR1.
        idr_[0] = (indirect(reg) >> (16 * (iar & 1))) & 0xFFFF;
        idr_[1] = (indirect(reg) >> 16) & 0xFFFF;
        return;
    }
    regs_[reg] = (idr_[0] & 0xFFFF) | ((idr_[1] & 0xFFFF) << 16);
    if (changed_) changed_();
}

u32 Hal2::read(u32 offset) {
    if (offset < kControlWindowEnd) {
        switch (offset) {
            case kIsr: return isr_;
            case kRev: return 0x4010;  // as fitted on Indys; bit 15 clear: audio present
            case kIar: return iar_;
            default:
                if (offset >= kIdr0 && offset <= kIdr3 && (offset & 0xF) == 0) return idr_[(offset - kIdr0) >> 4];
                return 0;
        }
    }
    const auto it = other_.find(offset);
    return it == other_.end() ? 0 : it->second;
}

void Hal2::write(u32 offset, u32 value) {
    if (offset < kControlWindowEnd) {
        switch (offset) {
            case kIsr:
                isr_ = value & kIsrWritable;
                if (!(value & 0x08)) {  // global reset
                    regs_.clear();
                    if (changed_) changed_();
                }
                return;
            case kIar:
                iar_ = value & 0xFFFF;
                indirect_access(iar_);
                return;
            default:
                if (offset >= kIdr0 && offset <= kIdr3 && (offset & 0xF) == 0) idr_[(offset - kIdr0) >> 4] = value & 0xFFFF;
                return;
        }
    }
    other_[offset] = value;
}

void Hal2::output_gain(double& left, double& right) const {
    const u32 c2 = indirect(kDacC2);
    if (c2 & kC2Mute) {
        left = right = 0;
        return;
    }
    // The codec's attenuators, then the volume levels (taken as linear).
    auto factor = [](u32 steps) { return std::pow(10.0, -1.5 * steps / 20.0); };
    auto level = [this](u32 reg) {
        const auto it = other_.find(reg);
        return (it == other_.end() ? 0 : it->second & 0xFF) / 255.0;
    };
    left = factor((c2 >> kC2LeftAttShift) & kC2AttMask) * level(kVolLeft);
    right = factor((c2 >> kC2RightAttShift) & kC2AttMask) * level(kVolRight);
}

void Hal2::input_gain(double& left, double& right) const {
    const u32 c2 = indirect(kAdcC2);
    auto factor = [](u32 steps, bool mic) {
        return std::pow(10.0, (1.5 * steps - (mic ? kMicTrimDb : 0.0)) / 20.0);
    };
    left = factor((c2 >> kC2LeftGainShift) & kC2GainMask, c2 & kC2LeftMic);
    right = factor((c2 >> kC2RightGainShift) & kC2GainMask, c2 & kC2RightMic);
}

Hal2::PortConfig Hal2::port(Port p) const {
    static constexpr u32 kC1[kNumPorts] = {kDacC1, kAdcC1, kAesTxC, kAesRxC};
    PortConfig c;
    const u32 c1 = indirect(kC1[p]);
    c.channel = c1 & kC1DmaMask;
    c.input = p == kAdc || p == kAesRx;
    c.little_endian = indirect(kDmaEnd) & kPortBit[p];
    const unsigned data = (c1 >> kC1DataShift) & kC1DataMask;
    c.voices = data == 3 ? 4 : data;
    // Clock 1-3: a Bresenham generator dividing 48 or 44.1 kHz by mod/inc.
    // Clock 0 (and the AES receiver) follows an external source: none here.
    const unsigned clock = (c1 >> kC1ClkShift) & kC1ClkMask;
    if (clock && p != kAesRx) {
        const u32 bres_c1 = indirect(kBresC1 + 0x100 * (clock - 1));
        const u32 bres_c2 = indirect(kBresC2 + 0x100 * (clock - 1));
        const unsigned master = bres_c1 & 3;
        // 16-bit fields where 0 stands for 65536: IRIX runs at the master
        // rate with inc = mod = 0, Linux with inc = mod = 4.
        auto field = [](u32 v) { return (v & 0xFFFF) ? (v & 0xFFFF) : 0x10000u; };
        const u32 inc = field(bres_c2);
        const u32 mod = field(inc - (bres_c2 >> 16) - 1);  // written as inc - mod - 1
        // A generator can only divide its master clock down (inc above mod
        // runs at the master rate); the AES-derived masters (2, 3) follow
        // an input that is not connected, taken as 48 kHz.
        c.rate = (master == 1 ? 44100.0 : 48000.0) * std::min(inc, mod) / mod;
    }
    c.enabled = (isr_ & 0x08) && (indirect(kDmaPortEn) & kPortBit[p]) && c.rate > 0 && c.voices > 0;
    return c;
}

}  // namespace sg::ip22
