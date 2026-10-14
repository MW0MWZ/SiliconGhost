// SPDX-License-Identifier: GPL-2.0-or-later
// HAL2 audio processor, on HPC3 PBUS PIO channels 0-3 (control and indirect
// registers, AES, volume, synth). It owns the codec (DAC/ADC) and AES ports;
// each port is tied to a PBUS DMA channel and clocked by one of three
// Bresenham rate generators. The DMA itself is HPC3's (see Hpc3); this class
// only says which channel moves data, in which direction and how fast.
//
// References: Linux sound/mips/hal2.[ch].
#pragma once

#include <array>
#include <functional>
#include <unordered_map>

#include "core/types.h"

namespace sg::ip22 {

class Hal2 {
public:
    enum Port : unsigned { kDac, kAdc, kAesTx, kAesRx, kNumPorts };
    struct PortConfig {
        bool enabled = false;      // DMA port enabled and clocked
        unsigned channel = 0;      // PBUS DMA channel
        double rate = 0;           // frames per second
        unsigned voices = 0;       // 1 mono, 2 stereo, 4 quad
        bool input = false;        // device to memory
        bool little_endian = false;
    };

    Hal2() { reset(); }
    void reset();

    // Registers in the PBUS PIO window, `offset` relative to channel 0's.
    u32 read(u32 offset);
    void write(u32 offset, u32 value);

    PortConfig port(Port p) const;
    // The DAC's output level as amplitude factors: DAC_C2 mute and
    // attenuators (1.5 dB a step), and the 8-bit volume levels.
    void output_gain(double& left, double& right) const;
    // Called whenever a port's configuration may have changed.
    void set_change_callback(std::function<void()> cb) { changed_ = std::move(cb); }

private:
    u32 indirect(u32 reg) const;
    void indirect_access(u32 iar);

    u32 isr_ = 0;
    u32 iar_ = 0;
    std::array<u32, 4> idr_{};
    std::unordered_map<u32, u32> regs_;  // indirect registers by type/number/parameter
    std::unordered_map<u32, u32> other_;  // AES, volume and synth windows: storage
    std::function<void()> changed_;
};

}  // namespace sg::ip22
