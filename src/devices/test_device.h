// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Bare-metal test device: lets a program loaded with --load-elf print to the
// host, read a host clock and the count of instructions retired, and end the
// emulator with an exit code.
//
// It sits in GIO expansion slot 0 (an empty slot on a stock machine). The
// registers are 32-bit, repeat every 0x40 bytes, and a byte read takes its
// lane from the containing word.
#pragma once

#include <chrono>
#include <functional>

#include "mem/bus.h"

namespace sg {

class TestDevice final : public Device {
public:
    static constexpr u32 kSignature = 0x53474854;  // "SGHT"
    enum Reg : u32 {
        kRegSignature = 0x00,
        kRegPutc = 0x04,
        kRegDump = 0x08,       // accepted, nothing is written
        kRegExit = 0x0C,
        kRegHostNsLo = 0x10,   // reading LO latches the 64-bit value, HI reads the latch
        kRegHostNsHi = 0x14,
        kRegIcountLo = 0x18,
        kRegIcountHi = 0x1C,
        kRegCaps = 0x20,
        kRegRunConfig = 0x24,  // groups << 16 | repeats << 12 | time percentage; 0 = everything
    };
    static constexpr u32 kCapTimebase = 1u << 0, kCapRunConfig = 1u << 1;
    static constexpr u32 kWindow = 0x40;

    struct Hooks {
        std::function<void(u8)> putc;
        std::function<void(u32)> exit;
        std::function<u64()> instructions;
    };
    TestDevice(Hooks hooks, u32 run_config) : hooks_(std::move(hooks)), run_config_(run_config) {}

    const char* name() const override { return "test-device"; }
    bool read(u32 offset, unsigned size, u64& value) override;
    bool write(u32 offset, unsigned size, u64 value) override;

private:
    u32 read_reg(u32 reg);

    Hooks hooks_;
    u32 run_config_;
    std::chrono::steady_clock::time_point epoch_ = std::chrono::steady_clock::now();
    u64 host_ns_latch_ = 0, icount_latch_ = 0;
};

}  // namespace sg
