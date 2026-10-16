// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "devices/test_device.h"

namespace sg {

u32 TestDevice::read_reg(u32 reg) {
    switch (reg) {
        case kRegSignature: return kSignature;
        case kRegHostNsLo:
            host_ns_latch_ = static_cast<u64>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - epoch_)
                    .count());
            return static_cast<u32>(host_ns_latch_);
        case kRegHostNsHi: return static_cast<u32>(host_ns_latch_ >> 32);
        case kRegIcountLo:
            icount_latch_ = hooks_.instructions ? hooks_.instructions() : 0;
            return static_cast<u32>(icount_latch_);
        case kRegIcountHi: return static_cast<u32>(icount_latch_ >> 32);
        case kRegCaps: return kCapTimebase | kCapRunConfig;
        case kRegRunConfig: return run_config_;
        default: return 0;
    }
}

bool TestDevice::read(u32 offset, unsigned size, u64& value) {
    const u32 off = offset & (kWindow - 1);
    const u32 word = read_reg(off & ~3u);
    if (size >= 4) value = size == 8 ? (u64{word} << 32) | read_reg((off + 4) & (kWindow - 4)) : word;
    else value = (word >> ((4 - size - (off & 3)) * 8)) & ((1u << (size * 8)) - 1);
    return true;
}

bool TestDevice::write(u32 offset, unsigned size, u64 value) {
    const u32 v = static_cast<u32>(size == 8 ? value >> 32 : value);
    switch (offset & (kWindow - 1) & ~3u) {
        case kRegPutc:
            if (hooks_.putc) hooks_.putc(static_cast<u8>(v));
            break;
        case kRegExit:
            if (hooks_.exit) hooks_.exit(v);
            break;
        default: break;
    }
    return true;
}

}  // namespace sg
