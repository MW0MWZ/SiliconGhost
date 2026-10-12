// SPDX-License-Identifier: GPL-2.0-or-later
#include "devices/stub_device.h"

#include "core/log.h"

namespace sg {

bool StubDevice::read(u32 offset, unsigned size, u64& value) {
    const auto it = regs_.find(offset);
    value = it != regs_.end() ? it->second : default_;
    if (size < 8) value &= (u64{1} << (size * 8)) - 1;
    SG_LOG(Info, name_.c_str(), "read%u  +%06x -> %llx", size * 8, offset, static_cast<unsigned long long>(value));
    return true;
}

bool StubDevice::write(u32 offset, unsigned size, u64 value) {
    regs_[offset] = value;
    SG_LOG(Info, name_.c_str(), "write%u +%06x <- %llx", size * 8, offset, static_cast<unsigned long long>(value));
    return true;
}

}  // namespace sg
