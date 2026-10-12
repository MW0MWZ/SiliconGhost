// SPDX-License-Identifier: GPL-2.0-or-later
// Placeholder for hardware that is not modelled yet. Registers behave as plain
// storage (reads return the last value written, else a default) and every
// access is logged, which is how the PROM tells us what to implement next.
#pragma once

#include <string>
#include <unordered_map>

#include "mem/bus.h"

namespace sg {

class StubDevice final : public Device {
public:
    StubDevice(std::string name, u64 default_value = 0) : name_(std::move(name)), default_(default_value) {}

    const char* name() const override { return name_.c_str(); }
    bool read(u32 offset, unsigned size, u64& value) override;
    bool write(u32 offset, unsigned size, u64 value) override;

    // Preload a register (e.g. an ID register the PROM checks).
    void preset(u32 offset, u64 value) { regs_[offset] = value; }

private:
    std::string name_;
    u64 default_;
    std::unordered_map<u32, u64> regs_;
};

}  // namespace sg
