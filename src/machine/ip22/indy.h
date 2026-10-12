// SPDX-License-Identifier: GPL-2.0-or-later
// SGI Indy (IP22) machine: owns the bus, memory, CPU and devices.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/scheduler.h"
#include "cpu/r4000/r4000.h"
#include "mem/bus.h"

namespace sg::ip22 {

struct IndyConfig {
    unsigned ram_mb = 64;  // 16..256, multiple of 16
};

class Indy {
public:
    explicit Indy(const IndyConfig& config);
    ~Indy();

    // Loads a raw big-endian PROM image (as dumped from the machine).
    bool load_prom(const std::string& path, std::string& error);

    void reset();
    // Runs the CPU for up to `cycles`, dispatching device events on time.
    u64 run(u64 cycles);

    mips::R4000& cpu() { return cpu_; }
    Bus& bus() { return bus_; }
    Scheduler& scheduler() { return scheduler_; }

private:
    IndyConfig config_;
    Bus bus_;
    Scheduler scheduler_;
    std::unique_ptr<u8[]> ram_;
    std::unique_ptr<u8[]> prom_;
    std::vector<std::unique_ptr<Device>> devices_;
    mips::R4000 cpu_;
};

}  // namespace sg::ip22
