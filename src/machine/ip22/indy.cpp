// SPDX-License-Identifier: GPL-2.0-or-later
#include "machine/ip22/indy.h"

#include <cstring>
#include <fstream>
#include <stdexcept>

#include "core/log.h"
#include "devices/stub_device.h"
#include "machine/ip22/memmap.h"

namespace sg::ip22 {

Indy::Indy(const IndyConfig& config) : config_(config), cpu_(bus_) {
    if (config_.ram_mb < 16 || config_.ram_mb > 256 || config_.ram_mb % 16 != 0)
        throw std::invalid_argument("Indy RAM must be 16..256 MiB in steps of 16");

    const u64 ram_size = u64{config_.ram_mb} << 20;
    ram_ = std::make_unique<u8[]>(ram_size);
    std::memset(ram_.get(), 0, ram_size);
    bus_.map_ram(kRamBase, ram_size, ram_.get());
    bus_.map_ram(kLowMemAlias, kLowMemAliasSize, ram_.get());

    prom_ = std::make_unique<u8[]>(kPromMaxSize);
    std::memset(prom_.get(), 0xFF, kPromMaxSize);
    bus_.map_ram(kProm, kPromMaxSize, prom_.get(), /*writable=*/false);

    // Chipset placeholders until real models land (see docs/ARCHITECTURE.md, M2/M3).
    auto add_stub = [this](const char* name, u32 base, u32 size) {
        devices_.push_back(std::make_unique<StubDevice>(name));
        bus_.map_device(base, size, devices_.back().get());
    };
    add_stub("mc", kMc, kMcSize);
    add_stub("hpc3", kHpc3, kHpc3Size);
    add_stub("newport", kNewportRex3, 0x10000);

    scheduler_.set_clock([this] { return cpu_.cycles(); });

    reset();
}

Indy::~Indy() = default;

bool Indy::load_prom(const std::string& path, std::string& error) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        error = "cannot open " + path;
        return false;
    }
    const std::streamsize size = f.tellg();
    if (size <= 0 || size > static_cast<std::streamsize>(kPromMaxSize)) {
        error = "PROM image must be 1..512 KiB";
        return false;
    }
    f.seekg(0);
    std::memset(prom_.get(), 0xFF, kPromMaxSize);
    if (!f.read(reinterpret_cast<char*>(prom_.get()), size)) {
        error = "read error on " + path;
        return false;
    }
    SG_LOG(Info, "indy", "loaded PROM %s (%lld bytes)", path.c_str(), static_cast<long long>(size));
    return true;
}

void Indy::reset() { cpu_.reset(); }

u64 Indy::run(u64 cycles) {
    const u64 start = cpu_.cycles();
    const u64 end = (cycles > ~u64{0} - start) ? ~u64{0} : start + cycles;
    cpu_.clear_stop();
    while (cpu_.cycles() < end && !cpu_.stop_requested()) {
        const u64 next = scheduler_.next_event();
        cpu_.run_until(next < end ? next : end);
        scheduler_.run_due(cpu_.cycles());
    }
    return cpu_.cycles() - start;
}

}  // namespace sg::ip22
