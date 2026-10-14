// SPDX-License-Identifier: GPL-2.0-or-later
// SGI Indy (IP22) machine: owns the bus, memory, CPU and devices.
#pragma once

#include <array>
#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "core/scheduler.h"
#include "cpu/r4000/r4000.h"
#include "machine/ip22/hpc3.h"
#include "machine/ip22/ioc2.h"
#include "machine/ip22/mc.h"
#include "machine/ip22/newport.h"
#include "mem/bus.h"

namespace sg::ip22 {

struct IndyConfig {
    unsigned ram_mb = 64;       // 16..256, multiple of 16
    double cpu_hz = 150.0e6;    // pipeline clock
    double sysclk_hz = 50.0e6;  // SysAD bus clock
    u32 prid = 0x00000450;      // R4400 revision 5.0
    bool graphics = true;       // Newport XL8 in the GFX slot
    std::string disk_image;     // SCSI disk (ID 1), raw 512-byte blocks
    std::string cdrom_image;    // SCSI CD-ROM (ID 4), raw 512-byte blocks (IRIX media layout)
    // Ethernet address, kept in NVRAM. Unset: keep the NVRAM's own address, or
    // use a default when the NVRAM has none (IRIX requires one).
    std::optional<std::array<u8, 6>> mac;
};

class Indy {
public:
    // Serial output from the Z85230: channel 0 = A, 1 = B.
    using SerialOutput = std::function<void(unsigned channel, u8 byte)>;

    explicit Indy(const IndyConfig& config);
    ~Indy();

    // Loads a raw big-endian PROM image (as dumped from the machine).
    bool load_prom(const std::string& path, std::string& error);
    void load_prom_image(const u8* data, std::size_t size);

    void reset();
    // Runs the CPU for up to `cycles`, dispatching device events on time.
    u64 run(u64 cycles);
    bool powered_off() const { return powered_off_; }
    double cycles_per_second() const { return config_.cpu_hz; }

    void set_serial_output(SerialOutput out) { serial_out_ = std::move(out); }
    // Thread-safe: queue a received byte for a serial channel.
    void serial_input(unsigned channel, u8 byte);

    mips::R4000& cpu() { return cpu_; }
    Bus& bus() { return bus_; }
    Scheduler& scheduler() { return scheduler_; }
    Ds1386& rtc() { return hpc3_.rtc(); }
    Hpc3& hpc3() { return hpc3_; }
    // The host side of the Ethernet port (null: unplugged); it must outlive
    // the machine or be detached first.
    void set_network(net::NetBackend* backend) { hpc3_.set_network(backend); }
    Ioc2& ioc() { return ioc_; }
    Mc& mc() { return mc_; }
    Newport* newport() { return config_.graphics ? &newport_ : nullptr; }

private:
    static constexpr unsigned kNvramBase = 0x40;   // NVRAM byte 0 within the DS1386
    static constexpr unsigned kNvramEaddr = 250;   // Ethernet address, 6 bytes
    void pump_serial_input();
    void apply_ethernet_address();

    IndyConfig config_;
    Scheduler scheduler_;
    Bus bus_;
    std::unique_ptr<u8[]> ram_;
    std::unique_ptr<u8[]> prom_;
    mips::R4000 cpu_;
    Ioc2 ioc_;
    Hpc3 hpc3_;
    Mc mc_;
    Newport newport_;
    Timer serial_timer_;
    std::unique_ptr<ScsiDisk> disk_, cdrom_;

    SerialOutput serial_out_;
    std::mutex serial_mutex_;
    std::deque<std::pair<unsigned, u8>> serial_in_;
    std::atomic<bool> serial_pending_{false};  // input queued: run() starts the pump
    bool reset_pending_ = false;
    bool powered_off_ = false;
};

}  // namespace sg::ip22
