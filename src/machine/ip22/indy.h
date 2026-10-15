// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
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
#include <vector>

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
    u32 cpu_config = mips::R4000::kDefaultConfig;  // Config register at reset (cache geometry)
    u32 scache_bytes = 1u << 20;                    // secondary cache; 0 for a PC module
    bool graphics = true;       // a Newport board in the GFX slot
    bool xl24 = false;          // the 24-bit board (XL24) rather than the 8-bit XL8
    std::string disk_image;     // SCSI disk (ID 1), raw 512-byte blocks
    // SCSI CD-ROM (ID 4) discs, raw 512-byte blocks (IRIX media layout). The
    // first is in the drive at power-on; ejecting moves on to the next.
    std::vector<std::string> cdrom_images;
    bool cdrom_drive = true;  // fit the drive even with no disc in it
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

    // Thread-safe: put the next CD-ROM disc in the drive (after the last,
    // the first again), as if the operator had swapped it.
    void request_next_disc();

    // The CD-ROM drive as the operator sees it (call between run() calls).
    // The discs given at start-up form a set; insert_disc() adds to it.
    const std::vector<std::string>& discs() const { return config_.cdrom_images; }
    // Index of the disc in the drive, or -1 if it is empty (or there is no drive).
    int disc_in_drive() const;
    bool has_cdrom_drive() const { return cdrom_ != nullptr; }
    // The drive's eject button: false if the guest has locked the disc in.
    bool eject_disc();
    // Ejects even a locked disc (a real drive's emergency eject hole).
    void force_eject_disc();
    bool insert_disc(const std::string& path);
    bool select_disc(std::size_t index);

    // Counters for activity lights and load meters.
    struct Activity {
        u64 disk = 0, cdrom = 0;          // medium reads and writes
        u64 net_tx = 0, net_rx = 0;       // Ethernet frames
        u64 cycles = 0, idle_cycles = 0;  // CPU cycles, and those idle-skipped
        u64 pixels = 0;                   // pixels drawn by the REX3
    };
    Activity activity() const;
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
    void next_disc();

    // The graphics slot as the board decodes it: REX3 at +0xF0000, and the
    // rest of the slot answering (reads as zero) rather than timing out, so
    // probes of the slot base find a board there.
    class GfxSlot final : public Device {
    public:
        explicit GfxSlot(Newport& board) : board_(board) {}
        const char* name() const override { return "gfx-slot"; }
        bool read(u32 offset, unsigned size, u64& value) override;
        bool write(u32 offset, unsigned size, u64 value) override;
        bool read_is_pure(u32 offset, unsigned size) const override;

    private:
        Newport& board_;
    };

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
    GfxSlot gfx_slot_{newport_};
    Timer serial_timer_;
    std::unique_ptr<ScsiDisk> disk_, cdrom_;

    SerialOutput serial_out_;
    std::mutex serial_mutex_;
    std::deque<std::pair<unsigned, u8>> serial_in_;
    std::atomic<bool> serial_pending_{false};  // input queued: run() starts the pump
    std::size_t disc_ = 0;                   // index into config_.cdrom_images
    bool disc_loaded_ = false;
    std::atomic<bool> disc_change_{false};  // host asked for the next disc
    bool reset_pending_ = false;
    bool powered_off_ = false;
};

}  // namespace sg::ip22
