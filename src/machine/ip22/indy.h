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

#include "core/host_memory.h"
#include "core/scheduler.h"
#include "devices/test_device.h"
#include "cpu/r4000/r4000.h"
#include "machine/ip22/hpc3.h"
#include "machine/ip22/ioc2.h"
#include "machine/ip22/mc.h"
#include "machine/ip22/gfx_board.h"
#include "mem/bus.h"

namespace sg::ip22 {

// The IP22 boards: the Indy ("Guinness", IP24 PROM) and the Indigo2
// ("Full House"). They share the chipset; the Indigo2 adds an EISA bus, a
// second SCSI channel, a third memory bank and slot interrupt routing, and
// keeps the PROM's settings in a serial EEPROM.
enum class Board { Indy, Indigo2 };

// Whether `mb` MiB of RAM can be fitted: one or two banks (three on the
// Indigo2) of 16, 32, 64 or 128 MiB.
bool ram_size_fits(unsigned mb, Board board = Board::Indy);

struct IndyConfig {
    Board board = Board::Indy;
    unsigned ram_mb = 64;       // 16..256, multiple of 16
    double cpu_hz = 150.0e6;    // pipeline clock
    double sysclk_hz = 50.0e6;  // SysAD bus clock
    mips::Core cpu_core = mips::Core::R4000;
    u32 prid = 0x00000450;      // R4400 revision 5.0
    u32 cpu_config = mips::R4000::kDefaultConfig;  // Config register at reset (cache geometry)
    u32 scache_bytes = 1u << 20;                    // secondary cache; 0 for a PC module
    bool graphics = true;       // a board in the GFX slot
    enum class GraphicsBoard { Xl24, SolidImpact, HighImpact, MaxImpact };
    GraphicsBoard graphics_board = GraphicsBoard::Xl24;  // which (IMPACT: Indigo2 only)
    int graphics_revision = -1;  // the Newport's board revision, 0-7; -1 for the usual one
    std::string disk_image;     // SCSI disk (ID 1), raw 512-byte blocks
    // SCSI CD-ROM (ID 4) discs, raw 512-byte blocks (IRIX media layout). The
    // first is in the drive at power-on; ejecting moves on to the next.
    std::vector<std::string> cdrom_images;
    bool cdrom_drive = true;  // fit the drive even with no disc in it
    // Ethernet address, kept in NVRAM. Unset: keep the NVRAM's own address, or
    // use a default when the NVRAM has none (IRIX requires one).
    std::optional<std::array<u8, 6>> mac;
    // The PROM's console variable, written to NVRAM at power-on: 'g'
    // (graphics) or 'd' (serial port 1); 0 leaves what the NVRAM has. Only
    // an environment the PROM has already set up is changed: a blank or
    // damaged one is left for the PROM to fill with its defaults.
    char console = 0;
    // The bare-metal test device in GIO expansion slot 0 (see TestDevice),
    // and the run configuration it reports.
    bool test_device = false;
    u32 test_run_config = 0;
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
    // Loads a static big-endian MIPS ELF executable into memory and starts
    // the CPU at its entry point instead of the PROM, with the memory banks
    // set up as the PROM would leave them. Call after reset().
    bool load_elf(const std::string& path, std::string& error);
    // Runs the CPU for up to `cycles`, dispatching device events on time.
    u64 run(u64 cycles);
    bool powered_off() const { return powered_off_; }
    // The code a program gave the test device's exit register, or -1.
    int exit_code() const { return exit_code_; }
    // True while software is timing the CPU against the 8254 (the PROM and
    // the kernels calibrate their delay loops at boot by latching counter 2),
    // and for half a second of guest time after its last latch. Each such
    // stretch starts by ending run(), so the CPU's speed can be fixed first.
    bool timing_cpu() const { return cpu_.cycles() < timing_cpu_until_; }

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
    Board board() const { return config_.board; }
    // What the machine keeps when powered off, as one image: the DS1386's
    // memory, after the serial EEPROM (big-endian words) on the Indigo2.
    // load_nvram_image() takes a shorter image as the start of one.
    std::size_t nvram_size() const;
    std::vector<u8> nvram_image();
    void load_nvram_image(const std::vector<u8>& image);
    u64 nvram_changes() const { return hpc3_.rtc().nvram_changes() + hpc3_.eeprom().changes(); }
    Hpc3& hpc3() { return hpc3_; }
    // The host side of the Ethernet port (null: unplugged); it must outlive
    // the machine or be detached first.
    void set_network(net::NetBackend* backend) { hpc3_.set_network(backend); }
    // Disk, Ethernet and the memory controller's DMA as fast as the host allows
    // (see Hpc3::set_fast_io, Mc::set_fast_dma).
    void set_fast_io(bool fast) {
        hpc3_.set_fast_io(fast);
        mc_.set_fast_dma(fast);
    }
    Ioc2& ioc() { return ioc_; }
    // Absent SCSI targets answer within a millisecond (see
    // Wd33c93::set_quick_selection_timeout).
    void set_quick_scsi_scan(bool quick) { hpc3_.set_quick_scsi_scan(quick); }
    Mc& mc() { return mc_; }
    GfxBoard* graphics() { return gfx_.get(); }

private:
    // The PROM's environment: 256 bytes, in the DS1386 from byte 0x40 on the
    // Indy, in the serial EEPROM (big-endian words) on the Indigo2.
    static constexpr unsigned kNvramBase = 0x40;   // NVRAM byte 0 within the DS1386
    static constexpr unsigned kNvramBytes = 256;
    static constexpr unsigned kNvramChecksum = 0;  // over every other byte
    static constexpr unsigned kNvramConsole = 2;   // "g" or "d", 2 bytes
    static constexpr unsigned kNvramEaddr = 250;   // Ethernet address, 6 bytes
    void pump_serial_input();
    u8 nvram_byte(unsigned offset);
    void set_nvram_byte(unsigned offset, u8 value);
    u8 nvram_checksum();
    void apply_nvram_settings();
    void next_disc();

    IndyConfig config_;
    Scheduler scheduler_;
    Bus bus_;
    HostMemory ram_;
    std::unique_ptr<u8[]> prom_;
    mips::R4000 cpu_;
    Ioc2 ioc_;
    Hpc3 hpc3_;
    Mc mc_;
    std::unique_ptr<GfxBoard> gfx_;  // in the graphics slot, if fitted
    Timer serial_timer_;
    std::unique_ptr<ScsiDisk> disk_, cdrom_;
    std::unique_ptr<TestDevice> test_device_;
    // An R4600SC module's secondary cache controller: the processor has no
    // cache interface of its own, so the module answers uncached accesses at
    // physical 0x80000000 | address (a byte store enables the cache, a
    // halfword store disables it, a word store invalidates a line). The
    // cache holds no data here, so they only need to complete.
    class ScacheControl final : public Device {
    public:
        const char* name() const override { return "r4600sc-cache-control"; }
        bool read(u32, unsigned, u64& value) override {
            value = 0;
            return true;
        }
        bool write(u32, unsigned, u64) override { return true; }
    };
    ScacheControl scache_control_;
    // The Indigo2's EISA I/O space with no cards: reads float high (all
    // ones), writes go nowhere.
    class EisaIo final : public Device {
    public:
        const char* name() const override { return "eisa-io"; }
        bool read(u32, unsigned size, u64& value) override {
            value = size == 8 ? ~u64{0} : (u64{1} << (size * 8)) - 1;
            return true;
        }
        bool write(u32, unsigned, u64) override { return true; }
        bool read_is_pure(u32, unsigned) const override { return true; }
    };
    EisaIo eisa_io_;
    int exit_code_ = -1;

    SerialOutput serial_out_;
    std::mutex serial_mutex_;
    std::deque<std::pair<unsigned, u8>> serial_in_;
    std::atomic<bool> serial_pending_{false};  // input queued: run() starts the pump
    std::size_t disc_ = 0;                   // index into config_.cdrom_images
    bool disc_loaded_ = false;
    std::atomic<bool> disc_change_{false};  // host asked for the next disc
    bool reset_pending_ = false;
    int cpu_reset_pending_ = 0;  // MC CPUCTRL0: 1 the processor's reset, 2 a warm reset
    bool powered_off_ = false;
    u64 timing_cpu_until_ = 0;
};

}  // namespace sg::ip22
