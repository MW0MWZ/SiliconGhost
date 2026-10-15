// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "machine/ip22/indy.h"

#include <algorithm>
#include <cstring>
#include <ctime>
#include <fstream>
#include <stdexcept>

#include "core/log.h"
#include "machine/ip22/memmap.h"

namespace sg::ip22 {

namespace {

mips::R4000::Options cpu_options(const IndyConfig& c) {
    mips::R4000::Options o;
    o.prid = c.prid;
    o.config = c.cpu_config;
    o.scache_bytes = c.scache_bytes;
    return o;
}

u64 checked_ram_bytes(const IndyConfig& c) {
    if (c.ram_mb < 16 || c.ram_mb > 256 || c.ram_mb % 16 != 0)
        throw std::invalid_argument("Indy RAM must be 16..256 MiB in steps of 16");
    return u64{c.ram_mb} << 20;
}

// Serial characters take a fixed, short time to send; faster than a real
// 9600 baud line so console output is not a bottleneck.
constexpr u64 kSccCharCycles = 2000;

}  // namespace

Indy::Indy(const IndyConfig& config)
    : config_(config),
      ram_(std::make_unique<u8[]>(checked_ram_bytes(config))),
      prom_(std::make_unique<u8[]>(kPromMaxSize)),
      cpu_(bus_, cpu_options(config)),
      ioc_(scheduler_, config.cpu_hz, kSccCharCycles),
      hpc3_(bus_, ioc_, scheduler_, config.cpu_hz),
      mc_(bus_, scheduler_, ram_.get(), checked_ram_bytes(config), config.cpu_hz, config.sysclk_hz),
      newport_(scheduler_, config.cpu_hz, Newport::Options{config.xl24}),
      serial_timer_(scheduler_, [this] { pump_serial_input(); }) {
    std::memset(ram_.get(), 0, checked_ram_bytes(config));
    std::memset(prom_.get(), 0xFF, kPromMaxSize);
    scheduler_.set_clock([this] { return cpu_.cycles(); });

    // Main memory appears only once the PROM programs the MC bank registers.
    bus_.map_ram(kProm, kPromMaxSize, prom_.get(), /*writable=*/false);
    bus_.map_device(kMc, kMcSize, &mc_);
    bus_.map_device(kHpc3, kHpc3Size, &hpc3_);
    if (config_.graphics) bus_.map_device(kGioGfx, kGioGfxSize, &gfx_slot_);
    bus_.set_error_handler([this](u64 paddr, unsigned size, bool is_write) {
        mc_.report_bus_error(paddr, size, is_write);
    });

    // Interrupt routing: INT3 LOCAL0/LOCAL1 and 8254 timers 0/1 drive
    // IP2..IP5; the MC bus error interrupt drives IP6.
    ioc_.set_cpu_line_callback([this](unsigned line, bool level) { cpu_.set_int_line(line, level); });
    mc_.set_irq_callback([this](bool level) { cpu_.set_int_line(4, level); });
    mc_.set_dma_irq_callback([this](bool level) { ioc_.set_local0(Ioc2::kL0McDma, level); });
    newport_.set_retrace_callback([this](bool level) { ioc_.set_local1(Ioc2::kL1Retrace, level); });
    mc_.set_reset_callback([this] {
        reset_pending_ = true;
        cpu_.request_stop();
    });
    ioc_.set_power_off_callback([this] {
        powered_off_ = true;
        cpu_.request_stop();
    });
    // SCSI devices on the built-in bus: disk at ID 1, CD-ROM at ID 4 (SGI's usual IDs).
    auto attach = [this](std::unique_ptr<ScsiDisk>& slot, ScsiDisk::Kind kind, const std::string& path, unsigned id) {
        if (path.empty()) return;
        slot = std::make_unique<ScsiDisk>(kind, path);
        if (!slot->ok()) throw std::runtime_error("cannot open " + path);
        hpc3_.scsi0().attach(id, slot.get());
        SG_LOG(Info, "indy", "SCSI ID %u: %s (%llu blocks)", id, path.c_str(),
               static_cast<unsigned long long>(slot->blocks()));
    };
    attach(disk_, ScsiDisk::Kind::Disk, config_.disk_image, 1);
    if (!config_.cdrom_images.empty()) {
        for (const auto& path : config_.cdrom_images)
            if (!std::ifstream(path, std::ios::binary)) throw std::runtime_error("cannot open " + path);
        attach(cdrom_, ScsiDisk::Kind::CdRom, config_.cdrom_images.front(), 4);
        disc_loaded_ = true;
    } else if (config_.cdrom_drive) {
        cdrom_ = std::make_unique<ScsiDisk>(ScsiDisk::Kind::CdRom, std::string());  // empty drive
        hpc3_.scsi0().attach(4, cdrom_.get());
    }
    if (cdrom_) cdrom_->set_eject_callback([this] { next_disc(); });

    // The RTC starts at the host's wall-clock time and then follows guest time.
    {
        const double wall = static_cast<double>(std::time(nullptr));
        hpc3_.rtc().set_year_base(1940);  // IRIX counts RTC years from 1940 (and keeps UTC)
        hpc3_.rtc().set_time_source([this, wall] { return wall + static_cast<double>(cpu_.cycles()) / config_.cpu_hz; });
    }
    ioc_.scc().set_tx_sink([this](unsigned channel, u8 byte) {
        if (serial_out_) serial_out_(channel, byte);
    });

    reset();
}

Indy::~Indy() = default;

void Indy::request_next_disc() {
    disc_change_.store(true, std::memory_order_release);
    cpu_.request_stop();
}

// Guest eject (or the host's "next disc"): the next disc of the set goes in,
// after the last the first again. With no set the drive stays empty.
void Indy::next_disc() {
    if (!cdrom_) return;
    if (config_.cdrom_images.empty()) {
        cdrom_->eject();
        disc_loaded_ = false;
        return;
    }
    select_disc((disc_ + (disc_loaded_ ? 1 : 0)) % config_.cdrom_images.size());
}

int Indy::disc_in_drive() const {
    return cdrom_ && disc_loaded_ && cdrom_->loaded() ? static_cast<int>(disc_) : -1;
}

bool Indy::eject_disc() {
    if (!cdrom_ || cdrom_->removal_prevented()) return false;
    cdrom_->eject();
    disc_loaded_ = false;
    SG_LOG(Info, "indy", "CD-ROM: ejected");
    return true;
}

void Indy::force_eject_disc() {
    if (!cdrom_) return;
    cdrom_->eject();
    disc_loaded_ = false;
    SG_LOG(Info, "indy", "CD-ROM: ejected (forced)");
}

bool Indy::select_disc(std::size_t index) {
    if (!cdrom_ || index >= config_.cdrom_images.size()) return false;
    disc_ = index;
    const std::string& path = config_.cdrom_images[disc_];
    disc_loaded_ = cdrom_->change_medium(path);
    if (disc_loaded_)
        SG_LOG(Info, "indy", "CD-ROM: %s (%llu blocks)", path.c_str(), static_cast<unsigned long long>(cdrom_->blocks()));
    else
        SG_LOG(Warn, "indy", "CD-ROM: cannot open %s, drive empty", path.c_str());
    return disc_loaded_;
}

bool Indy::insert_disc(const std::string& path) {
    if (!cdrom_) return false;
    auto& set = config_.cdrom_images;
    auto it = std::find(set.begin(), set.end(), path);
    if (it == set.end()) it = set.insert(set.end(), path);
    return select_disc(static_cast<std::size_t>(it - set.begin()));
}

Indy::Activity Indy::activity() const {
    Activity a;
    a.disk = disk_ ? disk_->activity() : 0;
    a.cdrom = cdrom_ ? cdrom_->activity() : 0;
    a.net_tx = hpc3_.enet_tx_frames();
    a.net_rx = hpc3_.enet_rx_frames();
    a.cycles = cpu_.cycles();
    a.idle_cycles = cpu_.idle_skipped_cycles();
    a.pixels = newport_.pixels_drawn();
    return a;
}

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

void Indy::load_prom_image(const u8* data, std::size_t size) {
    std::memset(prom_.get(), 0xFF, kPromMaxSize);
    std::memcpy(prom_.get(), data, size < kPromMaxSize ? size : kPromMaxSize);
}

// The factory programs each Indy's Ethernet address into NVRAM bytes 250-255.
void Indy::apply_ethernet_address() {
    static constexpr std::array<u8, 6> kDefaultMac = {0x08, 0x00, 0x69, 0x0C, 0x5E, 0x01};  // SGI OUI
    u8* nv = &rtc().memory()[kNvramBase + kNvramEaddr];
    const bool blank = std::all_of(nv, nv + 6, [](u8 b) { return b == 0x00; }) ||
                       std::all_of(nv, nv + 6, [](u8 b) { return b == 0xFF; });
    if (config_.mac) std::copy(config_.mac->begin(), config_.mac->end(), nv);
    else if (blank) std::copy(kDefaultMac.begin(), kDefaultMac.end(), nv);
}

void Indy::reset() {
    apply_ethernet_address();
    mc_.reset();
    ioc_.reset();
    hpc3_.reset();
    newport_.reset();
    cpu_.reset();
    for (unsigned line = 0; line < 5; ++line) cpu_.set_int_line(line, false);
    serial_timer_.cancel();  // run() starts it again while input is queued
    reset_pending_ = false;
    powered_off_ = false;
}

u64 Indy::run(u64 cycles) {
    const u64 start = cpu_.cycles();
    const u64 end = (cycles > ~u64{0} - start) ? ~u64{0} : start + cycles;
    cpu_.clear_stop();
    while (cpu_.cycles() < end && !powered_off_) {
        if (reset_pending_) {
            SG_LOG(Info, "indy", "system reset");
            reset();
        }
        if (disc_change_.exchange(false, std::memory_order_acq_rel)) next_disc();
        // Host input (serial bytes, Ethernet frames) is taken in by timers
        // that run only while there is some.
        if (serial_pending_.load(std::memory_order_acquire) && !serial_timer_.active()) serial_timer_.schedule_in(1);
        hpc3_.poll_network();
        cpu_.run_until(end, scheduler_.next_event_ptr());
        scheduler_.run_due(cpu_.cycles());
        if (cpu_.stop_requested() && !reset_pending_) break;
    }
    return cpu_.cycles() - start;
}

void Indy::serial_input(unsigned channel, u8 byte) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    serial_in_.emplace_back(channel, byte);
    serial_pending_.store(true, std::memory_order_release);
}

// Feed queued input at most one byte per channel per millisecond of guest
// time, and only into an enabled receiver with an empty FIFO, so typed-ahead
// and pasted text is not lost. The pump runs only while input is queued.
void Indy::pump_serial_input() {
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        bool fed[2] = {false, false};
        for (auto it = serial_in_.begin(); it != serial_in_.end();) {
            const unsigned ch = it->first & 1;
            if (!fed[ch] && ioc_.scc().rx_enabled(ch) && !ioc_.scc().rx_ready(ch)) {
                ioc_.scc().receive(ch, it->second);
                fed[ch] = true;
                it = serial_in_.erase(it);
            } else {
                fed[ch] = true;  // keep per-channel order
                ++it;
            }
        }
        if (serial_in_.empty()) {
            serial_pending_.store(false, std::memory_order_release);
            return;
        }
    }
    serial_timer_.schedule_in(static_cast<u64>(config_.cpu_hz / 1000));
}

namespace {
constexpr u32 kRex3InSlot = kNewportRex3 - kGioGfx, kRex3WindowSize = 0x10000;
}  // namespace

bool Indy::GfxSlot::read(u32 offset, unsigned size, u64& value) {
    if (offset - kRex3InSlot < kRex3WindowSize) return board_.read(offset - kRex3InSlot, size, value);
    value = 0;
    return true;
}

bool Indy::GfxSlot::write(u32 offset, unsigned size, u64 value) {
    if (offset - kRex3InSlot < kRex3WindowSize) return board_.write(offset - kRex3InSlot, size, value);
    return true;
}

bool Indy::GfxSlot::read_is_pure(u32 offset, unsigned size) const {
    if (offset - kRex3InSlot < kRex3WindowSize) return board_.read_is_pure(offset - kRex3InSlot, size);
    return true;
}

}  // namespace sg::ip22
