// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "machine/ip22/indy.h"

#include "machine/ip22/impact.h"
#include "machine/ip22/newport.h"

#include <algorithm>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <vector>

#include "core/log.h"
#include "machine/ip22/memmap.h"

namespace sg::ip22 {

namespace {

mips::R4000::Options cpu_options(const IndyConfig& c) {
    mips::R4000::Options o;
    o.core = c.cpu_core;
    o.prid = c.prid;
    o.config = c.cpu_config;
    o.scache_bytes = c.scache_bytes;
    // The IP28 keeps the R10000's (possibly speculative) cached accesses out
    // of GIO and the I/O devices: they end in a bus error. The PROM is cacheable.
    if (c.board == Board::Indigo2R10000) {
        o.uncached_only_base = 0x1F000000;
        o.uncached_only_end = 0x1FC00000;
    }
    return o;
}

}  // namespace

// An Indy has two memory banks, an Indigo2 three, each empty or holding
// 16, 32, 64 or 128 MiB (four SIMMs of 4, 8, 16 or 32 MiB), the larger
// first; the PROM sizes them. The R10000 Indigo2's banks hold 64, 128,
// 256 or 512 MiB (SIMMs of 16 to 128 MiB), 128 MiB SIMMs in two banks at
// most, and 1 GiB in all.
bool ram_size_fits(unsigned mb, Board board) {
    const bool large = board == Board::Indigo2R10000;
    const unsigned sizes[] = {0u, large ? 64u : 16u, large ? 128u : 32u, large ? 256u : 64u, large ? 512u : 128u};
    for (unsigned a : sizes)
        for (unsigned b : sizes)
            for (unsigned c : sizes)
                if (a && b <= a && c <= b && (full_house(board) || c == 0) && a + b + c == mb &&
                    (!large || (c < 512 && mb <= 1024)))
                    return true;
    return false;
}

namespace {

u64 checked_ram_bytes(const IndyConfig& c) {
    if (c.board == Board::Indigo2 && c.cpu_core != mips::Core::R4000)
        throw std::invalid_argument("the Indigo2 takes R4000 and R4400 processor modules");
    if ((c.board == Board::Indigo2R10000) != (c.cpu_core == mips::Core::R10000))
        throw std::invalid_argument("the R10000 fits the R10000 Indigo2 (IP28) only, and that takes nothing else");
    if (!ram_size_fits(c.ram_mb, c.board)) {
        if (c.board == Board::Indigo2R10000)
            throw std::invalid_argument("R10000 Indigo2 RAM must be one to three banks of 64, 128, 256 or 512 MiB (512 in two at most), 64-1024 MiB");
        if (full_house(c.board))
            throw std::invalid_argument("Indigo2 RAM must be one to three banks of 16, 32, 64 or 128 MiB (16-384 MiB)");
        throw std::invalid_argument("Indy RAM must be one or two banks of 16, 32, 64 or 128 MiB (16-256 MiB, not 112, 176, 208, 224 or 240)");
    }
    return u64{c.ram_mb} << 20;
}

constexpr u64 kTestDeviceSize = 0x10000;

// Serial characters take a fixed, short time to send; faster than a real
// 9600 baud line so console output is not a bottleneck.
constexpr u64 kSccCharCycles = 2000;

}  // namespace

// Control commands (bits 18:16): 0 fast (normal) mode, 1 slow mode, 2
// write the low bits to the chip, 3 clear interrupts, 5 / 6 enable /
// disable ECC error generation (OpenBSD imcreg.h).
bool Indy::EccBoard::write(u32 offset, unsigned, u64 value) {
    SG_LOG(Debug, "ecc", "control %03x <- %llx", offset, static_cast<unsigned long long>(value));
    if (offset != 0) return true;
    switch ((value >> 16) & 7) {
        case 0: indy_.cpu_.set_uncached_store_check(true); break;
        case 1: indy_.cpu_.set_uncached_store_check(false); break;
        default: break;
    }
    return true;
}

Indy::Indy(const IndyConfig& config)
    : config_(config),
      ram_(checked_ram_bytes(config)),
      prom_(std::make_unique<u8[]>(kPromMaxSize)),
      cpu_(bus_, cpu_options(config)),
      ioc_(scheduler_, config.cpu_hz, kSccCharCycles, full_house(config.board)),
      hpc3_(bus_, ioc_, scheduler_, config.cpu_hz, full_house(config.board)),
      mc_(bus_, scheduler_, ram_.get(), checked_ram_bytes(config), config.cpu_hz, config.sysclk_hz,
          full_house(config.board), config.board == Board::Indigo2R10000),
      serial_timer_(scheduler_, [this] { pump_serial_input(); }) {
    std::memset(prom_.get(), 0xFF, kPromMaxSize);
    scheduler_.set_clock([this] { return cpu_.cycles(); });

    // Main memory appears only once the PROM programs the MC bank registers.
    bus_.map_ram(kProm, kPromMaxSize, prom_.get(), /*writable=*/false);
    bus_.map_device(kMc, kMcSize, &mc_);
    bus_.map_device(kHpc3, kHpc3Size, &hpc3_);
    if (config_.graphics) {
        using G = IndyConfig::GraphicsBoard;
        if (config_.graphics_board == G::Xl24) {
            gfx_ = std::make_unique<Newport>(scheduler_, config.cpu_hz, Newport::Options{0xA, config.graphics_revision});
        } else {
            if (!full_house(config_.board)) throw std::invalid_argument("IMPACT graphics fit the Indigo2 only");
            auto impact = std::make_unique<Impact>(scheduler_, config.cpu_hz);
            impact->set_bus(&bus_);  // its DMA engine is a GIO bus master
            gfx_ = std::move(impact);
        }
        bus_.map_device(kGioGfx, kGioGfxSize, &gfx_->slot_device());
    }
    if (full_house(config_.board)) bus_.map_device(kEisaIo, kEisaIoSize, &eisa_io_);
    if (config_.board == Board::Indigo2R10000) {
        ioc_.set_sysid(kSysIdIp28);
        mc_.set_bank_device(kEccBoard, kEccBoardSize, &ecc_board_);
        cpu_.uncached_store_hook = [this](u64 paddr) { mc_.report_fast_uncached_write(paddr); };
    }
    // An R4600 or R5000 module with a secondary cache the processor does not
    // report: the PROM reads its size from the module's EEPROM.
    if (config_.cpu_core != mips::Core::R4000 && config_.scache_bytes) {
        mc_.cpu_eeprom()[Mc::kCpuEepromCacheSize] = static_cast<u16>(config_.scache_bytes >> 12);
        if (config_.cpu_core == mips::Core::R4600) bus_.map_device(kR4600ScControl, kR4600ScControlSize, &scache_control_);
    }
    if (config_.test_device) {
        TestDevice::Hooks hooks;
        hooks.putc = [this](u8 c) {
            if (serial_out_) serial_out_(1, c);
        };
        hooks.exit = [this](u32 code) {
            exit_code_ = static_cast<int>(code & 0xFF);
            powered_off_ = true;
            cpu_.request_stop();
        };
        hooks.instructions = [this] { return cpu_.instructions(); };
        test_device_ = std::make_unique<TestDevice>(std::move(hooks), config_.test_run_config);
        bus_.map_device(kGioExp0, kTestDeviceSize, test_device_.get());
    }
    // Uncached CPU reads of GIO devices (the graphics and expansion slots,
    // the HPC3 and its peripherals) wait for the GIO64 bus: the MC moves
    // PIO reads at 13 MB/s (MC specification), about 300 ns a word. Polling
    // loops sized for the real machine depend on it. (Writes are posted.)
    {
        const u16 stall = static_cast<u16>(4.0 / 13.0e6 * config_.cpu_hz + 0.5);
        bus_.set_read_stall(kGioGfx, 0xA00000, stall);
        bus_.set_read_stall(kHpc3, kHpc3Size, stall);
        if (test_device_) bus_.set_read_stall(kGioExp0, kTestDeviceSize, 0);  // not real hardware
    }
    bus_.set_error_handler([this](u64 paddr, unsigned size, bool is_write) {
        mc_.report_bus_error(paddr, size, is_write);
    });

    // Interrupt routing: INT3 LOCAL0/LOCAL1 and 8254 timers 0/1 drive
    // IP2..IP5; the MC bus error interrupt drives IP6.
    ioc_.set_cpu_line_callback([this](unsigned line, bool level) { cpu_.set_int_line(line, level); });
    mc_.set_irq_callback([this](bool level) {
        ioc_.set_mc_bus_error(level);
        cpu_.set_int_line(4, level);
    });
    mc_.set_dma_irq_callback([this](bool level) { ioc_.set_local0(Ioc2::kL0McDma, level); });
    // The graphics board's interrupt (the Newport's retrace): on the Indy
    // its own interrupt line; on the Indigo2 the graphics slot's GIO
    // interrupt 1 (the Newport driver reads the retrace from the REX3's
    // status there).
    if (gfx_ && config_.graphics_board != IndyConfig::GraphicsBoard::Xl24)  // IMPACT: its three GIO lines
        gfx_->set_irq_callback([this](unsigned line, bool level) { ioc_.set_gfx_slot_irq(line, level); });
    else if (gfx_ && full_house(config_.board))
        gfx_->set_irq_callback([this](unsigned, bool level) { ioc_.set_gfx_slot_irq(1, level); });
    else if (gfx_)
        gfx_->set_irq_callback([this](unsigned, bool level) { ioc_.set_gio_slot_irq(0, 2, level); });
    mc_.set_reset_callback([this] {
        reset_pending_ = true;
        cpu_.request_stop();
    });
    mc_.set_cpu_reset_callback([this](bool warm) {
        cpu_reset_pending_ = warm ? 2 : 1;
        cpu_.request_stop();
    });
    bus_.set_rom_write_handler([this](u64 paddr) {
        if (paddr >= kProm && paddr < kProm + kPromMaxSize) mc_.prom_written(static_cast<u32>(paddr));
    });
    ioc_.pit().set_latch_callback([this] {
        if (!timing_cpu()) cpu_.request_stop();
        timing_cpu_until_ = cpu_.cycles() + static_cast<u64>(config_.cpu_hz / 20);
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
    a.pixels = gfx_ ? gfx_->pixels_drawn() : 0;
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

u8 Indy::nvram_byte(unsigned offset) {
    if (full_house(config_.board)) {
        const u16 w = hpc3_.eeprom().words()[offset / 2];
        return static_cast<u8>((offset & 1) ? w : w >> 8);
    }
    return rtc().memory()[kNvramBase + offset];
}

void Indy::set_nvram_byte(unsigned offset, u8 value) {
    if (full_house(config_.board)) {
        u16& w = hpc3_.eeprom().words()[offset / 2];
        w = (offset & 1) ? static_cast<u16>((w & 0xFF00) | value) : static_cast<u16>((w & 0x00FF) | value << 8);
    } else {
        rtc().memory()[kNvramBase + offset] = value;
    }
}

// The PROM's check on its environment: from 0xA5, each 16-bit word XORed in
// (both bytes, but for the checksum byte itself), then rotated left by one.
u8 Indy::nvram_checksum() {
    u8 sum = 0xA5;
    for (unsigned i = 0; i < kNvramBytes; i += 2) {
        sum ^= nvram_byte(i + 1);
        if (i != kNvramChecksum) sum ^= nvram_byte(i);
        sum = static_cast<u8>(sum << 1 | sum >> 7);
    }
    return sum;
}

// The factory programs each machine's Ethernet address into NVRAM bytes
// 250-255. The checksum covers them, and the console variable, so it is
// brought up to date when the PROM's environment was valid to begin with.
void Indy::apply_nvram_settings() {
    static constexpr std::array<u8, 6> kDefaultMac = {0x08, 0x00, 0x69, 0x0C, 0x5E, 0x01};  // SGI OUI
    const bool valid = nvram_checksum() == nvram_byte(kNvramChecksum);
    std::array<u8, 6> nv{};
    for (unsigned i = 0; i < 6; ++i) nv[i] = nvram_byte(kNvramEaddr + i);
    const bool blank = std::all_of(nv.begin(), nv.end(), [](u8 b) { return b == 0x00; }) ||
                       std::all_of(nv.begin(), nv.end(), [](u8 b) { return b == 0xFF; });
    if (config_.mac) nv = *config_.mac;
    else if (blank) nv = kDefaultMac;
    for (unsigned i = 0; i < 6; ++i) set_nvram_byte(kNvramEaddr + i, nv[i]);
    if (valid && config_.console) {
        set_nvram_byte(kNvramConsole, static_cast<u8>(config_.console));
        set_nvram_byte(kNvramConsole + 1, 0);
    }
    if (valid) set_nvram_byte(kNvramChecksum, nvram_checksum());
}

std::size_t Indy::nvram_size() const {
    const std::size_t rtc_bytes = hpc3_.rtc().memory().size();
    return full_house(config_.board) ? 2 * hpc3_.eeprom().words().size() + rtc_bytes : rtc_bytes;
}

std::vector<u8> Indy::nvram_image() {
    std::vector<u8> image;
    if (full_house(config_.board))
        for (u16 w : hpc3_.eeprom().words()) image.push_back(static_cast<u8>(w >> 8)), image.push_back(static_cast<u8>(w));
    const auto& mem = rtc().memory();
    image.insert(image.end(), mem.begin(), mem.end());
    return image;
}

void Indy::load_nvram_image(const std::vector<u8>& image) {
    std::size_t at = 0;
    if (full_house(config_.board)) {
        for (u16& w : hpc3_.eeprom().words()) {
            if (at + 2 > image.size()) return;
            w = static_cast<u16>(image[at] << 8 | image[at + 1]);
            at += 2;
        }
    }
    auto& mem = rtc().memory();
    for (std::size_t i = 0; i < mem.size() && at < image.size(); ++i) mem[i] = image[at++];
}

void Indy::reset() {
    apply_nvram_settings();
    if (disk_) disk_->reset();
    if (cdrom_) cdrom_->reset();
    mc_.reset();
    ioc_.reset();
    hpc3_.reset();
    if (gfx_) gfx_->reset();
    cpu_.reset();
    cpu_.set_uncached_store_check(false);  // the ECC board starts in slow mode
    for (unsigned line = 0; line < 5; ++line) cpu_.set_int_line(line, false);
    serial_timer_.cancel();  // run() starts it again while input is queued
    reset_pending_ = false;
    cpu_reset_pending_ = 0;
    powered_off_ = false;
}

bool Indy::load_elf(const std::string& path, std::string& error) {
    std::ifstream f(path, std::ios::binary);
    std::vector<u8> elf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (!f && !f.eof()) {
        error = "cannot read " + path;
        return false;
    }
    auto be16 = [&](std::size_t at) { return static_cast<u32>(elf[at] << 8 | elf[at + 1]); };
    auto be32 = [&](std::size_t at) { return be16(at) << 16 | be16(at + 2); };
    if (elf.size() < 52 || std::memcmp(elf.data(), "\x7F" "ELF", 4) != 0 || elf[4] != 1 || elf[5] != 2 ||
        be16(16) != 2 || be16(18) != 8) {
        error = path + ": not a 32-bit big-endian MIPS executable";
        return false;
    }
    mc_.map_banks_as_post();
    const u32 phoff = be32(28), phentsize = be16(42), phnum = be16(44);
    for (u32 i = 0; i < phnum; ++i) {
        const std::size_t ph = phoff + std::size_t{i} * phentsize;
        if (ph + 32 > elf.size()) {
            error = path + ": truncated program headers";
            return false;
        }
        if (be32(ph) != 1) continue;  // PT_LOAD
        const u32 offset = be32(ph + 4), vaddr = be32(ph + 8), filesz = be32(ph + 16), memsz = be32(ph + 20);
        if (vaddr < 0x80000000u || vaddr >= 0xC0000000u || filesz > memsz || offset + u64{filesz} > elf.size()) {
            error = path + ": a segment is not in KSEG0/KSEG1 or lies outside the file";
            return false;
        }
        std::vector<u8> image(memsz, 0);
        std::memcpy(image.data(), elf.data() + offset, filesz);
        if (!bus_.write_block(vaddr & 0x1FFFFFFFu, image.data(), image.size())) {
            error = path + ": a segment does not fit in memory";
            return false;
        }
    }
    cpu_.set_pc(static_cast<u64>(static_cast<s64>(static_cast<s32>(be32(24)))));
    return true;
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
        if (cpu_reset_pending_) {  // the processor alone; the rest of the machine carries on
            if (cpu_reset_pending_ == 2) cpu_.soft_reset();
            else cpu_.reset();
            cpu_reset_pending_ = 0;
            cpu_.clear_stop();
        }
        if (disc_change_.exchange(false, std::memory_order_acq_rel)) next_disc();
        // Host input (serial bytes, Ethernet frames) is taken in by timers
        // that run only while there is some.
        if (serial_pending_.load(std::memory_order_acquire) && !serial_timer_.active()) serial_timer_.schedule_in(1);
        hpc3_.poll_network();
        cpu_.run_until(end, scheduler_.next_event_ptr());
        scheduler_.run_due(cpu_.cycles());
        if (cpu_.stop_requested() && !reset_pending_ && !cpu_reset_pending_) break;
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

}  // namespace sg::ip22
