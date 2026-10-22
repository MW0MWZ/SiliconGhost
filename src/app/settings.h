// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Everything that describes a machine and how to run it: filled from the
// command line, or kept in a settings file by the window's Settings dialog.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "core/pacer.h"
#include "core/types.h"
#include "machine/ip22/indy.h"
#include "net/slirp_backend.h"

namespace sg::app {

// The processor modules the emulator can present: the implementation, what
// the PROM and IRIX see in PRId and Config, the secondary cache, and the
// clocks each platform shipped the module at (none: not offered there).
struct CpuModel {
    const char* id;     // settings file / --cpu name
    const char* label;  // for menus
    mips::Core core;
    u32 prid;
    u32 config;
    u32 scache_bytes;
    std::vector<double> indy_mhz, indigo2_mhz, ip28_mhz;  // the first is the default
    const std::vector<double>& mhz(ip22::Board board) const {
        return board == ip22::Board::Indigo2         ? indigo2_mhz
               : board == ip22::Board::Indigo2R10000 ? ip28_mhz
                                                     : indy_mhz;
    }
    bool fits(ip22::Board board) const { return !mhz(board).empty(); }
};
const std::vector<CpuModel>& cpu_models();
const CpuModel* find_cpu_model(const std::string& id);

// The hardware platforms (settings file / --machine name, and menu label).
struct Platform {
    const char* id;
    const char* label;
    ip22::Board board;
};
const std::vector<Platform>& platforms();
const Platform* find_platform(const std::string& id);

struct Settings {
    std::string prom_dir;  // folder of PROM images; empty: see prom_folder()
    std::string nvram, disk;
    std::vector<std::string> cdroms;
    std::string machine = "indy";  // indy | indigo2 | indigo2-r10000
    std::string cpu = "r4400sc";
    double cpu_mhz = 150.0;
    unsigned ram_mb = 64;
    bool graphics = true;  // a graphics board fitted
    std::string graphics_board = "xl24";  // xl24, or max-impact (Indigo2)
    int graphics_rev = -1;  // its board revision, 0-7; -1 for the usual one
    // Where the PROM puts its console, written to NVRAM at power-on (see
    // IndyConfig::console): "graphics", "serial", or empty to leave it.
    std::string console;
    std::optional<std::array<u8, 6>> mac;
    std::string net = "user";  // user | tap:NAME | none
    std::vector<net::PortForward> forwards;
    Pacer::Mode timing = Pacer::Mode::Realtime;
    // Realtime timing, as multiples of the real CPU: its speed while the
    // guest times it (and its delay loops' from then on), and a cap on it
    // otherwise (0: none).
    bool idle_skip = true, block_cache = true, jit = true;
    // An absent SCSI target's selection times out within a millisecond
    // instead of its programmed period, shortening the PROM's bus scan: with
    // Realtime or Cycle timing, and with Max timing (or turbo).
    bool quick_scsi_scan = true;
    bool audio = true, audio_in = false;
    bool mouse_auto_capture = true;  // the window captures the mouse when the pointer moves onto the screen
    // The wheel, for a system whose mouse driver has none (IRIX), as keys:
    // "off" (none), "arrows" (Up and Down) or "pages" (Page Up and Down).
    std::string wheel_keys = "off";
    std::string log = "warn";
    // Command line only, never saved: a bare-metal program to run instead of
    // the PROM, and the test device it can talk to the host through.
    std::string load_elf;
    bool test_device = false;
    u32 test_run_config = 0;

    ip22::IndyConfig indy_config() const;
};

// What the window offers on first start, with no settings file yet: the
// best-equipped Indy (256 MiB, the 24-bit board, a 200 MHz CPU).
Settings first_run_settings();

ip22::Board board_of(const Settings& s);
// The memory sizes (MiB) the platform and processor allow, smallest first.
// (No IP22 processor module limits memory; the platform's banks do.)
std::vector<unsigned> ram_sizes(ip22::Board board, const CpuModel* cpu);
// After the platform or processor changed: a processor the platform takes,
// a clock that module shipped at and a memory size that fits, each kept if
// it already was valid (the clock also if it is not a shipped one but the
// processor is unchanged, so a deliberately faster clock survives).
void make_consistent(Settings& s, bool cpu_changed);
// Whether the platform takes the processor, the processor its clock and the
// platform the memory size; false with a message saying what would fit. A
// clock no such machine shipped with is refused unless `any_clock`.
bool check_hardware(const Settings& s, bool any_clock, std::string& error);
// What the chosen hardware means for the guest: needs and known limits.
std::vector<std::string> configuration_notes(const Settings& s);

// Text forms shared by the command line and the settings file.
const char* timing_name(Pacer::Mode mode);
const char* graphics_name(const Settings& s);  // the board, or none
bool parse_timing(const std::string& text, Pacer::Mode& mode);
std::string mac_text(const std::array<u8, 6>& mac);
bool parse_mac(const std::string& text, std::array<u8, 6>& mac);
std::string forward_text(const net::PortForward& f);
bool parse_forward(const std::string& text, net::PortForward& f);
void apply_log_level(const std::string& level);

// Sets one option from its settings-file key; false (with `error`) if the
// key is unknown or the value malformed.
bool set_option(Settings& s, const std::string& key, const std::string& value, std::string& error);

// The user's folder for SiliconGhost: the settings, the log, and roms,
// diskImages and nvram folders for machine files: SiliconGhost in the
// user's Documents folder. Empty if the home folder is unknown.
std::string user_data_folder();
// Makes the folder and its subfolders where they are missing.
void make_user_data_folders();

// The settings file: one "key=value" per line, repeated keys for lists.
bool load_settings(const std::string& path, Settings& s, std::string& error);
bool save_settings(const std::string& path, const Settings& s, std::string& error);

}  // namespace sg::app
