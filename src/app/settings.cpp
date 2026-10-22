// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "app/settings.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "core/log.h"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shlobj.h>
#endif

namespace sg::app {

const std::vector<CpuModel>& cpu_models() {
    // R4000 and R4400 share the pipeline; the R4000 has 8 KiB primary caches.
    // The Indigo2 shipped with R4000SC and R4400SC modules only.
    using mips::Core;
    using mips::R4000;
    static const std::vector<CpuModel> models = {
        {"r4400sc", "R4400SC (1 MiB secondary cache)", Core::R4000, 0x0450, R4000::kDefaultConfig, 1u << 20,
         {150, 175, 200}, {150, 175, 200, 250}, {}},
        {"r4400pc", "R4400PC (no secondary cache)", Core::R4000, 0x0450, R4000::kDefaultConfig, 0, {100}, {}, {}},
        {"r4000sc", "R4000SC (1 MiB secondary cache)", Core::R4000, 0x0430, 0x10C08263u, 1u << 20, {100}, {100}, {}},
        {"r4000pc", "R4000PC (no secondary cache)", Core::R4000, 0x0430, 0x10C08263u, 0, {100}, {}, {}},
        {"r4600pc", "R4600PC (no secondary cache)", Core::R4600, 0x2020, R4000::kR4600Config, 0, {100, 133}, {}, {}},
        {"r4600sc", "R4600SC (512 KiB secondary cache)", Core::R4600, 0x2020, R4000::kR4600Config, 512u << 10, {133},
         {}, {}},
        {"r5000pc", "R5000PC (no secondary cache)", Core::R5000, 0x2321, R4000::kR5000Config, 0, {150}, {}, {}},
        {"r5000sc", "R5000SC (512 KiB secondary cache)", Core::R5000, 0x2321, R4000::kR5000Config, 512u << 10,
         {180, 150}, {}, {}},
        {"r10000", "R10000 (1 MiB secondary cache)", Core::R10000, 0x0926, R4000::kR10000Config, 1u << 20, {}, {},
         {175, 195}},
    };
    return models;
}

const CpuModel* find_cpu_model(const std::string& id) {
    for (const auto& m : cpu_models())
        if (id == m.id) return &m;
    return nullptr;
}

const std::vector<Platform>& platforms() {
    static const std::vector<Platform> list = {
        {"indy", "Indy (IP24)", ip22::Board::Indy},
        {"indigo2", "Indigo2 (IP22)", ip22::Board::Indigo2},
        {"indigo2-r10000", "Indigo2 R10000 (IP28)", ip22::Board::Indigo2R10000},
    };
    return list;
}

const Platform* find_platform(const std::string& id) {
    for (const auto& p : platforms())
        if (id == p.id) return &p;
    return nullptr;
}

ip22::Board board_of(const Settings& s) {
    const Platform* p = find_platform(s.machine);
    return p ? p->board : ip22::Board::Indy;
}

std::vector<unsigned> ram_sizes(ip22::Board board, const CpuModel* cpu) {
    (void)cpu;
    std::vector<unsigned> sizes;
    for (unsigned mb = 16; mb <= 1024; mb += 16)
        if (ip22::ram_size_fits(mb, board)) sizes.push_back(mb);
    return sizes;
}

void make_consistent(Settings& s, bool cpu_changed) {
    const ip22::Board board = board_of(s);
    const CpuModel* cpu = find_cpu_model(s.cpu);
    if (!cpu || !cpu->fits(board)) {
        cpu = find_cpu_model(board == ip22::Board::Indigo2R10000 ? "r10000" : "r4400sc");
        s.cpu = cpu->id;
        cpu_changed = true;
    }
    const auto& speeds = cpu->mhz(board);
    if (cpu_changed && std::find(speeds.begin(), speeds.end(), s.cpu_mhz) == speeds.end()) s.cpu_mhz = speeds.front();
    const std::vector<unsigned> sizes = ram_sizes(board, cpu);
    if (std::find(sizes.begin(), sizes.end(), s.ram_mb) == sizes.end()) {
        unsigned best = sizes.front();
        for (unsigned mb : sizes)
            if (mb <= s.ram_mb) best = mb;
        s.ram_mb = best;
    }
    // The R10000 Indigo2 takes IMPACT graphics only.
    if (board == ip22::Board::Indigo2R10000 && s.graphics_board == "xl24") s.graphics_board = "max-impact";
}

bool check_hardware(const Settings& s, bool any_clock, std::string& error) {
    const Platform* platform = find_platform(s.machine);
    const CpuModel* cpu = find_cpu_model(s.cpu);
    if (!platform || !cpu) {
        error = "unknown platform or processor";
        return false;
    }
    const char* name = platform->board == ip22::Board::Indigo2         ? "Indigo2"
                       : platform->board == ip22::Board::Indigo2R10000 ? "R10000 Indigo2"
                                                                       : "Indy";
    auto list = [](const auto& items, const char* unit) {
        std::string text;
        for (const auto& i : items) {
            if (!text.empty()) text += ", ";
            char buf[32];
            std::snprintf(buf, sizeof buf, "%g%s", static_cast<double>(i), unit);
            text += buf;
        }
        return text;
    };
    if (!cpu->fits(platform->board)) {
        std::string fits;
        for (const auto& m : cpu_models())
            if (m.fits(platform->board)) fits += std::string(fits.empty() ? "" : ", ") + m.id;
        error = std::string("the ") + name + " does not take the " + cpu->id + " module; it takes " + fits;
        return false;
    }
    const auto& speeds = cpu->mhz(platform->board);
    if (!any_clock && std::find(speeds.begin(), speeds.end(), s.cpu_mhz) == speeds.end()) {
        char buf[64];
        std::snprintf(buf, sizeof buf, "%g MHz", s.cpu_mhz);
        error = std::string("no ") + name + " shipped with the " + cpu->id + " module at " + buf + "; it ran at " +
                list(speeds, " MHz") + " (add --any-clock to run it at " + buf + " anyway)";
        return false;
    }
    const std::vector<unsigned> sizes = ram_sizes(platform->board, cpu);
    if (std::find(sizes.begin(), sizes.end(), s.ram_mb) == sizes.end()) {
        error = std::string("the ") + name + " cannot hold " + std::to_string(s.ram_mb) + " MiB (" +
                (ip22::full_house(platform->board) ? "up to three" : "one or two") +
                (platform->board == ip22::Board::Indigo2R10000 ? " banks of 64, 128 or 256 MiB"
                                                               : " banks of 16, 32, 64 or 128 MiB") +
                ", the largest first); it takes " + list(sizes, "");
        return false;
    }
    if (s.graphics && platform->board == ip22::Board::Indigo2R10000 && s.graphics_board == "xl24") {
        error = "the R10000 Indigo2 takes IMPACT graphics (its PROM and IRIX have no Newport driver)";
        return false;
    }
    if (s.graphics && platform->board == ip22::Board::Indy && s.graphics_board != "xl24") {
        error = "IMPACT graphics fit the Indigo2 only";
        return false;
    }
    return true;
}

std::vector<std::string> configuration_notes(const Settings& s) {
    std::vector<std::string> notes;
    const ip22::Board board = board_of(s);
    const CpuModel* cpu = find_cpu_model(s.cpu);
    char text[160];
    if (board == ip22::Board::Indigo2R10000) {
        notes.push_back("Not finished: the R10000 and the IP28 board are being brought up (PROM first).");
        notes.push_back("Graphics: IMPACT (not finished: no picture yet), or none.");
    } else if (board == ip22::Board::Indigo2) {
        notes.push_back("IRIX 5.3 and 6.5 run (disks installed on an Indy boot unchanged).");
        notes.push_back("NetBSD and OpenBSD do not start with the 5.1 Indigo2 PROM: it loads only ECOFF programs and "
                        "their boot loaders are ELF.");
        notes.push_back("Graphics: Newport XL24, or IMPACT (not finished: no picture yet). The EISA slots are empty.");
    } else {
        notes.push_back("IRIX 5.3 and 6.5, NetBSD and OpenBSD run.");
    }
    if (cpu && cpu->core == mips::Core::R4600 && cpu->scache_bytes)
        notes.push_back("NetBSD does not use the R4600SC's secondary cache (IRIX does).");
    if (cpu) {
        const auto& speeds = cpu->mhz(board);
        if (std::find(speeds.begin(), speeds.end(), s.cpu_mhz) == speeds.end()) {
            std::snprintf(text, sizeof text, "No %s shipped with this module at %.0f MHz.",
                          ip22::full_house(board) ? "Indigo2" : "Indy", s.cpu_mhz);
            notes.push_back(text);
        }
    }
    if (!s.graphics) notes.push_back("Without a graphics board the console is the first serial port.");
    return notes;
}

ip22::IndyConfig Settings::indy_config() const {
    ip22::IndyConfig c;
    c.board = board_of(*this);
    c.ram_mb = ram_mb;
    c.cpu_hz = cpu_mhz * 1.0e6;
    if (const CpuModel* m = find_cpu_model(cpu)) {
        c.cpu_core = m->core;
        c.prid = m->prid;
        c.cpu_config = m->config;
        c.scache_bytes = m->scache_bytes;
    }
    c.graphics = graphics;
    using G = ip22::IndyConfig::GraphicsBoard;
    c.graphics_board = graphics_board == "max-impact" ? G::MaxImpact : G::Xl24;
    c.graphics_revision = graphics_rev;
    c.test_device = test_device;
    c.test_run_config = test_run_config;
    c.disk_image = disk;
    c.cdrom_images = cdroms;
    c.mac = mac;
    c.console = console == "graphics" ? 'g' : console == "serial" ? 'd' : 0;
    return c;
}

Settings first_run_settings() {
    Settings s;
    s.ram_mb = 256;
    s.cpu_mhz = 200;  // the fastest Indy R4400SC
    return s;
}

const char* graphics_name(const Settings& s) { return s.graphics ? s.graphics_board.c_str() : "none"; }

const char* timing_name(Pacer::Mode mode) {
    switch (mode) {
        case Pacer::Mode::Cycle: return "cycle";
        case Pacer::Mode::Max: return "max";
        case Pacer::Mode::Free: return "free";
        default: return "realtime";
    }
}

bool parse_timing(const std::string& text, Pacer::Mode& mode) {
    if (text == "realtime") mode = Pacer::Mode::Realtime;
    else if (text == "cycle") mode = Pacer::Mode::Cycle;
    else if (text == "max") mode = Pacer::Mode::Max;
    else if (text == "free") mode = Pacer::Mode::Free;
    else return false;
    return true;
}

std::string mac_text(const std::array<u8, 6>& mac) {
    char buf[18];
    std::snprintf(buf, sizeof buf, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return buf;
}

bool parse_mac(const std::string& text, std::array<u8, 6>& mac) {
    unsigned b[6];
    char extra;
    if (std::sscanf(text.c_str(), "%x:%x:%x:%x:%x:%x%c", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5], &extra) != 6)
        return false;
    for (unsigned k = 0; k < 6; ++k) {
        if (b[k] > 0xFF) return false;
        mac[k] = static_cast<u8>(b[k]);
    }
    return true;
}

std::string forward_text(const net::PortForward& f) {
    return std::string(f.udp ? "udp" : "tcp") + ":" + std::to_string(f.host_port) + ":" + std::to_string(f.guest_port);
}

bool parse_forward(const std::string& text, net::PortForward& f) {
    char proto[4] = {};
    unsigned host = 0, guest = 0;
    if (std::sscanf(text.c_str(), "%3[a-z]:%u:%u", proto, &host, &guest) != 3) return false;
    const std::string p = proto;
    if ((p != "tcp" && p != "udp") || !host || host > 65535 || !guest || guest > 65535) return false;
    f = {p == "udp", static_cast<u16>(host), static_cast<u16>(guest)};
    return true;
}

void apply_log_level(const std::string& l) {
    using log::Level;
    log::set_level(l == "trace" ? Level::Trace : l == "debug" ? Level::Debug : l == "info" ? Level::Info
                   : l == "error" ? Level::Error : Level::Warn);
}

namespace {

bool parse_bool(const std::string& v, bool& out) {
    if (v == "1" || v == "yes" || v == "true" || v == "on") out = true;
    else if (v == "0" || v == "no" || v == "false" || v == "off") out = false;
    else return false;
    return true;
}

bool parse_number(const std::string& v, double& out) {
    char* end = nullptr;
    out = std::strtod(v.c_str(), &end);
    return !v.empty() && end && *end == '\0';
}

}  // namespace

bool set_option(Settings& s, const std::string& key, const std::string& v, std::string& error) {
    double n = 0;
    bool ok = true;
    if (key == "prom_dir") s.prom_dir = v;
    else if (key == "nvram") s.nvram = v;
    else if (key == "disk") s.disk = v;
    else if (key == "cdrom") s.cdroms.push_back(v);
    else if (key == "machine") ok = find_platform(v) && (s.machine = v, true);
    else if (key == "cpu") ok = find_cpu_model(v) && (s.cpu = v, true);
    else if (key == "cpu_mhz") ok = parse_number(v, n) && n >= 10 && n <= 1000 && (s.cpu_mhz = n, true);
    else if (key == "ram_mb") ok = parse_number(v, n) && n >= 16 && n <= 1024 && n == static_cast<unsigned>(n) &&
                                  (ip22::ram_size_fits(static_cast<unsigned>(n), ip22::Board::Indigo2) ||
                                   ip22::ram_size_fits(static_cast<unsigned>(n), ip22::Board::Indigo2R10000)) &&
                                  (s.ram_mb = static_cast<unsigned>(n), true);  // checked against the board at power-on
    else if (key == "graphics") {
        // a board, or none (or 1 and 0)
        bool on = false;
        if (v == "none") s.graphics = false;
        else if (v == "xl24" || v == "max-impact")
            s.graphics = true, s.graphics_board = v;
        else if (parse_bool(v, on)) s.graphics = on;
        else ok = false;
    }
    else if (key == "graphics_rev") {
        if (v.empty()) s.graphics_rev = -1;
        else ok = parse_number(v, n) && n >= 0 && n <= 7 && n == static_cast<int>(n) &&
                  (s.graphics_rev = static_cast<int>(n), true);
    }
    else if (key == "console") ok = (v.empty() || v == "graphics" || v == "serial") && (s.console = v, true);
    else if (key == "mac") {
        std::array<u8, 6> mac{};
        if (v.empty()) s.mac.reset();
        else ok = parse_mac(v, mac) && (s.mac = mac, true);
    } else if (key == "net") ok = (v == "user" || v == "none" || (v.rfind("tap:", 0) == 0 && v.size() > 4)) && (s.net = v, true);
    else if (key == "hostfwd") {
        net::PortForward f;
        ok = parse_forward(v, f) && (s.forwards.push_back(f), true);
    } else if (key == "timing") ok = parse_timing(v, s.timing);
    else if (key == "idle_skip") ok = parse_bool(v, s.idle_skip);
    else if (key == "quick_scsi_scan") ok = parse_bool(v, s.quick_scsi_scan);
    else if (key == "block_cache") ok = parse_bool(v, s.block_cache);
    else if (key == "jit") ok = parse_bool(v, s.jit);
    else if (key == "audio") ok = parse_bool(v, s.audio);
    else if (key == "audio_in") ok = parse_bool(v, s.audio_in);
    else if (key == "mouse_auto_capture") ok = parse_bool(v, s.mouse_auto_capture);
    else if (key == "wheel_keys") ok = (v == "off" || v == "arrows" || v == "pages") && (s.wheel_keys = v, true);
    else if (key == "log") ok = (v == "trace" || v == "debug" || v == "info" || v == "warn" || v == "error") && (s.log = v, true);
    else {
        error = "unknown setting \"" + key + "\"";
        return false;
    }
    if (!ok) error = "bad value for " + key + ": \"" + v + "\"";
    return ok;
}

std::string user_data_folder() {
#if defined(_WIN32)
    PWSTR documents = nullptr;
    std::string folder;
    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_Documents, 0, nullptr, &documents)))
        folder = (std::filesystem::path(documents) / "SiliconGhost").string();
    CoTaskMemFree(documents);
    return folder;
#else
    const char* home = std::getenv("HOME");
    if (!home || !*home) return {};
    return (std::filesystem::path(home) / "Documents" / "SiliconGhost").string();
#endif
}

void make_user_data_folders() {
    const std::string base = user_data_folder();
    if (base.empty()) return;
    for (const char* sub : {"roms", "diskImages", "nvram"}) {
        std::error_code ec;
        std::filesystem::create_directories(std::filesystem::path(base) / sub, ec);
        if (ec) SG_LOG(Warn, "settings", "cannot make %s/%s: %s", base.c_str(), sub, ec.message().c_str());
    }
}

bool load_settings(const std::string& path, Settings& s, std::string& error) {
    std::ifstream f(path);
    if (!f) {
        error = "cannot open " + path;
        return false;
    }
    Settings loaded;
    std::string line;
    unsigned number = 0;
    while (std::getline(f, line)) {
        ++number;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        const auto eq = line.find('=');
        // Older files: a PROM chosen by hand, the CPU's speed while the guest
        // timed it and a cap on it otherwise, all no longer settings.
        if (line.compare(0, eq, "prom") == 0 || line.compare(0, eq, "timed_speed") == 0 ||
            line.compare(0, eq, "max_speed") == 0)
            continue;
        std::string why;
        if (eq == std::string::npos || !set_option(loaded, line.substr(0, eq), line.substr(eq + 1), why)) {
            // An unknown or bad line is skipped, so a file from another
            // version still loads.
            SG_LOG(Warn, "settings", "%s:%u: %s", path.c_str(), number, why.empty() ? "not key=value" : why.c_str());
        }
    }
    s = std::move(loaded);
    return true;
}

bool save_settings(const std::string& path, const Settings& s, std::string& error) {
    const std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp, std::ios::trunc);
        f << "# SiliconGhost settings\n";
        f << "prom_dir=" << s.prom_dir << "\nnvram=" << s.nvram << "\ndisk=" << s.disk << "\n";
        for (const auto& c : s.cdroms) f << "cdrom=" << c << "\n";
        f << "machine=" << s.machine << "\ncpu=" << s.cpu << "\ncpu_mhz=" << s.cpu_mhz << "\nram_mb=" << s.ram_mb << "\ngraphics=" << graphics_name(s)
          << "\ngraphics_rev=" << (s.graphics_rev >= 0 ? std::to_string(s.graphics_rev) : "")
          << "\nconsole=" << s.console << "\nmac=" << (s.mac ? mac_text(*s.mac) : "") << "\nnet=" << s.net << "\n";
        for (const auto& fw : s.forwards) f << "hostfwd=" << forward_text(fw) << "\n";
        f << "timing=" << timing_name(s.timing) << "\nidle_skip=" << (s.idle_skip ? 1 : 0) << "\nquick_scsi_scan=" << (s.quick_scsi_scan ? 1 : 0)
          << "\nblock_cache=" << (s.block_cache ? 1 : 0) << "\njit=" << (s.jit ? 1 : 0) << "\naudio=" << (s.audio ? 1 : 0)
          << "\naudio_in=" << (s.audio_in ? 1 : 0) << "\nlog=" << s.log
          << "\nmouse_auto_capture=" << (s.mouse_auto_capture ? 1 : 0) << "\nwheel_keys=" << s.wheel_keys << "\n";
        if (!f) {
            error = "cannot write " + tmp;
            return false;
        }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
        error = "cannot replace " + path + ": " + ec.message();
        return false;
    }
    return true;
}

}  // namespace sg::app
