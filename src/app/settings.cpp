// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "app/settings.h"

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
    static const std::vector<CpuModel> models = {
        {"r4400sc", "R4400SC (1 MiB secondary cache)", 0x0450, mips::R4000::kDefaultConfig, 1u << 20},
        {"r4400pc", "R4400PC (no secondary cache)", 0x0450, mips::R4000::kDefaultConfig, 0},
        {"r4000sc", "R4000SC (1 MiB secondary cache)", 0x0430, 0x10C08263u, 1u << 20},
        {"r4000pc", "R4000PC (no secondary cache)", 0x0430, 0x10C08263u, 0},
    };
    return models;
}

const CpuModel* find_cpu_model(const std::string& id) {
    for (const auto& m : cpu_models())
        if (id == m.id) return &m;
    return nullptr;
}

ip22::IndyConfig Settings::indy_config() const {
    ip22::IndyConfig c;
    c.ram_mb = ram_mb;
    c.cpu_hz = cpu_mhz * 1.0e6;
    if (const CpuModel* m = find_cpu_model(cpu)) {
        c.prid = m->prid;
        c.cpu_config = m->config;
        c.scache_bytes = m->scache_bytes;
    }
    c.graphics = graphics;
    c.xl24 = xl24;
    c.graphics_revision = graphics_rev;
    c.disk_image = disk;
    c.cdrom_images = cdroms;
    c.mac = mac;
    return c;
}

Settings first_run_settings() {
    Settings s;
    s.ram_mb = 256;
    s.xl24 = true;
    s.cpu_mhz = 250;
    s.max_speed = 1.2;
    return s;
}

const char* graphics_name(const Settings& s) { return !s.graphics ? "none" : s.xl24 ? "xl24" : "xl8"; }

const char* timing_name(Pacer::Mode mode) {
    switch (mode) {
        case Pacer::Mode::Cycle: return "cycle";
        case Pacer::Mode::Max: return "max";
        default: return "realtime";
    }
}

bool parse_timing(const std::string& text, Pacer::Mode& mode) {
    if (text == "realtime") mode = Pacer::Mode::Realtime;
    else if (text == "cycle") mode = Pacer::Mode::Cycle;
    else if (text == "max") mode = Pacer::Mode::Max;
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
    if (key == "prom") s.prom = v;
    else if (key == "nvram") s.nvram = v;
    else if (key == "disk") s.disk = v;
    else if (key == "cdrom") s.cdroms.push_back(v);
    else if (key == "cpu") ok = find_cpu_model(v) && (s.cpu = v, true);
    else if (key == "cpu_mhz") ok = parse_number(v, n) && n >= 10 && n <= 1000 && (s.cpu_mhz = n, true);
    else if (key == "ram_mb") ok = parse_number(v, n) && n >= 16 && n <= 256 && n == static_cast<unsigned>(n) &&
                                  ip22::ram_size_fits(static_cast<unsigned>(n)) && (s.ram_mb = static_cast<unsigned>(n), true);
    else if (key == "graphics") {
        // xl8, xl24 or none (1 and 0 from older files: XL8, none)
        bool on = false;
        if (v == "xl8" || v == "xl24" || v == "none") s.graphics = v != "none", s.xl24 = v == "xl24";
        else if (parse_bool(v, on)) s.graphics = on, s.xl24 = false;
        else ok = false;
    }
    else if (key == "graphics_rev") {
        if (v.empty()) s.graphics_rev = -1;
        else ok = parse_number(v, n) && n >= 0 && n <= 7 && n == static_cast<int>(n) &&
                  (s.graphics_rev = static_cast<int>(n), true);
    }
    else if (key == "mac") {
        std::array<u8, 6> mac{};
        if (v.empty()) s.mac.reset();
        else ok = parse_mac(v, mac) && (s.mac = mac, true);
    } else if (key == "net") ok = (v == "user" || v == "none" || (v.rfind("tap:", 0) == 0 && v.size() > 4)) && (s.net = v, true);
    else if (key == "hostfwd") {
        net::PortForward f;
        ok = parse_forward(v, f) && (s.forwards.push_back(f), true);
    } else if (key == "timing") ok = parse_timing(v, s.timing);
    else if (key == "max_speed") ok = parse_number(v, n) && n > 0 && (s.max_speed = n, true);
    else if (key == "idle_skip") ok = parse_bool(v, s.idle_skip);
    else if (key == "block_cache") ok = parse_bool(v, s.block_cache);
    else if (key == "jit") ok = parse_bool(v, s.jit);
    else if (key == "audio") ok = parse_bool(v, s.audio);
    else if (key == "audio_in") ok = parse_bool(v, s.audio_in);
    else if (key == "mouse_auto_capture") ok = parse_bool(v, s.mouse_auto_capture);
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
#if defined(__APPLE__)
    return (std::filesystem::path(home) / "Documents" / "SiliconGhost").string();
#else
    return (std::filesystem::path(home) / ".SiliconGhost").string();
#endif
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
        f << "prom=" << s.prom << "\nnvram=" << s.nvram << "\ndisk=" << s.disk << "\n";
        for (const auto& c : s.cdroms) f << "cdrom=" << c << "\n";
        f << "cpu=" << s.cpu << "\ncpu_mhz=" << s.cpu_mhz << "\nram_mb=" << s.ram_mb << "\ngraphics=" << graphics_name(s)
          << "\ngraphics_rev=" << (s.graphics_rev >= 0 ? std::to_string(s.graphics_rev) : "")
          << "\nmac=" << (s.mac ? mac_text(*s.mac) : "") << "\nnet=" << s.net << "\n";
        for (const auto& fw : s.forwards) f << "hostfwd=" << forward_text(fw) << "\n";
        f << "timing=" << timing_name(s.timing) << "\nmax_speed=" << s.max_speed << "\nidle_skip=" << (s.idle_skip ? 1 : 0)
          << "\nblock_cache=" << (s.block_cache ? 1 : 0) << "\njit=" << (s.jit ? 1 : 0) << "\naudio=" << (s.audio ? 1 : 0)
          << "\naudio_in=" << (s.audio_in ? 1 : 0) << "\nlog=" << s.log
          << "\nmouse_auto_capture=" << (s.mouse_auto_capture ? 1 : 0) << "\n";
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
