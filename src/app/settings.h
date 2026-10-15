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

// The processor modules the emulator can present (what the PROM and IRIX
// see in PRId and Config; the pipeline is the same).
struct CpuModel {
    const char* id;     // settings file / --cpu name
    const char* label;  // for menus
    u32 prid;
    u32 config;
    u32 scache_bytes;
};
const std::vector<CpuModel>& cpu_models();
const CpuModel* find_cpu_model(const std::string& id);

struct Settings {
    std::string prom, nvram, disk;
    std::vector<std::string> cdroms;
    std::string cpu = "r4400sc";
    double cpu_mhz = 150.0;
    unsigned ram_mb = 64;
    bool graphics = true;  // a Newport board fitted
    bool xl24 = false;     // and it is the 24-bit XL24
    std::optional<std::array<u8, 6>> mac;
    std::string net = "user";  // user | tap:NAME | none
    std::vector<net::PortForward> forwards;
    Pacer::Mode timing = Pacer::Mode::Realtime;
    double max_speed = 2.0;
    bool idle_skip = true, block_cache = true, jit = true;
    bool audio = true, audio_in = false;
    std::string log = "warn";

    ip22::IndyConfig indy_config() const;
};

// Text forms shared by the command line and the settings file.
const char* timing_name(Pacer::Mode mode);
const char* graphics_name(const Settings& s);  // xl8, xl24 or none
bool parse_timing(const std::string& text, Pacer::Mode& mode);
std::string mac_text(const std::array<u8, 6>& mac);
bool parse_mac(const std::string& text, std::array<u8, 6>& mac);
std::string forward_text(const net::PortForward& f);
bool parse_forward(const std::string& text, net::PortForward& f);
void apply_log_level(const std::string& level);

// Sets one option from its settings-file key; false (with `error`) if the
// key is unknown or the value malformed.
bool set_option(Settings& s, const std::string& key, const std::string& value, std::string& error);

// The settings file: one "key=value" per line, repeated keys for lists.
bool load_settings(const std::string& path, Settings& s, std::string& error);
bool save_settings(const std::string& path, const Settings& s, std::string& error);

}  // namespace sg::app
