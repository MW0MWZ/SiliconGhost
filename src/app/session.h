// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// A machine built from Settings that can be powered on and off: owns the
// Indy, its network connection, pacing and the NVRAM file. Used by the
// window and by the headless console alike.
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "app/settings.h"
#include "core/pacer.h"
#include "machine/ip22/indy.h"
#include "net/net_backend.h"

namespace sg::app {

// Writes a file of `bytes` zero bytes (sparse where the file system allows),
// for a blank disk or NVRAM image. Refuses to replace an existing file.
bool create_blank_image(const std::string& path, u64 bytes, std::string& error);
constexpr u64 kNvramBytes = 8192;  // DS1386-8K

class Session {
public:
    explicit Session(Settings settings) : settings_(std::move(settings)) {}
    ~Session() { power_off(); }
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    const Settings& settings() const { return settings_; }
    // Takes effect at the next power-on (the CD-ROM set is also kept in step
    // by the drive's own controls while running).
    void set_settings(Settings s) { settings_ = std::move(s); }

    bool power_on(std::string& error);
    // Immediately, as pulling the plug: the NVRAM is saved first.
    void power_off();
    bool on() const { return machine_ != nullptr; }
    ip22::Indy* machine() { return machine_.get(); }
    Pacer* pacer() { return pacer_.get(); }

    // Serial bytes from the machine (set before or after power-on).
    void set_serial_output(ip22::Indy::SerialOutput out);
    // Called just after each power-on and before each power-off, while the
    // machine exists (for audio and the like).
    void set_power_hooks(std::function<void(ip22::Indy&)> on, std::function<void(ip22::Indy&)> off) {
        on_hook_ = std::move(on);
        off_hook_ = std::move(off);
    }

    // Runs `cycles` of guest time, then lets the pacer relate it to the host
    // clock. The machine may have powered itself off afterwards.
    void run(u64 cycles);
    // About once a second: saves the NVRAM if the guest changed it.
    void periodic();
    // Saves the display as a binary PPM; false if there is no picture.
    bool save_screenshot(const std::string& path);

private:
    void save_nvram();

    Settings settings_;
    std::unique_ptr<net::NetBackend> network_;
    std::unique_ptr<ip22::Indy> machine_;
    std::unique_ptr<Pacer> pacer_;
    u64 nvram_saved_ = 0;
    ip22::Indy::SerialOutput serial_out_;
    std::function<void(ip22::Indy&)> on_hook_, off_hook_;
};

}  // namespace sg::app
