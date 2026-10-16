// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "app/session.h"

#include <filesystem>
#include <fstream>

#include "core/log.h"
#include "net/slirp_backend.h"
#include "net/tap_backend.h"

namespace sg::app {

bool create_blank_image(const std::string& path, u64 bytes, std::string& error) {
    std::error_code ec;
    if (std::filesystem::exists(path, ec)) {
        error = path + " already exists";
        return false;
    }
    {
        std::ofstream f(path, std::ios::binary);
        if (!f) {
            error = "cannot create " + path;
            return false;
        }
    }
    std::filesystem::resize_file(path, bytes, ec);  // a hole, where the file system has them
    if (ec) {
        error = "cannot size " + path + ": " + ec.message();
        std::filesystem::remove(path, ec);
        return false;
    }
    return true;
}

namespace {

bool load_file(const std::string& path, std::vector<u8>& data) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    f.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
    return true;
}

}  // namespace

bool Session::power_on(std::string& error) {
    if (machine_) return true;
    const Settings& s = settings_;
    if (s.prom.empty() && s.load_elf.empty()) {
        error = "no boot PROM image chosen";
        return false;
    }
    apply_log_level(s.log);

    std::unique_ptr<net::NetBackend> network;
    if (s.net == "user") {
        if (net::slirp_available()) network = net::make_slirp_backend(s.forwards, error);
        else if (!s.forwards.empty()) error = "port forwarding needs user-mode networking, which this build lacks";
    } else if (s.net.rfind("tap:", 0) == 0) {
        network = net::make_tap_backend(s.net.substr(4), error);
    } else if (s.net != "none") {
        error = "network expects user, tap:NAME or none";
    }
    if (!error.empty()) return false;

    try {
        auto machine = std::make_unique<ip22::Indy>(s.indy_config());
        if (!s.prom.empty() && !machine->load_prom(s.prom, error)) return false;
        machine->set_network(network.get());
        if (!s.nvram.empty() && !load_file(s.nvram, machine->rtc().memory()))
            SG_LOG(Warn, "session", "%s not found, starting with empty NVRAM", s.nvram.c_str());
        machine->rtc().power_on();  // the clock follows the host's (UTC)
        machine->set_serial_output(serial_out_);
        machine->reset();
        if (!s.load_elf.empty() && !machine->load_elf(s.load_elf, error)) return false;
        auto& cpu = machine->cpu();
        cpu.set_idle_skip(s.idle_skip);
        cpu.set_block_cache(s.block_cache);
        cpu.set_jit(s.jit);
        pacer_ = std::make_unique<Pacer>(s.timing, machine->cycles_per_second());
        pacer_->set_max_speed(s.max_speed);
        pacer_->update(cpu.cycles(), cpu.instructions());
        nvram_saved_ = machine->rtc().nvram_changes();
        network_ = std::move(network);
        machine_ = std::move(machine);
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }
    if (on_hook_) on_hook_(*machine_);
    return true;
}

void Session::power_off() {
    if (!machine_) return;
    if (off_hook_) off_hook_(*machine_);
    save_nvram();
    // Discs inserted while running stay in the set for next time.
    settings_.cdroms = machine_->discs();
    machine_->set_network(nullptr);
    machine_.reset();
    network_.reset();
    pacer_.reset();
}

void Session::set_serial_output(ip22::Indy::SerialOutput out) {
    serial_out_ = std::move(out);
    if (machine_) machine_->set_serial_output(serial_out_);
}

void Session::run(u64 cycles) {
    if (!machine_) return;
    machine_->run(cycles);
    auto& cpu = machine_->cpu();
    cpu.set_cycles_per_instruction(pacer_->update(cpu.cycles(), cpu.instructions(), machine_->timing_cpu()));
}

void Session::periodic() {
    if (machine_ && machine_->rtc().nvram_changes() != nvram_saved_) save_nvram();
}

// Each save goes to a temporary file that then replaces the old one, so a
// crash never leaves half an NVRAM behind.
void Session::save_nvram() {
    if (!machine_ || settings_.nvram.empty()) return;
    nvram_saved_ = machine_->rtc().nvram_changes();
    const std::string& path = settings_.nvram;
    const std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        const auto& mem = machine_->rtc().memory();
        f.write(reinterpret_cast<const char*>(mem.data()), static_cast<std::streamsize>(mem.size()));
        if (!f) {
            SG_LOG(Warn, "session", "cannot write %s", tmp.c_str());
            return;
        }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) SG_LOG(Warn, "session", "cannot replace %s: %s", path.c_str(), ec.message().c_str());
}

bool Session::save_screenshot(const std::string& path) {
    std::vector<u32> pixels;
    unsigned w = 0, h = 0;
    if (!machine_ || !machine_->newport() || !machine_->newport()->render(pixels, w, h)) return false;
    std::ofstream f(path, std::ios::binary);
    f << "P6\n" << w << " " << h << "\n255\n";
    for (u32 p : pixels) {
        const char rgb[3] = {static_cast<char>(p >> 16), static_cast<char>(p >> 8), static_cast<char>(p)};
        f.write(rgb, 3);
    }
    return static_cast<bool>(f);
}

}  // namespace sg::app
