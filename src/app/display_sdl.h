// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// The SDL3 front end: a window showing the Newport display, with menus for
// the machine's switches and media, a settings dialog, a status bar, serial
// terminal windows, PS/2 keyboard and mouse input and HAL2 audio.
#pragma once

#include <csignal>
#include <memory>
#include <string>

#include "app/settings.h"
#include "machine/ip22/indy.h"

namespace sg::app {

struct AudioOptions {
    bool output = true;  // what the HAL2 DAC plays goes to the default output device
    bool input = false;  // the HAL2 ADC records from the default recording device
};

// Connects the machine's audio to the host for as long as it lives (used on
// its own when running without a window).
class AudioOutput {
public:
    virtual ~AudioOutput() = default;
};
std::unique_ptr<AudioOutput> start_audio(ip22::Indy& machine, AudioOptions options);

struct GuiOptions {
    // Where the Settings dialog saves; empty: the user's settings file, which
    // is also loaded when `power_on` is false.
    std::string settings_file;
    bool power_on = false;  // power the machine on at once
    std::string screenshot;  // saved when the window closes
    std::string log_file;    // where messages go when not on a terminal (shown in About)
};

// A file in the user's SiliconGhost folder (where the settings live).
std::string user_file(const std::string& name);

// Runs the window until it is closed or `stop` becomes set. `next_disc` is
// a request (from a signal) to change to the next CD-ROM disc. Returns the
// exit status, or -1 if no window could be opened.
int run_gui(Settings settings, const GuiOptions& options, volatile std::sig_atomic_t& stop,
            volatile std::sig_atomic_t& next_disc);

}  // namespace sg::app
