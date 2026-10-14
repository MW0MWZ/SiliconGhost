// SPDX-License-Identifier: GPL-2.0-or-later
// SDL3 window for the Newport display, with PS/2 keyboard and mouse input and
// HAL2 audio output.
#pragma once

#include <csignal>
#include <functional>
#include <memory>

#include "core/pacer.h"
#include "machine/ip22/indy.h"

namespace sg::app {

// Sends what the HAL2 DAC plays to the default output device for as long as
// it lives (used on its own when running without a window).
class AudioOutput {
public:
    virtual ~AudioOutput() = default;
};
std::unique_ptr<AudioOutput> start_audio(ip22::Indy& machine);

// Runs the machine with a window, paced by `pacer`, until the window is
// closed, the machine powers off, or `stop` becomes set. The display is
// refreshed at 60 Hz of host time; with `audio`, what the HAL2 DAC plays is
// sent to the default output device. `periodic` is called about once a
// second. Returns false if SDL could not be initialised.
bool run_with_display(ip22::Indy& machine, Pacer& pacer, volatile std::sig_atomic_t& stop, bool audio,
                      const std::function<void()>& periodic);

}  // namespace sg::app
