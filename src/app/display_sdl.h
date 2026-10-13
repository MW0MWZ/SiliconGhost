// SPDX-License-Identifier: GPL-2.0-or-later
// SDL3 window for the Newport display, with PS/2 keyboard input.
#pragma once

#include <csignal>

#include "core/pacer.h"
#include "machine/ip22/indy.h"

namespace sg::app {

// Runs the machine with a window, paced by `pacer`, until the window is
// closed, the machine powers off, or `stop` becomes set. The display is
// refreshed at 60 Hz of host time. Returns false if SDL could not be
// initialised.
bool run_with_display(ip22::Indy& machine, Pacer& pacer, volatile std::sig_atomic_t& stop);

}  // namespace sg::app
