// SPDX-License-Identifier: GPL-2.0-or-later
// SDL3 window for the Newport display, with PS/2 keyboard input.
#pragma once

#include <csignal>

#include "machine/ip22/indy.h"

namespace sg::app {

// Runs the machine in real time with a window until it is closed, the
// machine powers off, or `stop` becomes set. Returns false if SDL could not
// be initialised.
bool run_with_display(ip22::Indy& machine, volatile std::sig_atomic_t& stop);

}  // namespace sg::app
