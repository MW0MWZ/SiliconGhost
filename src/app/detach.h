// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Letting the window run without holding the terminal it was started from:
// the program starts a copy of itself in the background, in a session of
// its own (Linux, macOS) or as a detached process (Windows), with its output
// going to a log file, and the original exits so the shell carries on.
#pragma once

#include <string>

namespace sg::app {

enum class Detach {
    Parent,  // the background copy is running: exit now
    Child,   // this is the background copy
    Stay,    // run here (no terminal to leave, or starting a copy failed)
};

// Call once, early (before any threads). With no terminal attached (started
// from a file browser) the program runs where it is, its output going to
// `log_path` too.
Detach detach_from_terminal(int argc, char** argv, const std::string& log_path);

}  // namespace sg::app
