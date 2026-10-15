// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Pieces shared by the SDL windows.
#pragma once

#include <SDL3/SDL.h>

#include "machine/ip22/indy.h"

namespace sg::app {

// Sends a host key press or release to the Indy's PS/2 keyboard, by position.
// `repeat`: the host's key repeat for a key still held, sent as the
// keyboard's typematic repeat (where the key's mode allows it).
void send_key(ip22::Indy& machine, SDL_Scancode scancode, bool down, bool repeat = false);

}  // namespace sg::app
