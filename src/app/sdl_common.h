// SPDX-License-Identifier: GPL-2.0-or-later
// Pieces shared by the SDL windows.
#pragma once

#include <SDL3/SDL.h>

#include "machine/ip22/indy.h"

namespace sg::app {

// Sends a host key press or release to the Indy's PS/2 keyboard, by position.
void send_key(ip22::Indy& machine, SDL_Scancode scancode, bool down);

}  // namespace sg::app
