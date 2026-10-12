// SPDX-License-Identifier: GPL-2.0-or-later
// Minimal levelled logging. Formatting cost is only paid when the level is enabled.
#pragma once

#include "core/types.h"

namespace sg::log {

enum class Level : int { Trace = 0, Debug, Info, Warn, Error, Off };

void set_level(Level level);
Level level();
void write(Level level, const char* tag, const char* fmt, ...) SG_PRINTF(3, 4);

}  // namespace sg::log

#define SG_LOG(lvl, tag, ...)                                                   \
    do {                                                                        \
        if (::sg::log::Level::lvl >= ::sg::log::level())                        \
            ::sg::log::write(::sg::log::Level::lvl, tag, __VA_ARGS__);          \
    } while (0)
