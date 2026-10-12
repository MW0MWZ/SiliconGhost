// SPDX-License-Identifier: GPL-2.0-or-later
#include "core/log.h"

#include <atomic>
#include <cstdarg>
#include <cstdio>

namespace sg::log {

namespace {
std::atomic<Level> g_level{Level::Info};

const char* level_name(Level level) {
    switch (level) {
        case Level::Trace: return "trace";
        case Level::Debug: return "debug";
        case Level::Info: return "info";
        case Level::Warn: return "warn";
        case Level::Error: return "error";
        case Level::Off: break;
    }
    return "?";
}
}  // namespace

void set_level(Level level) { g_level.store(level, std::memory_order_relaxed); }

Level level() { return g_level.load(std::memory_order_relaxed); }

void write(Level level, const char* tag, const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    std::fprintf(stderr, "[%s] %s: %s\n", level_name(level), tag, buf);
}

}  // namespace sg::log
