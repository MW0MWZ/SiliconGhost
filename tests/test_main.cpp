// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include <cstring>

#include "core/log.h"
#include "test.h"

int main(int argc, char** argv) {
    sg::log::set_level(sg::log::Level::Error);
    int run = 0, failed = 0;
    for (const auto& c : sg::test::registry()) {
        if (argc > 1 && std::strstr(c.name, argv[1]) == nullptr) continue;
        const int before = sg::test::failures();
        c.fn();
        ++run;
        const bool ok = sg::test::failures() == before;
        if (!ok) ++failed;
        std::printf("[%s] %s\n", ok ? " OK " : "FAIL", c.name);
    }
    std::printf("%d/%d tests passed\n", run - failed, run);
    return failed ? 1 : 0;
}
