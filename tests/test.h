// SPDX-License-Identifier: GPL-2.0-or-later
// Tiny dependency-free test harness.
#pragma once

#include <cstdio>
#include <vector>

namespace sg::test {

struct Case {
    const char* name;
    void (*fn)();
};

inline std::vector<Case>& registry() {
    static std::vector<Case> cases;
    return cases;
}

inline int& failures() {
    static int n = 0;
    return n;
}

struct Registrar {
    Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

}  // namespace sg::test

#define TEST(name)                                                        \
    static void name();                                                   \
    static const ::sg::test::Registrar name##_registrar(#name, name);     \
    static void name()

#define CHECK(cond)                                                                   \
    do {                                                                              \
        if (!(cond)) {                                                                \
            std::fprintf(stderr, "  %s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
            ++::sg::test::failures();                                                 \
        }                                                                             \
    } while (0)

#define CHECK_EQ(a, b)                                                                          \
    do {                                                                                        \
        const auto va_ = (a);                                                                   \
        const auto vb_ = (b);                                                                   \
        if (!(va_ == vb_)) {                                                                    \
            std::fprintf(stderr, "  %s:%d: CHECK_EQ(%s, %s) failed: 0x%llx != 0x%llx\n", __FILE__, \
                         __LINE__, #a, #b, static_cast<unsigned long long>(va_),                \
                         static_cast<unsigned long long>(vb_));                                 \
            ++::sg::test::failures();                                                           \
        }                                                                                       \
    } while (0)
