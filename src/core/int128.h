// SPDX-License-Identifier: GPL-2.0-or-later
// 64x64->128 multiply helpers (DMULT/DMULTU) with a portable fallback for
// compilers without __int128 (MSVC).
#pragma once

#include "core/types.h"

namespace sg {

inline void mul_u64(u64 a, u64 b, u64& hi, u64& lo) {
#if defined(__SIZEOF_INT128__)
    __extension__ using u128 = unsigned __int128;
    const u128 p = static_cast<u128>(a) * b;
    lo = static_cast<u64>(p);
    hi = static_cast<u64>(p >> 64);
#else
    const u64 a_lo = a & 0xFFFFFFFFu, a_hi = a >> 32;
    const u64 b_lo = b & 0xFFFFFFFFu, b_hi = b >> 32;
    const u64 p0 = a_lo * b_lo, p1 = a_lo * b_hi, p2 = a_hi * b_lo, p3 = a_hi * b_hi;
    const u64 mid = (p0 >> 32) + (p1 & 0xFFFFFFFFu) + (p2 & 0xFFFFFFFFu);
    lo = (mid << 32) | (p0 & 0xFFFFFFFFu);
    hi = p3 + (p1 >> 32) + (p2 >> 32) + (mid >> 32);
#endif
}

inline void mul_s64(s64 a, s64 b, u64& hi, u64& lo) {
    mul_u64(static_cast<u64>(a), static_cast<u64>(b), hi, lo);
    if (a < 0) hi -= static_cast<u64>(b);
    if (b < 0) hi -= static_cast<u64>(a);
}

}  // namespace sg
