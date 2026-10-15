// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Byte-order helpers. Guest memory is kept in guest (big-endian) byte order so
// that DMA, PROM images and disk images need no conversion; values are swapped
// on access, which is a single instruction (bswap/rev/movbe) on every host.
#pragma once

#include <bit>
#include <cstring>
#include <type_traits>

#include "core/types.h"

#if defined(_MSC_VER) && !defined(__clang__)
#include <stdlib.h>
#endif

namespace sg {

template <typename T>
SG_FORCEINLINE T byteswap(T v) {
    static_assert(std::is_unsigned_v<T>, "byteswap requires an unsigned integer type");
    if constexpr (sizeof(T) == 1) {
        return v;
    } else if constexpr (sizeof(T) == 2) {
#if defined(_MSC_VER) && !defined(__clang__)
        return _byteswap_ushort(v);
#else
        return __builtin_bswap16(v);
#endif
    } else if constexpr (sizeof(T) == 4) {
#if defined(_MSC_VER) && !defined(__clang__)
        return _byteswap_ulong(v);
#else
        return __builtin_bswap32(v);
#endif
    } else {
        static_assert(sizeof(T) == 8);
#if defined(_MSC_VER) && !defined(__clang__)
        return _byteswap_uint64(v);
#else
        return __builtin_bswap64(v);
#endif
    }
}

template <typename T>
SG_FORCEINLINE T load_be(const void* p) {
    T v;
    std::memcpy(&v, p, sizeof(T));
    if constexpr (std::endian::native == std::endian::little) v = byteswap(v);
    return v;
}

template <typename T>
SG_FORCEINLINE void store_be(void* p, T v) {
    if constexpr (std::endian::native == std::endian::little) v = byteswap(v);
    std::memcpy(p, &v, sizeof(T));
}

}  // namespace sg
