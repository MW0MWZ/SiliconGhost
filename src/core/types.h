// SPDX-License-Identifier: GPL-2.0-or-later
// SiliconGhost - fundamental types and helpers.
#pragma once

#include <cstddef>
#include <cstdint>

namespace sg {

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;
using s8 = std::int8_t;
using s16 = std::int16_t;
using s32 = std::int32_t;
using s64 = std::int64_t;

constexpr u64 sext8(u64 v) { return static_cast<u64>(static_cast<s64>(static_cast<s8>(v))); }
constexpr u64 sext16(u64 v) { return static_cast<u64>(static_cast<s64>(static_cast<s16>(v))); }
constexpr u64 sext32(u64 v) { return static_cast<u64>(static_cast<s64>(static_cast<s32>(v))); }

}  // namespace sg

#if defined(__GNUC__) || defined(__clang__)
#define SG_LIKELY(x) __builtin_expect(!!(x), 1)
#define SG_UNLIKELY(x) __builtin_expect(!!(x), 0)
#define SG_FORCEINLINE inline __attribute__((always_inline))
#define SG_NOINLINE __attribute__((noinline))
#define SG_PRINTF(fmt_idx, arg_idx) __attribute__((format(printf, fmt_idx, arg_idx)))
#elif defined(_MSC_VER)
#define SG_LIKELY(x) (x)
#define SG_UNLIKELY(x) (x)
#define SG_FORCEINLINE __forceinline
#define SG_NOINLINE __declspec(noinline)
#define SG_PRINTF(fmt_idx, arg_idx)
#else
#define SG_LIKELY(x) (x)
#define SG_UNLIKELY(x) (x)
#define SG_FORCEINLINE inline
#define SG_NOINLINE
#define SG_PRINTF(fmt_idx, arg_idx)
#endif
