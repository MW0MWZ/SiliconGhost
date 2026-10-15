// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// R4000/R4400 floating-point unit (COP1).
//
// Arithmetic runs on the host FPU with the guest rounding mode installed and
// host exception flags collected per instruction. Operands and results the
// R4000 hardware does not handle (denormals, quiet NaNs, tiny results,
// out-of-range integer conversions) raise the Unimplemented Operation
// exception exactly where the real FPU does, because the operating system
// completes those operations in software.
//
// This file must be compiled with strict floating-point semantics
// (-frounding-math / /fp:strict); see CMakeLists.txt.
#include <cfenv>
#include <cmath>
#include <cstring>

#include "core/log.h"
#include "cpu/r4000/r4000.h"

#if defined(_MSC_VER) && !defined(__clang__)
#pragma fenv_access(on)
#elif defined(__clang__)
#pragma STDC FENV_ACCESS ON
#endif

namespace sg::mips {

namespace {

// FCR31
constexpr u32 kFcrRoundMask = 0x3u;
constexpr u32 kFcrCond = 1u << 23;
constexpr u32 kFcrFS = 1u << 24;
constexpr u32 kFcrWriteMask = 0x0183FFFFu;
constexpr u32 kCauseMask = 0x0003F000u;
constexpr u32 kCauseE = 1u << 17;
constexpr u32 kCauseV = 1u << 16;
constexpr u32 kCauseZ = 1u << 15;
constexpr u32 kCauseO = 1u << 14;
constexpr u32 kCauseU = 1u << 13;
constexpr u32 kCauseI = 1u << 12;
constexpr u32 kEnableU = 1u << 8;
constexpr u32 kEnableI = 1u << 7;

constexpr u32 kFir = 0x00000500u;  // FCR0: implementation 0x05, revision 0.0

enum Fmt : unsigned { kFmtS = 16, kFmtD = 17, kFmtW = 20, kFmtL = 21 };

template <typename F>
struct Traits;

template <>
struct Traits<float> {
    using Bits = u32;
    static constexpr Bits kExp = 0x7F800000u;
    static constexpr Bits kFrac = 0x007FFFFFu;
    static constexpr Bits kSignalBit = 0x00400000u;  // set = signaling (legacy MIPS)
    static constexpr Bits kSign = 0x80000000u;
    static constexpr Bits kDefaultNaN = 0x7FBFFFFFu;
    static constexpr Bits kMinNormal = 0x00800000u;
};

template <>
struct Traits<double> {
    using Bits = u64;
    static constexpr Bits kExp = 0x7FF0000000000000ull;
    static constexpr Bits kFrac = 0x000FFFFFFFFFFFFFull;
    static constexpr Bits kSignalBit = 0x0008000000000000ull;
    static constexpr Bits kSign = 0x8000000000000000ull;
    static constexpr Bits kDefaultNaN = 0x7FF7FFFFFFFFFFFFull;
    static constexpr Bits kMinNormal = 0x0010000000000000ull;
};

enum class Class { Zero, Normal, Denormal, Infinity, QuietNaN, SignalingNaN };

template <typename F>
Class classify(typename Traits<F>::Bits b) {
    using T = Traits<F>;
    const auto exp = b & T::kExp, frac = b & T::kFrac;
    if (exp == 0) return frac ? Class::Denormal : Class::Zero;
    if (exp != T::kExp) return Class::Normal;
    if (!frac) return Class::Infinity;
    return (frac & T::kSignalBit) ? Class::SignalingNaN : Class::QuietNaN;
}

template <typename F>
F from_bits(typename Traits<F>::Bits b) {
    F f;
    std::memcpy(&f, &b, sizeof f);
    return f;
}

template <typename F>
typename Traits<F>::Bits to_bits(F f) {
    typename Traits<F>::Bits b;
    std::memcpy(&b, &f, sizeof b);
    return b;
}

[[maybe_unused]] int host_rounding(u32 rm) {
    switch (rm & 3) {
        case 0: return FE_TONEAREST;
        case 1: return FE_TOWARDZERO;
        case 2: return FE_UPWARD;
        default: return FE_DOWNWARD;
    }
}

// Forces a value through memory so the compiler cannot move the arithmetic
// that produces or consumes it across the fenv calls. (MSVC honours fenv
// ordering under /fp:strict.)
template <typename T>
SG_FORCEINLINE void fp_barrier(T& v) {
#if defined(__GNUC__) || defined(__clang__)
    asm volatile("" : "+m"(v));
#else
    (void)v;
#endif
}

// Runs `op(args...)` under the given rounding mode and returns the host
// exception flags it raised (as FE_* bits). On x86-64 and AArch64 with GCC
// or Clang this goes straight to the SSE / FP control and status registers
// that the arithmetic uses: the <cfenv> calls also manage the x87 unit and
// cost several times the operation itself.
#if (defined(__x86_64__) || defined(__aarch64__)) && (defined(__GNUC__) || defined(__clang__))
#define SG_DIRECT_FENV 1
#endif

#if defined(SG_DIRECT_FENV) && defined(__x86_64__)
SG_FORCEINLINE u32 get_fp_control() {
    u32 v;
    asm volatile("stmxcsr %0" : "=m"(v));
    return v;
}
SG_FORCEINLINE void set_fp_control(u32 v) { asm volatile("ldmxcsr %0" : : "m"(v)); }

// The host's MXCSR with no exception flags and rounding to nearest, as the
// emulator itself runs. Each guest operation starts from it with its own
// rounding mode, so flags left over from host code never leak in; the host's
// leftover flags are not restored (nothing on the host reads them).
const u32 kHostFpBase = get_fp_control() & ~0x603Fu;

template <typename R, typename Op, typename... Args>
R host_eval(u32 rm, int& flags, Op op, Args... args) {
    // MXCSR: exception flags in bits 5:0, rounding control in bits 14:13
    // (nearest, down, up, toward zero).
    static constexpr u32 kRound[4] = {0u << 13, 3u << 13, 2u << 13, 1u << 13};  // guest RN, RZ, RP, RM
    const u32 round = kRound[rm & 3];
    set_fp_control(kHostFpBase | round);
    (fp_barrier(args), ...);
    R r = op(args...);
    fp_barrier(r);
    const u32 status = get_fp_control();
    if (round) set_fp_control(kHostFpBase);  // back to nearest for the host
    flags = ((status & 0x01) ? FE_INVALID : 0) | ((status & 0x04) ? FE_DIVBYZERO : 0) |
            ((status & 0x08) ? FE_OVERFLOW : 0) | ((status & 0x10) ? FE_UNDERFLOW : 0) |
            ((status & 0x20) ? FE_INEXACT : 0);
    return r;
}
#elif defined(SG_DIRECT_FENV) && defined(__aarch64__)
// The host's FPCR with rounding to nearest, as the emulator itself runs; see
// the x86-64 version. FPSR's cumulative flags are cleared before each
// operation and not restored.
u64 read_fpcr() {
    u64 v;
    asm volatile("mrs %0, fpcr" : "=r"(v));
    return v;
}
const u64 kHostFpcr = read_fpcr() & ~(3ull << 22);

template <typename R, typename Op, typename... Args>
R host_eval(u32 rm, int& flags, Op op, Args... args) {
    // FPCR rounding mode in bits 23:22 (nearest, +inf, -inf, toward zero);
    // FPSR cumulative flags IOC, DZC, OFC, UFC, IXC in bits 4:0.
    static constexpr u64 kRound[4] = {0ull << 22, 3ull << 22, 1ull << 22, 2ull << 22};  // guest RN, RZ, RP, RM
    const u64 round = kRound[rm & 3];
    if (round) asm volatile("msr fpcr, %0" : : "r"(kHostFpcr | round));
    asm volatile("msr fpsr, xzr");
    (fp_barrier(args), ...);
    R r = op(args...);
    fp_barrier(r);
    u64 status;
    asm volatile("mrs %0, fpsr" : "=r"(status));
    if (round) asm volatile("msr fpcr, %0" : : "r"(kHostFpcr));
    flags = ((status & 0x01) ? FE_INVALID : 0) | ((status & 0x02) ? FE_DIVBYZERO : 0) |
            ((status & 0x04) ? FE_OVERFLOW : 0) | ((status & 0x08) ? FE_UNDERFLOW : 0) |
            ((status & 0x10) ? FE_INEXACT : 0);
    return r;
}
#else
template <typename R, typename Op, typename... Args>
R host_eval(u32 rm, int& flags, Op op, Args... args) {
    const int saved = std::fegetround();
    std::fesetround(host_rounding(rm));
    std::feclearexcept(FE_ALL_EXCEPT);
    (fp_barrier(args), ...);
    R r = op(args...);
    fp_barrier(r);
    flags = std::fetestexcept(FE_ALL_EXCEPT);
    std::fesetround(saved);
    return r;
}
#endif

// The integral value of `v` in rounding mode `mode` (FCR31.RM order:
// nearest-even, toward zero, up, down). Computed exactly without the host's
// rounding mode, since some C libraries round to integers with the x87
// unit's setting rather than the one the arithmetic uses.
double round_integral(double v, u32 mode) {
    switch (mode & 3) {
        case 1: return std::trunc(v);
        case 2: return std::ceil(v);
        case 3: return std::floor(v);
        default: {
            // From trunc() alone: v - trunc(v) is exact, so the comparison
            // with one half is too (some round() implementations add 0.5
            // first and round 0.49999999999999994 up).
            const double t = std::trunc(v);
            const double frac = std::fabs(v - t);
            const double away = t + (v < 0 ? -1.0 : 1.0);
            if (frac < 0.5) return t;
            if (frac > 0.5) return away;
            return std::fmod(t, 2.0) == 0 ? t : away;  // a tie: to the even neighbour
        }
    }
}

// Cause bits for host flags other than underflow, which is handled separately.
u32 cause_from_host(int flags) {
    u32 c = 0;
    if (flags & FE_INVALID) c |= kCauseV;
    if (flags & FE_DIVBYZERO) c |= kCauseZ;
    if (flags & FE_OVERFLOW) c |= kCauseO | kCauseI;
    if (flags & FE_INEXACT) c |= kCauseI;
    return c;
}

// Result delivered for an untrapped underflow with FS=1: zero or the smallest
// normal number of the right sign, depending on the rounding direction.
template <typename F>
typename Traits<F>::Bits flushed(bool negative, u32 rm) {
    using T = Traits<F>;
    switch (rm & 3) {
        case 2: return negative ? T::kSign : T::kMinNormal;              // toward +inf
        case 3: return negative ? (T::kSign | T::kMinNormal) : 0;        // toward -inf
        default: return negative ? T::kSign : 0;                         // nearest, toward zero
    }
}

}  // namespace

// ---------------------------------------------------------------------------

bool R4000::fp_commit(u32 cause) {
    fcr31 = (fcr31 & ~kCauseMask) | cause;
    const u32 ieee = (cause >> 12) & 0x1F;
    const u32 enables = (fcr31 >> 7) & 0x1F;
    if ((cause & kCauseE) || (ieee & enables)) {
        raise(Exc::FPE);
        return false;
    }
    fcr31 |= ieee << 2;  // sticky flags
    return true;
}

void R4000::fp_unimplemented() {
    fcr31 = (fcr31 & ~kCauseMask) | kCauseE;
    raise(Exc::FPE);
}

void R4000::fp_check_ctc1_trap() {
    const u32 cause = (fcr31 >> 12) & 0x3F;
    const u32 enables = ((fcr31 >> 7) & 0x1F) | 0x20;  // E cannot be masked
    if (cause & enables) raise(Exc::FPE);
}

template <typename F>
void R4000::fp_compute(unsigned func, unsigned ft, unsigned fs, unsigned fd) {
    using T = Traits<F>;
    using Bits = typename T::Bits;
    constexpr bool kSingle = sizeof(F) == 4;

    const u32 rm = fcr31 & kFcrRoundMask;
    auto read = [this](unsigned n) -> Bits {
        if constexpr (kSingle) return fpr_read32(n);
        else return fpr_read64(n);
    };
    auto write = [this](unsigned n, Bits v) {
        if constexpr (kSingle) fpr_write32(n, v);
        else fpr_write64(n, v);
    };

    if (func == 0x06) {  // MOV.fmt: a plain copy, no exceptions, Cause untouched
        write(fd, read(fs));
        return;
    }

    const Bits a = read(fs), b = read(ft);
    const Class ca = classify<F>(a), cb = classify<F>(b);

    // C.cond.fmt: denormals and quiet NaNs are handled in hardware.
    if (func >= 0x30) {
        const unsigned cond = func & 15;
        u32 cause = 0;
        const bool unordered = ca == Class::QuietNaN || ca == Class::SignalingNaN || cb == Class::QuietNaN ||
                               cb == Class::SignalingNaN;
        if (ca == Class::SignalingNaN || cb == Class::SignalingNaN || (unordered && (cond & 8))) cause |= kCauseV;
        const F fa = from_bits<F>(a), fb = from_bits<F>(b);
        const bool less = !unordered && fa < fb;
        const bool equal = !unordered && fa == fb;
        const bool c = ((cond & 4) && less) || ((cond & 2) && equal) || ((cond & 1) && unordered);
        if (fp_commit(cause)) fcr31 = c ? (fcr31 | kFcrCond) : (fcr31 & ~kFcrCond);
        return;
    }

    const bool unary = func >= 0x04;  // everything except ADD/SUB/MUL/DIV
    auto operand_check = [&](Class c, u32& cause) -> bool {
        if (c == Class::Denormal || c == Class::QuietNaN) return false;
        if (c == Class::SignalingNaN) cause |= kCauseV;
        return true;
    };
    u32 cause = 0;
    if (!operand_check(ca, cause) || (!unary && !operand_check(cb, cause))) {
        fp_unimplemented();
        return;
    }

    // Finishes an arithmetic result: NaN replacement, tininess and the commit.
    int flags = 0;  // host flags of the operation whose result is passed to finish()
    auto finish = [&](F r, unsigned dest) {
        cause |= cause_from_host(flags);
        Bits rb = to_bits(r);
        if (std::isnan(r)) rb = T::kDefaultNaN;
        const bool tiny = (flags & FE_UNDERFLOW) || classify<F>(rb) == Class::Denormal;
        if (tiny) {
            if (!(fcr31 & kFcrFS) || (fcr31 & (kEnableU | kEnableI))) {
                fp_unimplemented();
                return;
            }
            cause |= kCauseU | kCauseI;
            rb = flushed<F>((rb & T::kSign) != 0, rm);
        }
        if (fp_commit(cause)) write(dest, rb);
    };

    if (cause & kCauseV) {  // signaling NaN operand: invalid, default result
        if (!fp_commit(cause)) return;
        if ((func >= 0x08 && func <= 0x0F) || func == 0x24 || func == 0x25) {
            if (func == 0x25 || func < 0x0C) fpr_write64(fd, 0x7FFFFFFFFFFFFFFFull);
            else fpr_write32(fd, 0x7FFFFFFFu);
        } else if (func == 0x20) {
            fpr_write32(fd, Traits<float>::kDefaultNaN);
        } else if (func == 0x21) {
            fpr_write64(fd, Traits<double>::kDefaultNaN);
        } else {
            write(fd, T::kDefaultNaN);
        }
        return;
    }

    const F fa = from_bits<F>(a), fb = from_bits<F>(b);
    switch (func) {
        case 0x00: return finish(host_eval<F>(rm, flags, [](F x, F y) { return x + y; }, fa, fb), fd);
        case 0x01: return finish(host_eval<F>(rm, flags, [](F x, F y) { return x - y; }, fa, fb), fd);
        case 0x02: return finish(host_eval<F>(rm, flags, [](F x, F y) { return x * y; }, fa, fb), fd);
        case 0x03: return finish(host_eval<F>(rm, flags, [](F x, F y) { return x / y; }, fa, fb), fd);
        case 0x04: return finish(host_eval<F>(rm, flags, [](F x) { return std::sqrt(x); }, fa), fd);
        case 0x05:  // ABS
            if (fp_commit(0)) write(fd, a & ~T::kSign);
            return;
        case 0x07:  // NEG
            if (fp_commit(0)) write(fd, a ^ T::kSign);
            return;

        case 0x08: case 0x09: case 0x0A: case 0x0B:  // ROUND/TRUNC/CEIL/FLOOR.L
        case 0x0C: case 0x0D: case 0x0E: case 0x0F:  // ROUND/TRUNC/CEIL/FLOOR.W
        case 0x24:                                   // CVT.W
        case 0x25: {                                 // CVT.L
            const bool to_long = func == 0x25 || func < 0x0C;
            if (to_long && !ops64_enabled()) {
                fp_unimplemented();
                return;
            }
            // ROUND/TRUNC/CEIL/FLOOR encode their direction in the low two
            // function bits, in the same order as FCR31.RM; CVT uses FCR31.RM.
            const u32 mode = func >= 0x24 ? rm : (func & 3);
            if (ca == Class::Infinity) {  // out of range for any integer format
                fp_unimplemented();
                return;
            }
            const double v = static_cast<double>(fa);
            const double r = round_integral(v, mode);
            const bool inexact = r != v;
            if (to_long) {
                if (!(r >= -9223372036854775808.0 && r < 9223372036854775808.0)) {
                    fp_unimplemented();
                    return;
                }
                if (fp_commit(inexact ? kCauseI : 0)) fpr_write64(fd, static_cast<u64>(static_cast<s64>(r)));
            } else {
                if (!(r >= -2147483648.0 && r <= 2147483647.0)) {
                    fp_unimplemented();
                    return;
                }
                if (fp_commit(inexact ? kCauseI : 0)) fpr_write32(fd, static_cast<u32>(static_cast<s32>(r)));
            }
            return;
        }

        case 0x20:  // CVT.S.fmt (from D)
            if constexpr (kSingle) {
                fp_unimplemented();  // CVT.S.S is illegal
            } else {
                const float r = host_eval<float>(rm, flags, [](F x) { return static_cast<float>(x); }, fa);
                cause |= cause_from_host(flags);
                u32 rb = to_bits(r);
                const bool tiny = (flags & FE_UNDERFLOW) || classify<float>(rb) == Class::Denormal;
                if (tiny) {
                    if (!(fcr31 & kFcrFS) || (fcr31 & (kEnableU | kEnableI))) {
                        fp_unimplemented();
                        return;
                    }
                    cause |= kCauseU | kCauseI;
                    rb = flushed<float>((rb & Traits<float>::kSign) != 0, rm);
                }
                if (fp_commit(cause)) fpr_write32(fd, rb);
            }
            return;
        case 0x21:  // CVT.D.fmt (from S): exact
            if constexpr (kSingle) {
                if (fp_commit(0)) fpr_write64(fd, to_bits(static_cast<double>(fa)));
            } else {
                fp_unimplemented();  // CVT.D.D is illegal
            }
            return;

        default: fp_unimplemented(); return;
    }
}

void R4000::fp_convert_from_int(unsigned fmt, unsigned func, unsigned fs, unsigned fd) {
    if (func != 0x20 && func != 0x21) {
        fp_unimplemented();
        return;
    }
    if (fmt == kFmtL && !ops64_enabled()) {
        fp_unimplemented();
        return;
    }
    const s64 v = fmt == kFmtW ? static_cast<s64>(static_cast<s32>(fpr_read32(fs))) : static_cast<s64>(fpr_read64(fs));
    const u32 rm = fcr31 & kFcrRoundMask;
    int flags = 0;
    if (func == 0x20) {
        const float r = host_eval<float>(rm, flags, [](s64 x) { return static_cast<float>(x); }, v);
        if (fp_commit(cause_from_host(flags))) fpr_write32(fd, to_bits(r));
    } else {
        const double r = host_eval<double>(rm, flags, [](s64 x) { return static_cast<double>(x); }, v);
        if (fp_commit(cause_from_host(flags))) fpr_write64(fd, to_bits(r));
    }
}

void R4000::execute_cop1(u32 inst) {
    touch();  // FPU state is not part of the idle-loop comparison
    if (!(status_ & sr::CU1)) {
        raise(Exc::CpU, false, false, 1);
        return;
    }
    const unsigned fmt = (inst >> 21) & 31, ft = (inst >> 16) & 31, fs = (inst >> 11) & 31;
    const unsigned fd = (inst >> 6) & 31, func = inst & 63;
    switch (fmt) {
        case 0x00: gpr[ft] = sext32(fpr_read32(fs)); return;  // MFC1
        case 0x01:                                            // DMFC1
            if (!ops64_enabled()) return fp_unimplemented();
            gpr[ft] = fpr_read64(fs);
            return;
        case 0x02:  // CFC1
            gpr[ft] = sext32(fs == 0 ? kFir : fs == 31 ? fcr31 : 0);
            return;
        case 0x04: fpr_write32(fs, static_cast<u32>(gpr[ft])); return;  // MTC1
        case 0x05:                                                     // DMTC1
            if (!ops64_enabled()) return fp_unimplemented();
            fpr_write64(fs, gpr[ft]);
            return;
        case 0x06:  // CTC1
            if (fs == 31) {
                fcr31 = static_cast<u32>(gpr[ft]) & kFcrWriteMask;
                fp_check_ctc1_trap();
            }
            return;
        case 0x08: {  // BC1F / BC1T / BC1FL / BC1TL
            const bool cond = (fcr31 & kFcrCond) != 0;
            const bool want = (ft & 1) != 0;
            const u64 target = cur_pc_ + 4 + (sext16(inst & 0xFFFF) << 2);
            if (ft & 2) branch_likely(cond == want, target);
            else branch(cond == want, target);
            return;
        }
        case kFmtS: fp_compute<float>(func, ft, fs, fd); return;
        case kFmtD: fp_compute<double>(func, ft, fs, fd); return;
        case kFmtW:
        case kFmtL: fp_convert_from_int(fmt, func, fs, fd); return;
        case 18:
        case 19:
        case 22:
        case 23: fp_unimplemented(); return;
        default: raise(Exc::RI); return;
    }
}

// Used by the pre-decoded COP1 handlers (r4000.cpp).
template void R4000::fp_compute<float>(unsigned, unsigned, unsigned, unsigned);
template void R4000::fp_compute<double>(unsigned, unsigned, unsigned, unsigned);

}  // namespace sg::mips
