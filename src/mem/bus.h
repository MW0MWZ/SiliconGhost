// SPDX-License-Identifier: GPL-2.0-or-later
// Physical address space. A flat page table (64 KiB granularity over the low
// 4 GiB) resolves each physical page to either host memory (RAM/ROM, accessed
// directly) or a Device (MMIO, dispatched virtually). RAM accesses never leave
// the inline fast path.
#pragma once

#include <functional>
#include <vector>

#include "core/endian.h"
#include "core/types.h"

namespace sg {

class Device {
public:
    virtual ~Device() = default;
    virtual const char* name() const = 0;

    // `offset` is relative to the base address the device was mapped at.
    // `size` is 1, 2, 4 or 8 and accesses are naturally aligned. Values are
    // plain integers as the CPU sees them (byte order is already resolved).
    // Return false to signal a bus error to the requester.
    virtual bool read(u32 offset, unsigned size, u64& value) = 0;
    virtual bool write(u32 offset, unsigned size, u64 value) = 0;

    // True if reading this register has no side effects and its value can
    // only change through a scheduled event, a write, or input delivered
    // between run slices. The CPU's idle detection may then treat a loop
    // polling it as a spin. Conservative default: no.
    virtual bool read_is_pure(u32 offset, unsigned size) const {
        (void)offset;
        (void)size;
        return false;
    }
};

class Bus {
public:
    static constexpr unsigned kPageShift = 16;
    static constexpr u64 kPageSize = u64{1} << kPageShift;
    static constexpr u64 kPageMask = kPageSize - 1;
    static constexpr u64 kNumPages = (u64{1} << 32) >> kPageShift;

    Bus();

    // Base and size must be multiples of kPageSize. Later mappings replace earlier ones.
    void map_ram(u32 base, u64 size, u8* host, bool writable = true);
    void map_device(u32 base, u64 size, Device* device);
    void unmap(u32 base, u64 size);

    // Host pointer to `paddr` if it is backed by host memory, else nullptr.
    // The pointer is valid to the end of the containing bus page.
    u8* host_read_ptr(u64 paddr) const;
    u8* host_write_ptr(u64 paddr) const;  // nullptr for ROM as well as MMIO

    // Called for accesses that nothing answers (unmapped space, or a device
    // returning false). Reads then fail (the CPU takes a bus error exception);
    // writes are posted and complete normally, so error reporting for them is
    // up to the handler (on IP22 the MC raises an interrupt).
    using ErrorHandler = std::function<void(u64 paddr, unsigned size, bool is_write)>;
    void set_error_handler(ErrorHandler handler) { error_handler_ = std::move(handler); }

    // See Device::read_is_pure. RAM and ROM reads are always pure.
    bool read_is_pure(u64 paddr, unsigned size) const;

    // Debugger hook: every device (MMIO) read, after it completes.
    std::function<void(u64 paddr, unsigned size, u64 value)> device_read_hook;

    template <typename T>
    SG_FORCEINLINE bool read(u64 paddr, T& out) {
        if (SG_LIKELY((paddr >> 32) == 0)) {
            const Page& p = pages_[paddr >> kPageShift];
            if (SG_LIKELY(p.host != nullptr)) {
                out = load_be<T>(p.host + (paddr & kPageMask));
                return true;
            }
        }
        u64 v = 0;
        if (!read_slow(paddr, sizeof(T), v)) return false;
        out = static_cast<T>(v);
        return true;
    }

    template <typename T>
    SG_FORCEINLINE bool write(u64 paddr, T value) {
        if (SG_LIKELY((paddr >> 32) == 0)) {
            const Page& p = pages_[paddr >> kPageShift];
            if (SG_LIKELY(p.host != nullptr && p.writable)) {
                store_be<T>(p.host + (paddr & kPageMask), value);
                return true;
            }
        }
        return write_slow(paddr, sizeof(T), value);
    }

private:
    struct Page {
        u8* host = nullptr;
        bool writable = false;
        Device* device = nullptr;
        u32 device_base = 0;
    };

    void check_range(u32 base, u64 size) const;
    bool read_slow(u64 paddr, unsigned size, u64& out);
    bool write_slow(u64 paddr, unsigned size, u64 value);

    std::vector<Page> pages_;
    ErrorHandler error_handler_;
};

}  // namespace sg
