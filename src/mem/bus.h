// SPDX-License-Identifier: GPL-2.0-or-later
// Physical address space. A flat page table (64 KiB granularity over the low
// 4 GiB) resolves each physical page to either host memory (RAM/ROM, accessed
// directly) or a Device (MMIO, dispatched virtually). RAM accesses never leave
// the inline fast path.
#pragma once

#include <functional>
#include <memory>
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

    // Base and size must be multiples of kPageSize. Later mappings replace
    // earlier ones; the remap handler is told (cached host pointers, such as
    // the CPU's soft TLB, must then be dropped).
    void map_ram(u32 base, u64 size, u8* host, bool writable = true);
    void map_device(u32 base, u64 size, Device* device);
    void unmap(u32 base, u64 size);
    void set_remap_handler(std::function<void()> handler) { remap_handler_ = std::move(handler); }

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

    // Code tracking for the CPU's decoded-block cache. Host-backed memory has
    // one flag per 4 KiB ("decoded code was taken from here"), shared by all
    // physical aliases of the same host memory. Any write to a flagged page,
    // by the CPU or a device, calls the handler with the host address of the
    // start of that 4 KiB page so cached code can be dropped.
    static constexpr unsigned kCodePageShift = 12;
    using CodeWriteHandler = std::function<void(const u8* host_page)>;
    void set_code_write_handler(CodeWriteHandler handler) { code_write_handler_ = std::move(handler); }
    // Declares a block of host memory that will be mapped (in pieces, or more
    // than once). Optional, but it guarantees every mapping of it shares one
    // set of code flags however the pieces are later arranged.
    void add_host_region(u8* host, u64 size) { region_for(host, size); }
    // Flag byte for the 4 KiB page holding `host` (host-backed memory only).
    u8* code_flag(const u8* host) const;
    void code_written(const u8* page) const {
        if (code_write_handler_) code_write_handler_(page);
    }
    // Device DMA into memory: copies and invalidates cached code it overwrites.
    // Returns false if `paddr` is not writable host memory.
    bool write_block(u64 paddr, const u8* data, std::size_t length);

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
                u8* const h = p.host + (paddr & kPageMask);
                store_be<T>(h, value);
                if (SG_UNLIKELY(p.code[(paddr & kPageMask) >> kCodePageShift]))
                    code_written(h - (paddr & ((u64{1} << kCodePageShift) - 1)));
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
        u8* code = nullptr;  // code flags for this page's 4 KiB pieces (host-backed only)
    };
    // A distinct piece of host memory, however many times it is mapped.
    struct Region {
        u8* host;
        u64 size;
        std::unique_ptr<u8[]> code;
    };
    Region& region_for(u8* host, u64 size);
    const Region* find_region(const u8* host) const;

    void check_range(u32 base, u64 size) const;
    bool read_slow(u64 paddr, unsigned size, u64& out);
    bool write_slow(u64 paddr, unsigned size, u64 value);

    std::vector<Page> pages_;
    std::vector<std::unique_ptr<Region>> regions_;
    ErrorHandler error_handler_;
    CodeWriteHandler code_write_handler_;
    std::function<void()> remap_handler_;
};

}  // namespace sg
