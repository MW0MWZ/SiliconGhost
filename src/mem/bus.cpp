// SPDX-License-Identifier: GPL-2.0-or-later
#include "mem/bus.h"

#include <stdexcept>

#include "core/log.h"

namespace sg {

Bus::Bus() : pages_(kNumPages) {}

void Bus::check_range(u32 base, u64 size) const {
    if ((base & kPageMask) != 0 || (size & kPageMask) != 0 || size == 0 ||
        u64{base} + size > (u64{1} << 32)) {
        throw std::invalid_argument("Bus mapping must be 64 KiB aligned and within 4 GiB");
    }
}

void Bus::map_ram(u32 base, u64 size, u8* host, bool writable) {
    check_range(base, size);
    for (u64 off = 0; off < size; off += kPageSize) {
        pages_[(base + off) >> kPageShift] = Page{host + off, writable, nullptr, 0};
    }
}

void Bus::map_device(u32 base, u64 size, Device* device) {
    check_range(base, size);
    for (u64 off = 0; off < size; off += kPageSize) {
        pages_[(base + off) >> kPageShift] = Page{nullptr, false, device, base};
    }
}

void Bus::unmap(u32 base, u64 size) {
    check_range(base, size);
    for (u64 off = 0; off < size; off += kPageSize) pages_[(base + off) >> kPageShift] = Page{};
}

u8* Bus::host_read_ptr(u64 paddr) const {
    if ((paddr >> 32) != 0) return nullptr;
    const Page& p = pages_[paddr >> kPageShift];
    return p.host ? p.host + (paddr & kPageMask) : nullptr;
}

u8* Bus::host_write_ptr(u64 paddr) const {
    if ((paddr >> 32) != 0) return nullptr;
    const Page& p = pages_[paddr >> kPageShift];
    return (p.host && p.writable) ? p.host + (paddr & kPageMask) : nullptr;
}

bool Bus::read_slow(u64 paddr, unsigned size, u64& out) {
    if ((paddr >> 32) == 0) {
        const Page& p = pages_[paddr >> kPageShift];
        if (p.device && p.device->read(static_cast<u32>(paddr) - p.device_base, size, out)) return true;
    }
    SG_LOG(Debug, "bus", "bus error: read%u @ %09llx", size * 8, static_cast<unsigned long long>(paddr));
    if (error_handler_) error_handler_(paddr, size, false);
    return false;
}

bool Bus::write_slow(u64 paddr, unsigned size, u64 value) {
    if ((paddr >> 32) == 0) {
        const Page& p = pages_[paddr >> kPageShift];
        if (p.device) {
            if (p.device->write(static_cast<u32>(paddr) - p.device_base, size, value)) return true;
            SG_LOG(Debug, "bus", "bus error: write%u @ %09llx", size * 8, static_cast<unsigned long long>(paddr));
            if (error_handler_) error_handler_(paddr, size, true);
            return true;
        }
        if (p.host) {
            SG_LOG(Debug, "bus", "write%u to ROM @ %08llx ignored", size * 8,
                   static_cast<unsigned long long>(paddr));
            return true;
        }
    }
    SG_LOG(Debug, "bus", "bus error: write%u @ %09llx = %llx", size * 8, static_cast<unsigned long long>(paddr),
           static_cast<unsigned long long>(value));
    if (error_handler_) error_handler_(paddr, size, true);
    return true;
}

}  // namespace sg
