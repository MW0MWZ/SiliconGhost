// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "mem/bus.h"

#include <algorithm>
#include <cstring>
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

const Bus::Region* Bus::find_region(const u8* host) const {
    for (const auto& r : regions_)
        if (host >= r->host && host < r->host + r->size) return r.get();
    return nullptr;
}

// Aliases of memory already mapped share its region (and code flags).
Bus::Region& Bus::region_for(u8* host, u64 size) {
    for (auto& r : regions_)
        if (host >= r->host && host + size <= r->host + r->size) return *r;
    const u64 pages = (size + (u64{1} << kCodePageShift) - 1) >> kCodePageShift;
    auto r = std::make_unique<Region>(Region{host, size, std::make_unique<u8[]>(pages)});
    regions_.push_back(std::move(r));
    return *regions_.back();
}

void Bus::map_ram(u32 base, u64 size, u8* host, bool writable) {
    check_range(base, size);
    Region& r = region_for(host, size);
    for (u64 off = 0; off < size; off += kPageSize) {
        u8* const code = r.code.get() + ((host + off - r.host) >> kCodePageShift);
        pages_[(base + off) >> kPageShift] = Page{host + off, writable, nullptr, 0, code};
    }
    if (remap_handler_) remap_handler_();
}

u8* Bus::code_flag(const u8* host) const {
    const Region* r = find_region(host);
    return r ? r->code.get() + ((host - r->host) >> kCodePageShift) : nullptr;
}

bool Bus::write_block(u64 paddr, const u8* data, std::size_t length) {
    while (length) {
        if ((paddr >> 32) != 0) return false;
        const Page& p = pages_[paddr >> kPageShift];
        if (!p.host || !p.writable) return false;
        const u64 in_page = (paddr & kPageMask);
        const std::size_t chunk =
            static_cast<std::size_t>(std::min<u64>(length, (u64{1} << kCodePageShift) - (in_page & ((u64{1} << kCodePageShift) - 1))));
        u8* const h = p.host + in_page;
        std::memcpy(h, data, chunk);
        if (p.code[in_page >> kCodePageShift]) code_written(h - (in_page & ((u64{1} << kCodePageShift) - 1)));
        paddr += chunk;
        data += chunk;
        length -= chunk;
    }
    return true;
}

void Bus::map_device(u32 base, u64 size, Device* device) {
    check_range(base, size);
    for (u64 off = 0; off < size; off += kPageSize) {
        pages_[(base + off) >> kPageShift] = Page{nullptr, false, device, base};
    }
    if (remap_handler_) remap_handler_();
}

void Bus::unmap(u32 base, u64 size) {
    check_range(base, size);
    for (u64 off = 0; off < size; off += kPageSize) pages_[(base + off) >> kPageShift] = Page{};
    if (remap_handler_) remap_handler_();
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

bool Bus::read_is_pure(u64 paddr, unsigned size) const {
    if ((paddr >> 32) != 0) return false;
    const Page& p = pages_[paddr >> kPageShift];
    if (p.host) return true;
    return p.device && p.device->read_is_pure(static_cast<u32>(paddr) - p.device_base, size);
}

bool Bus::read_slow(u64 paddr, unsigned size, u64& out) {
    if ((paddr >> 32) == 0) {
        const Page& p = pages_[paddr >> kPageShift];
        if (p.device && p.device->read(static_cast<u32>(paddr) - p.device_base, size, out)) {
            if (device_read_hook) [[unlikely]]
                device_read_hook(paddr, size, out);
            return true;
        }
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
