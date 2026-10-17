// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "core/host_memory.h"

#include <cstring>
#include <new>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <sys/mman.h>
#endif

namespace sg {

HostMemory::HostMemory(std::size_t size) : size_(size) {
    if (size == 0) return;
#if defined(_WIN32)
    void* p = VirtualAlloc(nullptr, size, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    if (p) data_ = static_cast<u8*>(p), mapped_ = true;
#else
    void* p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (p != MAP_FAILED) {
        data_ = static_cast<u8*>(p), mapped_ = true;
#if defined(MADV_HUGEPAGE)
        madvise(p, size, MADV_HUGEPAGE);
#endif
    }
#endif
    if (!data_) {  // no page allocator: ordinary memory, cleared
        data_ = new u8[size];
        std::memset(data_, 0, size);
    }
}

HostMemory::~HostMemory() {
    if (!data_) return;
    if (!mapped_) {
        delete[] data_;
        return;
    }
#if defined(_WIN32)
    VirtualFree(data_, 0, MEM_RELEASE);
#else
    munmap(data_, size_);
#endif
}

}  // namespace sg
