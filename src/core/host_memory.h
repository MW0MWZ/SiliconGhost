// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Large zero-filled host memory (the guest's RAM): taken from the operating
// system page by page as it is first touched, so there is nothing to clear
// up front, and on Linux in huge pages where the system allows, which
// spares the host's TLB on guests that roam over their memory.
#pragma once

#include <cstddef>

#include "core/types.h"

namespace sg {

class HostMemory {
public:
    explicit HostMemory(std::size_t size);
    ~HostMemory();
    HostMemory(const HostMemory&) = delete;
    HostMemory& operator=(const HostMemory&) = delete;

    u8* get() const { return data_; }
    std::size_t size() const { return size_; }

private:
    u8* data_ = nullptr;
    std::size_t size_ = 0;
    bool mapped_ = false;  // from the system's page allocator (else operator new)
};

}  // namespace sg
