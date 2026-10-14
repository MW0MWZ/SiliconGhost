// SPDX-License-Identifier: GPL-2.0-or-later
// Memory for generated code, never writable and executable at once (W^X).
// Linux: the same pages are mapped twice, writable and executable; code is
// written through one view and run through the other. macOS: one MAP_JIT
// mapping, made writable for the calling thread while code is copied in.
// Elsewhere there is no buffer (ok() is false) and the recompiler is off.
#pragma once

#include <cstddef>

#include "core/types.h"

namespace sg::jit {

class ExecBuffer {
public:
    explicit ExecBuffer(std::size_t size);
    ~ExecBuffer();
    ExecBuffer(const ExecBuffer&) = delete;
    ExecBuffer& operator=(const ExecBuffer&) = delete;

    bool ok() const { return exec_ != nullptr; }
    // Copies `size` bytes of code in and returns its executable address, or
    // nullptr when the buffer is full.
    const void* add(const u8* code, std::size_t size);
    // Discards all code; addresses returned earlier become invalid.
    void clear() { used_ = 0; }

private:
    std::size_t size_ = 0, used_ = 0;
    u8* write_ = nullptr;
    u8* exec_ = nullptr;
    [[maybe_unused]] int fd_ = -1;  // Linux: the memfd behind both views
};

}  // namespace sg::jit
