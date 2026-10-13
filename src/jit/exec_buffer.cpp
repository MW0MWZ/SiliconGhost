// SPDX-License-Identifier: GPL-2.0-or-later
#include "jit/exec_buffer.h"

#include <cstring>

#if defined(__linux__)
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace sg::jit {

ExecBuffer::ExecBuffer(std::size_t size) : size_(size) {
#if defined(__linux__)
    fd_ = memfd_create("siliconghost-jit", MFD_CLOEXEC);
    if (fd_ < 0) return;
    if (ftruncate(fd_, static_cast<off_t>(size)) != 0) return;
    void* w = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (w == MAP_FAILED) return;
    void* x = mmap(nullptr, size, PROT_READ | PROT_EXEC, MAP_SHARED, fd_, 0);
    if (x == MAP_FAILED) {
        munmap(w, size);
        return;
    }
    write_ = static_cast<u8*>(w);
    exec_ = static_cast<u8*>(x);
#endif
}

ExecBuffer::~ExecBuffer() {
#if defined(__linux__)
    if (write_) munmap(write_, size_);
    if (exec_) munmap(exec_, size_);
    if (fd_ >= 0) close(fd_);
#endif
}

const void* ExecBuffer::add(const u8* code, std::size_t size) {
    constexpr std::size_t kAlign = 16;
    const std::size_t start = (used_ + kAlign - 1) & ~(kAlign - 1);
    if (!ok() || start + size > size_) return nullptr;
    std::memcpy(write_ + start, code, size);
    used_ = start + size;
    return exec_ + start;
}

}  // namespace sg::jit
