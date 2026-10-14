// SPDX-License-Identifier: GPL-2.0-or-later
#include "jit/exec_buffer.h"

#include <cstring>

#if defined(__linux__)
#include <sys/mman.h>
#include <unistd.h>
#elif defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#elif defined(__APPLE__)
#include <libkern/OSCacheControl.h>
#include <pthread.h>
#include <sys/mman.h>
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
#elif defined(_WIN32)
    // Windows: a section backed by the paging file, mapped twice like the
    // Linux memfd.
    HANDLE section = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_EXECUTE_READWRITE | SEC_COMMIT,
                                        static_cast<DWORD>(static_cast<u64>(size) >> 32), static_cast<DWORD>(size),
                                        nullptr);
    if (!section) return;
    void* w = MapViewOfFile(section, FILE_MAP_WRITE, 0, 0, size);
    void* x = w ? MapViewOfFile(section, FILE_MAP_READ | FILE_MAP_EXECUTE, 0, 0, size) : nullptr;
    if (!w || !x) {
        if (w) UnmapViewOfFile(w);
        CloseHandle(section);
        return;
    }
    section_ = section;
    write_ = static_cast<u8*>(w);
    exec_ = static_cast<u8*>(x);
#elif defined(__APPLE__)
    // macOS: one MAP_JIT mapping. On Apple silicon a thread sees it either
    // writable or executable, switched with pthread_jit_write_protect_np(),
    // so it is still never both at once.
    void* p = mmap(nullptr, size, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANON | MAP_JIT, -1, 0);
    if (p == MAP_FAILED) return;
    write_ = exec_ = static_cast<u8*>(p);
#endif
}

ExecBuffer::~ExecBuffer() {
#if defined(__linux__)
    if (write_) munmap(write_, size_);
    if (exec_) munmap(exec_, size_);
    if (fd_ >= 0) close(fd_);
#elif defined(_WIN32)
    if (write_) UnmapViewOfFile(write_);
    if (exec_) UnmapViewOfFile(exec_);
    if (section_) CloseHandle(section_);
#elif defined(__APPLE__)
    if (exec_) munmap(exec_, size_);
#endif
}

const void* ExecBuffer::add(const u8* code, std::size_t size) {
    constexpr std::size_t kAlign = 16;
    const std::size_t start = (used_ + kAlign - 1) & ~(kAlign - 1);
    if (!ok() || start + size > size_) return nullptr;
#if defined(__APPLE__)
#if defined(__aarch64__)
    pthread_jit_write_protect_np(0);  // this thread: writable
#endif
    std::memcpy(write_ + start, code, size);
#if defined(__aarch64__)
    pthread_jit_write_protect_np(1);  // and executable again
#endif
    sys_icache_invalidate(exec_ + start, size);
#elif defined(_WIN32)
    std::memcpy(write_ + start, code, size);
    FlushInstructionCache(GetCurrentProcess(), exec_ + start, size);
#else
    std::memcpy(write_ + start, code, size);
#if defined(__aarch64__)
    // The instruction cache does not see data writes: clean them to the point
    // of unification, then drop stale instructions at the executable view.
    __builtin___clear_cache(reinterpret_cast<char*>(write_ + start), reinterpret_cast<char*>(write_ + start + size));
    __builtin___clear_cache(reinterpret_cast<char*>(exec_ + start), reinterpret_cast<char*>(exec_ + start + size));
#endif
#endif
    used_ = start + size;
    return exec_ + start;
}

}  // namespace sg::jit
