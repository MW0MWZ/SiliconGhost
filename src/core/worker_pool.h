// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// A few threads that share out one job's range of work, for host-side work
// done while the machine is stopped (turning a frame into pixels). The
// calling thread takes a share too and returns when the whole range is done.
#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace sg {

class WorkerPool {
public:
    // `threads` helpers besides the caller (0: run everything on the caller).
    explicit WorkerPool(unsigned threads);
    ~WorkerPool();
    WorkerPool(const WorkerPool&) = delete;
    WorkerPool& operator=(const WorkerPool&) = delete;

    // Calls fn(begin, end) over [0, count) in pieces, at once on several
    // threads; fn must be safe to run concurrently on disjoint pieces.
    void run(unsigned count, const std::function<void(unsigned begin, unsigned end)>& fn);

    // One pool for the process, sized to the host (at most 8 threads in all).
    static WorkerPool& shared();

private:
    void loop();
    bool take(unsigned& begin, unsigned& end);  // the next piece of the job, with mutex_ held

    std::vector<std::thread> threads_;
    std::mutex mutex_;
    std::condition_variable start_, done_;
    const std::function<void(unsigned, unsigned)>* job_ = nullptr;
    unsigned count_ = 0, next_ = 0, piece_ = 1, busy_ = 0;
    unsigned long long generation_ = 0;
    bool quit_ = false;
};

}  // namespace sg
