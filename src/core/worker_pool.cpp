// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "core/worker_pool.h"

#include <algorithm>

namespace sg {

WorkerPool::WorkerPool(unsigned threads) {
    for (unsigned i = 0; i < threads; ++i) threads_.emplace_back([this] { loop(); });
}

WorkerPool::~WorkerPool() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        quit_ = true;
    }
    start_.notify_all();
    for (auto& t : threads_) t.join();
}

WorkerPool& WorkerPool::shared() {
    static WorkerPool pool(std::min(std::max(std::thread::hardware_concurrency(), 1u), 8u) - 1);
    return pool;
}

bool WorkerPool::take(unsigned& begin, unsigned& end) {
    if (!job_ || next_ >= count_) return false;
    begin = next_;
    end = std::min(count_, next_ + piece_);
    next_ = end;
    ++busy_;
    return true;
}

void WorkerPool::run(unsigned count, const std::function<void(unsigned, unsigned)>& fn) {
    if (threads_.empty() || count < 2) {
        if (count) fn(0, count);
        return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    job_ = &fn;
    count_ = count;
    next_ = 0;
    // Pieces small enough that a thread held up elsewhere does not hold up
    // the job, large enough to be worth the taking.
    piece_ = std::max(1u, count / (4 * static_cast<unsigned>(threads_.size() + 1)));
    ++generation_;
    start_.notify_all();
    unsigned begin, end;
    while (take(begin, end)) {
        lock.unlock();
        fn(begin, end);
        lock.lock();
        --busy_;
    }
    done_.wait(lock, [this] { return busy_ == 0; });
    job_ = nullptr;
}

void WorkerPool::loop() {
    unsigned long long seen = 0;
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
        start_.wait(lock, [&] { return quit_ || generation_ != seen; });
        if (quit_) return;
        seen = generation_;
        unsigned begin, end;
        while (take(begin, end)) {
            const auto* job = job_;  // stays set until busy_ is back to 0
            lock.unlock();
            (*job)(begin, end);
            lock.lock();
            if (--busy_ == 0 && next_ >= count_) done_.notify_all();
        }
    }
}

}  // namespace sg
