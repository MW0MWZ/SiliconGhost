// SPDX-License-Identifier: GPL-2.0-or-later
#include "core/scheduler.h"

#include <algorithm>

namespace sg {

Timer::~Timer() { cancel(); }

void Timer::schedule_at(u64 cycle) {
    if (active_) scheduler_.remove(this);
    when_ = cycle;
    active_ = true;
    scheduler_.insert(this);
}

void Timer::schedule_in(u64 cycles) { schedule_at(scheduler_.now() + cycles); }

void Timer::cancel() {
    if (!active_) return;
    scheduler_.remove(this);
    active_ = false;
}

void Scheduler::insert(Timer* t) {
    auto it = std::upper_bound(queue_.begin(), queue_.end(), t,
                               [](const Timer* a, const Timer* b) { return a->when_ < b->when_; });
    queue_.insert(it, t);
    update_next();
}

void Scheduler::remove(Timer* t) {
    auto it = std::find(queue_.begin(), queue_.end(), t);
    if (it != queue_.end()) queue_.erase(it);
    update_next();
}

void Scheduler::run_due(u64 now) {
    while (!queue_.empty() && queue_.front()->when_ <= now) {
        Timer* t = queue_.front();
        queue_.erase(queue_.begin());
        update_next();
        t->active_ = false;
        t->callback_();
    }
}

}  // namespace sg
