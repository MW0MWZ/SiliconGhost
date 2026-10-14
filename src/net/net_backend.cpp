// SPDX-License-Identifier: GPL-2.0-or-later
#include "net/net_backend.h"

namespace sg::net {

bool NetBackend::receive(std::vector<u8>& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return false;
    frame = std::move(queue_.front());
    queue_.pop_front();
    return true;
}

void NetBackend::deliver(const u8* frame, std::size_t length) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= kMaxQueued) queue_.pop_front();
    queue_.emplace_back(frame, frame + length);
}

}  // namespace sg::net
