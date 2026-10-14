// SPDX-License-Identifier: GPL-2.0-or-later
// Host side of an emulated Ethernet interface. Frames the guest sends go to
// send() (called on the emulation thread); a backend queues frames for the
// guest from any thread with deliver(), and the emulation thread collects
// them with receive().
#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

#include "core/types.h"

namespace sg::net {

class NetBackend {
public:
    virtual ~NetBackend() = default;
    virtual void send(const u8* frame, std::size_t length) = 0;
    // Takes the oldest frame for the guest; false if there is none.
    bool receive(std::vector<u8>& frame);

protected:
    // Thread-safe. When the guest is not collecting frames, the oldest are
    // dropped beyond a limit, as a full receive ring would.
    void deliver(const u8* frame, std::size_t length);

private:
    static constexpr std::size_t kMaxQueued = 256;
    std::mutex mutex_;
    std::deque<std::vector<u8>> queue_;
};

}  // namespace sg::net
