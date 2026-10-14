// SPDX-License-Identifier: GPL-2.0-or-later
// User-mode networking through libslirp: the guest sits behind a NAT on
// 10.0.2.0/24, where 10.0.2.2 is the host (and gateway) and 10.0.2.3 forwards
// DNS. Nothing needs privileges; ports on the host can be forwarded to the
// guest (at 10.0.2.15).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "net/net_backend.h"

namespace sg::net {

struct PortForward {
    bool udp = false;
    u16 host_port = 0;   // on the host's loopback interface
    u16 guest_port = 0;
};

// False if the emulator was built without libslirp.
bool slirp_available();
// Null (and `error` set) if it cannot be started.
std::unique_ptr<NetBackend> make_slirp_backend(const std::vector<PortForward>& forwards, std::string& error);

}  // namespace sg::net
