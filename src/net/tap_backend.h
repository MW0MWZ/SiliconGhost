// SPDX-License-Identifier: GPL-2.0-or-later
// A Linux TAP interface: the guest's frames appear on a host network device
// (set up beforehand, e.g. `ip tuntap add mode tap user $USER name tap0`,
// then bridged or routed as the host's network needs).
#pragma once

#include <memory>
#include <string>

#include "net/net_backend.h"

namespace sg::net {

bool tap_available();
std::unique_ptr<NetBackend> make_tap_backend(const std::string& name, std::string& error);

}  // namespace sg::net
