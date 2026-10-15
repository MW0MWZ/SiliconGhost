// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "net/tap_backend.h"

#if defined(__linux__)

#include <fcntl.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <thread>

namespace sg::net {

namespace {

class TapBackend final : public NetBackend {
public:
    bool start(const std::string& name, std::string& error) {
        fd_ = open("/dev/net/tun", O_RDWR | O_NONBLOCK);
        if (fd_ < 0) {
            error = std::string("cannot open /dev/net/tun: ") + std::strerror(errno);
            return false;
        }
        ifreq ifr{};
        ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
        std::strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ - 1);
        if (ioctl(fd_, TUNSETIFF, &ifr) < 0) {
            error = "cannot attach to TAP interface " + name + ": " + std::strerror(errno);
            return false;
        }
        if (pipe(stop_pipe_) != 0) {
            error = "cannot create a pipe";
            return false;
        }
        thread_ = std::thread([this] { loop(); });
        return true;
    }

    ~TapBackend() override {
        if (stop_pipe_[1] >= 0) {
            const char c = 0;
            [[maybe_unused]] const ssize_t n = write(stop_pipe_[1], &c, 1);
        }
        if (thread_.joinable()) thread_.join();
        for (int fd : {fd_, stop_pipe_[0], stop_pipe_[1]})
            if (fd >= 0) close(fd);
    }

    void send(const u8* frame, std::size_t length) override {
        [[maybe_unused]] const ssize_t n = write(fd_, frame, length);  // a full queue drops the frame
    }

private:
    void loop() {
        u8 buf[2048];
        for (;;) {
            pollfd fds[2] = {{fd_, POLLIN, 0}, {stop_pipe_[0], POLLIN, 0}};
            if (poll(fds, 2, -1) < 0 && errno != EINTR) return;
            if (fds[1].revents) return;
            if (fds[0].revents & POLLIN) {
                ssize_t n;
                while ((n = read(fd_, buf, sizeof buf)) > 0) deliver(buf, static_cast<std::size_t>(n));
            }
        }
    }

    int fd_ = -1;
    int stop_pipe_[2] = {-1, -1};
    std::thread thread_;
};

}  // namespace

bool tap_available() { return true; }

std::unique_ptr<NetBackend> make_tap_backend(const std::string& name, std::string& error) {
    auto backend = std::make_unique<TapBackend>();
    if (!backend->start(name, error)) return nullptr;
    return backend;
}

}  // namespace sg::net

#else

namespace sg::net {
bool tap_available() { return false; }
std::unique_ptr<NetBackend> make_tap_backend(const std::string&, std::string& error) {
    error = "TAP networking is only available on Linux";
    return nullptr;
}
}  // namespace sg::net

#endif
