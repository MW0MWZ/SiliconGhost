// SPDX-License-Identifier: GPL-2.0-or-later
#include "net/slirp_backend.h"

#if defined(SG_HAVE_SLIRP)

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <libslirp.h>  // pkg-config's include path is the slirp directory itself

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace sg::net {

namespace {

// libslirp is not thread-safe: every call into it holds `mutex_`. It runs on
// its own thread, which polls its sockets; the guest's frames are handed to
// it from the emulation thread.
class SlirpBackend final : public NetBackend {
public:
    bool start(const std::vector<PortForward>& forwards, std::string& error) {
        if (pipe(wake_) != 0) {
            error = "cannot create a pipe";
            return false;
        }
        for (int fd : wake_) fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);

        SlirpConfig cfg{};
        cfg.version = 4;
        cfg.in_enabled = true;
        inet_pton(AF_INET, "10.0.2.0", &cfg.vnetwork);
        inet_pton(AF_INET, "255.255.255.0", &cfg.vnetmask);
        inet_pton(AF_INET, "10.0.2.2", &cfg.vhost);
        inet_pton(AF_INET, "10.0.2.15", &cfg.vdhcp_start);
        inet_pton(AF_INET, "10.0.2.3", &cfg.vnameserver);
        cfg.vhostname = "IRIS";
        static const SlirpCb callbacks = {
            .send_packet = [](const void* buf, size_t len, void* opaque) -> slirp_ssize_t {
                static_cast<SlirpBackend*>(opaque)->deliver(static_cast<const u8*>(buf), len);
                return static_cast<slirp_ssize_t>(len);
            },
            .guest_error = [](const char*, void*) {},
            .clock_get_ns = [](void*) -> int64_t {
                return std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now().time_since_epoch())
                    .count();
            },
            .timer_new = [](SlirpTimerCb cb, void* cb_opaque, void* opaque) -> void* {
                auto* self = static_cast<SlirpBackend*>(opaque);
                self->timers_.push_back(std::make_unique<Timer>(Timer{cb, cb_opaque, -1}));
                return self->timers_.back().get();
            },
            .timer_free = [](void* timer, void* opaque) {
                auto& timers = static_cast<SlirpBackend*>(opaque)->timers_;
                for (auto it = timers.begin(); it != timers.end(); ++it)
                    if (it->get() == timer) {
                        timers.erase(it);
                        break;
                    }
            },
            .timer_mod = [](void* timer, int64_t expire_ms, void*) { static_cast<Timer*>(timer)->expire_ms = expire_ms; },
            .register_poll_fd = [](int, void*) {},
            .unregister_poll_fd = [](int, void*) {},
            .notify = [](void* opaque) { static_cast<SlirpBackend*>(opaque)->wake(); },
            .init_completed = nullptr,
            .timer_new_opaque = nullptr,
        };
        slirp_ = slirp_new(&cfg, &callbacks, this);
        if (!slirp_) {
            error = "libslirp failed to start";
            return false;
        }
        in_addr loopback{}, guest{};
        inet_pton(AF_INET, "127.0.0.1", &loopback);
        inet_pton(AF_INET, "10.0.2.15", &guest);
        for (const PortForward& f : forwards) {
            if (slirp_add_hostfwd(slirp_, f.udp, loopback, f.host_port, guest, f.guest_port) != 0) {
                error = "cannot forward host port " + std::to_string(f.host_port) + " (in use?)";
                return false;
            }
        }
        thread_ = std::thread([this] { loop(); });
        return true;
    }

    ~SlirpBackend() override {
        stop_ = true;
        wake();
        if (thread_.joinable()) thread_.join();
        if (slirp_) slirp_cleanup(slirp_);
        for (int fd : wake_)
            if (fd >= 0) close(fd);
    }

    void send(const u8* frame, std::size_t length) override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            slirp_input(slirp_, frame, static_cast<int>(length));
        }
        wake();  // new connections may need polling
    }

private:
    struct Timer {
        SlirpTimerCb cb;
        void* opaque;
        int64_t expire_ms;  // -1: not armed
    };

    void wake() {
        const char c = 0;
        [[maybe_unused]] const ssize_t n = write(wake_[1], &c, 1);
    }

    static int64_t now_ms() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    void loop() {
        std::vector<pollfd> fds;
        while (!stop_) {
            uint32_t timeout = 100;
            fds.assign(1, pollfd{wake_[0], POLLIN, 0});
            {
                std::lock_guard<std::mutex> lock(mutex_);
                slirp_pollfds_fill(
                    slirp_, &timeout,
                    [](int fd, int events, void* opaque) {
                        auto& v = static_cast<SlirpBackend*>(opaque)->fds_;
                        short e = 0;
                        if (events & SLIRP_POLL_IN) e |= POLLIN;
                        if (events & SLIRP_POLL_OUT) e |= POLLOUT;
                        if (events & SLIRP_POLL_PRI) e |= POLLPRI;
                        v->push_back(pollfd{fd, e, 0});
                        return static_cast<int>(v->size() - 1);
                    },
                    this_fds(fds));
                for (const auto& t : timers_)
                    if (t->expire_ms >= 0)
                        timeout = static_cast<uint32_t>(
                            std::max<int64_t>(0, std::min<int64_t>(timeout, t->expire_ms - now_ms())));
            }
            const int ready = poll(fds.data(), fds.size(), static_cast<int>(timeout));
            if (fds[0].revents & POLLIN) {
                char buf[64];
                while (read(wake_[0], buf, sizeof buf) > 0) {
                }
            }
            std::lock_guard<std::mutex> lock(mutex_);
            fds_ = &fds;
            slirp_pollfds_poll(
                slirp_, ready < 0,
                [](int idx, void* opaque) {
                    const pollfd& p = (*static_cast<SlirpBackend*>(opaque)->fds_)[static_cast<std::size_t>(idx)];
                    int r = 0;
                    if (p.revents & POLLIN) r |= SLIRP_POLL_IN;
                    if (p.revents & POLLOUT) r |= SLIRP_POLL_OUT;
                    if (p.revents & POLLPRI) r |= SLIRP_POLL_PRI;
                    if (p.revents & POLLERR) r |= SLIRP_POLL_ERR;
                    if (p.revents & POLLHUP) r |= SLIRP_POLL_HUP;
                    return r;
                },
                this);
            const int64_t now = now_ms();
            for (std::size_t i = 0; i < timers_.size(); ++i) {
                Timer& t = *timers_[i];
                if (t.expire_ms >= 0 && t.expire_ms <= now) {
                    t.expire_ms = -1;
                    const SlirpTimerCb cb = t.cb;
                    void* const opaque = t.opaque;
                    cb(opaque);  // may add or free timers: look again next time round
                    break;
                }
            }
        }
    }

    // slirp_pollfds_fill's callback appends to the vector being built.
    void* this_fds(std::vector<pollfd>& fds) {
        fds_ = &fds;
        return this;
    }

    Slirp* slirp_ = nullptr;
    std::mutex mutex_;
    std::vector<pollfd>* fds_ = nullptr;
    std::vector<std::unique_ptr<Timer>> timers_;
    int wake_[2] = {-1, -1};
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

}  // namespace

bool slirp_available() { return true; }

std::unique_ptr<NetBackend> make_slirp_backend(const std::vector<PortForward>& forwards, std::string& error) {
    auto backend = std::make_unique<SlirpBackend>();
    if (!backend->start(forwards, error)) return nullptr;
    return backend;
}

}  // namespace sg::net

#else

namespace sg::net {
bool slirp_available() { return false; }
std::unique_ptr<NetBackend> make_slirp_backend(const std::vector<PortForward>&, std::string& error) {
    error = "this build has no user-mode networking (libslirp was not found)";
    return nullptr;
}
}  // namespace sg::net

#endif
