// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// SiliconGhost front end. Started with no arguments it opens the window with
// the machine powered off and the saved settings; with arguments it powers on
// at once. With --headless (or without SDL3) both serial ports are connected
// to this terminal.
#include <atomic>
#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "app/detach.h"
#include "app/proms.h"
#include "app/session.h"
#include "app/settings.h"
#include "core/log.h"
#include "cpu/r4000/disasm.h"
#include "net/slirp_backend.h"
#include "net/tap_backend.h"

#if defined(SG_HAVE_SDL3)
#include "app/display_sdl.h"
#endif

#if !defined(_WIN32)
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace {

volatile std::sig_atomic_t g_interrupted = 0;
volatile std::sig_atomic_t g_next_disc = 0;

void on_stop_signal(int) { g_interrupted = 1; }

#if defined(SIGUSR1)
void on_disc_signal(int) { g_next_disc = 1; }
#endif

void usage(const char* argv0) {
    std::fprintf(stderr,
                 "usage: %s [options]\n"
                 "With no options the window opens with the machine off and the last settings.\n"
                 "  --prom-dir DIR   folder of boot PROM images; the one the machine needs is chosen\n"
                 "                   (default: ./roms, else roms in the SiliconGhost folder)\n"
                 "  --prom FILE      a specific boot PROM image instead (raw, big-endian)\n"
                 "  --nvram FILE     DS1386 NVRAM image, loaded at start and saved when it changes\n"
                 "  --disk FILE      SCSI disk image (ID 1)\n"
                 "  --cdrom FILE     SCSI CD-ROM image (ID 4), e.g. a raw dump of IRIX media; give it\n"
                 "                   more than once for a set of discs: ejecting one inserts the next\n"
#if defined(SIGUSR1)
                 "                   (as do Ctrl+Alt+E in the window and SIGUSR1)\n"
#else
                 "                   (as does Ctrl+Alt+E in the window)\n"
#endif
                 "  --settings FILE  start from a settings file saved by the window (options after it\n"
                 "                   override it)\n"
                 "  --mac ADDR       Ethernet address stored in NVRAM (default 08:00:69:0c:5e:01 if unset)\n",
                 argv0);
    // Only the network connections this build and host can make.
    const bool user = sg::net::slirp_available(), tap = sg::net::tap_available();
    std::fputs("  --net MODE       ", stderr);
    if (user)
        std::fputs("user:     NAT through the host, no privileges needed (the default;\n"
                   "                             the guest is 10.0.2.15, the host and gateway 10.0.2.2,\n"
                   "                             DNS 10.0.2.3)\n                   ",
                   stderr);
    if (tap) std::fputs("tap:NAME: an existing TAP interface\n                   ", stderr);
    std::fputs("none:     no cable\n", stderr);
    if (user)
        std::fputs("  --hostfwd tcp|udp:HOSTPORT:GUESTPORT\n"
                   "                   with --net user, forward a port on the host's loopback\n"
                   "                   interface to the guest (may be repeated)\n",
                   stderr);
    std::fprintf(stderr,
                 "  --machine BOARD  indy (default) or indigo2\n"
                 "  --ram MB         memory size: 16, 32, 48, 64, 80, 96, 128, 144, 160, 192 or 256\n"
                 "                   (one or two banks of 16, 32, 64 or 128; default 64); on the\n"
                 "                   indigo2 up to three banks, to 384\n"
                 "  --cpu MODEL      r4400sc (default), r4400pc, r4000sc, r4000pc, r4600sc, r4600pc,\n"
                 "                   r5000sc or r5000pc\n"
                 "  --cpu-mhz N      pipeline clock: one the module shipped at (default 150)\n"
                 "  --any-clock      allow a clock no such machine shipped with\n"
                 "  --graphics BOARD xl8 (8-bit Newport, the default), xl24 (24-bit) or none\n"
                 "  --graphics-rev N the board revision it reports, 0-7 (default 2 for xl8, 0 for xl24)\n"
                 "  --no-graphics    leave the GFX slot empty (serial console only), as --graphics none\n"
                 "  --headless       no window: both serial ports use this terminal\n"
                 "  --foreground     keep the window's messages on this terminal (the window otherwise\n"
                 "                   runs in the background, writing them to siliconghost.log)\n"
                 "  --screenshot F   on exit, save the display as a binary PPM image\n"
                 "  --max N          stop after N cycles (default: run until Ctrl-C); implies --headless\n"
                 "  --timing MODE    realtime: guest clocks follow the host clock and the CPU runs as\n"
                 "                             fast as the host allows, up to --max-speed (default)\n"
                 "                   cycle:    one cycle per instruction, reproducible; waits when ahead\n"
                 "                   max:      no waiting; guest time runs ahead (default with --max)\n"
                 "  --max-speed X    realtime: run the CPU at most X times as fast as the real one\n"
                 "                   (default 2; IRIX 5.3 boots with clock x X up to about 350 MHz)\n"
                 "  --no-idle-skip   execute idle loops instead of skipping to the next event\n"
                 "  --no-block-cache interpret every instruction individually (for comparison)\n"
                 "%s"
                 "  --no-audio       do not play the machine's audio output\n"
                 "  --audio-in       let the machine record from the host's audio input\n"
                 "  --trace          disassemble every instruction to stderr (very slow)\n"
                 "  --dump-state     print the CPU registers when the emulator stops\n"
                 "  --load-elf F     run bare-metal MIPS ELF program F instead of the PROM (no PROM needed);\n"
                 "                   implies --headless and, unless --timing is given, max\n"
                 "  --test-device    fit the bare-metal test device (console, host clock, exit code)\n"
                 "  --test-run-config N  value the test device reports in its run configuration register\n"
                 "  --log LEVEL      trace|debug|info|warn|error (default warn)\n"
                 "In the window, the Machine and Media menus hold the same choices; Ctrl-C quits.\n",
                 sg::mips::R4000::jit_available()
                     ? "  --no-jit         run decoded blocks without translating them to host code\n"
                     : "");
}

void dump_state(sg::mips::R4000& cpu) {
    using namespace sg::mips;
    std::fprintf(stderr, "pc=%016llx  cycles=%llu\n", static_cast<unsigned long long>(cpu.pc),
                 static_cast<unsigned long long>(cpu.cycles()));
    for (unsigned i = 0; i < 32; ++i) {
        std::fprintf(stderr, "%4s=%016llx%s", gpr_name(i), static_cast<unsigned long long>(cpu.gpr[i]),
                     (i % 4 == 3) ? "\n" : "  ");
    }
    for (unsigned reg : {cop0::Status, cop0::Cause, cop0::EPC, cop0::BadVAddr, cop0::ErrorEPC}) {
        std::fprintf(stderr, "%s=%016llx  ", cp0_name(reg), static_cast<unsigned long long>(cpu.cp0_read(reg)));
    }
    std::fprintf(stderr, "\n");
}

// Puts the terminal into character-at-a-time mode for the serial console and
// restores it on destruction. Ctrl-C keeps working.
class RawTerminal {
public:
    RawTerminal() {
#if !defined(_WIN32)
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0) return;
        termios raw = saved_;
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
        raw.c_iflag &= ~static_cast<tcflag_t>(ICRNL | IXON);
        raw.c_cc[VMIN] = 1;
        raw.c_cc[VTIME] = 0;
        active_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
#endif
    }
    ~RawTerminal() {
#if !defined(_WIN32)
        if (active_) tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
#endif
    }
    RawTerminal(const RawTerminal&) = delete;
    RawTerminal& operator=(const RawTerminal&) = delete;

private:
#if !defined(_WIN32)
    termios saved_{};
#endif
    bool active_ = false;
};

// Feeds terminal input to both serial ports. On POSIX the reader polls, so
// it can be stopped and joined before the machine goes away (a thread left
// blocked reading stdin would also hold up the C library's cleanup at exit).
class TerminalInput {
public:
    explicit TerminalInput(sg::ip22::Indy& machine)
        : thread_([this, &machine] {
#if !defined(_WIN32)
              while (!stop_.load(std::memory_order_relaxed)) {
                  pollfd p{STDIN_FILENO, POLLIN, 0};
                  if (poll(&p, 1, 100) <= 0) continue;
                  sg::u8 buf[256];
                  const ssize_t n = read(STDIN_FILENO, buf, sizeof buf);
                  if (n <= 0) break;  // end of input
                  for (ssize_t i = 0; i < n; ++i) {
                      machine.serial_input(0, buf[i]);
                      machine.serial_input(1, buf[i]);
                  }
              }
#else
              int c;
              while ((c = std::getchar()) != EOF) {  // detached: must not touch *this
                  machine.serial_input(0, static_cast<sg::u8>(c));
                  machine.serial_input(1, static_cast<sg::u8>(c));
              }
#endif
          }) {
    }
    ~TerminalInput() {
        stop_.store(true, std::memory_order_relaxed);
#if !defined(_WIN32)
        thread_.join();
#else
        thread_.detach();  // a blocked console read cannot be interrupted portably
#endif
    }
    TerminalInput(const TerminalInput&) = delete;
    TerminalInput& operator=(const TerminalInput&) = delete;

private:
    std::atomic<bool> stop_{false};
    std::thread thread_;
};


// Runs in 1 ms slices of guest time. Both serial ports use the terminal.
void run_headless(sg::app::Session& session, unsigned long long max_cycles) {
    auto* machine = session.machine();
    auto& cpu = machine->cpu();
    const sg::u64 slice = static_cast<sg::u64>(machine->cycles_per_second() / 1000);
    const sg::u64 start = cpu.cycles();
    auto next_periodic = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!g_interrupted && !machine->powered_off()) {
        const sg::u64 done = cpu.cycles() - start;
        if (done >= max_cycles) break;
        if (g_next_disc) {
            g_next_disc = 0;
            machine->request_next_disc();
        }
        session.run(std::min<sg::u64>(slice, max_cycles - done));
        if (const auto now = std::chrono::steady_clock::now(); now >= next_periodic) {
            next_periodic = now + std::chrono::seconds(1);
            session.periodic();
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    sg::app::Settings settings;
    std::string screenshot, settings_file;
    unsigned long long max_cycles = ~0ull;
    bool trace = false, dump = false, headless = false, timing_given = false, foreground = false, log_given = false;
    bool any_clock = false, cpu_given = false, mhz_given = false;
    sg::log::set_level(sg::log::Level::Warn);

    // Options that describe a machine. Finder on older macOS passes a process
    // serial number, which is not one, nor is --foreground.
    int real_args = 0;
    for (int i = 1; i < argc; ++i)
        if (std::strncmp(argv[i], "-psn_", 5) != 0 && std::strcmp(argv[i], "--foreground") != 0) ++real_args;
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (!std::strncmp(a, "-psn_", 5)) continue;
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                usage(argv[0]);
                std::exit(2);
            }
            return argv[++i];
        };
        // Options that map onto a setting.
        static const struct {
            const char* flag;
            const char* key;
        } kValued[] = {{"--prom", "prom"},   {"--prom-dir", "prom_dir"},  {"--nvram", "nvram"},     {"--disk", "disk"},
                       {"--cdrom", "cdrom"}, {"--mac", "mac"},         {"--net", "net"},
                       {"--hostfwd", "hostfwd"}, {"--ram", "ram_mb"},  {"--cpu", "cpu"},  {"--machine", "machine"},
                       {"--cpu-mhz", "cpu_mhz"}, {"--graphics", "graphics"},
                       {"--graphics-rev", "graphics_rev"}, {"--timing", "timing"}, {"--max-speed", "max_speed"},
                       {"--log", "log"}};
        static const struct {
            const char* flag;
            const char* key;
            const char* value;
        } kSwitches[] = {{"--no-graphics", "graphics", "0"}, {"--no-idle-skip", "idle_skip", "0"},
                         {"--no-block-cache", "block_cache", "0"}, {"--no-jit", "jit", "0"},
                         {"--no-audio", "audio", "0"},        {"--audio-in", "audio_in", "1"}};
        std::string key, value, error;
        for (const auto& v : kValued)
            if (!std::strcmp(a, v.flag)) key = v.key, value = next();
        for (const auto& v : kSwitches)
            if (!std::strcmp(a, v.flag)) key = v.key, value = v.value;
        if (!key.empty()) {
            if (key == "timing") timing_given = true;
            if (key == "cpu") cpu_given = true;
            if (key == "cpu_mhz") mhz_given = true;
            if (!sg::app::set_option(settings, key, value, error)) {
                std::fprintf(stderr, "error: %s %s\n", a, error.c_str());
                return 2;
            }
            if (key == "log") {
                sg::app::apply_log_level(value);
                log_given = true;
            }
        } else if (!std::strcmp(a, "--settings")) {
            settings_file = next();
            if (!sg::app::load_settings(settings_file, settings, error)) {
                std::fprintf(stderr, "error: %s\n", error.c_str());
                return 2;
            }
        } else if (!std::strcmp(a, "--load-elf")) settings.load_elf = next();
        else if (!std::strcmp(a, "--test-device")) settings.test_device = true;
        else if (!std::strcmp(a, "--test-run-config"))
            settings.test_run_config = static_cast<sg::u32>(std::strtoul(next().c_str(), nullptr, 0));
        else if (!std::strcmp(a, "--screenshot")) screenshot = next();
        else if (!std::strcmp(a, "--max")) max_cycles = std::strtoull(next().c_str(), nullptr, 0);
        else if (!std::strcmp(a, "--trace")) trace = true;
        else if (!std::strcmp(a, "--any-clock")) any_clock = true;
        else if (!std::strcmp(a, "--dump-state")) dump = true;
        else if (!std::strcmp(a, "--headless")) headless = true;
        else if (!std::strcmp(a, "--foreground")) foreground = true;
        else if (!std::strcmp(a, "--help") || !std::strcmp(a, "-h")) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    // A module chosen here without a clock runs at the one it shipped with.
    if (cpu_given && !mhz_given)
        if (const sg::app::CpuModel* m = sg::app::find_cpu_model(settings.cpu))
            if (const auto& mhz = m->mhz(sg::app::board_of(settings)); !mhz.empty() &&
                std::find(mhz.begin(), mhz.end(), settings.cpu_mhz) == mhz.end())
                settings.cpu_mhz = mhz.front();
    {
        std::string error;
        if (!sg::app::check_hardware(settings, any_clock, error)) {
            std::fprintf(stderr, "error: %s\n", error.c_str());
            return 2;
        }
        // A machine described here needs its PROM: say so before anything starts.
        sg::app::PromImage prom;
        if ((real_args > 0 || headless) && settings.load_elf.empty() && !sg::app::select_prom(settings, prom, error)) {
            std::fprintf(stderr, "error: %s\n", error.c_str());
            return 2;
        }
    }
    if (!settings.load_elf.empty()) {  // a bare-metal program: console only, as fast as possible
        headless = true;
        if (!timing_given) settings.timing = sg::Pacer::Mode::Max;
    }
    if (max_cycles != ~0ull) {
        headless = true;
        if (!timing_given) settings.timing = sg::Pacer::Mode::Max;
    }
    if (trace) {
        headless = true;
        settings.idle_skip = false;
    }
    sg::app::apply_log_level(settings.log);

#if defined(SG_HAVE_SDL3)
    // The window runs in the background, leaving the terminal free, unless
    // its messages were asked for (--log, --foreground).
    std::string log_file;
    if (!headless && !foreground && !log_given) {
        log_file = sg::app::user_file("siliconghost.log");
        if (sg::app::detach_from_terminal(argc, argv, log_file) == sg::app::Detach::Parent) return 0;
    }
#else
    (void)foreground;  // no window: the terminal is always the console
    (void)log_given;
#endif

    std::signal(SIGINT, on_stop_signal);
    std::signal(SIGTERM, on_stop_signal);  // stop cleanly (NVRAM is saved) when asked to terminate
#if defined(SIGUSR1)
    std::signal(SIGUSR1, on_disc_signal);  // next CD-ROM disc
#endif

#if defined(SG_HAVE_SDL3)
    if (!headless) {
        sg::app::GuiOptions options;
        options.settings_file = settings_file;
        options.power_on = real_args > 0;  // started with a machine described: power it on
        options.screenshot = screenshot;
        options.log_file = log_file;
        const int rc = sg::app::run_gui(settings, options, g_interrupted, g_next_disc);
        if (rc >= 0) return rc;
        std::fprintf(stderr, "note: no window could be opened; using the serial console\n");
    }
#else
    if (!headless)
        std::fprintf(stderr, "note: this build has no window or sound (SDL3 was not found when it was built); "
                             "using the serial console\n");
#endif

    sg::app::Session session(settings);
    session.set_serial_output([](unsigned, sg::u8 byte) {
        std::fputc(byte, stdout);
        std::fflush(stdout);
    });
    std::string error;
    if (!session.power_on(error)) {
        std::fprintf(stderr, "error: %s\n", error.c_str());
        return 1;
    }
    {
        auto& machine = *session.machine();
        RawTerminal terminal;
        TerminalInput input(machine);
#if defined(SG_HAVE_SDL3)
        const bool sound = (settings.audio || settings.audio_in) && max_cycles == ~0ull;
        const auto audio = sound ? sg::app::start_audio(machine, {settings.audio, settings.audio_in}) : nullptr;
#endif
        if (trace) {
            auto& cpu = machine.cpu();
            for (unsigned long long n = 0; n < max_cycles && !g_interrupted && !machine.powered_off(); ++n) {
                sg::u32 inst = 0;
                const bool ok = cpu.debug_read32(cpu.pc, inst);
                std::fprintf(stderr, "%016llx: %s\n", static_cast<unsigned long long>(cpu.pc),
                             ok ? sg::mips::disassemble(inst, cpu.pc).c_str() : "<unreadable>");
                machine.run(1);
            }
        } else {
            run_headless(session, max_cycles);
        }
        if (machine.powered_off()) std::fprintf(stderr, "\n[powered off]\n");
        if (dump) dump_state(machine.cpu());
        if (!screenshot.empty() && !session.save_screenshot(screenshot))
            std::fprintf(stderr, "screenshot: no video signal\n");
    }
    const int exit_code = session.machine()->exit_code();
    session.power_off();
    return exit_code >= 0 ? exit_code : 0;
}
