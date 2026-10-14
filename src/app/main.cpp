// SPDX-License-Identifier: GPL-2.0-or-later
// SiliconGhost command-line front end. The Indy's serial ports are connected
// to the terminal; a graphical front end comes with Newport support.
#include <algorithm>
#include <array>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "core/log.h"
#include "core/pacer.h"
#include "cpu/r4000/disasm.h"
#include "machine/ip22/indy.h"

#if defined(SG_HAVE_SDL3)
#include "app/display_sdl.h"
#endif

#if !defined(_WIN32)
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace {

sg::ip22::Indy* g_machine = nullptr;
volatile std::sig_atomic_t g_interrupted = 0;

void on_stop_signal(int) {
    g_interrupted = 1;
    if (g_machine) g_machine->cpu().request_stop();
}

void usage(const char* argv0) {
    std::fprintf(stderr,
                 "usage: %s --prom FILE [options]\n"
                 "  --prom FILE      IP22 boot PROM image (raw, big-endian)\n"
                 "  --nvram FILE     DS1386 NVRAM image, loaded at start and saved on exit\n"
                 "  --disk FILE      SCSI disk image (ID 1)\n"
                 "  --cdrom FILE     SCSI CD-ROM image (ID 4), e.g. a raw dump of IRIX media\n"
                 "  --mac ADDR       Ethernet address stored in NVRAM (default 08:00:69:0c:5e:01 if unset)\n"
                 "  --ram MB         memory size, 16..256 (default 64)\n"
                 "  --cpu-mhz N      R4400 pipeline clock (default 150)\n"
                 "  --no-graphics    leave the GFX slot empty (serial console only)\n"
                 "  --headless       no display window\n"
                 "  --screenshot F   on exit, save the display as a binary PPM image\n"
                 "  --max N          stop after N cycles (default: run until Ctrl-C)\n"
                 "  --timing MODE    realtime: guest clocks follow the host clock and the CPU runs as\n"
                 "                             fast as the host allows, up to --max-speed (default)\n"
                 "                   cycle:    one cycle per instruction, reproducible; waits when ahead\n"
                 "                   max:      no waiting; guest time runs ahead (default with --max)\n"
                 "  --max-speed X    realtime: run the CPU at most X times as fast as the real one\n"
                 "                   (default 2; IRIX 5.3 fails to boot beyond about 2.9)\n"
                 "  --no-idle-skip   execute idle loops instead of skipping to the next event\n"
                 "  --no-block-cache interpret every instruction individually (for comparison)\n"
                 "  --no-jit         run decoded blocks without translating them to host code\n"
                 "  --trace          disassemble every instruction to stderr (very slow)\n"
                 "  --log LEVEL      trace|debug|info|warn|error (default warn)\n"
                 "Both serial ports are connected to this terminal; Ctrl-C quits.\n",
                 argv0);
}

// Runs in 1 ms slices of guest time, letting the pacer relate them to the
// host clock after each.
void run_headless(sg::ip22::Indy& machine, sg::Pacer& pacer, unsigned long long max_cycles) {
    auto& cpu = machine.cpu();
    const sg::u64 slice = static_cast<sg::u64>(machine.cycles_per_second() / 1000);
    const sg::u64 start = cpu.cycles();
    pacer.update(cpu.cycles(), cpu.instructions());
    while (!g_interrupted && !machine.powered_off()) {
        const sg::u64 done = cpu.cycles() - start;
        if (done >= max_cycles) break;
        machine.run(std::min<sg::u64>(slice, max_cycles - done));
        cpu.set_cycles_per_instruction(pacer.update(cpu.cycles(), cpu.instructions()));
    }
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

bool load_file(const std::string& path, std::vector<sg::u8>& data) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    f.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    std::string prom, nvram, screenshot;
    sg::ip22::IndyConfig config;
    unsigned long long max_cycles = ~0ull;
    bool trace = false, headless = false, idle_skip = true, block_cache = true, jit = true;
    const char* timing = nullptr;
    double max_speed = 2.0;
    sg::log::set_level(sg::log::Level::Warn);

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                usage(argv[0]);
                std::exit(2);
            }
            return argv[++i];
        };
        if (!std::strcmp(a, "--prom")) prom = next();
        else if (!std::strcmp(a, "--nvram")) nvram = next();
        else if (!std::strcmp(a, "--screenshot")) screenshot = next();
        else if (!std::strcmp(a, "--disk")) config.disk_image = next();
        else if (!std::strcmp(a, "--cdrom")) config.cdrom_image = next();
        else if (!std::strcmp(a, "--mac")) {
            std::array<sg::u8, 6> mac{};
            unsigned b[6];
            if (std::sscanf(next(), "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
                std::fprintf(stderr, "error: --mac expects xx:xx:xx:xx:xx:xx\n");
                return 2;
            }
            for (unsigned k = 0; k < 6; ++k) mac[k] = static_cast<sg::u8>(b[k]);
            config.mac = mac;
        }
        else if (!std::strcmp(a, "--ram")) config.ram_mb = static_cast<unsigned>(std::strtoul(next(), nullptr, 0));
        else if (!std::strcmp(a, "--cpu-mhz")) config.cpu_hz = std::strtod(next(), nullptr) * 1.0e6;
        else if (!std::strcmp(a, "--max")) max_cycles = std::strtoull(next(), nullptr, 0);
        else if (!std::strcmp(a, "--trace")) trace = true;
        else if (!std::strcmp(a, "--headless")) headless = true;
        else if (!std::strcmp(a, "--timing")) timing = next();
        else if (!std::strcmp(a, "--max-speed")) max_speed = std::strtod(next(), nullptr);
        else if (!std::strcmp(a, "--no-idle-skip")) idle_skip = false;
        else if (!std::strcmp(a, "--no-block-cache")) block_cache = false;
        else if (!std::strcmp(a, "--no-jit")) jit = false;
        else if (!std::strcmp(a, "--no-graphics")) config.graphics = false;
        else if (!std::strcmp(a, "--log")) {
            const std::string l = next();
            using sg::log::Level;
            sg::log::set_level(l == "trace" ? Level::Trace : l == "debug" ? Level::Debug : l == "info" ? Level::Info
                               : l == "error" ? Level::Error : Level::Warn);
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (prom.empty()) {
        usage(argv[0]);
        return 2;
    }
    sg::Pacer::Mode timing_mode = max_cycles != ~0ull ? sg::Pacer::Mode::Max : sg::Pacer::Mode::Realtime;
    if (timing) {
        if (!std::strcmp(timing, "realtime")) timing_mode = sg::Pacer::Mode::Realtime;
        else if (!std::strcmp(timing, "cycle")) timing_mode = sg::Pacer::Mode::Cycle;
        else if (!std::strcmp(timing, "max")) timing_mode = sg::Pacer::Mode::Max;
        else {
            std::fprintf(stderr, "error: --timing expects realtime, cycle or max\n");
            return 2;
        }
    }

    try {
        sg::ip22::Indy machine(config);
        std::string err;
        if (!machine.load_prom(prom, err)) {
            std::fprintf(stderr, "error: %s\n", err.c_str());
            return 1;
        }
        if (!nvram.empty() && !load_file(nvram, machine.rtc().memory()))
            std::fprintf(stderr, "note: %s not found, starting with empty NVRAM\n", nvram.c_str());
        machine.rtc().power_on();  // the clock follows the host's (UTC)

        machine.set_serial_output([](unsigned, sg::u8 byte) {
            std::fputc(byte, stdout);
            std::fflush(stdout);
        });
        machine.reset();
        g_machine = &machine;
        std::signal(SIGINT, on_stop_signal);
        std::signal(SIGTERM, on_stop_signal);  // stop cleanly (NVRAM is saved) when asked to terminate

        RawTerminal terminal;
        TerminalInput input(machine);

        auto& cpu = machine.cpu();
        cpu.set_idle_skip(idle_skip && !trace);
        cpu.set_block_cache(block_cache);
        cpu.set_jit(jit);
        sg::Pacer pacer(timing_mode, machine.cycles_per_second());
        pacer.set_max_speed(max_speed);
        if (trace) {
            for (unsigned long long n = 0; n < max_cycles && !g_interrupted && !machine.powered_off(); ++n) {
                sg::u32 inst = 0;
                const bool ok = cpu.debug_read32(cpu.pc, inst);
                std::fprintf(stderr, "%016llx: %s\n", static_cast<unsigned long long>(cpu.pc),
                             ok ? sg::mips::disassemble(inst, cpu.pc).c_str() : "<unreadable>");
                machine.run(1);
            }
        } else {
#if defined(SG_HAVE_SDL3)
            const bool windowed = !headless && config.graphics && max_cycles == ~0ull;
            if (!windowed || !sg::app::run_with_display(machine, pacer, g_interrupted))
                run_headless(machine, pacer, max_cycles);
#else
            (void)headless;
            run_headless(machine, pacer, max_cycles);
#endif
        }
        g_machine = nullptr;
        if (machine.powered_off()) std::fprintf(stderr, "\n[powered off]\n");
        dump_state(cpu);

        if (!screenshot.empty()) {
            std::vector<sg::u32> pixels;
            unsigned w = 0, h = 0;
            if (machine.newport() && machine.newport()->render(pixels, w, h)) {
                std::ofstream f(screenshot, std::ios::binary);
                f << "P6\n" << w << " " << h << "\n255\n";
                for (sg::u32 p : pixels) {
                    const char rgb[3] = {static_cast<char>(p >> 16), static_cast<char>(p >> 8), static_cast<char>(p)};
                    f.write(rgb, 3);
                }
            } else {
                std::fprintf(stderr, "screenshot: no video signal\n");
            }
        }
        if (!nvram.empty()) {
            std::ofstream f(nvram, std::ios::binary);
            const auto& mem = machine.rtc().memory();
            f.write(reinterpret_cast<const char*>(mem.data()), static_cast<std::streamsize>(mem.size()));
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
