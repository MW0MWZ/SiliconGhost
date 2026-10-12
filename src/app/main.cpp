// SPDX-License-Identifier: GPL-2.0-or-later
// SiliconGhost command-line front end (headless for now; SDL3 display is M3).
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "core/log.h"
#include "cpu/r4000/disasm.h"
#include "machine/ip22/indy.h"

namespace {

sg::ip22::Indy* g_machine = nullptr;
volatile std::sig_atomic_t g_interrupted = 0;

void on_sigint(int) {
    g_interrupted = 1;
    if (g_machine) g_machine->cpu().request_stop();
}

void usage(const char* argv0) {
    std::fprintf(stderr,
                 "usage: %s --prom FILE [options]\n"
                 "  --prom FILE      IP22 boot PROM image (raw, big-endian)\n"
                 "  --ram MB         memory size, 16..256 (default 64)\n"
                 "  --max N          stop after N instructions (default: run until Ctrl-C)\n"
                 "  --trace          disassemble every instruction (very slow)\n"
                 "  --log LEVEL      trace|debug|info|warn|error (default info)\n",
                 argv0);
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

}  // namespace

int main(int argc, char** argv) {
    std::string prom;
    sg::ip22::IndyConfig config;
    unsigned long long max_insns = ~0ull;
    bool trace = false;

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
        else if (!std::strcmp(a, "--ram")) config.ram_mb = static_cast<unsigned>(std::strtoul(next(), nullptr, 0));
        else if (!std::strcmp(a, "--max")) max_insns = std::strtoull(next(), nullptr, 0);
        else if (!std::strcmp(a, "--trace")) trace = true;
        else if (!std::strcmp(a, "--log")) {
            const std::string l = next();
            using sg::log::Level;
            sg::log::set_level(l == "trace" ? Level::Trace : l == "debug" ? Level::Debug : l == "warn" ? Level::Warn
                               : l == "error" ? Level::Error : Level::Info);
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (prom.empty()) {
        usage(argv[0]);
        return 2;
    }

    try {
        sg::ip22::Indy machine(config);
        std::string err;
        if (!machine.load_prom(prom, err)) {
            std::fprintf(stderr, "error: %s\n", err.c_str());
            return 1;
        }
        machine.reset();
        g_machine = &machine;
        std::signal(SIGINT, on_sigint);

        auto& cpu = machine.cpu();
        if (trace) {
            for (unsigned long long n = 0; n < max_insns && !g_interrupted; ++n) {
                sg::u32 inst = 0;
                const bool ok = cpu.debug_read32(cpu.pc, inst);
                std::printf("%016llx: %s\n", static_cast<unsigned long long>(cpu.pc),
                            ok ? sg::mips::disassemble(inst, cpu.pc).c_str() : "<unreadable>");
                cpu.step();
            }
        } else {
            machine.run(max_insns);
        }
        g_machine = nullptr;
        dump_state(cpu);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
