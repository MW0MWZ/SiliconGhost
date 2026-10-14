<p align="center">
  <img src="graphics/Silicon_Ghost_Logo.png" alt="SiliconGhost logo" width="280"><br>
  <sub>Logo &copy; Andy Taylor (MW0MWZ)</sub>
</p>

# SiliconGhost

An open-source emulator for the SGI **Indy** (IP22): MIPS R4400 CPU, the
MC/HPC3/IOC2 chipset and **Newport (REX3)** graphics. The aim is to run the
original boot PROM and IRIX unmodified, at or above the speed of the real
machine. Linux is the primary host; macOS and Windows are planned.

## Status

SiliconGhost is under active development. What works today:

- **Boot PROM:** the IP24 PROM (5.3) passes its power-on tests, plays the
  start-up chime and reaches the System Maintenance Menu, on the serial
  console or on the Newport graphics console.
- **IRIX 5.3:** installs from its CD onto a blank disk image (labelled with
  `fx`, as on real hardware) and boots multi-user, on the serial console or
  to the graphical login and the IRIX desktop (4Dwm, Toolchest, winterm) on
  Newport, with the PS/2 keyboard and mouse. IRIS GL programs run, such as
  the flight simulator and the atlantis and ideas demos.
- **Audio:** what the machine plays goes to the host's sound output.
- **Networking:** the Ethernet port works (ping, telnet, file transfers),
  through user-mode NAT on the host (no privileges needed) or a Linux TAP
  interface.
- **CPU:** the full R4000/R4400 integer and floating-point instruction sets,
  TLB, exceptions, 32/64-bit addressing and cache tag operations.
- **Speed:** hot code is translated to x86-64 machine code, and idle loops
  are skipped exactly. On a modest host, IRIX runs at about one and a half
  times the speed of a 175 MHz Indy, and an idle IRIX uses about 2% of one
  host core.

See [Known limitations](#known-limitations) and the [Roadmap](#roadmap).

## Emulated hardware

| Component | Model | State |
|---|---|---|
| CPU | MIPS R4400 (R4000 family), FPU, 48-entry TLB | complete |
| Memory controller | MC: memory configuration, GIO bus errors | working |
| Peripheral controller | HPC3: SCSI, PBUS and Ethernet DMA | working |
| Interrupts/timers | IOC2 / INT3, 8254 PIT | working |
| Serial | Zilog 85230 SCC (two ports) | working |
| Real-time clock/NVRAM | Dallas DS1386 | working, NVRAM saved on exit |
| SCSI | WD33C93 with disk and CD-ROM targets | working |
| Graphics | Newport XL8 (REX3, VC2, XMAP9, CMAP, RAMDAC) | working (8-bit board) |
| Keyboard / mouse | 8042-compatible PS/2 | working |
| Ethernet | Seeq 80C03 | working |
| Audio | HAL2 | output works; inputs record silence |

## Building

Requirements: CMake 3.21 or later and a C++20 compiler (GCC 12+ or Clang
15+). Optional: SDL3 for the display window and sound (without it the
emulator runs on the serial console), and libslirp for user-mode networking.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build      # unit tests
```

## Running

SiliconGhost does not include any SGI software. You need your own dump of
the Indy boot PROM and, for IRIX, your own installation media.

```sh
# The PROM, with the serial console on the terminal
build/siliconghost --prom ip24prom.bin --nvram indy.nvram --no-graphics --headless

# IRIX from an installed disk, with the CD available
build/siliconghost --prom ip24prom.bin --nvram indy.nvram \
    --disk irix.img --cdrom irix53.img --cpu-mhz 175
```

Main options (run `siliconghost` with no arguments for the full list):

| Option | Meaning |
|---|---|
| `--prom FILE` | IP22 boot PROM image (raw, big-endian) |
| `--nvram FILE` | DS1386 NVRAM, loaded at start and saved on exit |
| `--disk FILE` / `--cdrom FILE` | SCSI disk (ID 1) / CD-ROM (ID 4) image |
| `--ram MB` | memory size, 16 to 256 (default 64) |
| `--cpu-mhz N` | CPU clock (default 150) |
| `--no-graphics`, `--headless` | serial console only / no display window |
| `--timing realtime\|cycle\|max` | how guest time relates to host time (see below) |
| `--max-speed X` | in realtime mode, run the CPU at most X times real speed (default 2) |
| `--no-audio` | do not play the machine's audio |
| `--net user\|tap:NAME\|none` | Ethernet through user-mode NAT (default), a TAP interface, or none |
| `--hostfwd tcp\|udp:HOST:GUEST` | with `--net user`, forward a host port to the guest |
| `--no-jit`, `--no-block-cache`, `--no-idle-skip` | turn off the speed-ups |

### The display window

Without `--headless`, the Newport screen appears in a window that can be
resized; it plays the machine's sound too.

- **Mouse:** click in the window to capture the pointer; Ctrl+Alt+G (or
  switching to another window) releases it. All three buttons work.
- **Keyboard:** keys are sent by position as a PS/2 keyboard would send
  them, including the numeric keypad and the UK keys `#~` and `\|`; IRIX's
  keyboard layout setting decides what they produce.
- **Console:** with the NVRAM's `console` set to `g` (the PROM's default on
  a machine with graphics), the PROM and IRIX use the screen; set it to `d`
  from the Command Monitor (`setenv console d`) to use the serial port.

### Installing IRIX on a new disk

IRIX CDs carry an SGI volume header and an EFS file system rather than
ISO 9660, so a raw dump of the disc is used as it is. To install:

1. Create a blank (sparse) disk image, e.g. `truncate -s 2G irix.img`.
2. From the PROM's Command Monitor, run `fx` from the CD:
   `boot -f dksc(0,4,8)sashARCS dksc(0,4,7)stand/fx.ARCS --x`, then
   `repartition/rootdrive` and `label/sync`.
3. Choose *Install System Software* from the maintenance menu and install
   from the CD-ROM as on a real Indy.

Install with the graphics board present (no `--no-graphics`): IRIX only
installs the Newport kernel driver and X server on a machine where it finds
the board. To add them to a system installed without graphics, start
*Install System Software* again with the board present, install
`eoe1.sw.unix`, `eoe1.sw.gfx_lib` and `x_eoe.sw.Server`, and let it rebuild
the kernel.

With `--no-graphics`, an NVRAM set to `console=g` disables autoboot; choose
*Start System* from the menu.

### Networking

With `--net user` the Indy sits behind a NAT on 10.0.2.0/24: use 10.0.2.15
for it, 10.0.2.2 (the host) as its gateway and 10.0.2.3 as its name server.
For example, as root on IRIX:

```sh
ifconfig ec0 10.0.2.15 netmask 255.255.255.0 up
route add default 10.0.2.2 1
```

To keep the address, put it in `/etc/hosts` under the system's name
(`/etc/sys_id`); IRIX 5.3 takes static routes such as the default gateway
from a script `/etc/init.d/network.local`, linked as `/etc/rc2.d/S31network`
(see the comments in `/etc/init.d/network`). Connections from the host
reach the guest through forwarded ports, e.g. `--hostfwd tcp:2323:23` and
`telnet localhost 2323`.

With `--net tap:NAME` frames go to an existing TAP interface (created with
`ip tuntap add mode tap user $USER name tap0` and bridged or routed as your
network needs), and the Indy is configured like any machine on that network.

### Timing

The CPU's cycle counter is the master clock for every timer in the machine,
so guest clocks always agree with each other.

- **realtime** (default): guest time follows the wall clock. A fast host
  makes the CPU look faster, while a guest second stays a second. The
  speed-up is capped by `--max-speed`, because IRIX 5.3 times the CPU
  against its timers at boot and fails on a CPU much faster than real.
- **cycle**: one cycle per instruction, reproducible from run to run.
- **max**: as fast as possible, with guest time running ahead of the wall
  clock.

## How it works

The CPU is an interpreter that serves as the reference implementation,
with performance layers on top that must give exactly the same results:

- a soft-TLB for fast address translation;
- a cache of decoded blocks, invalidated when code is written by the CPU or
  by DMA;
- an x86-64 recompiler for hot blocks, with guest registers held in host
  registers and direct links between translated blocks;
- exact idle-loop skipping: a loop whose iterations leave the machine in the
  same state is fast-forwarded to the next timer or device event.

[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) has the details.

## Known limitations

- Only the 8-bit Newport (XL8) is emulated. Blending and antialiased lines
  are drawn without the effect, which most of the desktop and the demos tried
  so far do not use.
- Audio input (microphone, line in, digital in) records silence.
- IRIX 5.3 is the system tested so far; 6.5 has not been tried.
- The recompiler is x86-64 only; on other hosts the (slower) block cache
  and interpreter run everything. Linux is the only host built and tested.
- The NVRAM is written back only when the emulator exits.

## Roadmap

- Newport 24-bit boards (XL24) and the remaining raster operations; audio input
- IRIX 6.5
- AArch64 recompiler; macOS and Windows builds
- Indigo² (IP22 "Fullhouse") machine profile

## Legal

SiliconGhost is free software, licensed under the GNU General Public License,
version 2 or (at your option) any later version; see [LICENSE](LICENSE).

It does not include SGI PROM images, IRIX or any other SGI software; users
supply their own. SGI, Indy, IRIX and related names are trademarks of their
respective owners. SiliconGhost is an independent project, not affiliated
with or endorsed by them.

The SiliconGhost logo is &copy; Andy Taylor (MW0MWZ).
