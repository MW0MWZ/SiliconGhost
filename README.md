<p align="center">
  <img src="graphics/Silicon_Ghost_Logo.png" alt="SiliconGhost logo" width="280"><br>
  <sub>Logo &copy; Andy Taylor (MW0MWZ)</sub>
</p>

# SiliconGhost

An open-source emulator for the SGI **Indy** and **Indigo²** (IP22, and the
R10000 Indigo², IP28): MIPS R4000, R4400, R4600, R5000 and R10000
processors, the MC/HPC3/IOC2 chipset, **Newport (REX3)** graphics and, on
the Indigo², **IMPACT** graphics. The aim is to run the
original boot PROM and IRIX unmodified, at or above the speed of the real
machine. Linux is the primary host; it also builds and runs on macOS, and
builds for Windows.

## Status

SiliconGhost is under active development. What works today:

- **Boot PROM:** the IP24 PROMs (5.0 and 5.3) pass their power-on tests, play the
  start-up chime and reach the System Maintenance Menu, on the serial
  console or on the Newport graphics console.
- **IRIX 5.3:** installs from its CD onto a blank disk image (labelled with
  `fx`, as on real hardware) and boots multi-user, on the serial console or
  to the graphical login and the IRIX desktop (4Dwm, Toolchest, winterm) on
  Newport, with the PS/2 keyboard and mouse. IRIS GL programs run, such as
  the flight simulator and the atlantis and ideas demos.
- **IRIX 6.5 (June 1998):** installs from its CD set (Installation Tools,
  Foundation 1 and 2, Applications, ONC3/NFS, Development Foundation and
  Libraries) onto an XFS disk, changing discs as the installer asks, and
  boots to the graphical login on Newport.
- **NetBSD 9.0/sgimips:** finds and attaches every device of the machine,
  installs to disk with its own installer (over the network) and boots
  multi-user on the serial console, with networking and sound.
- **OpenBSD 6.9/sgi:** boots its installer from the official install CD,
  installs to disk (sets over the network or from the CD) and boots
  multi-user on the serial console or the Newport text console, with
  networking; on the R10000 Indigo² likewise, with the IMPACT text console.
- **Audio:** what the machine plays goes to the host's sound output, with or
  without the display window, at the volume the guest sets; with
  `--audio-in` it can also record from the host's audio input.
- **Networking:** the Ethernet port works (DHCP, ping, DNS, telnet, ssh,
  file transfers in both directions), through user-mode NAT on the host (no
  privileges needed) or a Linux TAP interface.
- **Indigo² (IP22 "Full House"):** with `--machine indigo2` and the Indigo²
  PROM (5.1), IRIX 5.3 and 6.5 boot from the same disks as on the Indy, to
  the graphical desktop on a Newport (XL) board; IRIX finds the EISA bus and
  both SCSI controllers. Up to 384 MiB of memory in three banks.
- **R10000 Indigo² (IP28) with IMPACT (work in progress):** with
  `--machine indigo2-r10000` and the IP28 PROM, IRIX 6.5 installs from its
  CDs and boots to the 4Dwm desktop on a Maximum IMPACT board, with keyboard,
  mouse, networking and sound. OpenGL programs draw through the board's
  geometry engines (lit and smooth-shaded triangles, lines, points, depth,
  stencil, blending, mipmapped textures, pixel images, bitmaps and display
  lists).
- **CPU:** the full R4000/R4400 and R4600 (MIPS III) and R5000 (MIPS IV)
  integer and floating-point instruction sets, TLB, exceptions, 32/64-bit
  addressing and cache tag operations, with or without a secondary cache.
  IRIX 5.3 and 6.5, NetBSD and OpenBSD boot on the R4600 and the R5000.
- **Speed:** hot code is translated to x86-64 or AArch64 machine code, and idle loops
  are skipped exactly. On a modest host, IRIX runs at about one and a half
  times the speed of a 175 MHz Indy, and an idle IRIX uses about 2% of one
  host core.

See [Known limitations](#known-limitations) and the [Roadmap](#roadmap).

## Emulated hardware

| Component | Model | State |
|---|---|---|
| CPU | MIPS R4000, R4400, R4600 or R5000 (PC or SC module), FPU, 48-entry TLB | complete |
| CPU (IP28) | MIPS R10000 (MIPS IV, 64-entry TLB, primary and secondary caches) | working |
| Memory controller | MC: memory configuration, GIO bus errors | working |
| Peripheral controller | HPC3: SCSI, PBUS and Ethernet DMA | working |
| Interrupts/timers | IOC2 / INT3 (Indy), INT2 and EXTIO (Indigo²), 8254 PIT | working |
| Serial | Zilog 85C30 with the 85230's FIFOs (the IOC2's 85CX30, two ports): asynchronous mode, interrupts with acknowledge and under-service, loopback and echo | working |
| Real-time clock/NVRAM | Dallas DS1386; on the Indigo², settings in the 93CS56 serial EEPROM | working, saved as it changes |
| SCSI | WD33C93 with disk and CD-ROM targets (a second controller on the Indigo²) | working |
| EISA (Indigo²) | the bus, with no cards fitted | working |
| Graphics | Newport XL24 (REX3, VC2, XMAP9, CMAP, RAMDAC) | working |
| Graphics (Indigo²) | Maximum IMPACT with 4 MB texture memory (HQ3, two GE11s and RE4s, PP1, TE1, VC3, XMAP, CMAP, RAMDAC) | in progress: 2D and the X server complete; OpenGL mostly |
| Keyboard / mouse | 8042-compatible PS/2 | working |
| Ethernet | Seeq 80C03 | working |
| Audio | HAL2 | working (playback and recording) |

## Building

You need:

- **CMake** 3.21 or later, **Ninja** (or make) and **pkg-config**;
- a **C++20 compiler**: GCC 12 or later, or Clang 15 or later;
- **SDL3** for the display window, keyboard, mouse and sound. Without it the
  emulator still builds, but runs on the serial console only (CMake warns
  when it cannot find SDL3);
- **libslirp** for user-mode networking (`--net user`). Without it only
  `--net tap:NAME` is available.

On **Debian 13** or **Ubuntu 25.04 and later** (including 26.04 LTS):

```sh
sudo apt install build-essential cmake ninja-build pkg-config libsdl3-dev libslirp-dev
```

On **Fedora**:

```sh
sudo dnf install gcc-c++ cmake ninja-build pkgconf-pkg-config SDL3-devel libslirp-devel
```

On **macOS** (Apple silicon or Intel), with the Xcode command-line tools for
the compiler and [Homebrew](https://brew.sh) for the rest:

```sh
xcode-select --install
brew install cmake ninja pkgconf sdl3 libslirp
```

On macOS the build makes `build/Silicon Ghost.app`, a normal application
with the menus in the system menu bar (`build/siliconghost` links to the
program inside it, for running from the terminal). To make a copy that runs
on its own, with the libraries it uses copied in and signed for this Mac,
in the Applications folder:

```sh
cmake --install build --prefix /Applications
```

No `sudo` is needed: an administrator account can write to /Applications.
(The `install_name_tool` warnings about invalidated signatures are expected:
the copy is signed again as the last step.)

TAP networking is Linux-only; user-mode networking works everywhere.

Ubuntu 24.04 LTS and older releases do not package SDL3; build and install
it from source first ([SDL's build instructions](https://github.com/libsdl-org/SDL/blob/main/docs/README-cmake.md)),
or build without it for the serial console only. Distribution SDL3 packages
support both Wayland and X11 desktops; an SDL3 built from source needs the
Wayland and X11 development headers installed beforehand to support them.

Then:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build      # unit tests
```

Check the configure output: it says if SDL3 or libslirp was not found.

## Running

SiliconGhost does not include any SGI software. You need your own dumps of
the boot PROMs and, for IRIX, your own installation media.

Put your PROM dumps, under any names, in a `roms` folder: the one in the
SiliconGhost folder (below), or one in the folder you start it from, or one
named with `--prom-dir`. SiliconGhost recognises each dump (by its checksum,
or else the version string inside it) and starts each machine with the
newest one that suits it: an Indy (IP24) PROM for the Indy, 5.3 Rev B10
(070-9101-011) or later with an R5000, 5.0 Rev B6 (070-9101-007) or later
with an R4600, an IP22 PROM for the Indigo². An Indy without a graphics
board needs a 5.3 PROM: earlier ones set up the graphics board without first
checking that one is fitted, and stop with a bus error. If there is none
that fits, it says which it needs. Dumps read with each pair of
bytes swapped are recognised and put right as they are loaded. `--prom FILE`
(*Specific PROM* in the settings) chooses one by hand; it must suit the
machine too.

Started with no arguments, SiliconGhost opens its window with the machine
powered off: choose the NVRAM, disk and CD-ROM images and the rest in
*Machine > Settings*, then power on. The settings are kept for next time.
Given options on the command line, it powers on at once with them:

```sh
# The PROM (from ./roms), with the serial console on the terminal
build/siliconghost --nvram indy.nvram --no-graphics --headless

# IRIX from an installed disk, with the CD available
build/siliconghost --nvram indy.nvram --disk irix.img --cdrom irix53.img --cpu-mhz 175
```

Main options (`siliconghost --help` lists them all):

| Option | Meaning |
|---|---|
| `--prom-dir DIR` | folder of boot PROM dumps, from which the one the machine needs is chosen (default `./roms`, else `roms` in the SiliconGhost folder) |
| `--prom FILE` | a specific boot PROM image instead (raw, big-endian); it must suit the machine |
| `--machine indy\|indigo2\|indigo2-r10000` | the system board (default Indy); each Indigo² needs its own PROM; the IP22 Indigo² takes R4000/R4400 modules, the IP28 an R10000 at 175 or 195 MHz. Impossible combinations of platform, processor, clock and memory are refused with the choices that would fit |
| `--nvram FILE` | NVRAM (the DS1386, and on the Indigo² the serial EEPROM), loaded at start and saved whenever the guest changes it |
| `--disk FILE` / `--cdrom FILE` | SCSI disk (ID 1) / CD-ROM (ID 4) image; repeat `--cdrom` for a set of discs |
| `--ram MB` | memory size: 16, 32, 48, 64, 80, 96, 128, 144, 160, 192 or 256 (one or two banks of 16, 32, 64 or 128; default 64); on the Indigo² up to three banks (384); on the IP28 up to three banks of 64, 128, 256 or 512 (512 in two at most; 1024 in all) |
| `--cpu r4400sc\|r4400pc\|r4000sc\|r4000pc\|r4600sc\|r4600pc\|r5000sc\|r5000pc\|r10000` | processor module (default R4400SC; R10000 on the IP28, its only choice) |
| `--cpu-mhz N` | CPU clock: one the module shipped at on that platform (default: the module's usual clock) |
| `--any-clock` | allow a `--cpu-mhz` no such machine shipped with |
| `--graphics xl24\|max-impact\|none` | the 24-bit Newport (default), Maximum IMPACT with 4 MB of texture memory (Indigo²; the only graphics on the IP28, and its default), or no board (the console is then on the first serial port) |
| `--console graphics\|serial` | write the PROM's console setting to the NVRAM at power-on: the screen and keyboard, or serial port 1 (default: leave it as the NVRAM has it) |
| `--graphics-rev N` | the board revision the Newport reports, 0-7 (default 0); drivers pick timings and cursor offsets by it |
| `--headless` | no window: both serial ports use the terminal |
| `--foreground` | keep the window's messages on the terminal instead of in the background |
| `--settings FILE` | start from a settings file saved by the window |
| `--timing realtime\|cycle\|max\|free` | how guest time relates to host time (see below) |
| `--timed-speed X` | in realtime mode, the CPU's speed (X times real) while the system times it at boot (default 2); delay loops run at the real CPU's speed, or this one if slower |
| `--max-speed X` | in realtime mode, run the CPU at most X times real speed (default 0: as fast as the host allows) |
| `--no-audio` | do not play the machine's audio |
| `--audio-in` | let the machine record from the host's audio input (off by default) |
| `--net user\|tap:NAME\|none` | Ethernet through user-mode NAT (default), a TAP interface, or none |
| `--hostfwd tcp\|udp:HOST:GUEST` | with `--net user`, forward a host port to the guest |
| `--no-jit`, `--no-block-cache`, `--no-idle-skip` | turn off the speed-ups |
| `--quick-scsi-scan` | with realtime or cycle timing, absent SCSI devices answer within 1 ms instead of after the controller's programmed timeout (as with max timing) |
| `--no-quick-scsi-scan` | the full selection timeout with every timing, max included |
| `--dump-state` | print the CPU registers when the emulator stops (for debugging) |
| `--load-elf FILE` | run a bare-metal MIPS ELF program instead of the PROM (no PROM needed); implies `--headless` and, unless `--timing` is given, `free` |
| `--test-device` | fit the bare-metal test device in GIO expansion slot 0: console output, a host clock, an instruction count and an exit code, for bare-metal test and benchmark programs |
| `--test-run-config N` | the value the test device reports in its run configuration register |

### The window

SiliconGhost keeps its files in one folder, made on first start:
`~/Documents/SiliconGhost` on macOS, `Documents\SiliconGhost` on Windows and
`~/.SiliconGhost` on Linux. It holds `settings.ini`, the message log
`siliconghost.log`, and `roms`, `diskImages` and `nvram` folders for PROM
dumps, disk and CD-ROM images and NVRAM files, where the file choosers
start. (Settings from an earlier version are copied in.)

Started from a terminal, the window carries on in the background and the
terminal is free again at once; its messages go to `siliconghost.log`
(*Help > About* shows the path). `--foreground` or `--log` keeps them on
the terminal, and `--headless` always runs there.

The Newport screen fills the window between a menu bar and a status bar;
the window can be resized and made full screen. The machine's sound plays
with or without the window (unless turned off), about 80 ms behind the
machine so that a busy moment on the host does not break it up. A host too
slow to draw every frame and keep the machine at real time shows fewer frames
a second instead.

If no window appears, check the start-up messages: a build made without
SDL3 says so (CMake also warns when it cannot find SDL3), and SDL reports
why it could not open a window.

- **Machine menu:** power on; the front-panel power button (the system
  shuts down and powers off, as on a real Indy); power off at once; reset;
  the volume buttons; the Settings dialog, and opening or saving a
  settings file. Settings cover everything the command-line options do.
  New blank NVRAM and disk images can be made from the dialog.
- **Media menu:** the CD-ROM drive's eject button (refused while the system
  has the disc locked in; *Force Eject* works anyway), inserting another
  disc, and the discs of the set. Ejecting from the guest (`eject` in IRIX)
  puts the next disc of the set in; Ctrl+Alt+E, or `kill -USR1` to the
  emulator, does the same.
- **View menu:** a terminal window for each serial port (the first is the
  console, IRIX's `ttyd1`), the status bar and full screen. The terminals
  are VT100s with history (Shift+PgUp, the mouse wheel) and copy and paste
  (Ctrl+Shift+C/V, Cmd+C/V on a Mac). Without a graphics board the first
  one opens by itself. Only `--headless` connects the serial ports to the
  terminal SiliconGhost was started from.
- **Status bar:** power, disk, CD-ROM and network activity, the CPU's load
  (the time it is not idle), the pixels the Newport draws per second, and
  the CPU's speed as a multiple of the real one when busy, measured over
  its recent busy running so that it holds steady while the system idles.
  Guest clocks keep to real time however fast the CPU runs; only if the
  host cannot keep up and they fall a second or more behind does a
  "Behind" warning appear. The **Turbo** button switches a machine running
  with real-time or cycle timing to max timing (below) for a heavy job,
  and back when pressed again; with max or free timing set it is lit and
  cannot be pressed.
- **Mouse:** click in the window to capture the pointer; Ctrl+Alt+G
  (Ctrl+Option+G on a Mac), or switching to another window, releases it.
  All three buttons work. Where the system draws its pointer with the
  Newport's hardware cursor (the IRIX desktop does), simply moving onto the
  screen captures the mouse, with the system's pointer jumping to where the
  host pointer entered (Settings > Mouse, or View > Capture Mouse on Entry,
  sets this to capture on a click only), and
  pushing it against an edge of the screen releases the mouse there, as
  virtual machines do. A click captures it anywhere, with the pointer
  brought to where you clicked.
- **Keys kept by the window:** Ctrl+Alt+G (release the mouse), Ctrl+Alt+P
  (power button), Ctrl+Alt+E (next disc), Ctrl+Alt+F (full screen).
- **Keyboard:** keys are sent by position as a PS/2 keyboard would send
  them, including the numeric keypad and the UK keys `#~` and `\|`; IRIX's
  keyboard layout setting decides what they produce.
- **Console:** with the NVRAM's `console` set to `g` (the PROM's default on
  a machine with graphics), the PROM and IRIX use the screen; set to `d`
  they use serial port 1 (View → Serial Port 1). Choose it in Settings →
  Machine → Console or with `--console`, which write it to the NVRAM at
  power-on, or from the Command Monitor (`setenv console d`). A new NVRAM
  takes the setting from its second power-on, once the PROM has set it up.

### Installing IRIX on a new disk

IRIX CDs carry an SGI volume header and an EFS file system rather than
ISO 9660, so a raw dump of the disc is used as it is. To install:

1. Create a blank (sparse) disk image, e.g. `truncate -s 2G irix.img`.
2. From the PROM's Command Monitor, run `fx` from the CD:
   `boot -f dksc(0,4,8)sashARCS dksc(0,4,7)stand/fx.ARCS --x`, then
   `repartition/rootdrive` and `label/sync`.
3. Choose *Install System Software* from the maintenance menu and install
   from the CD-ROM as on a real Indy.

For a set of discs, such as IRIX 6.5, give `--cdrom` once for each disc, in
the order the installer reads them (Installation Tools first). When the
installer asks for another disc, press Ctrl+Alt+E (or send `SIGUSR1`) until
it is in the drive; the start-up messages (`--log info`) name each disc as
it goes in. The IRIX 6.5 Base Documentation disc is a plain ISO 9660 disc to
browse, not an installer distribution, so the installer cannot open it.

Install with the graphics board present (no `--no-graphics`): IRIX only
installs the Newport kernel driver and X server on a machine where it finds
the board. To add them to a system installed without graphics, start
*Install System Software* again with the board present, install
`eoe1.sw.unix`, `eoe1.sw.gfx_lib` and `x_eoe.sw.Server`, and let it rebuild
the kernel.

With `--no-graphics`, an NVRAM set to `console=g` disables autoboot; choose
*Start System* from the menu, or start with `--console serial`.

### Networking

With `--net user` the Indy sits behind a NAT on 10.0.2.0/24: use 10.0.2.15
for it, 10.0.2.2 (the host) as its gateway and 10.0.2.3 as its name server.
A DHCP server on the NAT hands out these settings, which systems with a DHCP
client (such as NetBSD and OpenBSD) pick up by themselves; otherwise put
10.0.2.2 in `/etc/mygate` for the default route. IRIX is configured by hand,
as root:

```sh
ifconfig ec0 10.0.2.15 netmask 255.255.255.0 up
route add default 10.0.2.2 1
echo nameserver 10.0.2.3 > /etc/resolv.conf
```

These last until the next restart. To keep the address, put it in
`/etc/hosts` under the system's name (`/etc/sys_id`). To keep the default
route, on IRIX 6.5 add this line to `/etc/config/static-route.options`:

```sh
$ROUTE $QUIET add -net default 10.0.2.2
```

IRIX 5.3 has no such file: it runs `/etc/init.d/network.local`, linked as
`/etc/rc2.d/S31network` (see the comments in `/etc/init.d/network`):

```sh
#!/bin/sh
case "$1" in
start) /usr/etc/route add default 10.0.2.2 1 ;;
esac
```

```sh
chmod 755 /etc/init.d/network.local
ln -s ../init.d/network.local /etc/rc2.d/S31network
```

To find where a connection stops, check in order: `netstat -rn` (a
`default` route to 10.0.2.2), `ping 10.0.2.2` (the gateway), `ping 1.1.1.1`
(the way out) and `ping www.sgi.com` (the name server). The same notes are
under *Setting up the guest* in Settings → Network.

Connections from the host reach the guest through forwarded ports, e.g.
`--hostfwd tcp:2323:23` and `telnet localhost 2323`.

The name server at 10.0.2.3 passes queries to the host's first IPv4 name
server; if the host has none (only IPv6 ones), give the guest a public name
server's address instead. On macOS 15 and later, reaching machines on the
local network, the router's name server among them, needs the program's
Local Network permission (System Settings → Privacy & Security → Local
Network: Silicon Ghost, or the terminal it was started from).

With `--net tap:NAME` frames go to an existing TAP interface (created with
`ip tuntap add mode tap user $USER name tap0` and bridged or routed as your
network needs), and the Indy is configured like any machine on that network.

### The BSDs at a glance

| | NetBSD 9.0/sgimips | OpenBSD 6.9/sgi |
|---|---|---|
| Installer boots from the CD | yes (from the Command Monitor) | yes (from the Command Monitor) |
| Installation sets read from the CD | no (see below) | yes |
| Installation over the network (HTTP/FTP) | yes | yes |
| Installed system boots multi-user from disk | yes | yes |
| Serial console | yes | yes |
| Newport (graphics) console | 9.0 and 11.0: no text shown; NetBSD-current: yes | yes, text console |
| PS/2 keyboard | yes | yes |
| PS/2 mouse | detected (not exercised without X) | detected (not exercised without X) |
| X11 | NetBSD-current: yes (see below); 9.0: not tried | no (OpenBSD's Newport driver has no frame buffer for X) |
| SCSI disk and CD-ROM | yes | yes |
| Ethernet | yes | yes |
| Audio playback | yes | no (no driver) |
| Audio recording | no (not supported by NetBSD's driver) | no (no driver) |
| Real-time clock and NVRAM | yes | yes |

NetBSD 11.0's installer boots the same way as 9.0's and attaches the same
devices, its Newport console showing no text either; the rest of the table
was checked on 9.0, apart from what is marked NetBSD-current.

X on NetBSD-current runs with its `newport` driver (twm, xterm, xclock and
the like). Start it with `-depth 8`: at its default depth of 24 the driver
gives the REX3 its fast-clear colour already in the frame buffer's bit
layout, which the REX3 produces itself (IRIX gives it as an RGB colour), so
the root window and other cleared areas show the wrong colours. Give it a
monitor section for 1280x1024
(without one it picks 1024x768). The kernel points the XMAP9's cursor
colour map at entries 0-3, so X's cursor colours overwrite palette entries 1
and 2 and the default white pixel shows as the cursor's black: give clients
explicit colours (`xterm -bg ... -fg ...`).

### Running NetBSD

NetBSD/sgimips 9.0 runs on the serial console (start with `--no-graphics
--console serial`; see the limitations below about its
graphics console). To start its installer from the CD image, from the
Command Monitor:

```
setenv OSLoadPartition dksc(0,4,0)
boot -f dksc(0,4,8)ip2xboot dksc(0,4,0)ip2x
```

(The PROM's default boot file name holds only 28 characters, too few for the
kernel name on the CD, hence the explicit command.) The installer cannot
read the installation sets from this CD on an Indy: its partition is
labelled with a fragment size that NetBSD's ISO 9660 code also takes as a
multi-session offset, real hardware included. Install over the network
instead, with the installer's *HTTP* or *FTP* source, either from a NetBSD
mirror or from the sets copied off the CD and served on the host (reached at
10.0.2.2 with `--net user`). Choose *Configure network* and accept the
automatic configuration.

To boot the installed system:

```
setenv SystemPartition dksc(0,1,8)
setenv OSLoader boot
setenv OSLoadPartition dksc(0,1,0)
setenv OSLoadFilename netbsd
setenv OSLoadOptions auto
setenv AutoLoad Yes
```

Without `OSLoadOptions auto`, NetBSD/sgimips stops in single-user mode
(type `exit` to carry on).

NetBSD's audio driver starts with the output volume at zero, so nothing is
heard until it is raised, e.g. `mixerctl -w outputs.master=200,200`.

### Running OpenBSD

OpenBSD/sgi 6.9 (`install69.iso`) starts its installer from the Command
Monitor:

```
boot -f dksc(0,4,8)bootecof dksc(0,4,0)bsd.rd.IP22
```

(The volume header holds the loader as `bootecof`, names being limited to
eight characters. The automatic boot picks a loader that does not fit an
Indy's memory, as OpenBSD's own notes say.)
The installer reads its sets from the CD (*cd0*, directory `6.9/sgi`), or
over the network (*http*, from an OpenBSD mirror's `pub/OpenBSD/6.9/sgi`).
To boot the installed system:

```
setenv OSLoader boot
setenv OSLoadFilename bsd
setenv AutoLoad Yes
```

The console follows the NVRAM's `console` setting: the Newport text console
(`g`) or the serial port (`d`). OpenBSD's X server does not run on the
Newport (its driver offers no frame buffer to map). OpenBSD has no driver
for the Indy's audio.

On the R10000 Indigo² (`--machine indigo2-r10000`) the loader is the 64-bit
one and the kernel `bsd.rd.IP28`; the CD-ROM is `scsi(0)cdrom(4)`:

```
boot -f scsi(0)cdrom(4)partition(8)boot64 scsi(0)cdrom(4)partition(0)/bsd.rd.IP28
```

The installed system boots as above, with the partitions named in full
(`setenv OSLoadPartition scsi(0)disk(1)rdisk(0)partition(0)` and `setenv
SystemPartition scsi(0)disk(1)rdisk(0)partition(8)`). OpenBSD uses the IMPACT
board as its console only when the PROM has brought it up (console `g`); its
X server does not run there either (no frame buffer to map).

### Timing

The CPU's cycle counter is the master clock for every timer in the machine,
so guest clocks always agree with each other.

- **realtime** (default): guest time follows the wall clock. A fast host
  makes the CPU look faster, while a guest second stays a second.
  While the PROM or a kernel times the CPU at boot, the CPU runs at
  `--timed-speed`. Delay loops (a register counted down, nothing else) run
  at the real CPU's speed, or the timed speed if that is slower: systems
  calibrate their delays by timing one against a timer, and at the real
  speed enough of it is timed for the timer's resolution, so delays come
  out their real length and never short. Everything else runs as fast as
  the host allows, or up to `--max-speed`.
  If the host falls behind (a busy start, a stall), guest time catches up
  afterwards; only a gap of more than 10 seconds, such as the host
  sleeping, is skipped.
- **cycle**: one cycle per instruction, reproducible from run to run.
- **max**: as realtime, with the CPU as fast as the host allows (the timed
  speed at boot and the speed of delay loops stay, keeping the guest's
  delays right, but `--max-speed` is ignored), and with disk and Ethernet
  not held to their real rates: SCSI data moves as fast as the host reads it (each
  command still takes its usual fraction of a millisecond, and by default
  selecting an absent device gives up within a millisecond, which takes
  seconds off the PROM's bus scan), Ethernet carries frames at 1 Gbit/s, holding each until
  the system has a buffer for it instead of dropping it, and the memory
  controller's DMA (with which the PROM clears memory) runs 32 times as fast.
- **free**: no waiting and one cycle per instruction, with guest time
  running ahead of the wall clock; for runs of a set length and
  benchmarks.

## How it works

The CPU is an interpreter that serves as the reference implementation,
with performance layers on top that must give exactly the same results:

- a soft-TLB for fast address translation;
- a cache of decoded blocks, invalidated when code is written by the CPU or
  by DMA;
- a recompiler for hot blocks (x86-64 and AArch64), with guest registers held in host
  registers and direct links between translated blocks;
- exact idle-loop skipping: a loop whose iterations leave the machine in the
  same state is fast-forwarded to the next timer or device event.

[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) has the details.

## Known limitations

- Newport: CID checking (CLIPMODE's CIDMATCH) is not applied; IRIX clears
  the CID planes to one code and clips windows with the screen masks. The
  order of the antialiased-line weight table's entries is assumed (the
  specification does not give it).
- NetBSD 9.0's and 11.0's Newport console shows no text: these releases
  read the board's revision just after a VC2 register access, a read whose
  result on real hardware is not known, and their console then draws
  nothing. A NetBSD-current kernel (11.99), which reads the revision
  directly, runs its console on the board. With 9.0 or 11.0, use the serial
  console.
- IRIX 5.3, IRIX 6.5, NetBSD 9.0 and 11.0 and OpenBSD 6.9 are the systems
  tested so far (on the Indigo², IRIX: its 5.1 PROM loads only ECOFF
  programs, and the NetBSD and OpenBSD boot loaders are ELF); Linux has not been tried. (Current Debian releases no longer include
  Indy support: the Debian 10 MIPS CD carries Malta and Octeon kernels only.)
- The recompiler runs on x86-64 (Linux, Windows) and AArch64 (Linux,
  macOS); elsewhere the slower block cache and interpreter run everything.
  Linux is the host tested most. macOS on Apple silicon has run the IRIX
  desktop. The Windows build is cross-compiled with MinGW-w64 and tested
  under Wine; a native Windows build (MSVC, SDL3, libslirp) has not been
  tried yet.

## Roadmap

- Linux/mips; NetBSD on the graphics console
- Native Windows builds (MSVC or MSYS2 with SDL3 and libslirp)
- IMPACT: the remaining OpenGL details; the IP22 PROM
  has no IMPACT console (IRIX still drives the board)
- Indigo² graphics GR2 (XZ/Extreme)

## Legal

Copyright (C) 2026 Andy Taylor.

SiliconGhost is free software, licensed under the GNU General Public License,
version 2 or (at your option) any later version; see [LICENSE](LICENSE).

It does not include SGI PROM images, IRIX or any other SGI software; users
supply their own. SGI, Indy, IRIX and related names are trademarks of their
respective owners. SiliconGhost is an independent project, not affiliated
with or endorsed by them.

The SiliconGhost logo is &copy; Andy Taylor (MW0MWZ).
