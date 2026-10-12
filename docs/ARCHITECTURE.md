# SiliconGhost architecture

SiliconGhost is an emulator for the SGI **Indy (IP22)**: MIPS R4000/R4400 CPU,
the MC/HPC3/IOC2 chipset and **Newport (REX3)** graphics. It aims to run the
original boot PROM and IRIX 5.3/6.5 unmodified at **≥100% of native speed**.
Linux is the primary host; macOS and Windows are supported targets.

> Why Indy and not Indigo: Newport/REX3 only exists on IP22 machines (Indy and
> Indigo²). The R4000 Indigo (IP20) used Express (Elan/XS/XZ) or LG1/LG2
> graphics. Indigo² (IP22 "Fullhouse") is a small follow-on target: same CPU
> board logic plus EISA and a GIO64 Newport XL.

## Guiding principles

1. **Correct first, fast second.** The interpreter is the reference
   implementation; every performance feature (soft-TLB, fastmem, JIT) must
   produce exactly the same architectural results.
2. **Guest memory stays in guest byte order** (big-endian). DMA, PROM and disk
   images need no conversion; loads/stores do one `bswap` (free on every host).
3. **No global state.** A `Machine` owns its bus, memory, CPU and devices, so
   tests can build many small machines and, later, save states are tractable.
4. **Portable core, platform code at the edges.** The emulation core is plain
   C++20 with no dependencies; SDL3, networking (TAP/slirp) and JIT memory
   management live behind small interfaces.

## Source layout

```
src/core/          types, endian helpers, 128-bit multiply, logging
src/mem/           physical bus (64 KiB page table → host RAM or Device)
src/cpu/r4000/     R4000/R4400: interpreter, COP0/TLB, disassembler (JIT later: src/cpu/r4000/jit/)
src/devices/       chipset and peripheral models (stub device for now)
src/machine/ip22/  IP22 memory map and Indy machine assembly
src/app/           command-line front end (SDL3 front end in M3)
tests/             dependency-free unit tests + hand assembler
docs/              this document, hardware notes
```

## CPU: R4000/R4400

### Interpreter (implemented)

* Full MIPS III integer ISA including 64-bit ops, `LWL/LWR/LDL/LDR/SWL/SWR/SDL/SDR`,
  `LL/SC/LLD/SCD`, traps and branch-likely nullification.
* Precise exceptions with branch-delay tracking (`EPC`/`Cause.BD`), BEV and
  normal vectors, TLB refill vs XTLB refill vs general vector selection.
* COP0: all architectural registers, 48-entry joint TLB with variable page
  size, `TLBR/TLBWI/TLBWR/TLBP/ERET`, Count/Compare timer on IP7, external
  interrupt lines on IP2–IP6.
* 32- and 64-bit addressing modes (kuseg/kseg0/1/ksseg/kseg3, xkuseg/xksseg/
  xkphys/xkseg, user/supervisor variants) with address-error checking.
* COP1: register moves, CFC1/CTC1 and FP loads/stores honour `Status.FR`.

* COP1: full R4000 FPU. Arithmetic runs on the host FPU under the guest
  rounding mode; denormal and quiet-NaN operands, tiny results and
  out-of-range integer conversions raise Unimplemented Operation exactly as
  the R4000 does, so the operating system's software completion path runs.
* `CACHE`: primary and secondary tag arrays (no cached data, so memory is
  always coherent) with index/hit/create-dirty/fill operations and Status.CH.

**Not yet:** watchpoints, reverse-endian user mode, NMI/soft reset,
pipeline timing (1 instruction = 1 cycle; Count = cycles/2).

### Memory access path

```
vaddr ──► soft-TLB (per mode × {read, write}, 256 entries, 4 KiB tags)
            hit + host ptr ──► load_be/store_be on host memory       (fast path)
            hit + MMIO     ──► Bus::read/write ──► Device virtual call
            miss           ──► segment decode ──► R4000 TLB search ──► fill
```

The soft-TLB is flushed on TLB writes, ASID changes and changes to
`Status.KX/SX/UX`; ERL gets its own table because it changes kuseg semantics.
Write-table entries are only filled through a successful store translation, so
the TLB `D` bit and ROM read-only status are enforced without per-access checks.

### Road to ≥100% native speed

A 150 MHz R4400 Indy delivers roughly 100–150 MIPS on real code (cache misses
included). A plain interpreter can approach that on simple code; IRIX
workloads (TLB-heavy, lots of MMIO, FP) need more:

1. **Interpreter tuning:** threaded dispatch with a predecoded instruction
   cache per physical page; fetch-page pointer caching; device-event scheduler
   instead of per-step checks.
2. **Fastmem:** reserve 4 GiB of host address space per address-space view and
   map guest RAM pages directly, using host page faults for MMIO/TLB misses
   (`mmap`/`mprotect` + SIGSEGV on Linux/macOS, `VirtualAlloc2` + VEH on
   Windows). That removes the soft-TLB compare from JIT code.
3. **JIT (dynarec):** basic-block → superblock translation with a
   platform-neutral IR, register allocation of hot GPRs, and back ends for
   **x86-64** first and then **AArch64**. On macOS/Apple Silicon this means
   `MAP_JIT` + `pthread_jit_write_protect_np`; on Linux/Windows, dual-mapped
   W^X code buffers. Self-modifying code is handled by write-protecting
   translated physical pages.
4. **Idle detection:** IRIX's idle loop and the PROM's polling loops skip ahead
   to the next scheduled event, so host CPU use stays low.

## Physical bus

`Bus` is a flat array of 65,536 entries (64 KiB pages over 4 GiB). Each entry
points at host memory (RAM/ROM, with a writable flag) or at a `Device` plus
its base. Aligned accesses never cross a page, so the fast path is one table
lookup. Devices see offsets relative to their mapping and already-resolved
integers. Physical addresses ≥ 4 GiB are bus errors (IP22 never uses them).

## IP22 (Indy) hardware model

Physical map (see `src/machine/ip22/memmap.h`; addresses marked VERIFY there
must be checked against Linux `arch/mips/sgi-ip22`, NetBSD `sgimips` and MAME
before device models depend on them):

| Range | Device | Status |
|---|---|---|
| `0000_0000–0007_FFFF` | Low-memory alias of first 512 KiB of RAM | done |
| `0800_0000–17FF_FFFF`, `2000_0000–2FFF_FFFF` | Main memory (16–256 MiB), decoded from MC MEMCFG bank registers | done |
| `1F00_0000–1F3F_FFFF` | GIO64 graphics slot. REX3 at `1F0F_0000` | not yet (slot reads as empty) |
| `1F40_0000`, `1F60_0000` | GIO expansion slots 0/1 | empty (bus error) |
| `1FA0_0000–1FA1_FFFF` | MC: control, SYSID, MEMCFG, error latches, RPSS, semaphores | done (virtual DMA: storage only) |
| `1FB8_0000–1FBF_FFFF` | HPC3: DMA channels, EEPROM, PBUS. IOC2/INT3/8254/Z85230/8042 at `1FBD_9800`, DS1386 at `1FBE_0000` | IOC2, INT3, timers, serial, keyboard controller, EEPROM, RTC/NVRAM done; DMA, SCSI, Ethernet, audio: storage only |
| `1FC0_0000–1FC7_FFFF` | Boot PROM (512 KiB) | done |

Planned device models, in the order the PROM needs them:

* **MC**: CPUCTRL0/1, SYSID, MEMCFG0/1 (bank decode drives `Bus` RAM mapping),
  RPSS counter, refresh, GIO/CPU error address/status (bus-error interrupt
  policy for unmapped writes), virtual DMA engine.
* **HPC3**: PBUS PIO/DMA channels, SCSI DMA (with **WD33C93A**), Ethernet DMA
  (with **Seeq 8003**), EEPROM serial interface (NVRAM/serial number), interrupt
  routing.
* **IOC2/INT3**: local interrupt status/mask → CPU Int0..Int4, 8254 PIT
  (timers 0/1 drive IP4/IP5 on IP22), **Z85230** SCC (serial console, the first
  thing we want working), **8042** keyboard/mouse.
* **DS1386** RTC + NVRAM (PROM environment variables).
* **HAL2** audio (late).
* **Newport**: REX3 (drawing engine), VC2 (video timing, cursor), XMAP9
  (pixel modes), CMAP (colour maps), RB2 (frame buffer), plus Bt445 RAMDAC.

## Graphics: Newport / REX3

Newport is a fixed-function 2D raster engine (lines, spans, blocks, patterns,
logic ops, Z is not present), driven by REX3 register writes. Plan:

* Model the frame buffer (1280×1024, 8-bit or 24-bit RGB configurations; XL8
  first) as host memory **owned by the emulation core**.
* Implement REX3 draw commands on the CPU first, with bulk fast paths for
  block fills and screen-to-screen copies (which dominate X11 under IRIX).
* Display: VC2 timing + XMAP9/CMAP translate the frame buffer to RGBA once per
  host frame (dirty-rectangle tracked), uploaded as an SDL3 texture. CMAP
  lookups move to a fragment shader if profiling says so.
* Software emulation of a 2D engine at 1280×1024 is far cheaper than
  emulating the CPU, so a GPU REX3 is not planned unless measured necessary.

## Timing and scheduling

The CPU is the master clock (`cycles`). Devices register events on a single
scheduler (min-heap keyed by CPU cycle). The run loop executes until the next
event, then dispatches it. This replaces per-instruction polling and is what
the JIT will exit to. Wall-clock pacing (for 100% speed rather than "as fast as
possible") throttles at frame boundaries.

## Front end and host integration

* **SDL3** for window, input, audio and timing (M3). The headless CLI stays for
  tests and CI.
* Disks: raw SCSI disk/CD images (`.img`, `.iso`); later CHD or qcow2-style
  sparse images.
* Networking: Seeq 8003 behind a host backend (TAP on Linux, user-mode slirp
  everywhere).
* Config: TOML machine profiles (RAM size, PROM path, disks, display size).

## Milestones

| | Goal | Done when |
|---|---|---|
| **M0** | Scaffold: bus, R4000 integer interpreter, TLB, tests, CLI | ✅ |
| **M1** | CPU complete: FPU arithmetic (IEEE + unimplemented-op traps), CACHE ops + primary/secondary cache tag model sufficient for PROM diagnostics | ✅ |
| **M2** | Chipset: MC, HPC3, IOC2/INT3, 8254, Z85230, DS1386, EEPROM. Event scheduler | PROM prints to serial console and reaches the command monitor |
| **M3** | Newport + SDL3 front end, 8042 keyboard/mouse | graphical PROM menu displays and accepts input |
| **M4** | WD33C93A SCSI + disk/CD images | IRIX 5.3/6.5 boots from disk image; install from CD |
| **M5** | Seeq ethernet, HAL2 audio, NVRAM persistence | networked IRIX desktop |
| **M6** | Performance: threaded interpreter, fastmem, x86-64 JIT, idle skip | ≥100% of a 150 MHz R4400 Indy |
| **M7** | AArch64 JIT, macOS/Windows packaging; Indigo² profile | |

## Legal

SiliconGhost is GPL-2.0-or-later. It does not ship SGI PROM images or IRIX;
users supply their own dumps and media. GPL-compatible references (Linux,
NetBSD, MAME's GPL-licensed code) may be consulted and adapted with attribution.
