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
4. **Decoded-block cache (in place):** straight-line runs are fetched and
   translated once and cached by host address, so physical aliases share
   them. A per-4 KiB "holds cached code" flag on the bus makes CPU stores and
   device DMA (`Bus::write_block`) drop affected blocks, so self-modifying
   code and freshly loaded programs behave exactly as with the interpreter.
   Each instruction is decoded once into a handler and its register fields;
   uncommon instructions fall back to the interpreter's `execute()`. Blocks
   are linked within a 4 KiB page: when a block ends, execution continues
   directly with the block at the new pc while the mode and the page's
   translation are unchanged.
5. **x86-64 recompiler (first stage, in place):** a block that has run eight
   times is translated into one host function (`src/cpu/r4000/jit_x64.cpp`,
   with a small encoder in `src/jit/`). Register operations and RAM loads
   and stores are emitted inline, the latter with the soft-TLB lookup;
   misses, MMIO, unaligned addresses and pages holding translated code take
   the handler path. Other instructions call their handlers with the same
   bookkeeping as the block loop, and the function returns early wherever
   the block loop would stop, so the two always agree. A block that branches
   back to itself loops inside the function while the next pass fits before
   the next timer or run-limit check. Generated code lives in a buffer
   mapped twice (writable, executable); when it fills, all translations are
   dropped. `--no-jit` turns the recompiler off. Next: branches and jumps
   inline, direct links between translated blocks, keeping guest registers
   in host registers, then an AArch64 back end.
4. **Idle detection:** loops are tracked per backward branch (so nested
   loops are seen too). A loop is a spin when two consecutive iterations end
   with the same registers and stored the same values to the same RAM
   locations, with no other side effect: no device access except reads that
   devices declare pure (status registers that only change at scheduled
   events), no exceptions, CP0 or FPU state changes. Machine state is then
   identical at every visit, so the loop is fast-forwarded in whole
   iterations to just before the next timer or device event, and the guest
   ends up exactly where it would have by spinning. IRIX's idle loop
   qualifies. The PROM's console wait does not: it increments a poll counter
   every pass, so its state never repeats.

## Timing and scheduling

The CPU's cycle counter is the master clock: Count/Compare, the 8254, the RTC
and every device delay derive from it, so guest clocks always agree with each
other. Devices register events on a single scheduler keyed by cycle. The run
loop executes until the next event, then dispatches it. This replaces
per-instruction polling and is what the JIT will exit to. The counter keeps
running through resets.

The counter has fractional precision, so the guest time charged per
instruction need not be one cycle. A pacer relates guest time to the host
clock after every 1 ms slice (`--timing`):

* `realtime` (default): guest time follows the wall clock. The CPU runs as
  many instructions as the host can, and the charge per instruction adapts
  (1/64 to 16 cycles), so a fast host looks like a faster R4400 while a guest
  second remains a second.
* `cycle`: one cycle per instruction, reproducible from run to run; waits
  when ahead of the wall clock and lags when the host is too slow.
* `max`: no waiting, one cycle per instruction (the default with `--max`).

## Front end and host integration

* **SDL3** for window, input, audio and timing (M3). The headless CLI stays for
  tests and CI.
* Disks: raw SCSI disk and CD-ROM images of 512-byte blocks (`--disk`,
  `--cdrom`). IRIX CDs are not ISO 9660: they carry an SGI volume header and
  EFS file system, so a raw dump of the disc is used as-is. A new disk is a
  blank (sparse) file, labelled as on real hardware with `fx` from the CD:
  `boot -f dksc(0,4,8)sashARCS dksc(0,4,7)stand/fx.ARCS --x`, then
  `repartition/rootdrive` and `label/sync`. "Install System Software" then
  copies the miniroot to the swap partition and runs `inst`. Later:
  copy-on-write disk images.
* Networking: Seeq 8003 behind a host backend (TAP on Linux, user-mode slirp
  everywhere).
* Config: TOML machine profiles (RAM size, PROM path, disks, display size).

## Milestones

| | Goal | Done when |
|---|---|---|
| **M0** | Scaffold: bus, R4000 integer interpreter, TLB, tests, CLI | ✅ |
| **M1** | CPU complete: FPU arithmetic (IEEE + unimplemented-op traps), CACHE ops + primary/secondary cache tag model sufficient for PROM diagnostics | ✅ |
| **M2** | Chipset: MC, HPC3, IOC2/INT3, 8254, Z85230, DS1386, EEPROM. Event scheduler | ✅ PROM 5.3 boots to the System Maintenance Menu on the serial console |
| **M3** | Newport + SDL3 front end, 8042 keyboard/mouse | ✅ graphical PROM menu with keyboard input; mouse pending |
| **M4** | WD33C93 SCSI, HPC3 SCSI DMA, disk and CD-ROM images | ✅ IRIX 5.3 installs from its CD and boots multi-user from disk (serial console) |
| **M5** | Seeq ethernet, HAL2 audio, NVRAM persistence | networked IRIX desktop |
| **M6** | Performance: threaded interpreter, fastmem, x86-64 JIT, idle skip | ≥100% of a 150 MHz R4400 Indy |
| **M7** | AArch64 JIT, macOS/Windows packaging; Indigo² profile | |

## Legal

SiliconGhost is GPL-2.0-or-later. It does not ship SGI PROM images or IRIX;
users supply their own dumps and media. GPL-compatible references (Linux,
NetBSD, MAME's GPL-licensed code) may be consulted and adapted with attribution.
