// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// SGI IP22 (Indy "Guinness" / Indigo2 "Fullhouse") physical memory map.
// Sources: SGI MC chip specification; Linux arch/mips/include/asm/sgi/*.h.
#pragma once

#include "core/types.h"

namespace sg::ip22 {

constexpr u32 kLowMemAlias = 0x00000000;      // first 512 KiB of bank 0 aliased here
constexpr u32 kLowMemAliasSize = 0x00080000;
constexpr u32 kRamBase = 0x08000000;          // low system memory window (256 MiB)
constexpr u32 kRamWindowSize = 0x10000000;
constexpr u32 kRamHighBase = 0x20000000;      // high system memory window (256 MiB)

constexpr u32 kGioGfx = 0x1F000000;           // GIO64 graphics slot
constexpr u32 kGioGfxSize = 0x00400000;
constexpr u32 kNewportRex3 = 0x1F0F0000;      // REX3 registers (Newport on the GFX slot)
constexpr u32 kGioExp0 = 0x1F400000;          // GIO expansion slot 0 (2 MiB)
constexpr u32 kGioExp0Size = 0x00200000;
constexpr u32 kGioExp1 = 0x1F600000;          // GIO expansion slot 1 (4 MiB)
constexpr u32 kGioExp1Size = 0x00400000;

constexpr u32 kMc = 0x1FA00000;               // Memory Controller (MC), user semaphores at +0x10000
constexpr u32 kMcSize = 0x00020000;

constexpr u32 kHpc3 = 0x1FB80000;             // HPC3 #0: DMA, SCSI, ethernet, PBUS
constexpr u32 kHpc3Size = 0x00080000;
// Within HPC3 space:
constexpr u32 kHpc3Wd33c93 = 0x1FBC0000;      // WD33C93 SCSI controller (SASR +3, data +7)
constexpr u32 kHpc3Seeq = 0x1FBD4000;         // Seeq 8003 ethernet
constexpr u32 kHpc3Hal2 = 0x1FBD8000;         // HAL2 audio
constexpr u32 kHpc3Ioc2 = 0x1FBD9800;         // IOC2: Z85230 serial, 8042 kbd/mouse, INT3, sysid
constexpr u32 kHpc3Int3 = 0x1FBD9880;         // INT3 interrupt controller + 8254 (inside IOC2)
constexpr u32 kHpc3Rtc = 0x1FBE0000;          // DS1386 RTC; NVRAM byte n at 0x1FBE0100 + 4n

constexpr u32 kProm = 0x1FC00000;             // boot PROM (512 KiB on Indy)
constexpr u32 kPromMaxSize = 0x00080000;

// R4600SC module: secondary cache control, uncached, at 0x80000000 | the
// physical address of the line (the module decodes the top address bit).
constexpr u32 kR4600ScControl = 0x80000000;
constexpr u64 kR4600ScControlSize = 0x80000000;

}  // namespace sg::ip22
