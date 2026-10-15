// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// References: SGI "MC Chip Specification"; Linux arch/mips/include/asm/sgi/mc.h
// and arch/mips/sgi-ip22/ip22-mc.c; NetBSD sys/arch/sgimips/dev/imc.c.
#include "machine/ip22/mc.h"

#include <algorithm>

#include "core/log.h"

namespace sg::ip22 {

namespace {

constexpr u32 kSysIdIndy = 0x03;  // MC revision C, no EISA

constexpr u32 kCpuCtrl0SysInit = 1u << 9;
constexpr u32 kErrStatAddr = 1u << 10;  // CPU: bad address; GIO: timeout uses the same bit position
constexpr u32 kEeromSi = 1u << 4;

// MEMCFG half-word fields
constexpr u32 kBankValid = 1u << 13;
constexpr u32 kBankSubbanks = 1u << 14;

// Main memory may only be decoded into the low and high system memory windows.
constexpr u32 kLowWindow = 0x08000000, kLowWindowEnd = 0x18000000;
constexpr u32 kHighWindow = 0x20000000, kHighWindowEnd = 0x30000000;
constexpr u32 kLowAliasSize = 0x80000;

constexpr u32 kGioSpaceStart = 0x1F000000, kGioSpaceEnd = 0x1FA00000;

}  // namespace

Mc::Mc(Bus& bus, Scheduler& scheduler, u8* ram, u64 ram_size, double cpu_hz, double sysclk_hz)
    : bus_(bus),
      scheduler_(scheduler),
      ram_(ram),
      cycles_per_sysclk_(cpu_hz / sysclk_hz),
      dma_timer_(scheduler, [this] { finish_dma(); }) {
    bus_.add_host_region(ram, ram_size);  // however the banks get mapped, it is one memory
    // Split installed memory into decodable banks, largest first.
    u64 remaining = ram_size, offset = 0;
    for (u64 size = u64{128} << 20; size >= (u64{4} << 20) && banks_.size() < 4;) {
        if (remaining >= size) {
            const bool double_sided = size == (u64{128} << 20) || size == (u64{32} << 20) || size == (u64{8} << 20);
            Bank b;
            b.offset = offset;
            b.subbanks = double_sided ? 2 : 1;
            b.subbank_size = size / b.subbanks;
            banks_.push_back(b);
            offset += size;
            remaining -= size;
        } else {
            size >>= 1;
        }
    }
    if (remaining) SG_LOG(Warn, "mc", "%llu MiB of RAM cannot be expressed as SIMM banks and is ignored",
                          static_cast<unsigned long long>(remaining >> 20));
    reset();
}

void Mc::reset() {
    regs_.clear();
    regs_[kCpuCtrl0] = 0;
    regs_[kCpuCtrl1] = 0;
    regs_[kRpssDivider] = 0x0104;
    regs_[kCtrld] = 0x0C30;
    regs_[kGio64Arb] = 0x1;  // HPC64, from the boot configuration
    regs_[kCpuTime] = 0x1000;
    regs_[kLbTime] = 0x200;
    regs_[kLockMemory] = 1;
    regs_[kEisaLock] = 1;
    regs_[kMemCfg0] = 0;
    regs_[kMemCfg1] = 0;
    cpu_error_addr_ = cpu_error_stat_ = gio_error_addr_ = gio_error_stat_ = 0;
    sys_semaphore_ = 0;
    user_semaphores_.fill(0);
    rpss_base_ = 0;
    rpss_cycle_base_ = scheduler_.now();
    dma_timer_.cancel();
    dma_mem_addr_ = dma_size_ = dma_stride_ = dma_gio_addr_ = dma_mode_ = dma_count_ = 0;
    dma_run_ = dma_ctl_ = dma_cause_ = 0;
    dma_tlb_hi_.fill(0);
    dma_tlb_lo_.fill(0);
    if (dma_irq_cb_) dma_irq_cb_(false);
    remap_memory();
    update_irq();
}

bool Mc::read(u32 offset, unsigned size, u64& value) {
    if (size == 8) {  // doubleword access: the register is the low (odd) word
        value = read_reg(offset + 4);
        return true;
    }
    const u32 word = offset & ~3u;
    if (!(word & 4)) {
        SG_LOG(Debug, "mc", "read of little-endian register address +%05x", word);
        value = 0;
        return true;
    }
    const u32 v = read_reg(word);
    const unsigned shift = (4 - size - (offset & 3)) * 8;
    value = (v >> shift) & (size == 4 ? 0xFFFFFFFFu : (1u << (size * 8)) - 1);
    return true;
}

bool Mc::write(u32 offset, unsigned size, u64 value) {
    if (size == 8) {
        write_reg(offset + 4, static_cast<u32>(value));
        return true;
    }
    const u32 word = offset & ~3u;
    if (!(word & 4)) {
        SG_LOG(Debug, "mc", "write to little-endian register address +%05x", word);
        return true;
    }
    if (size != 4) {  // sub-word write: merge into the register
        const unsigned shift = (4 - size - (offset & 3)) * 8;
        const u32 mask = ((1u << (size * 8)) - 1) << shift;
        const u32 old = regs_.count(word) ? regs_[word] : 0;
        value = (old & ~mask) | ((static_cast<u32>(value) << shift) & mask);
    }
    write_reg(word, static_cast<u32>(value));
    return true;
}

u32 Mc::read_reg(u32 reg) {
    if (reg >= kUserSemaphores && reg < kUserSemaphores + 0x10000 && (reg & 0xFFF) == 4) {
        u32& sem = user_semaphores_[(reg - kUserSemaphores) >> 12];
        const u32 v = sem;
        sem = 1;  // reading acquires
        return v;
    }
    switch (reg) {
        case kSysId: return kSysIdIndy;
        case kDogc: return 0;
        case kRefCtr: return regs_[kCtrld];
        case kEerom: return regs_[kEerom] & ~kEeromSi;
        case kCpuErrorAddr: return cpu_error_addr_;
        case kCpuErrorStat: return cpu_error_stat_;
        case kGioErrorAddr: return gio_error_addr_;
        case kGioErrorStat: return gio_error_stat_;
        case kSysSemaphore: {
            const u32 v = sys_semaphore_;
            sys_semaphore_ = 1;
            return v;
        }
        case kRpssCounter: return rpss_counter();
        case kDmaCause: return dma_cause_;
        case kDmaCtl: return dma_ctl_;
        case kDmaMemAdr:
        case kDmaMemAdrD: return dma_mem_addr_;
        case kDmaSize: return dma_size_;
        case kDmaStride: return dma_stride_;
        case kDmaGioAdr:
        case kDmaGioAdrS: return dma_gio_addr_;
        case kDmaMode: return dma_mode_;
        case kDmaCount: return dma_count_;
        case kDmaStart: return 0;
        case kDmaRun: return dma_run_;
        default: {
            const auto it = regs_.find(reg);
            if (it == regs_.end()) {
                SG_LOG(Debug, "mc", "read of unknown register +%05x", reg);
                return 0;
            }
            return it->second;
        }
    }
}

void Mc::write_reg(u32 reg, u32 value) {
    if (reg >= kUserSemaphores && reg < kUserSemaphores + 0x10000 && (reg & 0xFFF) == 4) {
        user_semaphores_[(reg - kUserSemaphores) >> 12] = value & 1;
        return;
    }
    switch (reg) {
        case kCpuCtrl0:
            regs_[reg] = value & ~kCpuCtrl0SysInit;
            if (value & kCpuCtrl0SysInit) {
                SG_LOG(Info, "mc", "system reset requested");
                if (reset_cb_) reset_cb_();
            }
            return;
        case kDmaCause:
            dma_cause_ = value;
            if (dma_cause_ == 0 && dma_irq_cb_) dma_irq_cb_(false);
            return;
        case kDmaCtl: dma_ctl_ = value; return;
        case kDmaMemAdr:
        case kDmaMemAdrD: dma_mem_addr_ = value; return;
        case kDmaSize:
            dma_size_ = value;
            dma_count_ = (dma_count_ & 0xFFFF0000u) | (value & 0xFFFF);
            return;
        case kDmaStride:
            dma_stride_ = value;
            dma_count_ = (dma_count_ & 0x0000FFFFu) | (value & 0x03FF0000u);
            return;
        case kDmaGioAdr: dma_gio_addr_ = value; return;
        case kDmaGioAdrS:
            dma_gio_addr_ = value;
            start_dma();
            return;
        case kDmaMode: dma_mode_ = value; return;
        case kDmaCount: dma_count_ = value; return;
        case kDmaStart:
            if (value & 1) start_dma();
            return;
        case kDmaMemAdrDS:
            dma_gio_addr_ = value;
            dma_size_ = 0x0001000C;
            dma_stride_ = 0x00010000;
            dma_count_ = 0x0001000C;
            dma_mode_ = 0x00000028;
            start_dma();
            return;
        case kSysId:
        case kRefCtr:
        case kRpssCounter:
        case kDmaRun:
        case kCpuErrorAddr:
        case kGioErrorAddr: return;  // read-only
        case kDogc: return;           // any write clears the watchdog (never modelled to fire)
        case kRpssDivider:
            rebase_rpss();
            regs_[reg] = value & 0xFFFF;
            return;
        case kMemCfg0:
        case kMemCfg1:
            regs_[reg] = value & 0x7FFF7FFF;
            remap_memory();
            return;
        case kCpuErrorStat:
            cpu_error_stat_ = 0;
            cpu_error_addr_ = 0;
            update_irq();
            return;
        case kGioErrorStat:
            gio_error_stat_ = 0;
            gio_error_addr_ = 0;
            update_irq();
            return;
        case kSysSemaphore: sys_semaphore_ = value; return;
        default:
            if (reg >= kDmaTlbHi0 && reg < kDmaTlbHi0 + 0x40 && ((reg - kDmaTlbHi0) & 7) == 0) {
                const unsigned i = (reg - kDmaTlbHi0) >> 4;
                ((reg - kDmaTlbHi0) & 8 ? dma_tlb_lo_ : dma_tlb_hi_)[i] = value;
                return;
            }
            if (!regs_.count(reg)) SG_LOG(Debug, "mc", "write to unknown register +%05x = %08x", reg, value);
            regs_[reg] = value;
            return;
    }
}

// RPSS counts INC every DIV+1 system clocks (100 ns units when programmed per spec).
u32 Mc::rpss_counter() const {
    const u32 div = regs_.at(kRpssDivider);
    const u64 sysclks = static_cast<u64>(static_cast<double>(scheduler_.now() - rpss_cycle_base_) / cycles_per_sysclk_);
    return rpss_base_ + static_cast<u32>(sysclks / ((div & 0xFF) + 1)) * ((div >> 8) & 0xFF);
}

void Mc::rebase_rpss() {
    rpss_base_ = rpss_counter();
    rpss_cycle_base_ = scheduler_.now();
}

void Mc::remap_memory() {
    for (const auto& [base, size] : mapped_) bus_.unmap(base, size);
    mapped_.clear();
    bus_.unmap(0, kLowAliasSize);

    for (unsigned bank = 0; bank < 4; ++bank) {
        const u32 memcfg = regs_[bank < 2 ? kMemCfg0 : kMemCfg1];
        const u32 field = (bank & 1) ? (memcfg & 0xFFFF) : (memcfg >> 16);
        const Bank fitted = bank < banks_.size() ? banks_[bank] : Bank{};
        if (!(field & kBankValid)) continue;

        const u32 base = (field & 0xFF) << 22;
        const u64 size = static_cast<u64>(((field >> 8) & 0x1F) + 1) << 22;
        const bool in_low = base >= kLowWindow && base + size <= kLowWindowEnd;
        const bool in_high = base >= kHighWindow && base + size <= kHighWindowEnd;
        if (!in_low && !in_high) {
            SG_LOG(Warn, "mc", "bank %u configured outside the memory windows (%08x + %llu MiB); ignored", bank,
                   base, static_cast<unsigned long long>(size >> 20));
            continue;
        }
        // The configured range is split into one or two sub-banks (BNK). A
        // sub-bank with SIMMs repeats them when configured larger (unused
        // address lines); a sub-bank without SIMMs is decoded but undriven.
        const unsigned configured_subbanks = (field & kBankSubbanks) ? 2 : 1;
        const u64 part = size / configured_subbanks;
        for (unsigned s = 0; s < configured_subbanks; ++s) {
            const u64 part_base = base + s * part;
            if (s >= fitted.subbanks) {
                bus_.map_device(static_cast<u32>(part_base), part, &empty_bank_);
                mapped_.emplace_back(static_cast<u32>(part_base), part);
                continue;
            }
            u8* const host = ram_ + fitted.offset + s * fitted.subbank_size;
            for (u64 off = 0; off < part; off += fitted.subbank_size) {
                const u64 chunk = std::min(fitted.subbank_size, part - off);
                bus_.map_ram(static_cast<u32>(part_base + off), chunk, host);
                mapped_.emplace_back(static_cast<u32>(part_base + off), chunk);
            }
        }
        SG_LOG(Debug, "mc", "bank %u: %llu MiB at %08x as %u sub-bank(s); fitted %u x %llu MiB", bank,
               static_cast<unsigned long long>(size >> 20), base, configured_subbanks, fitted.subbanks,
               static_cast<unsigned long long>(fitted.subbank_size >> 20));
    }
    // The first 512 KiB of whatever is decoded at 0x08000000 also appear at 0.
    if (u8* low = bus_.host_write_ptr(kLowWindow)) bus_.map_ram(0, kLowAliasSize, low);
}

void Mc::report_bus_error(u64 paddr, unsigned size, bool is_write) {
    u32 lanes = 0;
    for (unsigned i = 0; i < size; ++i) lanes |= 1u << (7 - ((paddr + i) & 7));
    const bool gio = paddr >= kGioSpaceStart && paddr < kGioSpaceEnd;
    if (gio) {
        if (!gio_error_stat_) {
            gio_error_addr_ = static_cast<u32>(paddr);
            gio_error_stat_ = kErrStatAddr | lanes;  // GIO timeout
        }
    } else if (!cpu_error_stat_) {
        cpu_error_addr_ = static_cast<u32>(paddr);
        cpu_error_stat_ = kErrStatAddr | lanes;
    }
    // Reads fault synchronously in the CPU; writes are posted and reported by interrupt.
    if (is_write) {
        irq_ = true;
        if (irq_cb_) irq_cb_(true);
    }
}

void Mc::update_irq() {
    const bool pending = irq_ && (cpu_error_stat_ || gio_error_stat_);
    if (!pending && irq_) {
        irq_ = false;
        if (irq_cb_) irq_cb_(false);
    }
}

// ---------------------------------------------------------------------------
// Virtual DMA: moves rectangular memory areas to or from a GIO64 address, or
// fills memory with a constant. The GIO address stays fixed: it names a
// device's data port (such as REX3's host data register with GO), which
// takes every doubleword in turn. Memory may start at any byte; IRIX sets
// the GIO address's low bits to match, but a doubleword on GIO64 has no
// byte address, so the device sees the aligned address. Memory addresses go
// through a 4-entry TLB of 4 MiB segments whose PTEs live in main memory;
// addresses no entry covers are used as physical addresses (the boot PROM
// clears memory this way).
// ---------------------------------------------------------------------------

namespace {
constexpr u32 kDmaModeToHost = 1u << 1;
constexpr u32 kDmaModeFill = 1u << 3;
constexpr u32 kDmaModeIncrement = 1u << 4;
constexpr u32 kDmaRunActive = 1u << 6;
constexpr u32 kDmaRunDone = 1u << 3;
constexpr u32 kDmaCtlIntEnable = 1u << 4;
constexpr u32 kDmaCauseDone = 1u << 3;
}  // namespace

u32 Mc::dma_translate(u32 vaddr) {
    for (unsigned i = 0; i < 4; ++i) {
        if (!dma_tlb_hi_[i] && !dma_tlb_lo_[i]) continue;
        if ((vaddr & 0xFFC00000u) != (dma_tlb_hi_[i] & 0xFFC00000u)) continue;
        u32 pte = 0;
        bus_.read<u32>(((dma_tlb_lo_[i] & 0x03FFFFC0u) << 6) + (((vaddr >> 12) & 0x3FF) << 2), pte);
        return ((pte & 0x03FFFFC0u) << 6) + (vaddr & 0xFFF);
    }
    return vaddr;
}

// The transfer is timed as one 64-bit word per system clock and its effect
// is applied when it completes.
void Mc::start_dma() {
    dma_run_ = (dma_run_ & ~kDmaRunDone) | kDmaRunActive;
    const u64 lines = dma_size_ >> 16, width = dma_size_ & 0xFFFF, zoom = std::max<u64>(1, (dma_stride_ >> 16) & 0x3FF);
    const u64 words = std::max<u64>(1, lines * zoom * width / 8);
    dma_timer_.schedule_in(static_cast<u64>(static_cast<double>(words) * cycles_per_sysclk_) + 1);
}

void Mc::finish_dma() {
    const u32 lines = dma_size_ >> 16;
    const u32 width = dma_size_ & 0xFFFF;
    const u32 zoom = (dma_stride_ >> 16) & 0x3FF;
    const s32 stride = static_cast<s16>(dma_stride_);
    const bool to_host = dma_mode_ & kDmaModeToHost, fill = dma_mode_ & kDmaModeFill;
    const s32 step = (dma_mode_ & kDmaModeIncrement) ? 1 : -1;
    u32 addr = dma_mem_addr_;
    const u64 gio = dma_gio_addr_ & ~u64{7};
    for (u32 line = 0; line < lines; ++line) {
        for (u32 z = 0; z < std::max(zoom, 1u); ++z) {
            u32 a = addr;
            if (fill && to_host) {
                for (u32 n = 0; n < width; n += 4, a += static_cast<u32>(4 * step))
                    bus_.write<u32>(dma_translate(a), dma_gio_addr_);
            } else if (to_host) {
                for (u32 n = 0; n < width; n += 8) {
                    u64 data = 0;
                    bus_.read<u64>(gio, data);
                    for (u32 b = 0; b < 8 && n + b < width; ++b, a += static_cast<u32>(step))
                        bus_.write<u8>(dma_translate(a), static_cast<u8>(data >> (56 - 8 * b)));
                }
            } else {
                for (u32 n = 0; n < width; n += 8) {
                    u64 data = 0;
                    for (u32 b = 0; b < 8 && n + b < width; ++b, a += static_cast<u32>(step)) {
                        u8 v = 0;
                        bus_.read<u8>(dma_translate(a), v);
                        data |= u64{v} << (56 - 8 * b);
                    }
                    bus_.write<u64>(gio, data);
                }
            }
            if (z + 1 == std::max(zoom, 1u)) addr = a;
        }
        addr += static_cast<u32>(stride);
    }
    dma_mem_addr_ = addr;
    dma_run_ = (dma_run_ & ~kDmaRunActive) | kDmaRunDone;
    if (dma_ctl_ & kDmaCtlIntEnable) {
        dma_cause_ |= kDmaCauseDone;
        if (dma_irq_cb_) dma_irq_cb_(true);
    }
}

}  // namespace sg::ip22
