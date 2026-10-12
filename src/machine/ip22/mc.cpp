// SPDX-License-Identifier: GPL-2.0-or-later
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

// Main memory may only be decoded into the low and high system memory windows.
constexpr u32 kLowWindow = 0x08000000, kLowWindowEnd = 0x18000000;
constexpr u32 kHighWindow = 0x20000000, kHighWindowEnd = 0x30000000;
constexpr u32 kLowAliasSize = 0x80000;

constexpr u32 kGioSpaceStart = 0x1F000000, kGioSpaceEnd = 0x1FA00000;

}  // namespace

Mc::Mc(Bus& bus, Scheduler& scheduler, u8* ram, u64 ram_size, double cpu_hz, double sysclk_hz)
    : bus_(bus), scheduler_(scheduler), ram_(ram), cycles_per_sysclk_(cpu_hz / sysclk_hz) {
    // Split installed memory into decodable banks, largest first.
    u64 remaining = ram_size;
    for (u64 size = u64{128} << 20; size >= (u64{4} << 20) && bank_sizes_.size() < 4;) {
        if (remaining >= size) {
            bank_sizes_.push_back(size);
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
        case kDmaRun: return 0;
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

    u64 bank_offset = 0;
    for (unsigned bank = 0; bank < 4; ++bank) {
        const u32 memcfg = regs_[bank < 2 ? kMemCfg0 : kMemCfg1];
        const u32 field = (bank & 1) ? (memcfg & 0xFFFF) : (memcfg >> 16);
        const u64 installed = bank < bank_sizes_.size() ? bank_sizes_[bank] : 0;
        u8* const host = ram_ + bank_offset;
        bank_offset += installed;
        if (!(field & kBankValid) || installed == 0) continue;

        const u32 base = (field & 0xFF) << 22;
        const u64 size = static_cast<u64>(((field >> 8) & 0x1F) + 1) << 22;
        const bool in_low = base >= kLowWindow && base + size <= kLowWindowEnd;
        const bool in_high = base >= kHighWindow && base + size <= kHighWindowEnd;
        if (!in_low && !in_high) {
            SG_LOG(Warn, "mc", "bank %u configured outside the memory windows (%08x + %llu MiB); ignored", bank,
                   base, static_cast<unsigned long long>(size >> 20));
            continue;
        }
        // A bank configured larger than the SIMMs fitted mirrors them; smaller
        // shows only the configured part.
        for (u64 off = 0; off < size; off += installed) {
            const u64 chunk = std::min(installed, size - off);
            bus_.map_ram(static_cast<u32>(base + off), chunk, host);
            mapped_.emplace_back(static_cast<u32>(base + off), chunk);
        }
        SG_LOG(Debug, "mc", "bank %u: %llu MiB at %08x (%llu MiB fitted)", bank,
               static_cast<unsigned long long>(size >> 20), base, static_cast<unsigned long long>(installed >> 20));
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

}  // namespace sg::ip22
