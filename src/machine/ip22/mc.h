// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// IP22 MC (memory controller): CPU/GIO control, SIMM bank decode, bus error
// reporting, RPSS counter and semaphores. Mapped at 0x1FA00000 (128 KiB).
//
// Registers are 32 bits wide on 8-byte strides; a big-endian CPU uses the
// odd word, so register R lives at 0x1FA00000 + R (R ends in 4 or C).
#pragma once

#include <array>
#include <functional>
#include <unordered_map>
#include <vector>

#include "core/scheduler.h"
#include "devices/eeprom93cxx.h"
#include "mem/bus.h"

namespace sg::ip22 {

class Mc final : public Device {
public:
    // Register offsets (big-endian word addresses relative to the MC base).
    enum Reg : u32 {
        kCpuCtrl0 = 0x004,
        kCpuCtrl1 = 0x00C,
        kDogc = 0x014,
        kSysId = 0x01C,
        kRpssDivider = 0x02C,
        kEerom = 0x034,
        kCtrld = 0x044,
        kRefCtr = 0x04C,
        kGio64Arb = 0x084,
        kCpuTime = 0x08C,
        kLbTime = 0x09C,
        kMemCfg0 = 0x0C4,
        kMemCfg1 = 0x0CC,
        kCpuMemAcc = 0x0D4,
        kGioMemAcc = 0x0DC,
        kCpuErrorAddr = 0x0E4,
        kCpuErrorStat = 0x0EC,
        kGioErrorAddr = 0x0F4,
        kGioErrorStat = 0x0FC,
        kSysSemaphore = 0x104,
        kLockMemory = 0x10C,
        kEisaLock = 0x114,
        kDmaGioMask = 0x154,
        kDmaGioSubst = 0x15C,
        kDmaCause = 0x164,
        kDmaCtl = 0x16C,
        kDmaTlbHi0 = 0x184,  // + n * 0x10; LO at +8
        kRpssCounter = 0x1004,
        kDmaMemAdr = 0x2004,
        kDmaMemAdrD = 0x200C,
        kDmaSize = 0x2014,
        kDmaStride = 0x201C,
        kDmaGioAdr = 0x2024,
        kDmaGioAdrS = 0x202C,
        kDmaMode = 0x2034,
        kDmaCount = 0x203C,
        kDmaStart = 0x2044,
        kDmaRun = 0x204C,
        kDmaMemAdrDS = 0x2074,
        kUserSemaphores = 0x10004,  // + n * 0x1000, n = 0..15
    };

    // `ram` holds all installed SIMM memory; it is split into banks of the
    // sizes the MC can decode (4..128 MiB, largest first, at most 4 banks).
    // 128, 32 and 8 MiB banks are built from double-sided SIMMs (two
    // sub-banks); 64, 16 and 4 MiB banks from single-sided ones.
    // `eisa`: the board has an EISA bus (the Indigo2), reported in SYSID.
    // `large_banks`: the later MC of the R8000 and R10000 Indigo2 boards
    // (revision 5): bank registers count in 16 MiB rather than 4 MiB, the
    // high memory window is 512 MiB, and banks of 64, 128 and 256 MiB are
    // fitted (256 MiB double-sided).
    Mc(Bus& bus, Scheduler& scheduler, u8* ram, u64 ram_size, double cpu_hz, double sysclk_hz, bool eisa = false,
       bool large_banks = false);
    // A device the board decodes through a bank register, at `base` (the
    // IP28's ECC board, mapped by a bank at 0x60000000 to reach its register).
    void set_bank_device(u32 base, u64 size, Device* device) {
        bank_device_ = device;
        bank_device_base_ = base;
        bank_device_size_ = size;
    }

    void reset();
    const char* name() const override { return "mc"; }
    bool read(u32 offset, unsigned size, u64& value) override;
    bool write(u32 offset, unsigned size, u64 value) override;

    // Called by the bus for accesses nobody answered.
    void report_bus_error(u64 paddr, unsigned size, bool is_write);

    // IP6: asynchronous bus error interrupt.
    void set_irq_callback(std::function<void(bool)> cb) { irq_cb_ = std::move(cb); }
    // DMA-done interrupt (INT3 LOCAL0 bit 4).
    void set_dma_irq_callback(std::function<void(bool)> cb) { dma_irq_cb_ = std::move(cb); }
    // The virtual DMA engine moves 32 times as much per clock as the real
    // one's 64 bits (the PROM clears all of memory with it at power-on).
    void set_fast_dma(bool fast) { fast_dma_ = fast; }
    // CPUCTRL0.SIN (or the watchdog running out): a full system reset.
    void set_reset_callback(std::function<void()> cb) { reset_cb_ = std::move(cb); }
    // CPUCTRL0.WR_ST and WRST: the processor alone is reset, from its reset
    // sequence (warm = false) or with a warm (soft) reset.
    void set_cpu_reset_callback(std::function<void(bool warm)> cb) { cpu_reset_cb_ = std::move(cb); }
    // The CPU wrote to the boot PROM: a bus error interrupt (GIO_ERROR_STAT
    // PROM) unless CPUCTRL0.PROM_WR_EN is set. The write itself goes nowhere.
    void prom_written(u32 paddr);

    struct Bank {
        u64 offset = 0;         // into ram_
        u64 subbank_size = 0;
        unsigned subbanks = 0;  // 0 = slot empty
    };
    const std::vector<Bank>& banks() const { return banks_; }
    // The CPU module's serial EEPROM (93CS56) behind the EEROM register: the
    // processor's boot-mode bits, and (word 17) the size of a secondary
    // cache the processor itself cannot report, in 4 KiB pages.
    static constexpr unsigned kCpuEepromCacheSize = 17;
    std::vector<u16>& cpu_eeprom() { return cpu_eeprom_.words(); }
    // Programs the bank registers as the PROM's memory sizing leaves them
    // (the fitted banks one after another from 0x08000000), for programs
    // started without the PROM. Does nothing if they are already set.
    void map_banks_as_post();

private:
    // Registers answer at the odd word (big-endian) or, with CPUCTRL0.LITTLE,
    // the even word.
    bool register_word(u32 word) const { return ((word & 4) != 0) != ((regs_.at(kCpuCtrl0) & (1u << 18)) != 0); }
    u32 read_reg(u32 reg);
    void write_reg(u32 reg, u32 value);
    void remap_memory();
    u32 rpss_counter() const;
    void rebase_rpss();
    void update_irq();
    void start_dma();
    void finish_dma();
    // Physical address for a DMA memory address, or 0 with `fault` set to
    // the cause bit (TLB miss, page fault, clean page written).
    u32 dma_translate(u32 vaddr, bool to_memory, u32& fault);
    void dma_load_defaults();
    // Refresh counter and watchdog (see read_reg kRefCtr / kDogc).
    u64 sysclks_since(u64 cycle) const;
    u64 refresh_bursts() const;
    u32 dog_count() const;
    void clear_dog();
    void schedule_dog();

    // A bank that is configured valid but has no SIMMs fitted: the MC still
    // decodes it, so accesses complete (reads see an undriven bus, here zero)
    // instead of raising bus errors. The PROM sizes memory this way.
    class EmptyBank final : public Device {
    public:
        const char* name() const override { return "empty-simm-bank"; }
        bool read(u32, unsigned, u64& value) override {
            value = 0;
            return true;
        }
        bool write(u32, unsigned, u64) override { return true; }
    };

    Bus& bus_;
    Scheduler& scheduler_;
    EmptyBank empty_bank_;
    Eeprom93Cxx cpu_eeprom_;
    bool eisa_;
    bool large_banks_;
    unsigned bank_shift_;  // of a bank register's base and size fields
    Device* bank_device_ = nullptr;
    u32 bank_device_base_ = 0;
    u64 bank_device_size_ = 0;
    u8* ram_;
    std::vector<Bank> banks_;
    std::vector<std::pair<u32, u64>> mapped_;  // memory windows currently mapped
    double cycles_per_sysclk_;

    std::unordered_map<u32, u32> regs_;
    u32 cpu_error_addr_ = 0, cpu_error_stat_ = 0;
    u32 gio_error_addr_ = 0, gio_error_stat_ = 0;
    u32 sys_semaphore_ = 0;
    std::array<u32, 16> user_semaphores_{};
    u32 rpss_base_ = 0;
    u64 rpss_cycle_base_ = 0;
    bool irq_ = false;
    std::function<void(bool)> irq_cb_;
    std::function<void(bool)> dma_irq_cb_;

    // Virtual DMA engine (graphics DMA master)
    u32 dma_mem_addr_ = 0, dma_size_ = 0, dma_stride_ = 0, dma_gio_addr_ = 0, dma_mode_ = 0, dma_count_ = 0;
    u32 dma_run_ = 0, dma_ctl_ = 0, dma_cause_ = 0;
    std::array<u32, 4> dma_tlb_hi_{}, dma_tlb_lo_{};
    Timer dma_timer_;
    bool fast_dma_ = false;
    std::function<void()> reset_cb_;
    std::function<void(bool)> cpu_reset_cb_;
    u64 refresh_base_ = 0;    // when the refresh counter last loaded CTRLD
    s64 dog_cleared_at_ = 0;  // refresh_bursts() when the watchdog was last cleared
    Timer dog_timer_;
};

}  // namespace sg::ip22
