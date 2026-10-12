// SPDX-License-Identifier: GPL-2.0-or-later
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
        kRpssCounter = 0x1004,
        kDmaRun = 0x204C,
        kUserSemaphores = 0x10004,  // + n * 0x1000, n = 0..15
    };

    // `ram` holds all installed SIMM memory; it is split into banks of the
    // sizes the MC can decode (4..128 MiB, largest first, at most 4 banks).
    Mc(Bus& bus, Scheduler& scheduler, u8* ram, u64 ram_size, double cpu_hz, double sysclk_hz);

    void reset();
    const char* name() const override { return "mc"; }
    bool read(u32 offset, unsigned size, u64& value) override;
    bool write(u32 offset, unsigned size, u64 value) override;

    // Called by the bus for accesses nobody answered.
    void report_bus_error(u64 paddr, unsigned size, bool is_write);

    // IP6: asynchronous bus error interrupt.
    void set_irq_callback(std::function<void(bool)> cb) { irq_cb_ = std::move(cb); }
    // CPUCTRL0.SIN: software requested a full system reset.
    void set_reset_callback(std::function<void()> cb) { reset_cb_ = std::move(cb); }

    const std::vector<u64>& bank_sizes() const { return bank_sizes_; }

private:
    u32 read_reg(u32 reg);
    void write_reg(u32 reg, u32 value);
    void remap_memory();
    u32 rpss_counter() const;
    void rebase_rpss();
    void update_irq();

    Bus& bus_;
    Scheduler& scheduler_;
    u8* ram_;
    std::vector<u64> bank_sizes_;  // installed bank sizes, bank n at ram_ + sum of earlier banks
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
    std::function<void()> reset_cb_;
};

}  // namespace sg::ip22
