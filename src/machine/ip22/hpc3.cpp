// SPDX-License-Identifier: GPL-2.0-or-later
// References: Linux arch/mips/include/asm/sgi/hpc3.h, ip22-nvram.c;
// NetBSD sys/arch/sgimips/hpc/hpcreg.h.
#include "machine/ip22/hpc3.h"

#include "core/log.h"

namespace sg::ip22 {

namespace {

constexpr u32 kIntStat0 = 0x30000;
constexpr u32 kGioMisc = 0x30004;
constexpr u32 kEepromReg = 0x30008;
constexpr u32 kIntStat1 = 0x3000C;
constexpr u32 kBeStat = 0x30010;

constexpr u32 kIocWindow = 0x59800;  // PBUS PIO channel 6
constexpr u32 kIocWindowEnd = kIocWindow + 0x400;
constexpr u32 kScsi0Window = 0x40000;  // WD33C93: SASR at word 0, data at word 1
constexpr u32 kScsi0WindowEnd = 0x48000;
constexpr u32 kScsi0Ctrl = 0x11004;
constexpr u32 kScsiCtrlReset = 1u << 6;
constexpr u32 kRtcWindow = 0x60000;
constexpr u32 kRtcWindowEnd = 0x68000;

// EEPROM register bits
constexpr u32 kEepromCs = 1u << 1;
constexpr u32 kEepromClk = 1u << 2;
constexpr u32 kEepromDataOut = 1u << 3;  // CPU -> EEPROM
constexpr u32 kEepromDataIn = 1u << 4;   // EEPROM -> CPU

}  // namespace

Hpc3::Hpc3(Ioc2& ioc, Scheduler& scheduler, double cpu_hz) : ioc_(ioc), scsi0_(scheduler, cpu_hz / 1000.0) {
    scsi0_.set_irq_callback([this](bool level) { ioc_.set_local0(Ioc2::kL0Scsi0, level); });
    reset();
}

void Hpc3::reset() {
    regs_.clear();
    eeprom_reg_ = 0;
    eeprom_.set_lines(false, false, false);
    scsi0_.reset();
}

int Hpc3::byte_index(u32 offset, unsigned size, u32 window_base) {
    const u32 rel = offset - window_base;
    if (size == 1) return (rel & 3) == 3 ? static_cast<int>(rel >> 2) : -1;
    if (size == 2) return (rel & 3) == 2 ? static_cast<int>(rel >> 2) : -1;
    return static_cast<int>(rel >> 2);  // word or doubleword: data in the low byte of the (last) word
}

bool Hpc3::read(u32 offset, unsigned size, u64& value) {
    if (offset >= kScsi0Window && offset < kScsi0WindowEnd) {
        const u32 o = size == 8 ? offset + 4 : offset;
        const int idx = byte_index(o, size == 8 ? 4 : size, kScsi0Window);
        value = idx < 0 ? 0 : (idx & 1) ? scsi0_.read_data() : scsi0_.read_aux_status();
        return true;
    }
    if (offset >= kIocWindow && offset < kIocWindowEnd) {
        const u32 o = size == 8 ? offset + 4 : offset;
        const int idx = byte_index(o, size == 8 ? 4 : size, kIocWindow);
        value = idx >= 0 ? ioc_.read(static_cast<unsigned>(idx)) : 0;
        return true;
    }
    if (offset >= kRtcWindow && offset < kRtcWindowEnd) {
        const u32 o = size == 8 ? offset + 4 : offset;
        const int idx = byte_index(o, size == 8 ? 4 : size, kRtcWindow);
        value = idx >= 0 ? rtc_.read(static_cast<unsigned>(idx)) : 0;
        return true;
    }
    if (size == 8) {
        value = (u64{read_word(offset)} << 32) | read_word(offset + 4);
        return true;
    }
    const u32 w = read_word(offset & ~3u);
    if (size == 4) {
        value = w;
    } else {
        const unsigned shift = (4 - size - (offset & 3)) * 8;
        value = (w >> shift) & ((1u << (size * 8)) - 1);
    }
    return true;
}

bool Hpc3::write(u32 offset, unsigned size, u64 value) {
    if (offset >= kScsi0Window && offset < kScsi0WindowEnd) {
        const u32 o = size == 8 ? offset + 4 : offset;
        const int idx = byte_index(o, size == 8 ? 4 : size, kScsi0Window);
        if (idx >= 0) {
            if (idx & 1) scsi0_.write_data(static_cast<u8>(value));
            else scsi0_.write_address(static_cast<u8>(value));
        }
        return true;
    }
    if (offset >= kIocWindow && offset < kIocWindowEnd) {
        const u32 o = size == 8 ? offset + 4 : offset;
        const int idx = byte_index(o, size == 8 ? 4 : size, kIocWindow);
        if (idx >= 0) ioc_.write(static_cast<unsigned>(idx), static_cast<u8>(value));
        return true;
    }
    if (offset >= kRtcWindow && offset < kRtcWindowEnd) {
        const u32 o = size == 8 ? offset + 4 : offset;
        const int idx = byte_index(o, size == 8 ? 4 : size, kRtcWindow);
        if (idx >= 0) rtc_.write(static_cast<unsigned>(idx), static_cast<u8>(value));
        return true;
    }
    if (size == 8) {
        write_word(offset, static_cast<u32>(value >> 32));
        write_word(offset + 4, static_cast<u32>(value));
        return true;
    }
    if (size == 4) {
        write_word(offset, static_cast<u32>(value));
    } else {
        const u32 word = offset & ~3u;
        const unsigned shift = (4 - size - (offset & 3)) * 8;
        const u32 mask = ((1u << (size * 8)) - 1) << shift;
        write_word(word, (read_word(word) & ~mask) | ((static_cast<u32>(value) << shift) & mask));
    }
    return true;
}

u32 Hpc3::read_word(u32 offset) {
    switch (offset) {
        case kIntStat0:
        case kIntStat1: return 0;  // no DMA channel has completed
        case kEepromReg: return (eeprom_reg_ & ~kEepromDataIn) | (eeprom_.data_out() ? kEepromDataIn : 0);
        case kBeStat: return 0;
        default: {
            const auto it = regs_.find(offset);
            if (it != regs_.end()) return it->second;
            SG_LOG(Debug, "hpc3", "read of unmodelled register +%05x", offset);
            return 0;
        }
    }
}

void Hpc3::write_word(u32 offset, u32 value) {
    switch (offset) {
        case kEepromReg:
            eeprom_reg_ = value & 0x1F;
            eeprom_.set_lines(value & kEepromCs, value & kEepromClk, value & kEepromDataOut);
            return;
        case kGioMisc: regs_[offset] = value & 3; return;
        case kScsi0Ctrl:
            if (value & kScsiCtrlReset) scsi0_.reset();
            regs_[offset] = value & ~kScsiCtrlReset;
            return;
        case kIntStat0:
        case kIntStat1:
        case kBeStat: return;
        default:
            SG_LOG(Debug, "hpc3", "write to unmodelled register +%05x = %08x", offset, value);
            regs_[offset] = value;
            return;
    }
}

}  // namespace sg::ip22
