// SPDX-License-Identifier: GPL-2.0-or-later
// IP22 HPC3 peripheral controller at 0x1FB80000 (512 KiB window): PBUS,
// SCSI and Ethernet DMA channels, interrupt status, the serial EEPROM, the
// PBUS device windows (IOC2 on channel 6) and the DS1386 RTC/NVRAM.
//
// SCSI channel 0 (WD33C93 plus its DMA channel) is modelled; the PBUS and
// Ethernet DMA engines and the Seeq Ethernet controller are register storage
// for now.
#pragma once

#include <unordered_map>

#include "devices/ds1386.h"
#include "devices/eeprom93cxx.h"
#include "devices/wd33c93.h"
#include "machine/ip22/ioc2.h"
#include "mem/bus.h"

namespace sg::ip22 {

class Hpc3 final : public Device {
public:
    Hpc3(Bus& bus, Ioc2& ioc, Scheduler& scheduler, double cpu_hz);

    void reset();
    const char* name() const override { return "hpc3"; }
    bool read(u32 offset, unsigned size, u64& value) override;
    bool write(u32 offset, unsigned size, u64 value) override;

    Ds1386& rtc() { return rtc_; }
    Eeprom93Cxx& eeprom() { return eeprom_; }
    Wd33c93& scsi0() { return scsi0_; }

private:
    // Byte-wide devices on 4-byte strides: returns the byte-register index
    // for an access, or -1 if the access does not touch the data byte.
    static int byte_index(u32 offset, unsigned size, u32 window_base);

    u32 read_word(u32 offset);
    void write_word(u32 offset, u32 value);

    // SCSI channel 0 DMA: walks descriptor chains {buffer, count/flags, next}.
    u32 scsi_dma(const u8* src, u8* dst, u32 length);
    void update_dma_irq();

    Bus& bus_;
    Ioc2& ioc_;
    struct ScsiDma {
        u32 ndbp = 0, cbp = 0, bc = 0, flags = 0, ctrl = 0;
        bool active = false, loaded = false, irq = false;
    } scsi_dma_;
    Ds1386 rtc_;
    Eeprom93Cxx eeprom_;
    Wd33c93 scsi0_;
    u32 eeprom_reg_ = 0;
    std::unordered_map<u32, u32> regs_;  // registers without behaviour yet
};

}  // namespace sg::ip22
