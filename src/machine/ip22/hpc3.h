// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// IP22 HPC3 peripheral controller at 0x1FB80000 (512 KiB window): PBUS,
// SCSI and Ethernet DMA channels, interrupt status, the serial EEPROM, the
// PBUS device windows (IOC2 on channel 6) and the DS1386 RTC/NVRAM.
//
// The SCSI channels (a WD33C93 plus its DMA channel each; the Indy wires
// only channel 0), the PBUS DMA channels serving the HAL2 audio processor
// and the Ethernet DMA channels with their Seeq 8003 controller are
// modelled. On the Indigo2 the INT2 sits on PBUS channel 4 and the PROM
// keeps its settings in the serial EEPROM.
#pragma once

#include <array>
#include <functional>
#include <memory>
#include <unordered_map>

#include "devices/ds1386.h"
#include "devices/eeprom93cxx.h"
#include "devices/seeq8003.h"
#include "devices/wd33c93.h"
#include "machine/ip22/hal2.h"
#include "machine/ip22/ioc2.h"
#include "mem/bus.h"

namespace sg::net {
class NetBackend;
}

namespace sg::ip22 {

class Hpc3 final : public Device {
public:
    // `full_house`: the Indigo2's wiring (second SCSI channel, INT2, DS1286).
    Hpc3(Bus& bus, Ioc2& ioc, Scheduler& scheduler, double cpu_hz, bool full_house = false);

    void reset();
    const char* name() const override { return "hpc3"; }
    bool read(u32 offset, unsigned size, u64& value) override;
    bool write(u32 offset, unsigned size, u64 value) override;
    bool read_is_pure(u32 offset, unsigned size) const override;

    Ds1386& rtc() { return rtc_; }
    const Ds1386& rtc() const { return rtc_; }
    Eeprom93Cxx& eeprom() { return eeprom_; }
    const Eeprom93Cxx& eeprom() const { return eeprom_; }
    Wd33c93& scsi0() { return scsi0_; }
    Wd33c93& scsi1() { return scsi1_; }
    Hal2& hal2() { return hal2_; }
    Seeq8003& seeq() { return seeq_; }
    // Ethernet frames go to and come from `backend` (null: no cable).
    void set_network(net::NetBackend* backend);
    // SCSI and Ethernet as fast as the host allows instead of at their real
    // rates (Ethernet then takes in a frame only when the driver has a
    // buffer ready for it, rather than losing it, unless the channel is
    // held in reset).
    void set_fast_io(bool fast);
    // See Wd33c93::set_quick_selection_timeout.
    void set_quick_scsi_scan(bool quick) {
        scsi0_.set_quick_selection_timeout(quick);
        scsi1_.set_quick_selection_timeout(quick);
    }
    // Frames sent and received on the Ethernet port (for activity lights).
    u64 enet_tx_frames() const { return enet_tx_frames_; }
    u64 enet_rx_frames() const { return enet_rx_frames_; }
    // Called by the machine's run loop: starts taking in frames when the
    // host has some (the poll then runs only while frames keep coming).
    void poll_network();

    // Samples played by the HAL2 DAC (the top 16 bits of its 24-bit samples):
    // `frames` frames of `voices` interleaved samples at `rate` frames per
    // second.
    using AudioSink = std::function<void(const s16* samples, std::size_t frames, unsigned voices, double rate)>;
    void set_audio_sink(AudioSink sink) { audio_sink_ = std::move(sink); }
    // Fills `samples` with what the HAL2 ADC records (`frames` frames of
    // `voices` interleaved samples at `rate`); without a source it records
    // silence.
    using AudioSource = std::function<void(s16* samples, std::size_t frames, unsigned voices, double rate)>;
    void set_audio_source(AudioSource source) { audio_source_ = std::move(source); }

private:
    // Byte-wide devices on 4-byte strides: returns the byte-register index
    // for an access, or -1 if the access does not touch the data byte.
    static int byte_index(u32 offset, unsigned size, u32 window_base);

    u32 read_word(u32 offset);
    void write_word(u32 offset, u32 value);

    // SCSI DMA: walks descriptor chains {buffer, count/flags, next}.
    u32 scsi_dma(unsigned ch, const u8* src, u8* dst, u32 length);
    u32 scsi_read(unsigned ch, u32 reg);
    void scsi_write(unsigned ch, u32 reg, u32 value);
    void update_dma_irq();

    // PBUS DMA: a channel walks its descriptor chain as fast as the HAL2
    // port tied to it moves data (one timer event per descriptor).
    static constexpr unsigned kPbusChannels = 8;
    struct PbusChannel {
        u32 bptr = 0, dptr = 0, ctrl = 0, bc = 0, flags = 0;
        u32 filled = 0;  // audio: bytes of the current descriptor already moved
        bool active = false, irq = false, loaded = false;
        double bytes_per_cycle = 0;  // of the current transfer; 0 = waiting for the device
        u64 started = 0;             // when the current descriptor began moving
    };
    bool pbus_load(unsigned ch);
    void pbus_schedule(unsigned ch, bool restart);
    void pbus_complete(unsigned ch);
    void pbus_reconfigure();
    unsigned pbus_sample_bytes(unsigned ch) const;  // memory bytes per audio sample
    const Hal2::Port* pbus_port(unsigned ch) const;  // the HAL2 port this channel serves, if any
    void pbus_transfer(unsigned ch, u32 upto, bool last = false);
    void play(unsigned ch, u32 addr, u32 frames, const Hal2::PortConfig& dac);
    u32 pbus_read(u32 offset);
    void pbus_write(u32 offset, u32 value);

    // Ethernet DMA: receive writes each accepted frame into the next
    // descriptor(s); transmit sends a frame per end-of-packet descriptor,
    // spaced by its time on a 10 Mbit/s wire.
    struct EnetDma {
        u32 cbp = 0, ndbp = 0, ctrl = 0;
        u32 cdp = 0;  // descriptor last fetched (where receiving stopped)
        bool active = false;
    };
    void enet_receive(const u8* frame, std::size_t length);
    void enet_transmit();
    void enet_poll();
    void enet_interrupt();
    void enet_reset();
    u32 enet_read(u32 offset);
    void enet_write(u32 offset, u32 value);

    Bus& bus_;
    Ioc2& ioc_;
    Scheduler& scheduler_;
    double cpu_hz_;
    bool full_house_;
    struct ScsiDma {
        u32 ndbp = 0, cbp = 0, bc = 0, flags = 0, ctrl = 0;
        bool active = false, loaded = false, irq = false;
        bool in_reset = false;  // the reset bit is held
    };
    std::array<ScsiDma, 2> scsi_dma_{};
    Ds1386 rtc_;
    Eeprom93Cxx eeprom_;
    Wd33c93 scsi0_, scsi1_;
    std::array<Wd33c93*, 2> scsi_{&scsi0_, &scsi1_};
    Hal2 hal2_;
    std::array<PbusChannel, kPbusChannels> pbus_{};
    std::array<std::unique_ptr<Timer>, kPbusChannels> pbus_timer_;
    AudioSink audio_sink_;
    AudioSource audio_source_;
    Seeq8003 seeq_;
    EnetDma enet_rx_, enet_tx_;
    bool enet_irq_ = false, enet_loopback_ = false;
    bool enet_rx_ready() const;
    double enet_bits_per_second() const { return fast_io_ ? 1e9 : 10e6; }
    bool fast_io_ = false;
    bool enet_held_ = true;  // channel reset bit: set at power-on, holds the channel and the Seeq in reset
    u32 enet_dconfig_ = 0, enet_pconfig_ = 0;
    net::NetBackend* net_ = nullptr;
    u64 enet_tx_frames_ = 0, enet_rx_frames_ = 0;
    std::unique_ptr<Timer> enet_tx_timer_, enet_poll_timer_;
    u32 eeprom_reg_ = 0;
    std::unordered_map<u32, u32> regs_;  // registers without behaviour yet
    std::unordered_map<u32, u32> written_;  // last word written by sub-word writes, per register
};

}  // namespace sg::ip22
