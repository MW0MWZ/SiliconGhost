// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// References: Linux arch/mips/include/asm/sgi/hpc3.h, ip22-nvram.c;
// NetBSD sys/arch/sgimips/hpc/hpcreg.h.
#include "machine/ip22/hpc3.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "core/log.h"
#include "net/net_backend.h"

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
constexpr u32 kScsi1WindowEnd = 0x50000;  // second WD33C93 port: not connected on the Indy
constexpr u32 kScsi0Cbp = 0x10000;
constexpr u32 kScsi0Ndbp = 0x10004;
constexpr u32 kScsi0Bc = 0x11000;
constexpr u32 kScsi0Ctrl = 0x11004;
// SCSI DMA control bits
constexpr u32 kScsiCtrlIrq = 1u << 0;
constexpr u32 kScsiCtrlEndian = 1u << 1;
constexpr u32 kScsiCtrlDir = 1u << 2;  // memory -> device
constexpr u32 kScsiCtrlFlush = 1u << 3;
constexpr u32 kScsiCtrlActive = 1u << 4;
constexpr u32 kScsiCtrlActiveMask = 1u << 5;
constexpr u32 kScsiCtrlReset = 1u << 6;
// Descriptor count word
constexpr u32 kDescEox = 1u << 31;
constexpr u32 kDescXie = 1u << 29;
constexpr u32 kDescCountMask = 0x3FFF;
constexpr u32 kIntStatScsi0 = 1u << 8;
constexpr u32 kHal2Window = 0x58000;  // PBUS PIO channels 0-3
constexpr u32 kHal2WindowEnd = 0x59000;
// PBUS DMA channel n: buffer and descriptor pointers at n * 0x2000, control at +0x1000
constexpr u32 kPbusDmaEnd = 0x10000;
constexpr u32 kPbusStride = 0x2000;
constexpr u32 kPbusCtrl = 0x1000;
// Control register: written...
constexpr u32 kPbusCtrlLittleEndian = 1u << 1;
constexpr u32 kPbusCtrlReceive = 1u << 2;  // device to memory
constexpr u32 kPbusCtrlAct = 1u << 4;
constexpr u32 kPbusCtrlLoad = 1u << 5;  // ACT is written only with this set
// ...and read
constexpr u32 kPbusStatInt = 1u << 0;
constexpr u32 kPbusStatActive = 1u << 1;
// Per-channel DMA configuration (device timing and bus width), 0x200 apart
constexpr u32 kPbusDmaCfg = 0x5C000, kPbusDmaCfgStride = 0x200;
constexpr u32 kPbusCfgDs16 = 1u << 18;
// Ethernet DMA (receive channel at 0x14000, transmit at 0x16000) and the
// Seeq's registers on 4-byte strides
constexpr u32 kEnetStart = 0x14000, kEnetEnd = 0x18000;
// The receive channel's current descriptor: where receiving stopped (IRIX
// restarts after it once the driver has caught up).
constexpr u32 kEnetRxCurDesc = 0x18000;
constexpr u32 kEnetRxCbp = 0x14000, kEnetRxNdbp = 0x14004, kEnetRxBcd = 0x15000, kEnetRxCtrl = 0x15004;
constexpr u32 kEnetReset = 0x15014, kEnetDmaCfg = 0x15018, kEnetPioCfg = 0x1501C;
constexpr u32 kEnetTxCbp = 0x16000, kEnetTxNdbp = 0x16004, kEnetTxBcd = 0x17000, kEnetTxCtrl = 0x17004;
constexpr u32 kSeeqWindow = 0x54000, kSeeqWindowEnd = 0x54500;
constexpr u32 kEnetCtrlEndian = 1u << 8, kEnetCtrlActive = 1u << 9, kEnetCtrlActiveMask = 1u << 10;
constexpr u32 kEnetResetChannel = 1u << 0, kEnetResetClearIrq = 1u << 1, kEnetResetLoopback = 1u << 2;
// Ethernet descriptor count word
constexpr u32 kEnetDescEnd = 1u << 31;        // end of chain (transmit) / ring (receive)
constexpr u32 kEnetDescEndPacket = 1u << 30;  // end of packet
constexpr u32 kEnetDescXie = 1u << 29;
constexpr u32 kEnetDescTxDone = 1u << 15;     // set by the HPC once sent
constexpr u32 kEnetDescOwn = 1u << 14;        // receive buffer handed to the HPC
constexpr std::size_t kEnetMinFrame = 60;     // without the FCS
constexpr std::size_t kEnetMaxFrame = 1518;
constexpr u32 kRtcWindow = 0x60000;
constexpr u32 kRtcWindowEnd = 0x68000;

// EEPROM register bits
constexpr u32 kEepromPre = 1u << 0;  // protect register enable
constexpr u32 kEepromCs = 1u << 1;
constexpr u32 kEepromClk = 1u << 2;
constexpr u32 kEepromDataOut = 1u << 3;  // CPU -> EEPROM
constexpr u32 kEepromDataIn = 1u << 4;   // EEPROM -> CPU

}  // namespace

Hpc3::Hpc3(Bus& bus, Ioc2& ioc, Scheduler& scheduler, double cpu_hz)
    : bus_(bus), ioc_(ioc), scheduler_(scheduler), cpu_hz_(cpu_hz), scsi0_(scheduler, cpu_hz / 1000.0) {
    for (unsigned ch = 0; ch < kPbusChannels; ++ch)
        pbus_timer_[ch] = std::make_unique<Timer>(scheduler, [this, ch] { pbus_complete(ch); });
    hal2_.set_change_callback([this] { pbus_reconfigure(); });
    enet_tx_timer_ = std::make_unique<Timer>(scheduler, [this] { enet_transmit(); });
    enet_poll_timer_ = std::make_unique<Timer>(scheduler, [this] { enet_poll(); });
    scsi0_.set_irq_callback([this](bool level) { ioc_.set_local0(Ioc2::kL0Scsi0, level); });
    scsi0_.set_dma([this](const u8* data, u32 n) { return scsi_dma(data, nullptr, n); },
                   [this](u8* data, u32 n) { return scsi_dma(nullptr, data, n); });
    reset();
}

void Hpc3::reset() {
    regs_.clear();
    written_.clear();
    eeprom_reg_ = 0;
    eeprom_.set_lines(false, false, false);
    scsi0_.reset();
    scsi_dma_ = ScsiDma{};
    scsi_dma_.in_reset = true;  // the channel reset bit is set at power-on
    for (unsigned ch = 0; ch < kPbusChannels; ++ch) {
        pbus_timer_[ch]->cancel();
        pbus_[ch] = PbusChannel{};
    }
    hal2_.reset();
    update_dma_irq();
    enet_reset();
    enet_held_ = true;
    enet_loopback_ = false;
    enet_dconfig_ = enet_pconfig_ = 0;
    enet_rx_.cbp = enet_rx_.ndbp = enet_rx_.cdp = enet_tx_.cbp = enet_tx_.ndbp = 0;
}

// ---------------------------------------------------------------------------
// Ethernet
// ---------------------------------------------------------------------------

void Hpc3::set_network(net::NetBackend* backend) {
    net_ = backend;
    enet_poll_timer_->cancel();
}

void Hpc3::poll_network() {
    if (net_ && !enet_poll_timer_->active() && net_->pending()) enet_poll_timer_->schedule_in(1);
}

// Resets both channels and the Seeq (the interrupt is cleared as well).
void Hpc3::enet_reset() {
    enet_rx_.active = enet_tx_.active = false;
    enet_rx_.ctrl = enet_tx_.ctrl = 0;
    enet_tx_timer_->cancel();
    seeq_.reset();
    enet_irq_ = false;
    ioc_.set_local0(Ioc2::kL0Ethernet, false);
}

void Hpc3::enet_interrupt() {
    enet_irq_ = true;
    ioc_.set_local0(Ioc2::kL0Ethernet, true);
}

// Frames from the host arrive no faster than a 10 Mbit/s wire carries them
// (with the preamble, FCS and gap between frames). When the host has none,
// the poll stops until poll_network() sees more.
void Hpc3::enet_poll() {
    std::vector<u8> frame;
    if (net_ && net_->receive(frame)) {
        enet_receive(frame.data(), frame.size());
        const std::size_t wire = std::max(frame.size(), kEnetMinFrame) + 24;
        enet_poll_timer_->schedule_in(std::max<u64>(1, static_cast<u64>(wire * 8 / 10e6 * cpu_hz_)));
    }
}

// The frame is written as two bytes of padding, the frame (padded to the
// minimum length) and the receive status, starting a new descriptor. Only
// descriptors the driver has handed over (OWN set) take data; the HPC clears
// OWN and leaves the unused byte count in each one it fills.
void Hpc3::enet_receive(const u8* frame, std::size_t length) {
    if (!seeq_.accepts(frame, length) || length > kEnetMaxFrame) return;
    ++enet_rx_frames_;
    if (!enet_rx_.active) return;  // no buffers: the frame is lost
    std::vector<u8> data(2 + std::max(length, kEnetMinFrame) + 1, 0);
    std::memcpy(&data[2], frame, length);
    u8 status = Seeq8003::kRxGood | Seeq8003::kRxEndOfFrame;
    std::size_t done = 0;
    bool irq = false;
    for (int descs = 0; descs < 64; ++descs) {
        const u32 desc = enet_rx_.ndbp;
        enet_rx_.cdp = desc;
        u32 pbuf = 0, cnt = 0, next = 0;
        bus_.read<u32>(desc, pbuf);
        bus_.read<u32>(desc + 4, cnt);
        bus_.read<u32>(desc + 8, next);
        if (!(cnt & kEnetDescOwn)) {
            // The driver has not handed this buffer back yet (OWN is the
            // drivers' convention; the HPC3 itself only stops at EOX). The
            // frame is dropped rather than written over a buffer still in
            // use, and the channel stays ready for the next one.
            if (done == 0) return;
            status = Seeq8003::kRxOverflow;
            break;
        }
        const u32 room = cnt & kDescCountMask;
        const u32 n = static_cast<u32>(std::min<std::size_t>(room, data.size() - done));
        if (n == data.size() - done) data.back() = status;  // the status byte lands in this buffer
        if (!bus_.write_block(pbuf, &data[done], n))
            for (u32 i = 0; i < n; ++i) bus_.write<u8>(pbuf + i, data[done + i]);
        done += n;
        const bool last = done == data.size();
        bus_.write<u32>(desc + 4, (cnt & ~(kDescCountMask | kEnetDescOwn)) | (room - n) | (last ? kEnetDescEndPacket : 0));
        enet_rx_.cbp = pbuf + n;
        irq |= (cnt & kEnetDescXie) != 0;
        if (cnt & kEnetDescEnd) {  // end of the ring: receiving stops
            enet_rx_.active = false;
            if (!last) status = Seeq8003::kRxOverflow;
            break;
        }
        enet_rx_.ndbp = next;
        if (last) break;
    }
    irq |= seeq_.received(status);
    if (irq) enet_interrupt();
}

// Sends the frame starting at the next descriptor, then times the next one.
void Hpc3::enet_transmit() {
    if (!enet_tx_.active) return;
    std::vector<u8> frame;
    bool irq = false, end_of_packet = false;
    for (int descs = 0; descs < 64 && !end_of_packet; ++descs) {
        const u32 desc = enet_tx_.ndbp;
        u32 pbuf = 0, cnt = 0, next = 0;
        bus_.read<u32>(desc, pbuf);
        bus_.read<u32>(desc + 4, cnt);
        bus_.read<u32>(desc + 8, next);
        const u32 n = cnt & kDescCountMask;
        const std::size_t at = frame.size();
        frame.resize(std::min(at + n, kEnetMaxFrame));
        for (std::size_t i = at; i < frame.size(); ++i) bus_.read<u8>(static_cast<u32>(pbuf + (i - at)), frame[i]);
        bus_.write<u32>(desc + 4, cnt | kEnetDescTxDone);
        enet_tx_.cbp = pbuf + n;
        irq |= (cnt & kEnetDescXie) != 0;
        end_of_packet = cnt & kEnetDescEndPacket;
        if (cnt & kEnetDescEnd) {
            enet_tx_.active = false;
            break;
        }
        enet_tx_.ndbp = next;
    }
    if (frame.size() < kEnetMinFrame) frame.resize(kEnetMinFrame, 0);
    ++enet_tx_frames_;
    if (enet_loopback_) enet_receive(frame.data(), frame.size());
    else if (net_) net_->send(frame.data(), frame.size());
    irq |= seeq_.transmitted(end_of_packet ? Seeq8003::kTxDone : Seeq8003::kTxUnderflow);
    if (irq) enet_interrupt();
    // Preamble, FCS and the gap between frames: 24 bytes more on the wire.
    if (enet_tx_.active) enet_tx_timer_->schedule_in(static_cast<u64>((frame.size() + 24) * 8 / 10e6 * cpu_hz_));
}

u32 Hpc3::enet_read(u32 offset) {
    switch (offset) {
        case kEnetRxCbp: return enet_rx_.cbp;
        case kEnetRxNdbp: return enet_rx_.ndbp;
        case kEnetRxCtrl:
            return seeq_.rx_status() | (enet_rx_.ctrl & kEnetCtrlEndian) | (enet_rx_.active ? kEnetCtrlActive : 0);
        case kEnetReset:
            return (enet_held_ ? kEnetResetChannel : 0) | (enet_irq_ ? kEnetResetClearIrq : 0) |
                   (enet_loopback_ ? kEnetResetLoopback : 0);
        case kEnetDmaCfg: return enet_dconfig_;
        case kEnetPioCfg: return enet_pconfig_;
        case kEnetTxCbp: return enet_tx_.cbp;
        case kEnetTxNdbp: return enet_tx_.ndbp;
        case kEnetTxCtrl:
            return seeq_.tx_status() | (enet_tx_.ctrl & kEnetCtrlEndian) | (enet_tx_.active ? kEnetCtrlActive : 0);
        case kEnetRxBcd:
        case kEnetTxBcd:
        default: return 0;
    }
}

void Hpc3::enet_write(u32 offset, u32 value) {
    switch (offset) {
        case kEnetRxCbp: enet_rx_.cbp = value; return;
        case kEnetRxNdbp: enet_rx_.ndbp = value; return;
        case kEnetRxCtrl:
            enet_rx_.ctrl = value;
            if (!(value & kEnetCtrlActiveMask)) enet_rx_.active = (value & kEnetCtrlActive) && !enet_held_;
            return;
        case kEnetReset:
            enet_held_ = value & kEnetResetChannel;
            if (enet_held_) enet_reset();
            if (value & kEnetResetClearIrq) {
                enet_irq_ = false;
                ioc_.set_local0(Ioc2::kL0Ethernet, false);
            }
            enet_loopback_ = value & kEnetResetLoopback;
            return;
        case kEnetDmaCfg: enet_dconfig_ = value; return;
        case kEnetPioCfg: enet_pconfig_ = value; return;
        case kEnetTxCbp: enet_tx_.cbp = value; return;
        case kEnetTxNdbp: enet_tx_.ndbp = value; return;
        case kEnetTxCtrl: {
            enet_tx_.ctrl = value;
            if (value & kEnetCtrlActiveMask) return;
            const bool active = (value & kEnetCtrlActive) && !enet_held_;
            const bool start = active && !enet_tx_.active;
            enet_tx_.active = active;
            if (start) enet_tx_timer_->schedule_in(static_cast<u64>(cpu_hz_ * 2e-6));  // after a short setup
            if (!enet_tx_.active) enet_tx_timer_->cancel();
            return;
        }
        default: return;
    }
}

// ---------------------------------------------------------------------------
// PBUS DMA
// ---------------------------------------------------------------------------

// Loads the descriptor at dptr; false at the end of the chain.
bool Hpc3::pbus_load(unsigned ch) {
    PbusChannel& c = pbus_[ch];
    if (c.loaded && (c.flags & kDescEox)) return false;
    u32 pbuf = 0, cnt = 0, next = 0;
    bus_.read<u32>(c.dptr, pbuf);
    bus_.read<u32>(c.dptr + 4, cnt);
    bus_.read<u32>(c.dptr + 8, next);
    c.bptr = pbuf;
    c.bc = cnt & kDescCountMask;
    c.filled = 0;
    c.flags = cnt;
    c.dptr = next;
    c.loaded = true;
    return true;
}

// Times the current descriptor at the rate of the HAL2 port on this channel
// (a channel without one waits). `restart`: the timing of the descriptor
// begins now even if the rate is unchanged.
void Hpc3::pbus_schedule(unsigned ch, bool restart) {
    PbusChannel& c = pbus_[ch];
    double bytes_per_cycle = 0;
    if (c.active) {
        for (unsigned p = 0; p < Hal2::kNumPorts; ++p) {
            const Hal2::PortConfig port = hal2_.port(static_cast<Hal2::Port>(p));
            if (port.enabled && port.channel == ch) {
                bytes_per_cycle = port.rate * pbus_sample_bytes(ch) * port.voices / cpu_hz_;
                break;
            }
        }
    }
    if (!restart && bytes_per_cycle == c.bytes_per_cycle) return;
    c.bytes_per_cycle = bytes_per_cycle;
    pbus_timer_[ch]->cancel();
    if (bytes_per_cycle <= 0) return;
    // Move on to a descriptor with data. A chain of empty ones (which the
    // hardware would also spin on) is followed a few at a time.
    for (int empty = 0; c.bc == 0; ++empty) {
        if (c.loaded && (c.flags & kDescXie)) {
            c.irq = true;
            update_dma_irq();
        }
        if (empty == 16 && !(c.loaded && (c.flags & kDescEox))) {
            c.loaded = false;  // the descriptor at dptr is next
            c.started = scheduler_.now();
            pbus_timer_[ch]->schedule_in(std::max<u64>(1, static_cast<u64>(4 / bytes_per_cycle)));
            return;
        }
        if (!pbus_load(ch)) {
            c.active = false;
            c.bytes_per_cycle = 0;
            return;
        }
    }
    c.started = scheduler_.now();
    pbus_timer_[ch]->schedule_in(std::max<u64>(1, static_cast<u64>(std::ceil(c.bc / bytes_per_cycle))));
}

// The current descriptor's data has moved.
void Hpc3::pbus_complete(unsigned ch) {
    PbusChannel& c = pbus_[ch];
    if (!c.active) return;
    if (!c.bc) {  // after a run of empty descriptors
        pbus_schedule(ch, true);
        return;
    }
    pbus_transfer(ch, c.bc, true);
    const u32 n = c.bc;
    c.bptr += n;
    c.bc = 0;
    c.filled = 0;
    pbus_schedule(ch, true);
}

// Sends `frames` frames of DAC samples at `addr` to the audio sink. With DS16
// set, each sample is a 24-bit value in a 32-bit word and the codec takes its
// top 16 bits; otherwise samples are packed 16-bit values.
void Hpc3::play(unsigned ch, u32 addr, u32 frames, const Hal2::PortConfig& dac) {
    const unsigned size = pbus_sample_bytes(ch);
    std::vector<s16> samples(static_cast<std::size_t>(frames) * dac.voices);
    // HAL2 and the HPC3 each swap bytes when set for little-endian: both cancel.
    const bool little = dac.little_endian != ((pbus_[ch].ctrl & kPbusCtrlLittleEndian) != 0);
    for (u32 i = 0; i < samples.size(); ++i) {
        u8 b[4] = {0, 0, 0, 0};
        for (u32 k = 0; k < size; ++k) bus_.read<u8>(addr + size * i + k, b[k]);
        if (size == 2) {
            samples[i] = static_cast<s16>(little ? b[0] | (b[1] << 8) : (b[0] << 8) | b[1]);
            continue;
        }
        const u32 word = little ? u32{b[0]} | (u32{b[1]} << 8) | (u32{b[2]} << 16) | (u32{b[3]} << 24)
                                : (u32{b[0]} << 24) | (u32{b[1]} << 16) | (u32{b[2]} << 8) | u32{b[3]};
        samples[i] = static_cast<s16>(word >> 8);
    }
    double gain[2];
    hal2_.output_gain(gain[0], gain[1]);
    if (gain[0] != 1.0 || gain[1] != 1.0) {
        for (std::size_t i = 0; i < samples.size(); ++i)
            samples[i] = static_cast<s16>(std::lround(samples[i] * gain[(i % dac.voices) & 1]));
    }
    audio_sink_(samples.data(), frames, dac.voices, dac.rate);
}

const Hal2::Port* Hpc3::pbus_port(unsigned ch) const {
    static constexpr Hal2::Port kPorts[] = {Hal2::kDac, Hal2::kAdc, Hal2::kAesTx, Hal2::kAesRx};
    for (const Hal2::Port& p : kPorts) {
        const Hal2::PortConfig port = hal2_.port(p);
        if (port.enabled && port.channel == ch) return &p;
    }
    return nullptr;
}

// Moves the audio of the current descriptor up to `upto` bytes into it
// (whole frames; with `last`, the rest of it). As on the hardware, data moves
// as the transfer goes: software that follows the buffer pointer refills
// (or empties) the memory behind it, so it must be done with by then.
// The channel's receive bit sets the direction. The codec input (ADC)
// records from the host when a source is set and the digital input (AES) has
// nothing connected: silence otherwise, as from an output port. What the DAC
// plays goes to the audio sink.
void Hpc3::pbus_transfer(unsigned ch, u32 upto, bool last) {
    PbusChannel& c = pbus_[ch];
    const Hal2::Port* port = pbus_port(ch);
    if (!port) return;
    const Hal2::PortConfig in = hal2_.port(*port);
    const unsigned size = pbus_sample_bytes(ch);
    const u32 frame = size * std::max(1u, in.voices);
    upto = last ? c.bc : std::min(upto, c.bc) / frame * frame;
    if (upto <= c.filled) return;
    const u32 n = upto - c.filled;
    if (!(c.ctrl & kPbusCtrlReceive)) {
        if (*port == Hal2::kDac && audio_sink_ && n >= frame) play(ch, c.bptr + c.filled, n / frame, in);
        c.filled = upto;
        return;
    }
    std::vector<u8> bytes(n, 0);
    if (audio_source_ && *port == Hal2::kAdc && in.input && in.voices && n >= frame) {
        std::vector<s16> samples(n / frame * in.voices, 0);
        audio_source_(samples.data(), samples.size() / in.voices, in.voices, in.rate);
        const bool little = in.little_endian != ((c.ctrl & kPbusCtrlLittleEndian) != 0);
        double gain[2];
        hal2_.input_gain(gain[0], gain[1]);
        const bool unity = gain[0] == 1.0 && gain[1] == 1.0;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            // The codec's input gain (left on even samples), clipping at full scale; then as
            // played: a 24-bit value in a 32-bit word (DS16) or a packed 16-bit sample.
            s32 s = static_cast<s32>(samples[i]) * (size == 4 ? 256 : 1);
            if (!unity) {
                const double g = gain[in.voices >= 2 ? i % 2 : 0];
                const double full = size == 4 ? 8388607.0 : 32767.0;
                s = static_cast<s32>(std::lround(std::clamp(s * g, -full - 1, full)));
            }
            const u32 v = size == 4 ? static_cast<u32>(s) : static_cast<u16>(s);
            for (unsigned k = 0; k < size; ++k) bytes[i * size + k] = static_cast<u8>(v >> (8 * (little ? k : size - 1 - k)));
        }
    }
    const u32 at = c.bptr + c.filled;
    if (!bus_.write_block(at, bytes.data(), n))
        for (u32 i = 0; i < n; ++i) bus_.write<u8>(at + i, bytes[i]);
    c.filled = upto;
}

unsigned Hpc3::pbus_sample_bytes(unsigned ch) const {
    const auto it = regs_.find(kPbusDmaCfg + ch * kPbusDmaCfgStride);
    return it != regs_.end() && (it->second & kPbusCfgDs16) ? 4 : 2;
}

void Hpc3::pbus_reconfigure() {
    for (unsigned ch = 0; ch < kPbusChannels; ++ch)
        if (pbus_[ch].active) pbus_schedule(ch, false);
}

u32 Hpc3::pbus_read(u32 offset) {
    const unsigned ch = offset / kPbusStride;
    PbusChannel& c = pbus_[ch];
    switch (offset % kPbusStride) {
        case 0: {  // buffer pointer: how far the current descriptor has got
            u32 moved = 0;
            if (c.active && c.bytes_per_cycle > 0 && c.bc) {
                moved = std::min<u32>(c.bc, static_cast<u32>((scheduler_.now() - c.started) * c.bytes_per_cycle) & ~1u);
                pbus_transfer(ch, moved);  // the audio has moved up to here
            }
            return c.bptr + moved;
        }
        case 4: return c.dptr;
        case kPbusCtrl: {
            const u32 v = (c.irq ? kPbusStatInt : 0) | (c.active ? kPbusStatActive : 0);
            c.irq = false;  // reading acknowledges the interrupt
            update_dma_irq();
            return v;
        }
        default: return 0;
    }
}

void Hpc3::pbus_write(u32 offset, u32 value) {
    const unsigned ch = offset / kPbusStride;
    PbusChannel& c = pbus_[ch];
    switch (offset % kPbusStride) {
        case 0:
            c.bptr = value;
            c.filled = 0;
            return;
        case 4:
            c.dptr = value;
            return;
        case kPbusCtrl: {
            c.ctrl = value;
            if (!(value & kPbusCtrlLoad)) return;
            const bool start = (value & kPbusCtrlAct) && !c.active;
            c.active = value & kPbusCtrlAct;
            if (start) {  // from the descriptor at dptr
                c.bc = 0;
                c.loaded = false;
                c.bytes_per_cycle = 0;
            }
            if (c.active) {
                pbus_schedule(ch, start);
            } else {
                pbus_timer_[ch]->cancel();
                c.bytes_per_cycle = 0;
            }
            return;
        }
        default: return;
    }
}

// Moves bytes between the SCSI controller and memory along the descriptor
// chain: `src` set = device to memory, `dst` set = memory to device.
u32 Hpc3::scsi_dma(const u8* src, u8* dst, u32 length) {
    ScsiDma& d = scsi_dma_;
    if (!d.active) return 0;  // not started yet: the controller waits for it
    u32 moved = 0;
    while (moved < length && d.active) {
        if (d.bc == 0) {
            if (d.loaded && (d.flags & kDescEox)) {
                d.active = false;
                break;
            }
            u32 pbuf = 0, cnt = 0, next = 0;
            bus_.read<u32>(d.ndbp, pbuf);
            bus_.read<u32>(d.ndbp + 4, cnt);
            bus_.read<u32>(d.ndbp + 8, next);
            d.cbp = pbuf;
            d.bc = cnt & kDescCountMask;
            d.flags = cnt;
            d.ndbp = next;
            d.loaded = true;
            continue;
        }
        const u32 n = std::min(d.bc, length - moved);
        for (u32 done = 0; done < n;) {
            const u32 addr = d.cbp + done;
            const u32 chunk = std::min<u32>(n - done, static_cast<u32>(Bus::kPageSize - (addr & Bus::kPageMask)));
            if (src) {
                if (!bus_.write_block(addr, src + moved + done, chunk))  // also drops cached code it overwrites
                    for (u32 i = 0; i < chunk; ++i) bus_.write<u8>(addr + i, src[moved + done + i]);
            } else {
                if (const u8* host = bus_.host_read_ptr(addr)) std::memcpy(dst + moved + done, host, chunk);
                else for (u32 i = 0; i < chunk; ++i) bus_.read<u8>(addr + i, dst[moved + done + i]);
            }
            done += chunk;
        }
        d.cbp += n;
        d.bc -= n;
        moved += n;
        if (d.bc == 0 && (d.flags & kDescXie)) {
            d.irq = true;
            update_dma_irq();
        }
        if (d.bc == 0 && (d.flags & kDescEox)) d.active = false;  // end of the chain
    }
    if (moved < length) SG_LOG(Debug, "hpc3", "SCSI DMA chain ended after %u of %u bytes", moved, length);
    return moved;
}

void Hpc3::update_dma_irq() {
    bool pbus = false;
    for (const PbusChannel& c : pbus_) pbus |= c.irq;
    ioc_.set_local1(Ioc2::kL1HpcDma, scsi_dma_.irq || pbus);
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
    if (offset >= kScsi0WindowEnd && offset < kScsi1WindowEnd) {  // no chip: reads float low
        value = 0;
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

bool Hpc3::read_is_pure(u32 offset, unsigned size) const {
    if (offset >= kIocWindow && offset < kIocWindowEnd) {
        const u32 o = size == 8 ? offset + 4 : offset;
        const int idx = byte_index(o, size == 8 ? 4 : size, kIocWindow);
        return idx < 0 || ioc_.read_is_pure(static_cast<unsigned>(idx));
    }
    return false;
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
    if (offset >= kScsi0WindowEnd && offset < kScsi1WindowEnd) return true;
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
        // A PIO access is always a whole word on the HPC3's side, and a write
        // never reads: the other lanes are taken from the last word written,
        // so a byte at lane 3 reaches an 8-bit device as is.
        u32& last = written_[word];
        last = (last & ~mask) | ((static_cast<u32>(value) << shift) & mask);
        write_word(word, last);
    }
    return true;
}

u32 Hpc3::read_word(u32 offset) {
    if (offset < kPbusDmaEnd) return pbus_read(offset);
    if (offset >= kEnetStart && offset < kEnetEnd) return enet_read(offset);
    if (offset == kEnetRxCurDesc) return enet_rx_.cdp;
    if (offset >= kSeeqWindow && offset < kSeeqWindowEnd) return seeq_.read((offset - kSeeqWindow) >> 2);
    if (offset >= kHal2Window && offset < kHal2WindowEnd) return hal2_.read(offset - kHal2Window);
    switch (offset) {
        case kIntStat0:
        case kIntStat1: {
            u32 v = scsi_dma_.irq ? kIntStatScsi0 : 0;
            for (unsigned ch = 0; ch < kPbusChannels; ++ch)
                if (pbus_[ch].irq) v |= 1u << ch;
            return v;
        }
        case kScsi0Cbp: return scsi_dma_.cbp;
        case kScsi0Ndbp: return scsi_dma_.ndbp;
        case kScsi0Bc: return (scsi_dma_.flags & ~kDescCountMask) | scsi_dma_.bc;
        case kScsi0Ctrl: {
            const u32 v = (scsi_dma_.ctrl & (kScsiCtrlEndian | kScsiCtrlDir | kScsiCtrlFlush)) |
                          (scsi_dma_.active ? kScsiCtrlActive : 0) | (scsi_dma_.irq ? kScsiCtrlIrq : 0) |
                          (scsi_dma_.in_reset ? kScsiCtrlReset : 0);
            scsi_dma_.irq = false;  // reading the control register acknowledges the interrupt
            update_dma_irq();
            return v;
        }
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
    if (offset < kPbusDmaEnd) return pbus_write(offset, value);
    if (offset >= kEnetStart && offset < kEnetEnd) return enet_write(offset, value);
    if (offset >= kSeeqWindow && offset < kSeeqWindowEnd) return seeq_.write((offset - kSeeqWindow) >> 2, static_cast<u8>(value));
    if (offset >= kHal2Window && offset < kHal2WindowEnd) return hal2_.write(offset - kHal2Window, value);
    if (offset >= kPbusDmaCfg && offset < kPbusDmaCfg + kPbusChannels * kPbusDmaCfgStride) {
        regs_[offset] = value;
        if (offset % kPbusDmaCfgStride == 0) pbus_reconfigure();  // the sample size may have changed
        return;
    }
    switch (offset) {
        case kEepromReg:
            eeprom_reg_ = value & 0x1F;
            eeprom_.set_lines(value & kEepromCs, value & kEepromClk, value & kEepromDataOut, value & kEepromPre);
            return;
        case kGioMisc: regs_[offset] = value & 3; return;
        case kScsi0Ndbp:  // the descriptor in use, if any, is finished first
            scsi_dma_.ndbp = value;
            if (!scsi_dma_.active) {
                scsi_dma_.bc = 0;
                scsi_dma_.loaded = false;
            }
            return;
        case kScsi0Ctrl: {
            ScsiDma& d = scsi_dma_;
            if (value & kScsiCtrlReset) {  // holds the channel and the WD33C93 in reset, and resets the bus
                if (!d.in_reset) scsi0_.bus_reset();
                scsi0_.reset();
                d = ScsiDma{};
                d.in_reset = true;
                update_dma_irq();
                return;
            }
            if (d.in_reset) {
                d.in_reset = false;
                scsi0_.reset_released();
            }
            d.ctrl = value & (kScsiCtrlEndian | kScsiCtrlDir | kScsiCtrlFlush);
            if (!(value & kScsiCtrlActiveMask)) {
                const bool start = (value & kScsiCtrlActive) && !d.active;
                d.active = value & kScsiCtrlActive;
                if (start) {  // (re)start from the descriptor at NDBP
                    d.bc = 0;
                    d.loaded = false;
                }
            }
            if (value & kScsiCtrlFlush) d.active = false;
            if (d.active) scsi0_.dma_kick();  // the controller may be waiting for it
            return;
        }
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
