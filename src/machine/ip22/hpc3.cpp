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

constexpr u32 kInt2Window = 0x59000;  // PBUS PIO channel 4 (Indigo2: INT2)
constexpr u32 kInt2WindowEnd = kInt2Window + 0x400;
constexpr u32 kIocWindow = 0x59800;  // PBUS PIO channel 6
constexpr u32 kIocWindowEnd = kIocWindow + 0x400;
constexpr unsigned kIocRegisters = 64;  // the IOC decodes 6 address bits: repeated through the window
constexpr u32 kExtIo = kIocWindow + 4 * Ioc2::kExtIoIndex;  // Indigo2: 16 bits, in the low half of the word
constexpr u32 kScsiWindow = 0x40000;  // WD33C93s, 0x8000 apart: SASR at word 0, data at word 1
constexpr u32 kScsiWindowSize = 0x8000;
constexpr u32 kScsiRegs = 0x10000;  // channel n's DMA registers at + n * 0x2000
constexpr u32 kScsiRegsEnd = 0x14000;
constexpr u32 kScsiCbp = 0x0000, kScsiNdbp = 0x0004, kScsiBc = 0x1000, kScsiCtrl = 0x1004;
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
constexpr u32 kIntStatScsi0 = 1u << 8;  // and channel 1 at bit 9
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
constexpr u32 kEnetCtrlRbo = 1u << 11;  // receive buffer overflow, cleared by reading the register
constexpr u32 kEnetResetChannel = 1u << 0, kEnetResetClearIrq = 1u << 1, kEnetResetLoopback = 1u << 2;
// Ethernet descriptor count word
constexpr u32 kEnetDescEnd = 1u << 31;        // end of chain (transmit) / ring (receive)
constexpr u32 kEnetDescEndPacket = 1u << 30;  // end of packet
constexpr u32 kEnetDescXie = 1u << 29;
constexpr u32 kEnetDescTxDone = 1u << 15;     // set by the HPC once sent
constexpr u32 kEnetDescOwn = 1u << 14;        // receive buffer handed to the HPC
constexpr std::size_t kEnetMinFrame = 60;     // without the FCS
constexpr std::size_t kEnetMaxFrame = 1518;
// Battery-backed RAM: 32K words, of which the DS1386-8K decodes 13 address
// lines, so it repeats four times.
constexpr u32 kRtcWindow = 0x60000;
constexpr u32 kRtcWindowEnd = 0x80000;

// EEPROM register bits
constexpr u32 kEepromPre = 1u << 0;  // protect register enable
constexpr u32 kEepromCs = 1u << 1;
constexpr u32 kEepromClk = 1u << 2;
constexpr u32 kEepromDataOut = 1u << 3;  // CPU -> EEPROM
constexpr u32 kEepromDataIn = 1u << 4;   // EEPROM -> CPU

}  // namespace

Hpc3::Hpc3(Bus& bus, Ioc2& ioc, Scheduler& scheduler, double cpu_hz, bool full_house)
    : bus_(bus), ioc_(ioc), scheduler_(scheduler), cpu_hz_(cpu_hz), full_house_(full_house),
      scsi0_(scheduler, cpu_hz / 1000.0), scsi1_(scheduler, cpu_hz / 1000.0) {
    for (unsigned ch = 0; ch < kPbusChannels; ++ch)
        pbus_timer_[ch] = std::make_unique<Timer>(scheduler, [this, ch] { pbus_complete(ch); });
    hal2_.set_change_callback([this] { pbus_reconfigure(); });
    enet_tx_timer_ = std::make_unique<Timer>(scheduler, [this] { enet_transmit(); });
    enet_poll_timer_ = std::make_unique<Timer>(scheduler, [this] { enet_poll(); });
    for (unsigned ch = 0; ch < 2; ++ch) {
        scsi_[ch]->set_irq_callback(
            [this, ch](bool level) { ioc_.set_local0(ch ? Ioc2::kL0Scsi1 : Ioc2::kL0Scsi0, level); });
        scsi_[ch]->set_dma([this, ch](const u8* data, u32 n) { return scsi_dma(ch, data, nullptr, n); },
                           [this, ch](u8* data, u32 n) { return scsi_dma(ch, nullptr, data, n); });
    }
    reset();
}

void Hpc3::reset() {
    regs_.clear();
    written_.clear();
    eeprom_reg_ = 0;
    enet_wire_free_ = 0;
    eeprom_.set_lines(false, false, false);
    for (unsigned ch = 0; ch < 2; ++ch) {
        scsi_[ch]->reset();
        scsi_dma_[ch] = ScsiDma{};
        scsi_dma_[ch].in_reset = true;  // the channel reset bit is set at power-on
    }
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

void Hpc3::set_fast_io(bool fast) {
    fast_io_ = fast;
    scsi0_.set_fast_transfers(fast);
    scsi1_.set_fast_transfers(fast);
}

// The next receive descriptor has been handed to the HPC.
bool Hpc3::enet_rx_ready() const {
    u32 cnt = 0;
    return enet_rx_.active && bus_.read<u32>(enet_rx_.ndbp + 4, cnt) && (cnt & kEnetDescOwn);
}

// The least time from a frame leaving to the first frame back: the other
// end's turnaround (200 us, about as quick as a host on the wire answered),
// then a minimum frame's time on the wire. IRIX misses some replies that
// come back much sooner (pings to the NAT's own addresses, with fast I/O
// at 50 us).
u64 Hpc3::enet_turnaround_cycles() const {
    return static_cast<u64>(200e-6 * cpu_hz_) + enet_wire_cycles(kEnetMinFrame);
}

u64 Hpc3::enet_wire_cycles(std::size_t bytes) const {
    const std::size_t wire = std::max(bytes, kEnetMinFrame) + 24;
    return std::max<u64>(1, static_cast<u64>(static_cast<double>(wire) * 8 / enet_bits_per_second() * cpu_hz_));
}

// A frame from the host is in the guest's buffer only once it has crossed
// the wire, which it can start doing only after the frame being sent is off
// it and the other end has had time to answer: a reply the host makes at
// once (the NAT answering a ping to its own address, an ARP reply) must not
// land as the request leaves. IRIX loses such replies.
void Hpc3::poll_network() {
    if (net_ && !enet_poll_timer_->active() && net_->pending()) {
        const u64 now = scheduler_.now();
        enet_poll_timer_->schedule_in(std::max(now, enet_wire_free_) - now + enet_turnaround_cycles());
    }
}

// Resets both channels and the Seeq (the interrupt is cleared as well).
void Hpc3::enet_reset() {
    enet_rx_.active = enet_tx_.active = false;
    enet_rx_.ctrl = enet_tx_.ctrl = 0;
    enet_rx_.rbo = false;
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
// the poll stops until poll_network() sees more. With fast I/O a frame waits
// (looked at again every 50 us) while the driver has no buffer ready, which
// includes the channel stopped at the end of its ring until restarted.
void Hpc3::enet_poll() {
    if (!net_) return;
    if (const u64 now = scheduler_.now(); now < enet_wire_free_) {  // still sending
        enet_poll_timer_->schedule_in(enet_wire_free_ - now + enet_turnaround_cycles());
        return;
    }
    if (fast_io_ && !enet_held_ && !enet_rx_ready()) {
        enet_poll_timer_->schedule_in(std::max<u64>(1, static_cast<u64>(cpu_hz_ * 50e-6)));
        return;
    }
    std::vector<u8> frame;
    if (net_->receive(frame)) {
        enet_receive(frame.data(), frame.size());
        enet_poll_timer_->schedule_in(enet_wire_cycles(frame.size()));
    }
}

// The frame is written as two bytes of padding, the frame (padded to the
// minimum length) and the receive status, starting a new descriptor. Only
// descriptors the driver has handed over (OWN set) take data; the HPC clears
// OWN and leaves the unused byte count in each one it fills.
// Each packet goes into one descriptor's buffer (the HPC3 takes exactly one
// per packet): two bytes of padding, the frame, then the Seeq's status byte.
// A packet larger than the buffer fills it, sets RBO and stops the channel.
void Hpc3::enet_receive(const u8* frame, std::size_t length) {
    if (!seeq_.accepts(frame, length) || length > kEnetMaxFrame) return;
    ++enet_rx_frames_;
    if (!enet_rx_.active) return;  // no buffers: the frame is lost
    const u32 desc = enet_rx_.ndbp;
    enet_rx_.cdp = desc;
    u32 pbuf = 0, cnt = 0, next = 0;
    bus_.read<u32>(desc, pbuf);
    bus_.read<u32>(desc + 4, cnt);
    bus_.read<u32>(desc + 8, next);
    // The driver has not handed this buffer back yet (OWN is the drivers'
    // convention; the HPC3 itself only stops at EOX): the frame is dropped
    // rather than written over a buffer still in use, and the channel stays
    // ready for the next one.
    if (!(cnt & kEnetDescOwn)) return;
    std::vector<u8> data(2 + std::max(length, kEnetMinFrame) + 1, 0);
    std::memcpy(&data[2], frame, length);
    data.back() = Seeq8003::kRxGood | Seeq8003::kRxEndOfFrame;
    const u32 room = cnt & kDescCountMask;
    const bool fits = data.size() <= room;
    const u32 n = fits ? static_cast<u32>(data.size()) : room;
    // Little-endian transfers carry byte n of each doubleword in its bits
    // 8n+7..8n, which in the big-endian memory is the byte at address ^ 7.
    if (enet_rx_.ctrl & kEnetCtrlEndian) {
        for (u32 i = 0; i < n; ++i) bus_.write<u8>((pbuf + i) ^ 7, data[i]);
    } else if (!bus_.write_block(pbuf, data.data(), n)) {
        for (u32 i = 0; i < n; ++i) bus_.write<u8>(pbuf + i, data[i]);
    }
    bus_.write<u32>(desc + 4, (cnt & ~(kDescCountMask | kEnetDescOwn)) | (room - n) | kEnetDescEndPacket);
    enet_rx_.cbp = pbuf + n;
    bool irq = (cnt & kEnetDescXie) != 0;
    if (!fits) {
        enet_rx_.rbo = true;
        enet_rx_.active = false;
        irq = true;
    } else if (cnt & kEnetDescEnd) {  // end of the ring: receiving stops
        enet_rx_.active = false;
        irq = true;
    } else {
        enet_rx_.ndbp = next;
    }
    irq |= seeq_.received(data.back());  // the Seeq took the frame whole
    if (irq) enet_interrupt();
}

// Sends the frame starting at the next descriptor, then times the next one.
void Hpc3::enet_transmit() {
    if (!enet_tx_.active) return;
    std::vector<u8> frame;
    const bool little = enet_tx_.ctrl & kEnetCtrlEndian;  // see enet_receive
    bool irq = false, end_of_packet = false;
    u32 first_desc = 0, first_cnt = 0;
    for (int descs = 0; descs < 64 && !end_of_packet; ++descs) {
        const u32 desc = enet_tx_.ndbp;
        u32 pbuf = 0, cnt = 0, next = 0;
        bus_.read<u32>(desc, pbuf);
        bus_.read<u32>(desc + 4, cnt);
        bus_.read<u32>(desc + 8, next);
        const u32 n = cnt & kDescCountMask;
        const std::size_t at = frame.size();
        frame.resize(std::min(at + n, kEnetMaxFrame));
        for (std::size_t i = at; i < frame.size(); ++i)
            bus_.read<u8>(static_cast<u32>(pbuf + (i - at)) ^ (little ? 7u : 0u), frame[i]);
        if (descs == 0) first_desc = desc, first_cnt = cnt;
        enet_tx_.cbp = pbuf + n;
        end_of_packet = cnt & kEnetDescEndPacket;
        irq |= end_of_packet && (cnt & kEnetDescXie);  // XIE counts on a packet's last descriptor only
        if (cnt & kEnetDescEnd) {
            enet_tx_.active = false;
            break;
        }
        enet_tx_.ndbp = next;
    }
    // Sent: TXD goes into the packet's first descriptor, clearing its count
    // (bits 14:0).
    bus_.write<u32>(first_desc + 4, (first_cnt & ~0x7FFFu) | kEnetDescTxDone);
    if (frame.size() < kEnetMinFrame) frame.resize(kEnetMinFrame, 0);
    ++enet_tx_frames_;
    if (enet_loopback_) {
        if (seeq_.receives_own()) enet_receive(frame.data(), frame.size());
    } else if (net_) {
        net_->send(frame.data(), frame.size());
        enet_wire_free_ = scheduler_.now() + enet_wire_cycles(frame.size());
    }
    irq |= seeq_.transmitted(end_of_packet ? Seeq8003::kTxDone : Seeq8003::kTxUnderflow, enet_loopback_ || net_);
    if (irq) enet_interrupt();
    // Preamble, FCS and the gap between frames: 24 bytes more on the wire.
    if (enet_tx_.active) enet_tx_timer_->schedule_in(enet_wire_cycles(frame.size()));
}

u32 Hpc3::enet_read(u32 offset) {
    switch (offset) {
        case kEnetRxCbp: return enet_rx_.cbp;
        case kEnetRxNdbp: return enet_rx_.ndbp;
        case kEnetRxCtrl: {
            const u32 v = seeq_.rx_status() | (enet_rx_.ctrl & kEnetCtrlEndian) |
                          (enet_rx_.active ? kEnetCtrlActive : 0) | (enet_rx_.rbo ? kEnetCtrlRbo : 0);
            enet_rx_.rbo = false;
            return v;
        }
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
u32 Hpc3::scsi_dma(unsigned ch, const u8* src, u8* dst, u32 length) {
    ScsiDma& d = scsi_dma_[ch];
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
        if (d.ctrl & kScsiCtrlEndian) {  // little-endian: each byte at its address ^ 7 (see enet_receive)
            for (u32 i = 0; i < n; ++i) {
                const u32 addr = (d.cbp + i) ^ 7;
                if (src) bus_.write<u8>(addr, src[moved + i]);
                else bus_.read<u8>(addr, dst[moved + i]);
            }
        }
        for (u32 done = (d.ctrl & kScsiCtrlEndian) ? n : 0; done < n;) {
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
    ioc_.set_local1(Ioc2::kL1HpcDma, scsi_dma_[0].irq || scsi_dma_[1].irq || pbus);
}

// A register PIO moves one 32-bit word whatever the size asked for (the
// address's two low bits are ignored), so every access reaches the device.
// PBUS devices' registers are replicated to fill the word: an 8-bit
// register's byte in all four lanes, a 16-bit one's halfword in both
// halves. (Other external registers, the WD33C93's, use bits 7:0 and read
// zero elsewhere.)
namespace {
// The register a read addresses (a doubleword read takes its low word's).
u32 pbus_word(u32 offset, unsigned size, u32 window_base) {
    return ((size == 8 ? offset + 4 : offset) - window_base) >> 2;
}
u64 replicated8(u8 v, unsigned size) {
    const u64 w = v * 0x01010101ull;
    return size == 8 ? (w << 32) | w : size == 4 ? w : size == 2 ? v * 0x0101u : v;
}
u64 lanes_of_word(u32 w, u32 offset, unsigned size) {
    if (size == 8) return (u64{w} << 32) | w;
    if (size == 4) return w;
    const unsigned shift = (4 - size - (offset & 3)) * 8;
    return (w >> shift) & ((1u << (size * 8)) - 1);
}
}  // namespace

int Hpc3::byte_index(u32 offset, unsigned size, u32 window_base) {
    const u32 rel = offset - window_base;
    if (size == 1) return (rel & 3) == 3 ? static_cast<int>(rel >> 2) : -1;
    if (size == 2) return (rel & 3) == 2 ? static_cast<int>(rel >> 2) : -1;
    return static_cast<int>(rel >> 2);  // word or doubleword: data in the low byte of the (last) word
}

bool Hpc3::read(u32 offset, unsigned size, u64& value) {
    if (offset >= kScsiWindow && offset < kScsiWindow + 2 * kScsiWindowSize) {
        const unsigned ch = (offset - kScsiWindow) / kScsiWindowSize;
        if (ch == 1 && !full_house_) {  // no chip: reads float low
            value = 0;
            return true;
        }
        const u32 word = pbus_word(offset, size, kScsiWindow + ch * kScsiWindowSize);
        const u8 v = (word & 1) ? scsi_[ch]->read_data() : scsi_[ch]->read_aux_status();
        value = size == 8 ? v : lanes_of_word(v, offset, size);
        return true;
    }
    if (full_house_ && offset >= kInt2Window && offset < kInt2WindowEnd) {
        value = replicated8(ioc_.int2_read(pbus_word(offset, size, kInt2Window)), size);
        return true;
    }
    if (full_house_ && (offset & ~3u) == kExtIo) {
        const u32 w = ioc_.ext_io() & 0xFFFFu;
        value = lanes_of_word(w | (w << 16), offset, size);
        return true;
    }
    if (offset >= kIocWindow && offset < kIocWindowEnd) {
        value = replicated8(ioc_.read(pbus_word(offset, size, kIocWindow) % kIocRegisters), size);
        return true;
    }
    if (offset >= kRtcWindow && offset < kRtcWindowEnd) {
        value = replicated8(rtc_.read(pbus_word(offset, size, kRtcWindow) % rtc_.memory().size()), size);
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
    if (full_house_ && offset >= kInt2Window && offset < kInt2WindowEnd) {
        return ioc_.int2_read_is_pure(pbus_word(offset, size, kInt2Window));
    }
    if (full_house_ && (offset & ~3u) == kExtIo) return true;
    if (offset >= kIocWindow && offset < kIocWindowEnd) {
        return ioc_.read_is_pure(pbus_word(offset, size, kIocWindow) % kIocRegisters);
    }
    return false;
}

bool Hpc3::write(u32 offset, unsigned size, u64 value) {
    if (offset >= kScsiWindow && offset < kScsiWindow + 2 * kScsiWindowSize) {
        const unsigned ch = (offset - kScsiWindow) / kScsiWindowSize;
        if (ch == 1 && !full_house_) return true;
        const u32 o = size == 8 ? offset + 4 : offset;
        const int idx = byte_index(o, size == 8 ? 4 : size, kScsiWindow + ch * kScsiWindowSize);
        if (idx >= 0) {
            if (idx & 1) scsi_[ch]->write_data(static_cast<u8>(value));
            else scsi_[ch]->write_address(static_cast<u8>(value));
        }
        return true;
    }
    if (full_house_ && offset >= kInt2Window && offset < kInt2WindowEnd) {
        const u32 o = size == 8 ? offset + 4 : offset;
        const int idx = byte_index(o, size == 8 ? 4 : size, kInt2Window);
        if (idx >= 0) ioc_.int2_write(static_cast<unsigned>(idx), static_cast<u8>(value));
        return true;
    }
    if (full_house_ && (offset & ~3u) == kExtIo) {
        ioc_.ext_io_write(static_cast<u16>(value));
        return true;
    }
    if (offset >= kIocWindow && offset < kIocWindowEnd) {
        const u32 o = size == 8 ? offset + 4 : offset;
        const int idx = byte_index(o, size == 8 ? 4 : size, kIocWindow);
        if (idx >= 0) ioc_.write(static_cast<unsigned>(idx) % kIocRegisters, static_cast<u8>(value));
        return true;
    }
    if (offset >= kRtcWindow && offset < kRtcWindowEnd) {
        const u32 o = size == 8 ? offset + 4 : offset;
        const int idx = byte_index(o, size == 8 ? 4 : size, kRtcWindow);
        if (idx >= 0) rtc_.write(static_cast<unsigned>(idx) % rtc_.memory().size(), static_cast<u8>(value));
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

u32 Hpc3::scsi_read(unsigned ch, u32 reg) {
    ScsiDma& d = scsi_dma_[ch];
    switch (reg) {
        case kScsiCbp: return d.cbp;
        case kScsiNdbp: return d.ndbp;
        case kScsiBc: return (d.flags & ~kDescCountMask) | d.bc;
        case kScsiCtrl: {
            const u32 v = (d.ctrl & (kScsiCtrlEndian | kScsiCtrlDir | kScsiCtrlFlush)) |
                          (d.active ? kScsiCtrlActive : 0) | (d.irq ? kScsiCtrlIrq : 0) |
                          (d.in_reset ? kScsiCtrlReset : 0);
            d.irq = false;  // reading the control register acknowledges the interrupt
            update_dma_irq();
            return v;
        }
        default: {
            const auto it = regs_.find(kScsiRegs + ch * 0x2000 + reg);
            return it != regs_.end() ? it->second : 0;
        }
    }
}

void Hpc3::scsi_write(unsigned ch, u32 reg, u32 value) {
    ScsiDma& d = scsi_dma_[ch];
    Wd33c93& wd = *scsi_[ch];
    switch (reg) {
        case kScsiNdbp:  // the descriptor in use, if any, is finished first
            d.ndbp = value;
            if (!d.active) {
                d.bc = 0;
                d.loaded = false;
            }
            return;
        case kScsiCtrl:
            if (value & kScsiCtrlReset) {  // holds the channel and the WD33C93 in reset, and resets the bus
                if (!d.in_reset) wd.bus_reset();
                wd.reset();
                d = ScsiDma{};
                d.in_reset = true;
                update_dma_irq();
                return;
            }
            if (d.in_reset) {
                d.in_reset = false;
                wd.reset_released();
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
            if (d.active) wd.dma_kick();  // the controller may be waiting for it
            return;
        default: regs_[kScsiRegs + ch * 0x2000 + reg] = value; return;
    }
}

u32 Hpc3::read_word(u32 offset) {
    if (offset < kPbusDmaEnd) return pbus_read(offset);
    if (offset >= kScsiRegs && offset < kScsiRegsEnd) return scsi_read((offset - kScsiRegs) / 0x2000, offset & 0x1FFF);
    if (offset >= kEnetStart && offset < kEnetEnd) return enet_read(offset);
    if (offset == kEnetRxCurDesc) return enet_rx_.cdp;
    if (offset >= kSeeqWindow && offset < kSeeqWindowEnd) return seeq_.read((offset - kSeeqWindow) >> 2);
    if (offset >= kHal2Window && offset < kHal2WindowEnd) {  // a 16-bit PBUS device: both halves
        const u32 v = hal2_.read(offset - kHal2Window) & 0xFFFFu;
        return v | (v << 16);
    }
    switch (offset) {
        case kIntStat0:
        case kIntStat1: {
            u32 v = (scsi_dma_[0].irq ? kIntStatScsi0 : 0) | (scsi_dma_[1].irq ? kIntStatScsi0 << 1 : 0);
            for (unsigned ch = 0; ch < kPbusChannels; ++ch)
                if (pbus_[ch].irq) v |= 1u << ch;
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
    if (offset >= kScsiRegs && offset < kScsiRegsEnd) return scsi_write((offset - kScsiRegs) / 0x2000, offset & 0x1FFF, value);
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
