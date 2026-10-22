// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "machine/ip22/impact.h"

#include <algorithm>
#include <cstring>
#include <deque>
#include <utility>

#include "core/log.h"
#include "core/worker_pool.h"

namespace sg::ip22 {

namespace {

// Window offsets
constexpr u32 kWindow = 0x100000;
constexpr u32 kUcode = 0x40000, kUcodeWords = 6144;
constexpr u32 kPriv = 0x50000, kPrivSize = 0x10000;
constexpr u32 kDcb = 0x60000, kDcbTiming = 0x68000, kDcbEnd = 0x70000;
constexpr u32 kUser = 0x70000, kUserEnd = 0x7C000;
constexpr u32 kRss = 0x7C000, kRssExec = 0x7D000, kRssEnd = 0x7E000;

// HQ3 privileged registers (offsets from 0x50000)
constexpr u32 kHqFlagSet = 0x08, kHqFlagClear = 0x0C, kHqFlagEnSet = 0x10, kHqFlagEnClear = 0x14;
constexpr u32 kHqIntEnSet = 0x18, kHqIntEnClear = 0x1C, kHqCfifoHigh = 0x20, kHqCfifoLow = 0x24;
constexpr u32 kHqDfifoLow = 0x30, kHqFifo0 = 0x80, kHqFifo1 = 0x84;
constexpr u32 kHqGe0Data = 0x40, kHqGe0Addr = 0x44, kHqGe1Data = 0x48, kHqGe1Addr = 0x4C;
constexpr u32 kHqGeDiagRead = 0x22C;  // (0x230, the pad word, reads as last written)
constexpr u32 kFlagGeData = 1u << 17;  // a GE answer is waiting in GE readback high/low
constexpr u32 kFlagGeDiag = 1u << 18;  // diagnostic readback words remain
constexpr u32 kHqContextSwitch = 0x50;
constexpr u32 kHqReifContext = 0x200, kHqDmaContext = 0x300;
// Host DMA engine registers (command 0x800 + n)
constexpr unsigned kDmaPageList = 0x00, kDmaStride = 0x04, kDmaRowOffset = 0x05, kDmaRowStart = 0x06;
constexpr unsigned kDmaLines = 0x07, kDmaLineBytes = 0x08, kDmaLastBytes = 0x09, kDmaStart = 0x0B;
constexpr unsigned kDmaDlJump = 0x0C, kDmaDlPush = 0x37, kDmaDlReturn = 0x39, kDmaPoolTable = 0x20, kDmaDlTable = 0x28;
constexpr unsigned kDlMaxSegments = 1u << 20, kDlMaxDepth = 64;  // (GL nests lists 64 deep)
constexpr u32 kFlagContextSaved = 1u << 19, kFlagContextLoaded = 1u << 6;
constexpr u32 kFlagCp0 = 1u << 10;  // command-processor flag 0: a scheduled buffer swap has happened
constexpr u32 kCtxFirstLoad = 1u << 31;
// User registers (offsets from 0x70000)
constexpr u32 kUStatus = 0x00, kUFifoStatus = 0x04, kUFlagSet = 0x08, kUFlagClear = 0x0C;
constexpr u32 kUGeHigh = 0x10, kUGeLow = 0x14, kUFifo0 = 0x80, kUFifo1 = 0x84, kUFifoGl0 = 0x4080, kUFifoGl1 = 0x4084;
constexpr u32 kUGioStatus = 0x100, kUDmaBusy = 0x104;

// Flags and interrupt enables
constexpr u32 kFlagDone = 1u << 16;
constexpr u32 kIntCfifoLow = 1u << 27, kIntDfifoLow = 1u << 31;  // (26 and 30: the high-water marks)

// DCB devices
constexpr unsigned kDevCmapAll = 3, kDevCmap0 = 4, kDevCmap1 = 5, kDevDac = 6, kDevXmap = 7, kDevVc3 = 8;
constexpr unsigned kDevBoardVersion = 9, kDevI2c = 11;
// VC3 registers
constexpr unsigned kVc3CursorGlyph = 1, kVc3CursorX = 2, kVc3CursorY = 3, kVc3MainDids = 4, kVc3OverlayDids = 5;
constexpr unsigned kVc3SramPtr = 7, kVc3Scanline = 0xB, kVc3CursorControl = 0x1D, kVc3DisplayControl = 0x1E;
constexpr unsigned kLinesPerFrame = 1066;

constexpr u64 kWord36 = (u64{1} << 36) - 1;

// 4x4 ordered dither thresholds (0-15).
constexpr u8 kBayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};

u64 logic_op(unsigned op, u64 src, u64 dst) {
    switch (op & 15) {
        case 0x0: return 0;
        case 0x1: return src & dst;
        case 0x2: return src & ~dst;
        case 0x3: return src;
        case 0x4: return ~src & dst;
        case 0x5: return dst;
        case 0x6: return src ^ dst;
        case 0x7: return src | dst;
        case 0x8: return ~(src | dst);
        case 0x9: return ~(src ^ dst);
        case 0xA: return ~dst;
        case 0xB: return src | ~dst;
        case 0xC: return ~src;
        case 0xD: return ~src | dst;
        case 0xE: return ~(src & dst);
        default: return ~u64{0};
    }
}

int half(u32 v) { return static_cast<s16>(v & 0xFFFF); }  // a signed 16-bit field

bool pixel_type_ci(u32 pp1) { return ((pp1 >> 8) & 7) >= 4; }  // PP1FILLMODE pixel types 4-7

}  // namespace

Impact::Impact(Scheduler& scheduler, double cpu_hz)
    : scheduler_(scheduler),
      cpu_hz_(cpu_hz),
      ucode_(kUcodeWords),
      priv_(kPrivSize / 4),
      retrace_timer_(scheduler, [this] { retrace(); }),
      retrace_end_timer_(scheduler, [this] { set_line(kIrqRetrace, false); }) {
    for (unsigned e = 0; e < 2; ++e) {
        pixmem_[e].assign(std::size_t{kPages} * kTileWidth * kTileRows, 0);
        row_written_[e].assign(std::size_t{kPages} * kTileRows, 0);
    }
    reset();
}

void Impact::reset() {
    std::fill(ucode_.begin(), ucode_.end(), 0);
    std::fill(priv_.begin(), priv_.end(), 0);
    flags_ = flag_enable_ = int_enable_ = 0;
    cfifo_high_ = cfifo_low_ = 0;
    user_port_ = FifoPort{};
    priv_port_ = FifoPort{};
    dma_regs_.fill(0);
    reif_regs_.fill(0);
    formatter_ = 0;
    for (GeDiag& g : ge_) {
        g.addr = 0;
        g.word = 0;
        std::fill(g.ucode.begin(), g.ucode.end(), std::array<u32, 3>{});
    }
    ge_readback_.clear();
    ge_answer_.fill(0);
    ctx_words_ = 0;
    ctx_image_.fill(0);
    ctx_slot_.reset();
    gl_window_ = GlWindow{};
    gl_.reset();
    for (auto& e : eram_) std::fill(e.begin(), e.end(), 0);
    nosave_next_ = false;
    eram_in_.reset();
    eram_write_ge0_.reset();
    eram_read_ge0_.reset();
    eram_write_ge_.reset();
    eram_out_.clear();
    pixel_buf_.clear();
    gl_pixels_.clear();
    zst_.clear();
    std::fill(cid_.begin(), cid_.end(), u8{0});
    dcb_timing_.fill(0);
    for (Cmap& c : cmap_) {
        std::fill(c.entries.begin(), c.entries.end(), 0);
        c.addr = 0;
        c.command = 0;
    }
    dac_ = Dac{};
    for (unsigned i = 0; i < 256; ++i)
        for (unsigned c = 0; c < 3; ++c) dac_.gamma[i][c] = static_cast<u16>((i << 2) | (i >> 6));
    dac_.regs[4] = 0xFF;  // pixel read mask
    xmap_ = Xmap{};
    put_be_bytes(xmap_.dib.data(), 4, 0x1C0C8240);  // the PROM's 1280x1024 layout
    vc3_.index = 0;
    vc3_.regs.fill(0);
    std::fill(vc3_.sram.begin(), vc3_.sram.end(), 0);
    // Board version (DCB device 9): the second raster board at revision 0,
    // four TRAMs (4 MB) on each raster board; two GEs.
    board_version_[0] = 0x0A, board_version_[1] = 0x02;
    board_version_written_ = 0;
    rss_.fill(0);
    rss_[kDrbSize] = 0x31E | 1u << 6;  // the PROM's 1280x1024 layout, two engines
    rss_[kDrbPointers] = 0xC8240;
    stipple_ = Stipple{};
    for (auto& m : pixmem_) std::fill(m.begin(), m.end(), 0);
    for (unsigned l = 0; l < line_.size(); ++l) set_line(l, false);
    retrace_end_timer_.cancel();
    last_retrace_ = scheduler_.now();
    retrace_timer_.schedule_in(static_cast<u64>(cpu_hz_ / 60));
    full_redraw_ = true;
    ++display_version_;
}

void Impact::unknown(const char* what, u32 offset, u64 value) const {
    SG_LOG(Debug, "impact", "%s %05x = %llx", what, offset, static_cast<unsigned long long>(value));
}

void Impact::set_line(unsigned line, bool level) {
    if (line_[line] == level) return;
    line_[line] = level;
    if (line != kIrqRetrace) SG_LOG(Debug, "impact", "interrupt line %u %s", line, level ? "raised" : "lowered");
    set_irq(line, level);
}

// The FIFO never holds anything between accesses: its level is 0.
u32 Impact::status() const {
    u32 s = 0x43;  // HQ3 idle, board idle, bit 6
    if (scheduler_.now() - last_retrace_ < static_cast<u64>(cpu_hz_ / 1000)) s |= 4;
    if (cfifo_low_ > 0) s |= 0x10;  // level below the low-water mark
    return s;
}

void Impact::update_irqs() {
    set_line(kIrqGraphics, (flags() & int_enable_ & 0x7FFFFF) != 0);
    // The FIFOs are empty: below any non-zero low-water mark, never at a
    // high-water mark.
    const bool dfifo_low = priv_[kHqDfifoLow / 4] > 0;
    set_line(kIrqFifo, ((int_enable_ & kIntCfifoLow) && cfifo_low_ > 0) || ((int_enable_ & kIntDfifoLow) && dfifo_low));
}

// ---- bus ------------------------------------------------------------------

bool Impact::read(u32 offset, unsigned size, u64& value) {
    if (size == 8 && !(offset >= kDcb && offset < kDcbTiming)) {
        u64 hi = 0, lo = 0;
        read(offset, 4, hi);
        read(offset + 4, 4, lo);
        value = (hi << 32) | lo;
        return true;
    }
    value = 0;
    if (offset >= kWindow) return true;
    if (offset < 4) {
        value = size == 4 ? kGioId : (kGioId >> (8 * (4 - size - (offset & 3)))) & ((1u << (8 * size)) - 1);
        return true;
    }
    if (offset >= kDcb && offset < kDcbEnd) {
        value = dcb_read(offset - kDcb, size);
        if (size == 8) value <<= 32;
        return true;
    }
    if (size != 4) {
        if (offset >= kRss && offset < kRssEnd && size < 4) {  // a byte or halfword of a raster register
            u64 w = 0;
            read(offset & ~3u, 4, w);
            value = (w >> (8 * (4 - size - (offset & 3)))) & ((1u << (8 * size)) - 1);
            return true;
        }
        unknown("narrow read", offset, size);
        return true;
    }
    if (offset == kPriv + kHqGeDiagRead && !ge_readback_.empty()) {  // reading takes the word
        value = ge_readback_.front();
        ge_readback_.pop_front();
        if (ge_readback_.empty()) update_irqs();
    } else if (offset >= kUcode && offset < kUcode + 4 * kUcodeWords) value = ucode_[(offset - kUcode) / 4];
    else if (offset >= kPriv && offset < kPriv + kPrivSize) value = priv_read(offset - kPriv);
    else if (offset >= kUser && offset < kUserEnd) value = user_read(offset - kUser);
    else if (offset >= kRss && offset < kRssEnd) {
        const u32 reg = ((offset - kRss) & 0xFFF) / 4;
        // PIO reads: char_h through the execute alias takes the next
        // doubleword, char_l (either alias) reads its low half.
        if (xfer_.armed && xfer_.pio && xfer_.read && reg == kCharH && offset >= kRssExec) value = pio_read_hi();
        else if (xfer_.armed && xfer_.pio && xfer_.read && reg == kCharL) value = xfer_.lo;
        else value = rss_read(reg);
    } else unknown("read", offset, 0);
    if (offset >= kPriv && offset < kUserEnd)
        SG_LOG(Trace, "impact", "read %05x = %08llx", offset, static_cast<unsigned long long>(value));
    return true;
}

bool Impact::write(u32 offset, unsigned size, u64 value) {
    if (offset >= kWindow) return true;
    if (offset >= kDcb && offset < kDcbEnd && !(size == 8 && offset >= kDcbTiming)) {
        dcb_write(offset - kDcb, size, value);
        return true;
    }
    if (size == 8) {
        const u32 hi = static_cast<u32>(value >> 32), lo = static_cast<u32>(value);
        if (offset >= kRss && offset < kRssEnd) {  // a register pair; any execute after the second
            const u32 reg = ((offset - kRss) & 0xFFF) / 4;
            SG_LOG(Trace, "impact", "rss %03x pair%s = %08x %08x", reg, offset >= kRssExec ? " go" : "", hi, lo);
            rss_write(reg, hi, false);
            rss_write(reg + 1, lo, offset >= kRssExec);
            return true;
        }
        write(offset, 4, hi);
        write(offset + 4, 4, lo);
        return true;
    }
    if (size != 4) {
        unknown("narrow write", offset, value);
        return true;
    }
    const u32 v = static_cast<u32>(value);
    if (offset >= kPriv && offset < kUserEnd && (offset & 0xFFF) != 0x080 && (offset & 0xFFF) != 0x084)
        SG_LOG(Trace, "impact", "write %05x = %08x", offset, v);
    if (offset >= kUcode && offset < kUcode + 4 * kUcodeWords) ucode_[(offset - kUcode) / 4] = v & 0xFFFFFF;
    else if (offset >= kPriv && offset < kPriv + kPrivSize) priv_write(offset - kPriv, v);
    else if (offset >= kUser && offset < kUserEnd) user_write(offset - kUser, v);
    else if (offset >= kRss && offset < kRssEnd) {
        SG_LOG(Trace, "impact", "rss %03x%s = %08x", ((offset - kRss) & 0xFFF) / 4, offset >= kRssExec ? " go" : "", v);
        rss_write(((offset - kRss) & 0xFFF) / 4, v, offset >= kRssExec);
    } else unknown("write", offset, v);
    return true;
}

bool Impact::read_is_pure(u32 offset, unsigned size) const {
    (void)size;
    return offset < 4 || (offset >= kUser + kUFlagSet && offset < kUser + kUGeHigh) ||
           (offset >= kPriv + kHqFlagSet && offset < kPriv + kHqCfifoHigh);
}

// ---- HQ3 ----------------------------------------------------------------------

u32 Impact::flags() const { return flags_ | (ge_readback_.empty() ? 0 : kFlagGeDiag); }

u32 Impact::priv_read(u32 offset) const {
    switch (offset) {
        case kHqFlagSet:
        case kHqFlagClear: return flags();
        case kHqGeDiagRead: return ge_readback_.empty() ? 0 : ge_readback_.front();
        case kHqFlagEnSet:
        case kHqFlagEnClear: return flag_enable_;
        case kHqIntEnSet:
        case kHqIntEnClear: return int_enable_;
        case kHqCfifoHigh: return cfifo_high_;
        case kHqCfifoLow: return cfifo_low_;
        default:
            // The DMA engine's and raster interface's context RAM: their
            // registers as the command FIFO set them (the FIFO is always
            // drained, so the HQ is idle when they are read).
            if (offset >= kHqDmaContext && offset < kHqDmaContext + 4 * dma_regs_.size())
                return dma_regs_[(offset - kHqDmaContext) / 4];
            if (offset >= kHqReifContext && offset < kHqReifContext + 4 * reif_regs_.size())
                return reif_regs_[(offset - kHqReifContext) / 4];
            return priv_[offset / 4];
    }
}

void Impact::priv_write(u32 offset, u32 value) {
    switch (offset) {
        case kHqFlagSet: flags_ |= value; break;
        case kHqFlagClear: flags_ &= ~value; break;
        case kHqFlagEnSet: flag_enable_ |= value; break;
        case kHqFlagEnClear: flag_enable_ &= ~value; break;
        case kHqIntEnSet: int_enable_ |= value; break;
        case kHqIntEnClear: int_enable_ &= ~value; break;
        case kHqCfifoHigh: cfifo_high_ = value & 0x7F; break;
        case kHqCfifoLow: cfifo_low_ = value & 0x7F; break;
        case kHqFifo0:
        case kHqFifo1: fifo_push(priv_port_, value); return;
        case kHqContextSwitch:
            priv_[offset / 4] = value;  // the switch routine's microcode address
            begin_context_switch();
            break;
        case kHqGe0Data:
        case kHqGe0Addr:
        case kHqGe1Data:
        case kHqGe1Addr: ge_diag_write(offset >= kHqGe1Data ? 1 : 0, offset & 4, value); break;
        // Stored only: hq_config, flag_ctx, the DFIFO and BFIFO marks,
        // gio_config, and the DMA TLB (its entries, current address and
        // valid bits; transfers go through the page tables).
        case 0x00: case 0x04: case 0x28: case 0x2C: case 0x30: case 0x34:
        case 0x108: case 0x10C: case 0x110: case 0x114: case 0x880: case 0x900: priv_[offset / 4] = value; break;
        default:
            priv_[offset / 4] = value;
            if (offset >= kHqDmaContext && offset < kHqDmaContext + 4 * dma_regs_.size())
                dma_regs_[(offset - kHqDmaContext) / 4] = value;
            else if (offset >= kHqReifContext && offset < kHqReifContext + 4 * reif_regs_.size())
                reif_regs_[(offset - kHqReifContext) / 4] = value;
            else if (!(offset >= 0x800 && offset < 0x880))  // (the DMA TLB entries: stored only)
                unknown("hq3 write", kPriv + offset, value);
            break;
    }
    update_irqs();
}

// A context switch: the save completes at once. The outgoing context's
// record goes to its ERAM slot: its GE state (unless the kernel asked for
// no save), its half-received user command (a process can be switched out
// midway through one), any GE answer it has not read, and its DMA and
// raster interface registers. The image of the incoming one follows
// through the FIFO.
void Impact::begin_context_switch() {
    gl_.end_bracket();
    if (ctx_slot_) save_record(*ctx_slot_);
    nosave_next_ = false;
    user_port_ = FifoPort{};
    flags_ = (flags_ & ~kFlagGeData) | kFlagContextSaved;
    ge_answer_.fill(0);
    ctx_words_ = kContextImageWords;
}

void Impact::save_record(u32 slot) {
    if (std::size_t{slot} * 4 + sizeof(ContextRecord) > kEramWords * 4) return;
    ContextRecord r;
    if (nosave_next_) std::memcpy(static_cast<void*>(&r.ge), eram_[0].data() + slot, sizeof r.ge);
    else r.ge = gl_.state();
    r.user_port = user_port_;
    r.ge_flags = flags_ & kFlagGeData;
    r.ge_answer = ge_answer_;
    r.dma_regs = dma_regs_;
    r.reif_regs = reif_regs_;
    for (auto& e : eram_) std::memcpy(e.data() + slot, static_cast<const void*>(&r), sizeof r);
}

bool Impact::load_record(u32 slot) {
    if (std::size_t{slot} * 4 + sizeof(ContextRecord) > kEramWords * 4) return false;
    ContextRecord r;
    std::memcpy(static_cast<void*>(&r), eram_[0].data() + slot, sizeof r);
    gl_.state() = r.ge;
    user_port_ = r.user_port;
    if (user_port_.need > user_port_.data.size() || user_port_.got > user_port_.need) user_port_ = FifoPort{};
    flags_ |= r.ge_flags & kFlagGeData;
    ge_answer_ = r.ge_answer;
    dma_regs_ = r.dma_regs;
    reif_regs_ = r.reif_regs;
    return true;
}

// The incoming context's image has arrived: resume what it left (or start
// from OpenGL's initial state on its first load: the kernel recycles
// slots), and take its window.
void Impact::load_context() {
    const u32 slot = ctx_image_[0];
    const bool first = ctx_image_[1] & kCtxFirstLoad;
    ctx_slot_ = slot;
    if (first || !load_record(slot)) {
        gl_.reset();
        user_port_ = FifoPort{};
    }
    GlWindow& w = gl_window_;
    w.origin = ctx_image_[2];
    w.mode = ctx_image_[3];
    w.pp1_mode = ctx_image_[4];
    for (unsigned m = 0; m < 4; ++m) w.masks[3 - m] = {ctx_image_[5 + 2 * m], ctx_image_[6 + 2 * m]};  // masks 4..1
    w.drb = ctx_image_[13];
    w.banks[0] = ctx_image_[16];
    w.banks[1] = ctx_image_[17];
    gl_.set_window(w.origin, w.mode, w.pp1_mode, w.masks, w.drb);
    gl_.set_draw_bank(w.banks[0]);
    flags_ |= kFlagContextLoaded;
    update_irqs();
}

// A write to a geometry engine's diagnostic address or data port. An
// address with bit 31 set asks for (address & 0x7FFFFFFF) microcode lines
// from the current address, replacing any words not yet read: for each pair
// of lines, word 0 and the low byte of word 2 of the first, then word 1 of
// the second, after two empty words when it starts on an odd line (the
// order the driver's verifier reads). Data written at a microcode address
// fills its line, three words a line; at the execution control register,
// bit 0 starts the engine, whose version program answers revision 1.
void Impact::ge_diag_write(unsigned ge, bool address_port, u32 value) {
    GeDiag& g = ge_[ge];
    auto line = [&](u32 addr) -> const std::array<u32, 3>* {
        const u32 i = addr - kGeUcodeBase;
        return addr >= kGeUcodeBase && i < kGeUcodeLines ? &g.ucode[i] : nullptr;
    };
    if (address_port) {
        if (!(value & 0x80000000u)) {
            g.addr = value;
            g.word = 0;
            return;
        }
        ge_readback_.clear();
        u32 l = g.addr;
        if ((l - kGeUcodeBase) & 1) ge_readback_.insert(ge_readback_.end(), {0u, 0u});
        for (u32 n = 0; n < (value & 0x7FFFFFFFu) / 2; ++n, l += 2) {
            const auto* a = line(l);
            const auto* b = line(l + 1);
            ge_readback_.push_back(a ? (*a)[0] : 0);
            ge_readback_.push_back(a ? (*a)[2] & 0xFF : 0);
            ge_readback_.push_back(b ? (*b)[1] : 0);
        }
        return;
    }
    if (g.addr >= kGeUcodeBase) {
        if (g.addr - kGeUcodeBase < kGeUcodeLines) g.ucode[g.addr - kGeUcodeBase][g.word] = value;
        if (++g.word == 3) {
            g.word = 0;
            ++g.addr;
        }
    } else if (g.addr == kGeExecControl && (value & 1)) {
        ge_answer_ = {0, 1};
        flags_ |= kFlagGeData;
    } else {
        unknown("ge diag write", kPriv + (ge ? kHqGe1Data : kHqGe0Data), value);
    }
}

u32 Impact::user_read(u32 offset) const {
    switch (offset) {
        case kUStatus: return status();
        case kUFifoStatus: return cfifo_high_ << 22;  // level 0
        case kUFlagSet:
        case kUFlagClear: return flags();
        case kUGeHigh: return ge_answer_[0];
        case kUGeLow: return ge_answer_[1];
        case kUGioStatus:
        case kUDmaBusy: return 0;
        default: unknown("user read", kUser + offset, 0); return 0;
    }
}

void Impact::user_write(u32 offset, u32 value) {
    switch (offset) {
        case kUFlagSet: flags_ |= value; break;
        case kUFlagClear: flags_ &= ~value; break;
        case kUFifo0:
        case kUFifo1:
        case kUFifoGl0:
        case kUFifoGl1: fifo_push(user_port_, value); return;
        default: unknown("user write", kUser + offset, value); return;
    }
    update_irqs();
}

// A command word (bit 31 clear): data conversion in 29:23 (GE tokens),
// command number in 20:8, byte count in 7:0, then that many bytes of data
// in whole words. A pixel data header (bit 31 set) announces a byte count
// (19:0) starting at a byte offset (24:22) into its first word; the data
// follows as whole doublewords and belongs to the command before it.
void Impact::fifo_push(FifoPort& port, u32 word) {
    if (ctx_words_) {  // a context switch's incoming image
        ctx_image_[kContextImageWords - ctx_words_] = word;
        if (--ctx_words_ == 0) load_context();
        return;
    }
    if (port.pixel_words) {
        pixel_buf_.push_back(word);
        if (--port.pixel_words) return;
        const u32 count = port.pixel & 0xFFFFF, align = (port.pixel >> 22) & 7;
        std::vector<u8> bytes(count);
        for (u32 i = 0; i < count; ++i) {
            const u32 b = align + i;
            bytes[i] = static_cast<u8>(pixel_buf_[b / 4] >> (24 - 8 * (b % 4)));
        }
        pixel_buf_.clear();
        port.pixel = 0;
        pixel_data(port, bytes);
        return;
    }
    if (!port.in_command) {
        if (word & 0x80000000u) {
            const u32 bytes = word & 0xFFFFF, align = (word >> 22) & 7;
            port.pixel = word;
            port.pixel_words = (bytes + align + 7) / 8 * 2;
            pixel_buf_.clear();
            if (!port.pixel_words) port.pixel = 0;
            return;
        }
        port.cmd = (word >> 8) & 0x1FFF;
        port.conv = (word >> 23) & 0x7F;
        port.bytes = word & 0xFF;
        port.need = (port.bytes + 3) / 4;
        port.got = 0;
        if (!port.need) {
            run_command(port, port.cmd, nullptr, 0, 0);
            return;
        }
        port.in_command = true;
        return;
    }
    port.data[port.got++] = word;
    if (port.got < port.need) return;
    port.in_command = false;
    run_command(port, port.cmd, port.data.data(), port.got, port.bytes);
}

// Pixel data: ERAM write words, or the data of the GE command before it
// (raster register lists, images, bitmaps).
void Impact::pixel_data(FifoPort& port, const std::vector<u8>& bytes) {
    std::vector<u32> words((bytes.size() + 3) / 4);
    for (std::size_t i = 0; i < bytes.size(); ++i) words[i / 4] |= u32{bytes[i]} << (24 - 8 * (i % 4));
    if (eram_in_) {
        eram_in_->insert(eram_in_->end(), words.begin(), words.end());
        return;
    }
    if ((port.last_token == 0x080 || port.last_token == 0x0CD) && !port.last_words) {  // a GE pixel state block
        gl_.pixel_state_block(words.data(), static_cast<unsigned>(words.size()));
        port.last_token = 0;
        return;
    }
    if (port.last_token == 0x089) {  // INIT_COLOR_TABLE
        gl_.colour_table(words.data(), static_cast<unsigned>(words.size()));
        port.last_token = 0;
        return;
    }
    if (gl_.wants_pixel_data()) {  // a bitmap's or image's rows, perhaps over several commands
        if (gl_.wants_image() && (formatter_ & 0xC0)) {
            std::vector<u8> b = bytes;
            format_bytes(b);
            std::fill(words.begin(), words.end(), 0);
            for (std::size_t i = 0; i < b.size(); ++i) words[i / 4] |= u32{b[i]} << (24 - 8 * (i % 4));
        }
        gl_pixels_.insert(gl_pixels_.end(), words.begin(), words.end());
        if (gl_pixels_.size() >= gl_.pixel_words_needed()) {
            gl_.pixel_data(gl_pixels_.data(), static_cast<unsigned>(gl_pixels_.size()));
            gl_pixels_.clear();
        }
        return;
    }
    switch (port.last_token) {
        case 0x07F: gl_.rss_list(words.data(), static_cast<unsigned>(words.size()), 2); break;  // RSS_REG_SET: (register, value) pairs
        case 0x07C: gl_.rss_list(words.data(), static_cast<unsigned>(words.size()), 3); break;  // RSS_REG_SHADOW: (register, value, shadow) triples
        case 0x082: break;  // (with the GE's DMA set-up: nothing drawn)
        default: unknown("pixel data", port.last_token, bytes.size()); break;
    }
}

void Impact::run_command(FifoPort& port, u32 cmd, const u32* data, unsigned words, unsigned bytes) {
    SG_LOG(Trace, "impact", "cmd %04x [%u] %08x %08x", cmd, words, words ? data[0] : 0, words > 1 ? data[1] : 0);
    port.last_token = cmd;
    port.last_words = words;
    if (cmd >= 0x1000) {  // raster register write; 0x400 = execute after it
        const u32 reg = cmd & 0x3FF;
        const bool go = (cmd & 0x400) != 0;
        if (!words) {
            rss_write(reg, 0, go);
            return;
        }
        for (unsigned i = 0; i < words; ++i) rss_write(reg + i, data[i], go && i + 1 == words);
        return;
    }
    if (cmd == 0xE04) {  // SET_DONE_FLAG
        flags_ |= kFlagDone;
        update_irqs();
        return;
    }
    if (cmd >= 0x800 && cmd < 0x880) {  // host DMA engine registers
        for (unsigned i = 0; i < words && (cmd & 0x7F) + i < dma_regs_.size(); ++i) {
            const unsigned reg = (cmd & 0x7F) + i;
            dma_regs_[reg] = data[i];
            if (reg == kDmaStart) dma_start(data[i]);
        }
        // From the FIFO, a branch (eight bytes: offset, words) starts a
        // display list; inside one, the links are the list engine's.
        if (&port != &dl_port_ && ((cmd & 0x7F) == kDmaDlJump || (cmd & 0x7F) == kDmaDlJump + 2))
            run_display_list(dma_regs_[cmd & 0x7F], dma_regs_[(cmd & 0x7F) + 1]);
        return;
    }
    if (cmd >= 0xA00 && cmd < 0xA10) {  // raster interface
        for (unsigned i = 0; i < words && (cmd & 0xF) + i < reif_regs_.size(); ++i) {
            const unsigned reg = (cmd & 0xF) + i;
            reif_regs_[reg] = data[i];
            // The raster side of a board-to-host transfer, as libGLcore starts it.
            if (reg == 5 && (data[i] & 0xF) == 9 && dma_read_pending_) dma_run(std::exchange(dma_read_pending_, 0));
        }
        return;
    }
    if (cmd == 0xC00) {  // formatter mode
        if (words) formatter_ = data[0];
        return;
    }
    if (cmd < 0x200) {
        ge_command(port, cmd, data, words, bytes);
        return;
    }
    unknown("command", cmd, words ? data[0] : 0);
}

// A display list: segments fetched from host memory through the display-
// list page table (DMA registers 0x28-0x29, a 32-bit frame number per 4 KB
// page), run as a third command stream. libGLcore compiles a list into
// segments that begin with their link, DMA engine writes: a jump to the
// next segment (0x0C, 0x0E), a push of a return segment (0x37, before the
// jump into a called list), or a return to the last pushed segment (0x39),
// or to the FIFO when none is left. The body runs first, then the link.
void Impact::run_display_list(u32 offset, u32 words) {
    if (!bus_) return;
    const u32 table = dma_regs_[kDmaDlTable + 1] & ~3u;
    std::vector<std::pair<u32, u32>> stack;
    std::optional<std::pair<u32, u32>> cur{{offset, words}};
    dl_port_ = FifoPort{};
    std::vector<u32> seg;
    for (unsigned segments = 0; cur && segments < kDlMaxSegments; ++segments) {
        const auto [off, n] = *cur;
        cur.reset();
        if (n > (1u << 20)) break;
        seg.assign(n, 0);
        u32 cached_page = ~0u, frame = 0;
        for (u32 i = 0; i < n; ++i) {
            const u32 l = off + 4 * i;
            if ((l >> 12) != cached_page) {
                u8 f[4];
                host_read(table + 4ull * (l >> 12), f, 4);
                cached_page = l >> 12, frame = be_bytes(f, 4);
            }
            u8 b[4];
            host_read((u64{frame} << 12) | (l & 0xFFF), b, 4);
            seg[i] = be_bytes(b, 4);
        }
        std::vector<std::array<u32, 3>> links;  // register, offset, words
        std::size_t at = 0;
        while (at < seg.size()) {
            const u32 w = seg[at], c = (w >> 8) & 0x1FFF, r = c & 0x7F;
            if ((w & 0x80000000u) || c < 0x800 || c >= 0xA00 ||
                !(r == kDmaDlJump || r == kDmaDlJump + 2 || r == kDmaDlPush || r == kDmaDlReturn))
                break;
            links.push_back({r, at + 1 < seg.size() ? seg[at + 1] : 0, at + 2 < seg.size() ? seg[at + 2] : 0});
            at += 1 + ((w & 0xFF) + 3) / 4;
        }
        for (std::size_t i = at; i < seg.size(); ++i) fifo_push(dl_port_, seg[i]);
        for (const auto& [r, a, b] : links) {
            if (r == kDmaDlPush) {
                if (stack.size() >= kDlMaxDepth) break;
                stack.emplace_back(a, b);
            } else if (r == kDmaDlReturn) {
                if (!stack.empty()) cur = stack.back(), stack.pop_back();
            } else {
                cur = std::pair<u32, u32>{a, b};
            }
        }
    }
}

// Command-processor tokens: the HQ3's own, and the GE's (GL).
void Impact::ge_command(FifoPort& port, u32 cmd, const u32* data, unsigned words, unsigned bytes) {
    const u32 w0 = words ? data[0] : 0;
    switch (cmd) {
        case 0x037:  // SCHEDULE_SWAP: reported done at once; the display flips by the kernel's buffer select
            gl_.swap_buffers();
            flags_ |= kFlagCp0;
            update_irqs();
            return;
        case 0x0A1:  // SPIN_AND_RETURN
        case 0x0A2:  // __MGR_RETURN_MODE: the state word at a GE address (glGet*, glIsEnabled, glFinish)
            ge_answer_ = {0, gl_.state_word(w0).value_or(cmd == 0x0A1 ? w0 : 0)};
            if (!gl_.state_word(w0)) unknown("ge readback", cmd, w0);
            flags_ |= kFlagGeData;
            update_irqs();
            return;
        case 0x0E4: {  // KERN_WINDOW: origin, window mode, PP1 window mode, masks 4..1, DRB pointers, ...
            if (words < 12) break;
            std::array<std::array<u32, 2>, 4> masks{};
            for (unsigned m = 0; m < 4; ++m) masks[3 - m] = {data[3 + 2 * m], data[4 + 2 * m]};
            gl_.set_window(data[0], data[1], data[2], masks, data[11]);
            return;
        }
        case 0x0F4: nosave_next_ = true; return;  // GE_NOSAVE_NEXT_SWITCH
        case 0x0FA:
        case 0x0FB:
        case 0x0FC:
        case 0x0FD: eram_pass_through(cmd, data, words); return;
        default: break;
    }
    // Data conversion (command word bits 29:23) for vertex, colour, normal
    // and index tokens: bit 5 to float, 4 signed, 3 normalise, 2:0 the
    // source size (0 32-bit, 1 byte, 2 short, 4 double); bit 6 appends a
    // fourth component of 1.0.
    const unsigned conv = port.conv;
    if ((conv & 0x60) && bytes) {
        std::vector<u32> f;
        auto put = [&](double v) {
            const float x = static_cast<float>(v);
            u32 b;
            std::memcpy(&b, &x, 4);
            f.push_back(b);
        };
        const bool sign = conv & 0x10, norm = conv & 0x08;
        auto scale = [&](double v, unsigned nbits) {
            const double max = static_cast<double>((u64{1} << nbits) - 1);
            return !norm ? v : sign ? (2 * v + 1) / max : v / max;
        };
        if (!(conv & 0x20)) {
            f.assign(data, data + words);
        } else if ((conv & 7) == 4) {
            for (unsigned i = 0; i + 1 < words; i += 2) {
                const u64 w = (u64{data[i]} << 32) | data[i + 1];
                double v;
                std::memcpy(&v, &w, 8);
                put(v);
            }
        } else if ((conv & 7) == 1) {
            for (unsigned i = 0; i < bytes; ++i) {
                const u32 b = (data[i / 4] >> (24 - 8 * (i % 4))) & 0xFF;
                put(scale(sign ? static_cast<s8>(b) : static_cast<double>(b), 8));
            }
        } else if ((conv & 7) == 2) {
            for (unsigned i = 0; i < bytes / 2; ++i) {
                const u32 h = (data[i / 2] >> (16 - 16 * (i % 2))) & 0xFFFF;
                put(scale(sign ? static_cast<s16>(h) : static_cast<double>(h), 16));
            }
        } else {
            for (unsigned i = 0; i < bytes / 4; ++i)
                put(scale(sign ? static_cast<double>(static_cast<s32>(data[i])) : static_cast<double>(data[i]), 32));
        }
        if (conv & 0x40) put(1.0);
        if (gl_.token(cmd, f.data(), static_cast<unsigned>(f.size()))) return;
    } else if (gl_.token(cmd, data, words)) {
        return;
    }
    unknown("ge token", cmd, w0);
}

// The kernel's way into GE external RAM: write = ERAM_WRITE (word count),
// the words (FIFO pixel data or a host to board DMA), ERAM_WRITE_AT
// (address, count) for each chunk, ERAM_WRITE_END after the last; read = a
// board to host DMA armed, then ERAM_READ (address, count) feeds it. With
// two GEs every transfer comes once per GE, GE0 first: a transfer that
// repeats GE0's last one is GE1's (whose read output runs a word late).
void Impact::eram_pass_through(u32 cmd, const u32* data, unsigned words) {
    const u32 a0 = words > 0 ? data[0] : 0, a1 = words > 1 ? data[1] : 0;
    switch (cmd) {
        case 0x0FD: eram_in_.emplace(); return;
        case 0x0FB: {
            if (!eram_write_ge_) {
                const unsigned ge = eram_write_ge0_ == a0;
                if (ge == 0) eram_write_ge0_ = a0;
                else eram_write_ge0_.reset();
                eram_write_ge_ = ge;
            }
            std::vector<u32> buf = eram_in_ ? std::move(*eram_in_) : std::vector<u32>{};
            eram_in_.emplace();
            const u32 addr = std::min(a0, kEramWords);
            const u32 n = std::min<u32>({a1, static_cast<u32>(buf.size()), kEramWords - addr});
            std::copy(buf.begin(), buf.begin() + n, eram_[*eram_write_ge_].begin() + addr);
            return;
        }
        case 0x0FA:
            eram_in_.reset();
            eram_write_ge_.reset();
            return;
        default: {  // 0x0FC: read
            const unsigned ge = eram_read_ge0_ == std::pair<u32, u32>{a0 + 1, a1};
            if (ge == 0) eram_read_ge0_ = std::pair<u32, u32>{a0, a1};
            else eram_read_ge0_.reset();
            const u32 addr = std::min(a0 + ge, kEramWords), n = std::min(a1, kEramWords - addr);
            eram_out_.clear();
            for (u32 i = 0; i < n; ++i)
                for (unsigned b = 0; b < 4; ++b) eram_out_.push_back(static_cast<u8>(eram_[ge][addr + i] >> (24 - 8 * b)));
            if (dma_read_pending_) dma_run(std::exchange(dma_read_pending_, 0));
            else unknown("eram read with no host DMA armed", cmd, a0);
            return;
        }
    }
}

// ---- display control bus ----------------------------------------------------
//
// Device N has a 1 KB window at 0x60000 + N * 0x400; within it address bits
// 9:7 are the chip's register select (CRS), bit 5 asks it to increment and
// bits 4:3 give the transfer width in bytes (0 = 4). Data rides in the most
// significant bytes of a word access, the low bytes of narrower ones.

u32 Impact::be_bytes(const u8* p, unsigned n) {
    u32 v = 0;
    for (unsigned i = 0; i < n; ++i) v = (v << 8) | p[i];
    return v;
}

void Impact::put_be_bytes(u8* p, unsigned n, u32 v) {
    for (unsigned i = 0; i < n; ++i) p[i] = static_cast<u8>(v >> (8 * (n - 1 - i)));
}

u32 Impact::dcb_read(u32 offset, unsigned size) {
    if (offset >= kDcbTiming - kDcb) return dcb_timing_[((offset - (kDcbTiming - kDcb)) / 4) & 15];
    const unsigned dev = (offset >> 10) & 31, crs = (offset >> 7) & 7;
    unsigned width = (offset >> 3) & 3;
    if (!width) width = 4;
    const u32 data = dcb_device_read(dev, crs, width);
    if (size < 4) return data;
    return width == 4 ? data : data << (8 * (4 - width));  // (of a doubleword: the high word)
}

void Impact::dcb_write(u32 offset, unsigned size, u64 value) {
    if (offset >= kDcbTiming - kDcb) {
        const unsigned dev = ((offset - (kDcbTiming - kDcb)) / 4) & 15;
        dcb_timing_[dev] = static_cast<u32>(value);
        return;
    }
    const unsigned dev = (offset >> 10) & 31, crs = (offset >> 7) & 7;
    unsigned width = (offset >> 3) & 3;
    if (!width) width = 4;
    u32 data;
    if (size == 4 || size == 8) {
        const u32 word = size == 8 ? static_cast<u32>(value >> 32) : static_cast<u32>(value);
        data = width == 4 ? word : word >> (8 * (4 - width));
    } else {
        data = static_cast<u32>(value) & ((1u << (8 * size)) - 1);
    }
    dcb_device_write(dev, crs, width, data);
}

u32 Impact::dcb_device_read(unsigned dev, unsigned crs, unsigned width) {
    switch (dev) {
        case kDevCmapAll:
        case kDevCmap0: return cmap_read(0, crs);
        case kDevCmap1: return cmap_read(1, crs);
        case kDevDac: return dac_read(crs);
        case kDevXmap: return xmap_read(crs, width);
        case kDevVc3: return vc3_read(crs, width);
        case kDevBoardVersion: return crs < 2 ? board_version_[crs] : 0;
        case kDevI2c: return 0;  // no flat panel
        default: unknown("dcb read", dev << 10 | crs << 7, width); return 0;
    }
}

void Impact::dcb_device_write(unsigned dev, unsigned crs, unsigned width, u32 data) {
    switch (dev) {
        case kDevCmapAll:
            cmap_write(0, crs, width, data);
            cmap_write(1, crs, width, data);
            break;
        case kDevCmap0: cmap_write(0, crs, width, data); break;
        case kDevCmap1: cmap_write(1, crs, width, data); break;
        case kDevDac: dac_write(crs, width, data); break;
        // The X server sets the XMAP's index every frame to poll it: only a
        // write that changes what is shown makes the picture out of date.
        case kDevXmap:
            if (!xmap_write(crs, width, data)) return;
            break;
        case kDevVc3:
            if (!vc3_write(crs, width, data)) return;
            break;
        case kDevBoardVersion:
            if (crs == 1) board_version_written_ = static_cast<u8>(data);
            return;
        default: unknown("dcb write", dev << 10 | crs << 7, data); return;
    }
    // The VC3 says for itself (a cursor move changes only its rows); the
    // XMAP's config word shows only as the cursor's colours, which the X
    // server switches back and forth while idle.
    if (dev != kDevVc3 && !(dev == kDevXmap && crs == 2)) full_redraw_ = true;
    ++display_version_;
}

// Colour maps: 8192 entries of 0xRRGGBB. A 2-byte address goes low byte first.
void Impact::cmap_write(unsigned which, unsigned crs, unsigned width, u32 data) {
    Cmap& c = cmap_[which];
    switch (crs) {
        case 0:
            if (width == 1) c.addr = (c.addr & 0x1F00) | (data & 0xFF);
            else c.addr = ((data >> 8) & 0xFF) | ((data & 0x1F) << 8);
            break;
        case 1: c.addr = (c.addr & 0xFF) | ((data & 0x1F) << 8); break;
        case 2:
            c.entries[c.addr] = data & 0xFFFFFF;
            c.addr = (c.addr + 1) & 0x1FFF;
            break;
        case 3: c.command = static_cast<u8>(data); break;
        default: unknown("cmap write", crs, data); break;
    }
}

u32 Impact::cmap_read(unsigned which, unsigned crs) {
    const Cmap& c = cmap_[which];
    switch (crs) {
        case 0: return c.addr & 0xFF;
        case 1: return c.addr >> 8;
        case 2: return c.entries[c.addr];
        case 3: return c.command;
        case 4: return 0x08;  // ready for writes
        case 6: return 0xFB;  // revision
        default: return 0;
    }
}

// RAMDAC: 256 gamma entries of 10-bit R, G, B, written in turn; indexed
// registers (4 = pixel read mask).
void Impact::dac_write(unsigned crs, unsigned width, u32 data) {
    switch (crs) {
        case 0:
            if (width == 1) dac_.addr = (dac_.addr & 0xFF00) | (data & 0xFF);
            else dac_.addr = ((data >> 8) & 0xFF) | ((data & 0xFF) << 8);
            dac_.phase = 0;
            break;
        case 1: {
            const u16 v = width == 1 ? static_cast<u16>((data & 0xFF) << 2)
                                     : static_cast<u16>((((data >> 8) & 0xFF) << 2) | (data & 3));
            dac_.gamma[dac_.addr & 0xFF][dac_.phase] = v;
            if (++dac_.phase == 3) {
                dac_.phase = 0;
                dac_.addr = (dac_.addr + 1) & 0xFFFF;
            }
            break;
        }
        case 2: dac_.regs[dac_.addr & 0xFF] = static_cast<u8>(data); break;
        case 3: dac_.mode = static_cast<u8>(data); break;
        default: unknown("dac write", crs, data); break;
    }
}

u32 Impact::dac_read(unsigned crs) {
    switch (crs) {
        case 0: return dac_.addr & 0xFF;
        case 1: {
            const u32 v = dac_.gamma[dac_.addr & 0xFF][dac_.phase] >> 2;
            if (++dac_.phase == 3) {
                dac_.phase = 0;
                dac_.addr = (dac_.addr + 1) & 0xFFFF;
            }
            return v;
        }
        case 2: return dac_.regs[dac_.addr & 0xFF];
        case 3: return dac_.mode;
        default: return 0;
    }
}

// XMAP (in the PP1s): register files addressed by INDEX, written big-endian
// a transfer's width at a time.
bool Impact::xmap_write(unsigned crs, unsigned width, u32 data) {
    Xmap& x = xmap_;
    u8* file = nullptr;
    unsigned size = 0;
    switch (crs) {
        case 0: x.pp1_select = static_cast<u8>(data); return false;
        case 1: x.index = static_cast<u8>(data); return false;
        case 2: file = x.config.data(), size = 4; break;
        case 3: file = x.buf_select.data(), size = 4; break;
        case 4: file = x.main_mode.data(), size = 128; break;
        case 5: file = x.overlay_mode.data(), size = 128; break;
        case 6: file = x.dib.data(), size = static_cast<unsigned>(x.dib.size()); break;
        default: unknown("xmap write", crs, data); return false;
    }
    bool changed = false;
    if (x.index + width <= size) {
        changed = be_bytes(file + x.index, width) != (width < 4 ? data & ((1u << (8 * width)) - 1) : data);
        put_be_bytes(file + x.index, width, data);
    } else {
        unknown("xmap index", crs << 8 | x.index, data);
    }
    if (crs >= 3 && (be_bytes(x.config.data(), 4) & (1u << 19))) x.index = static_cast<u8>(x.index + width);
    return changed;
}

u32 Impact::xmap_read(unsigned crs, unsigned width) {
    Xmap& x = xmap_;
    u32 v = 0;
    switch (crs) {
        case 0: return x.pp1_select;
        case 1: return x.index;
        case 2:
            if (x.index == 4) return 1;     // PP1 revision
            if (x.index == 8) return 0x10;  // room for a mode update
            return x.index + width <= 4 ? be_bytes(x.config.data() + x.index, width) : 0;
        case 3: v = x.index + width <= 4 ? be_bytes(x.buf_select.data() + x.index, width) : 0; break;
        case 4: v = x.index + width <= 128 ? be_bytes(x.main_mode.data() + x.index, width) : 0; break;
        case 5: v = x.index + width <= 128 ? be_bytes(x.overlay_mode.data() + x.index, width) : 0; break;
        case 6: v = x.index + width <= x.dib.size() ? be_bytes(x.dib.data() + x.index, width) : 0; break;
        case 7: return 0x00010101;  // RE-to-PP1 links in sync
        default: return 0;
    }
    if (be_bytes(x.config.data(), 4) & (1u << 19)) x.index = static_cast<u8>(x.index + width);
    return v;
}

// VC3: 32 indexed 16-bit registers and 32K words of SRAM (timing, window ID
// tables, cursor glyph).
bool Impact::vc3_write(unsigned crs, unsigned width, u32 data) {
    Vc3& v = vc3_;
    // A register write shows only if its value changes (the SRAM pointer
    // never shows); an SRAM write likewise. Only a move of the cursor
    // leaves the rest of the picture as it was.
    auto set_reg = [this, &v](unsigned r, u16 value) {
        const bool changed = v.regs[r] != value && r != kVc3SramPtr;
        v.regs[r] = value;
        if (changed && r != kVc3CursorX && r != kVc3CursorY) full_redraw_ = true;
        return changed;
    };
    auto set_sram = [this, &v](u32 at, u16 value) {
        const bool changed = v.sram[at & 0x7FFF] != value;
        v.sram[at & 0x7FFF] = value;
        if (changed) full_redraw_ = true;
        return changed;
    };
    switch (crs) {
        case 0:
            if (width == 3) return set_reg((data >> 16) & 31, static_cast<u16>(data));
            if (width == 1) v.index = data & 31;
            else unknown("vc3 crs0", width, data);
            return false;
        case 1: return set_reg(v.index, static_cast<u16>(data));
        case 3: {
            u32 ptr = v.regs[kVc3SramPtr] & 0x7FFF;
            bool changed;
            if (width == 4) {
                changed = set_sram(ptr, static_cast<u16>(data >> 16));
                changed |= set_sram(ptr + 1, static_cast<u16>(data));
                ptr += 2;
            } else {
                changed = set_sram(ptr, static_cast<u16>(data));
                ptr += 1;
            }
            v.regs[kVc3SramPtr] = static_cast<u16>(ptr & 0x7FFF);
            return changed;
        }
        default: unknown("vc3 write", crs, data); return false;
    }
}

u32 Impact::vc3_read(unsigned crs, unsigned width) {
    Vc3& v = vc3_;
    switch (crs) {
        case 0: return v.index;
        case 1: return v.index == kVc3Scanline ? vc3_scanline() : v.regs[v.index];
        case 3: {
            u32 ptr = v.regs[kVc3SramPtr] & 0x7FFF, r;
            if (width == 4) {
                r = (u32{v.sram[ptr]} << 16) | v.sram[(ptr + 1) & 0x7FFF];
                ptr += 2;
            } else {
                r = v.sram[ptr];
                ptr += 1;
            }
            v.regs[kVc3SramPtr] = static_cast<u16>(ptr & 0x7FFF);
            return r;
        }
        default: return 0;
    }
}

u32 Impact::vc3_scanline() const {
    const u64 frame = static_cast<u64>(cpu_hz_ / 60);
    const u64 since = (scheduler_.now() - last_retrace_) % frame;
    return static_cast<u32>(since * kLinesPerFrame / frame);
}

void Impact::retrace() {
    last_retrace_ = scheduler_.now();
    ++frames_;
    if (vc3_.regs[kVc3DisplayControl] & 1) {
        set_line(kIrqRetrace, true);
        retrace_end_timer_.schedule_in(static_cast<u64>(cpu_hz_ / 2000));  // 500 us
    }
    retrace_timer_.schedule_in(static_cast<u64>(cpu_hz_ / 60));
}

// ---- raster engine ------------------------------------------------------------

u32 Impact::rss_read(u32 reg) const {
    if (reg == kStatus) return 0x110;  // idle
    return rss_[reg & 0x3FF];
}

void Impact::rss_write(u32 reg, u32 value, bool go) {
    reg &= 0x3FF;
    rss_[reg] = value;
    if (reg == kIrAlias) rss_[kIr] = value;
    if (reg == kXfrControl && (value & 0xF) == 9 && dma_read_pending_)  // the raster side of a DMA read
        dma_run(std::exchange(dma_read_pending_, 0));
    if (!go) return;
    if (reg == kCharH && stipple_.armed) return char_stipple(u64{value} << 32, 32);
    if (reg == kCharL && stipple_.armed) return char_stipple((u64{rss_[kCharH]} << 32) | value, 64);
    if (reg == kCharH && xfer_.armed && xfer_.pio && !xfer_.read) return pio_write(u64{value} << 32, 4);
    if (reg == kCharL && xfer_.armed && xfer_.pio && !xfer_.read) return pio_write((u64{rss_[kCharH]} << 32) | value, 8);
    execute();
}

void Impact::execute() {
    switch (rss_[kIr] & 0xF) {
        case 0x8: block(); break;
        case 0x4: point(); break;
        case 0x5: xline(); break;
        default: unknown("raster op", kIr, rss_[kIr]); break;
    }
}

bool Impact::to_framebuffer(int x, int y, int& fx, int& fy) const {
    const u32 win = rss_[kXyWin];
    fx = half(win) + x;
    fy = (rss_[kConfig] & 8) ? half(win >> 16) - y : half(win >> 16) + y;
    return fx >= 0 && fx < 2048 && fy >= 0 && fy < 2048;
}

u32 Impact::draw_colour() const {
    const bool ci = pixel_type_ci(rss_[kPp1FillMode]);
    if (rss_[kFillMode] & (1u << 20)) {  // fast fill: the PIXcolor registers (12 bits each)
        if (ci) return rss_[kPixColorR] & 0xFFF;
        auto c8 = [&](unsigned i) { return ((rss_[kPixColorR + i] & 0xFFF) >> 4) << (8 * i); };
        return c8(0) | c8(1) | c8(2) | c8(3);
    }
    if (ci) return (rss_[kRed] >> 12) & 0xFFF;
    return rss_[kPackedColor] | (((rss_[kRed + 3] >> 12) & 0xFF) << 24);
}

u32 Impact::background_colour() const {
    const u32 rg = rss_[kBkgrdRg], ba = rss_[kBkgrdBa];
    if (pixel_type_ci(rss_[kPp1FillMode])) return rg & 0xFFF;
    return ((rg & 0xFFF) >> 4) | ((((rg >> 12) & 0xFFF) >> 4) << 8) | (((ba & 0xFFF) >> 4) << 16) |
           ((((ba >> 12) & 0xFFF) >> 4) << 24);
}

void Impact::plot(int x, int y, u32 colour) {
    int fx, fy;
    if (!to_framebuffer(x, y, fx, fy)) return;
    store_pixel(fx, fy, colour);
}

// A block from start to end, both corners inclusive, in either direction:
// filled now, or armed for character stipple data (or a pixel transfer).
void Impact::block() {
    const u32 fm = rss_[kFillMode];
    const unsigned type = (fm >> 22) & 7;
    const bool fast = fm & (1u << 20), char_stipple = fm & 8;
    const int sx = half(rss_[kBlockStart] >> 16), sy = half(rss_[kBlockStart]);
    const int ex = half(rss_[kBlockEnd] >> 16), ey = half(rss_[kBlockEnd]);
    const int dx = ex >= sx ? 1 : -1, dy = ey >= sy ? 1 : -1;
    stipple_.armed = false;
    xfer_.armed = false;
    const u32 colour = draw_colour();
    if (fast || type == 0 || (type == 1 && !char_stipple)) {
        for (int y = sy;; y += dy) {
            for (int x = sx;; x += dx) {
                plot(x, y, colour);
                if (x == ex) break;
            }
            if (y == ey) break;
        }
        ++display_version_;
        return;
    }
    if (type == 1) {
        stipple_ = {true, (fm & 0x10) != 0, sx, ex, ey, sx, sy, dx, dy, colour, background_colour()};
        return;
    }
    arm_transfer(sx, sy, ex, ey, type);
}

// Stipple bits, most significant first, across the armed block row by row;
// the rest of a chunk is dropped where a row ends.
void Impact::char_stipple(u64 bits, unsigned count) {
    Stipple& s = stipple_;
    for (unsigned i = 0; i < count && s.armed; ++i) {
        if ((bits >> (63 - i)) & 1) plot(s.x, s.y, s.colour);
        else if (s.opaque) plot(s.x, s.y, s.background);
        if (s.x != s.x1) {
            s.x += s.dx;
            continue;
        }
        s.x = s.x0;
        if (s.y == s.y1) s.armed = false;
        else s.y += s.dy;
        break;
    }
    ++display_version_;
}

void Impact::point() {
    const u32 p = rss_[kXlineStart];
    plot(half(p >> 16), half(p), draw_colour());
    ++display_version_;
}

// Bresenham between the integer end points (the last left out when FILLMODE
// bit 10 asks), with the 32-bit line stipple when bit 5 is set.
void Impact::xline() {
    const u32 fm = rss_[kFillMode];
    int x0 = half(rss_[kXlineStart] >> 16), y0 = half(rss_[kXlineStart]);
    const int x1 = half(rss_[kXlineEnd] >> 16), y1 = half(rss_[kXlineEnd]);
    const int adx = std::abs(x1 - x0), ady = std::abs(y1 - y0);
    const int sx = x1 >= x0 ? 1 : -1, sy = y1 >= y0 ? 1 : -1;
    const bool skip_last = fm & (1u << 10), stipple = fm & 0x20, opaque = fm & 0x40;
    const u32 colour = draw_colour(), background = background_colour(), pattern = rss_[kLsPat];
    int err = adx - ady;
    for (unsigned k = 0;; ++k) {
        const bool last = x0 == x1 && y0 == y1;
        if (last && skip_last) break;
        if (!stipple || ((pattern >> (31 - (k & 31))) & 1)) plot(x0, y0, colour);
        else if (opaque) plot(x0, y0, background);
        if (last) break;
        const int e2 = 2 * err;
        if (e2 > -ady) err -= ady, x0 += sx;
        if (e2 < adx) err += adx, y0 += sy;
    }
    ++display_version_;
}

// ---- pixel transfers --------------------------------------------------------
//
// A block of type 2-5 arms a transfer of its pixels: PIO (2 read, 3 write)
// through the char registers, or DMA (4 read, 5 write) by the HQ3's DMA
// engine. XFRMODE gives the host format: component type (3:0), pixel
// format (7:4: 0 colour index, 7 RGB, 8 RGBA, 3-6 one component), the
// first pixel's byte offset in its doubleword (10:8), padding after each
// line (17:14), PIO reads in DMA order (18) and RGBorder (22: RGBA red
// first rather than ABGR). XFRSIZE's low half is the pixels per line.

namespace {

unsigned comp_size(unsigned t) { return (t & 3) == 1 ? 2 : (t & 3) == 3 ? 4 : 1; }

// Formats converted component by component, and their components (0 red
// .. 3 alpha) in host order.
bool generic_format(unsigned f, unsigned t, bool rgborder) {
    const bool types = t == 0 || t == 1 || t == 3 || t == 4 || t == 5 || t == 7;
    if (f == 8) return rgborder ? types : (t == 1 || t == 3 || t == 4 || t == 5 || t == 7);
    if (f == 7) return t == 3 || t == 4 || t == 5 || t == 7;
    return f >= 3 && f <= 6 && types;
}

std::vector<unsigned> format_comps(unsigned f, bool rgborder) {
    switch (f) {
        case 3: return {0};
        case 4: return {1};
        case 5: return {2};
        case 6: return {3};
        case 7: return {0, 1, 2};
        default: return rgborder ? std::vector<unsigned>{0, 1, 2, 3} : std::vector<unsigned>{3, 2, 1, 0};
    }
}

u32 be_value(const u8* p, unsigned n) {
    u32 v = 0;
    for (unsigned i = 0; i < n; ++i) v = (v << 8) | p[i];
    return v;
}

}  // namespace

unsigned Impact::transfer_bpp(u32 mode) {
    const unsigned f = (mode >> 4) & 15, t = mode & 15;
    if (generic_format(f, t, mode & (1u << 22))) return static_cast<unsigned>(format_comps(f, mode & (1u << 22)).size()) * comp_size(t);
    if (f == 0) return comp_size(t);
    if (f == 8 && (t == 8 || t == 10)) return 2;
    if ((f == 8 && t == 0) || (f == 2 && t == 3)) return 4;
    if (f == 7 && t == 0) return 3;
    if (f == 7 && t == 1) return 6;
    if (f == 8 && t == 1) return 8;
    return 1;
}

// A host pixel as store_pixel takes it: a colour index, or 0xAABBGGRR.
u32 Impact::host_to_pixel(const u8* p, u32 mode) const {
    const unsigned f = (mode >> 4) & 15, t = mode & 15;
    const bool rgborder = mode & (1u << 22);
    if (generic_format(f, t, rgborder)) {
        const unsigned n = comp_size(t);
        u32 px = 0;
        const std::vector<unsigned> comps = format_comps(f, rgborder);
        for (std::size_t i = 0; i < comps.size(); ++i) {
            const u8* c = p + i * n;
            u32 v = c[0];
            if (t & 4) v = (c[0] & 0x80) ? 0 : ((c[0] << 1) | (n > 1 ? c[1] >> 7 : c[0] >> 6)) & 0xFF;  // signed: one bit less
            px |= v << (8 * comps[i]);
        }
        return px;
    }
    if (f == 0) return be_value(p, comp_size(t)) & (comp_size(t) == 1 ? 0xFF : 0xFFF);
    if (f == 8 && t == 8) {  // packed 4:4:4(:4), red lowest
        const u32 v = be_value(p, 2);
        auto c = [&](unsigned s) { return ((v >> s) & 15) * 0x11; };
        return c(0) | (c(4) << 8) | (c(8) << 16);
    }
    if (f == 8 && t == 10) {  // packed 5:5:5
        const u32 v = be_value(p, 2);
        auto c = [&](unsigned s) { return (((v >> s) & 31) << 3) | (((v >> s) & 31) >> 2); };
        return c(0) | (c(5) << 8) | (c(10) << 16);
    }
    if (f == 8 && t == 0) return (p[3]) | (p[2] << 8) | (p[1] << 16) | (u32{p[0]} << 24);  // A B G R
    if (f == 7 && t == 0) return p[0] | (p[1] << 8) | (p[2] << 16);                          // R G B
    if (f == 7 && t == 1) return p[0] | (p[2] << 8) | (p[4] << 16);                          // 16-bit R G B
    if (f == 8 && t == 1) return p[0] | (p[2] << 8) | (p[4] << 16) | (u32{p[6]} << 24);
    return p[0];
}

void Impact::pixel_to_host(u32 px, std::vector<u8>& out) const {
    const u32 mode = rss_[kXfrMode];
    const unsigned f = (mode >> 4) & 15, t = mode & 15;
    const bool rgborder = mode & (1u << 22);
    auto put = [&](u32 v, unsigned n) {
        for (unsigned i = 0; i < n; ++i) out.push_back(static_cast<u8>(v >> (8 * (n - 1 - i))));
    };
    if (generic_format(f, t, rgborder)) {
        for (unsigned k : format_comps(f, rgborder)) {
            const u32 c = (px >> (8 * k)) & 0xFF;
            switch (t & 7) {
                case 0: put(c, 1); break;
                case 4: put(c >> 1, 1); break;
                case 1: put(c * 0x101, 2); break;
                case 5: put((c * 0x101) >> 1, 2); break;
                case 3: put(c * 0x01010101u, 4); break;
                default: put((c * 0x01010101u) >> 1, 4); break;
            }
        }
        return;
    }
    const u32 r = px & 0xFF, g = (px >> 8) & 0xFF, b = (px >> 16) & 0xFF, a = px >> 24;
    if (gl_read_zst_ == 1) return put(px, transfer_bpp(mode));
    if (f == 0) return put(px & (comp_size(t) == 1 ? 0xFF : 0xFFF), comp_size(t));
    if (f == 8 && t == 8) return put((r >> 4) | ((g >> 4) << 4) | ((b >> 4) << 8), 2);
    if (f == 8 && t == 10) return put((r >> 3) | ((g >> 3) << 5) | ((b >> 3) << 10), 2);
    if (f == 8 && t == 0) return put((a << 24) | (b << 16) | (g << 8) | r, 4);
    if (f == 7 && t == 0) return put((r << 16) | (g << 8) | b, 3);
    if (f == 7 && t == 1) {
        put(r * 0x101, 2), put(g * 0x101, 2), put(b * 0x101, 2);
        return;
    }
    if (f == 8 && t == 1) {
        put(r * 0x101, 2), put(g * 0x101, 2), put(b * 0x101, 2), put(a * 0x101, 2);
        return;
    }
    put(px, 1);
}

// The read buffer's pixel (PP1FILLMODE bits 25:21: 0 A, 1 B, 4 the overlay)
// in store_pixel's terms.
u32 Impact::read_pixel(int fx, int fy) const {
    if (fx < 0 || fy < 0 || fx >= static_cast<int>(kFbSize) || fy >= static_cast<int>(kFbSize)) return 0;
    if (gl_read_zst_) {  // the depth word as it is, or the stencil index
        const u32 z = zst_.empty() ? 0 : zst_[std::size_t{static_cast<unsigned>(fy)} * kFbSize + static_cast<unsigned>(fx)];
        return gl_read_zst_ == 2 ? z >> 24 : z;
    }
    const u32 pp1 = rss_[kPp1FillMode], drb = rss_[kDrbPointers];
    const unsigned rf = (pp1 >> 21) & 31, type = (pp1 >> 8) & 7;
    if (rf == 4) return overlay_pixel(drb & 0x3FF, fx, fy) & 0xFF;
    const bool same_word = type <= 1 || type >= 4;
    const unsigned page = rf == 1 && !same_word && ((drb >> 10) & 0x3FF) ? (drb >> 10) & 0x3FF : drb & 0x3FF;
    const u64 w = pixel_word(page, fx, fy);
    const bool b_half = rf == 1 && same_word;
    switch (type) {
        case 0: {
            const u32 h = static_cast<u32>(w >> (b_half ? 12 : 0));
            auto c = [&](unsigned s) { return ((h >> s) & 15) * 0x11; };
            return c(0) | (c(4) << 8) | (c(8) << 16) | ((static_cast<u32>(w >> (b_half ? 28 : 24)) & 15) * 0x11 << 24);
        }
        case 1: {
            const u32 h = static_cast<u32>(w >> (b_half ? 15 : 0));
            auto c = [&](unsigned s) { return (((h >> s) & 31) << 3) | (((h >> s) & 31) >> 2); };
            return c(0) | (c(5) << 8) | (c(10) << 16);
        }
        case 2: return static_cast<u32>(w);
        case 3: return static_cast<u32>(w) & 0xFFFFFF;
        default: return static_cast<u32>(w >> (b_half ? 12 : 0)) & 0xFFF;
    }
}

void Impact::arm_transfer(int sx, int sy, int ex, int ey, unsigned type) {
    const u32 mode = rss_[kXfrMode];
    Xfer& x = xfer_;
    x = Xfer{};
    x.armed = true;
    x.read = type == 2 || type == 4;
    x.pio = type == 2 || type == 3;
    x.x0 = sx, x.y0 = sy, x.x1 = ex, x.y1 = ey;
    x.dx = ex >= sx ? 1 : -1, x.dy = ey >= sy ? 1 : -1;
    x.width = rss_[kXfrSize] & 0xFFFF;
    if (!x.width) x.width = static_cast<unsigned>(std::abs(ex - sx) + 1);
    x.lines = static_cast<unsigned>(std::abs(ey - sy) + 1);
    x.bpp = transfer_bpp();
    x.begin = (mode >> 8) & 7;
    x.stride = (mode >> 14) & 15;
    x.skip = x.begin;
}

// A line of host pixels into the write transfer (dropped with none armed).
void Impact::transfer_line_in(const u8* bytes, std::size_t n) {
    Xfer& x = xfer_;
    if (!x.armed || x.read || x.line >= x.lines) return;
    const u32 mode = rss_[kXfrMode];
    if (((mode >> 4) & 15) == 2 || ((mode >> 4) & 15) == 1) {  // depth, stencil: not modelled
        ++x.line;
        return;
    }
    const int y = x.y0 + static_cast<int>(x.line) * x.dy;
    for (unsigned k = 0; k < x.width && (k + 1) * x.bpp <= n; ++k) plot(x.x0 + static_cast<int>(k) * x.dx, y, host_to_pixel(bytes + k * x.bpp));
    ++x.line;
    ++display_version_;
}

// The read transfer's next line of host pixels.
std::vector<u8> Impact::transfer_line_out() {
    Xfer& x = xfer_;
    std::vector<u8> out;
    if (!x.armed || !x.read || x.line >= x.lines) return out;
    const int y = x.y0 + static_cast<int>(x.line) * x.dy;
    for (unsigned k = 0; k < x.width; ++k) {
        int fx, fy;
        const u32 px = to_framebuffer(x.x0 + static_cast<int>(k) * x.dx, y, fx, fy) ? read_pixel(fx, fy) : 0;
        pixel_to_host(px, out);
    }
    ++x.line;
    return out;
}

// A PIO write doubleword (or its high half alone): each line starts in a
// fresh doubleword after its begin offset; the rest of a line's last
// doubleword is dropped.
void Impact::pio_write(u64 dw, unsigned bytes) {
    Xfer& x = xfer_;
    const unsigned line_bytes = x.width * x.bpp;
    for (unsigned i = 0; i < bytes && x.line < x.lines; ++i) {
        if (x.skip) {
            --x.skip;
            continue;
        }
        x.buf.push_back(static_cast<u8>(dw >> (56 - 8 * i)));
        if (x.buf.size() == line_bytes) {
            transfer_line_in(x.buf.data(), x.buf.size());
            x.buf.clear();
            x.begin = (x.begin + line_bytes + x.stride) & 7;
            x.skip = x.begin;
            break;
        }
    }
}

// A PIO read: the high half takes the stream's next doubleword (each line
// filler up to its begin offset, the pixels, padding to the doubleword),
// the low half reads what it took.
u32 Impact::pio_read_hi() {
    Xfer& x = xfer_;
    if (!x.armed || !x.read || !x.pio) return 0;
    while (x.out.size() - x.out_pos < 8 && x.line < x.lines) {
        const unsigned begin = x.begin;
        std::vector<u8> line = transfer_line_out();
        // RGB(A) PIO reads come in the reverse of the DMA component order unless bit 18 asks for DMA order.
        const u32 mode = rss_[kXfrMode];
        const unsigned f = (mode >> 4) & 15;
        if ((f == 7 || f == 8) && !(mode & (1u << 18))) {
            const unsigned per = static_cast<unsigned>(format_comps(f, mode & (1u << 22)).size()), n = x.bpp / std::max(1u, per);
            for (std::size_t p = 0; p + x.bpp <= line.size(); p += x.bpp)
                for (unsigned c = 0; c < per / 2; ++c)
                    for (unsigned b = 0; b < n; ++b) std::swap(line[p + c * n + b], line[p + (per - 1 - c) * n + b]);
        }
        x.out.insert(x.out.end(), begin, 0);
        x.out.insert(x.out.end(), line.begin(), line.end());
        while ((x.out.size() - x.out_pos) % 8) x.out.push_back(0);
        x.begin = (begin + x.width * x.bpp + x.stride) & 7;
    }
    u64 dw = 0;
    for (unsigned i = 0; i < 8; ++i) dw = (dw << 8) | (x.out_pos + i < x.out.size() ? x.out[x.out_pos + i] : 0);
    x.out_pos = std::min(x.out.size(), x.out_pos + 8);
    if (x.out_pos == x.out.size()) x.out.clear(), x.out_pos = 0;
    x.lo = static_cast<u32>(dw);
    return static_cast<u32>(dw >> 32);
}

// ---- host DMA ---------------------------------------------------------------
//
// The DMA engine's start word: bit 0 run, 2:1 the pool, 3 board to host.
// Host addresses are logical in the pool; its page table (DMA registers
// 0x20 + 2 * pool, the address in the low word) holds a 32-bit frame number
// per 4 KB page. Host to board runs at once into the armed write transfer;
// board to host waits for the raster side (XFRCONTROL 9, or raster
// interface register 5) as well.

void Impact::dma_start(u32 word) {
    if (!(word & 1)) return;
    if (word & 8) {
        if (xfer_.armed && xfer_.read && !xfer_.pio && (rss_[kXfrControl] & 0xF) == 9) dma_run(word);
        else dma_read_pending_ = word;
        return;
    }
    dma_run(word);
}

bool Impact::host_read(u64 paddr, u8* out, std::size_t n) const {
    if (!bus_) return false;
    for (std::size_t i = 0; i < n; ++i) {
        const u8* h = bus_->host_read_ptr(paddr + i);
        out[i] = h ? *h : 0;
    }
    return true;
}

void Impact::format_bytes(std::vector<u8>& bytes) const {
    // The formatter (command 0xC00): within each pixel of N components of
    // 1, 2 or 4 bytes, swap each component's bytes (bit 6; byte components
    // in pairs) and reverse the components (bit 7).
    const u32 m = formatter_;
    if (!(m & 0xC0)) return;
    const unsigned size = (m & 3) == 1 ? 1 : (m & 3) == 2 ? 2 : (m & 3) == 3 ? 4 : 0;
    if (!size) return;
    const unsigned n = ((m >> 2) & 3) + 1, px = size * n;
    for (std::size_t p = 0; p + px <= bytes.size(); p += px) {
        u8* q = bytes.data() + p;
        if (m & 0x40) {
            if (size == 1)
                for (unsigned c = 0; c + 1 < n; c += 2) std::swap(q[c], q[c + 1]);
            else
                for (unsigned c = 0; c < n; ++c) std::reverse(q + c * size, q + (c + 1) * size);
        }
        if (m & 0x80)
            for (unsigned c = 0; c < n / 2; ++c)
                std::swap_ranges(q + c * size, q + (c + 1) * size, q + (n - 1 - c) * size);
    }
}

void Impact::dma_run(u32 word) {
    if (!bus_) return;
    const bool read = word & 8;
    const unsigned pool = (word >> 1) & 3;
    const u32 table = dma_regs_[kDmaPoolTable + 2 * pool + 1] & ~3u;
    if (table < 0x1000) return;  // a parked pool (the kernel's dummy table): no transfer
    // List mode: up to eight 16-bit indices into the pool's table, bit 15
    // on the last (just [0x8000]: normal mode).
    std::vector<u32> list;
    for (unsigned i = 0; i < 4 && (list.empty() || !(list.back() & 0x8000)); ++i)
        for (u32 h : {dma_regs_[kDmaPageList + i] >> 16, dma_regs_[kDmaPageList + i] & 0xFFFF})
            if (list.empty() || !(list.back() & 0x8000)) list.push_back(h);
    if (list.empty() || !(list.back() & 0x8000) || (list.size() == 1 && list[0] == 0x8000)) list.clear();
    u32 base, stride, lines, len;
    if (list.empty()) {
        base = dma_regs_[kDmaRowStart] + dma_regs_[kDmaRowOffset];
        stride = dma_regs_[kDmaStride], lines = dma_regs_[kDmaLines], len = dma_regs_[kDmaLineBytes];
    } else {
        const u32 n = static_cast<u32>(list.size());
        base = dma_regs_[kDmaRowOffset] & 0xFFF, stride = 0, lines = 1;
        len = n == 1 ? dma_regs_[kDmaLineBytes] : dma_regs_[kDmaLineBytes] + 0x1000 * (n - 2) + dma_regs_[kDmaLastBytes];
        for (u32& h : list) h &= 0x7FFF;
    }
    if (lines > 4096 || len > (1u << 24)) return;
    u32 cached_page = ~0u, cached_frame = 0;
    auto phys = [&](u32 logical, u64& pa) {
        const u32 page = logical >> 12;
        if (page != cached_page) {
            const u32 index = list.empty() ? page : page < list.size() ? list[page] : ~0u;
            if (index == ~0u) return false;
            u8 f[4];
            host_read(table + 4ull * index, f, 4);
            cached_page = page, cached_frame = be_value(f, 4);
        }
        pa = (u64{cached_frame} << 12) | (logical & 0xFFF);
        return true;
    };
    std::deque<u8> stream;  // board to host: the raster side's bytes across its lines
    std::vector<u8> bytes;
    for (u32 i = 0; i < lines; ++i) {
        const u32 a = base + i * stride;
        bytes.assign(len, 0);
        if (read) {
            while (stream.size() < len && !eram_out_.empty()) stream.push_back(eram_out_.front()), eram_out_.pop_front();
            while (stream.size() < len && xfer_.armed && xfer_.read && xfer_.line < xfer_.lines) {
                const std::vector<u8> l = transfer_line_out();
                stream.insert(stream.end(), l.begin(), l.end());
            }
            for (u32 k = 0; k < len && !stream.empty(); ++k) bytes[k] = stream.front(), stream.pop_front();
            format_bytes(bytes);
            for (u32 k = 0; k < len;) {
                u64 pa;
                if (!phys(a + k, pa)) return;
                const u32 run = std::min<u32>(len - k, 0x1000 - ((a + k) & 0xFFF));
                bus_->write_block(pa, bytes.data() + k, run);
                k += run;
            }
        } else {
            for (u32 k = 0; k < len;) {
                u64 pa;
                if (!phys(a + k, pa)) return;
                const u32 run = std::min<u32>(len - k, 0x1000 - ((a + k) & 0xFFF));
                host_read(pa, bytes.data() + k, run);
                k += run;
            }
            if (eram_in_) {  // ERAM write data
                for (std::size_t k = 0; k + 3 < bytes.size(); k += 4) eram_in_->push_back(be_value(&bytes[k], 4));
                continue;
            }
            if (gl_.wants_pixel_data()) {  // an image for the GE (glDrawPixels by DMA)
                format_bytes(bytes);
                for (std::size_t k = 0; k + 3 < bytes.size(); k += 4) gl_pixels_.push_back(be_value(&bytes[k], 4));
                if (gl_pixels_.size() >= gl_.pixel_words_needed()) {
                    gl_.pixel_data(gl_pixels_.data(), static_cast<unsigned>(gl_pixels_.size()));
                    gl_pixels_.clear();
                }
                continue;
            }
            format_bytes(bytes);
            transfer_line_in(bytes.data(), bytes.size());
        }
    }
}

u64 Impact::pixel_word(unsigned page, int fx, int fy) const { return *word_at(page, fx, fy); }

// The overlay: 768 x 16 pixel pages of 9-bit pixels, four to a word.
const u64* Impact::overlay_word(unsigned page, int fx, int fy, unsigned& shift) const {
    const unsigned two = engines() == 2 && !pixmem_[1].empty();
    const unsigned engine = two ? fy & 1 : 0, row = two ? static_cast<unsigned>(fy) >> 1 : static_cast<unsigned>(fy);
    const unsigned tiles = std::max(1u, rss_[kDrbSize] & 3);
    const unsigned pg = (page + row / kTileRows * tiles + static_cast<unsigned>(fx) / kOverlayTileWidth) & (kPages - 1);
    const unsigned p = (row % kTileRows) * kOverlayTileWidth + fx % kOverlayTileWidth;
    shift = 9 * (p % 4);
    return &pixmem_[engine][std::size_t{pg} * kTileWidth * kTileRows + p / 4];
}

// The writable form notes the page row written for render().
u64* Impact::overlay_word(unsigned page, int fx, int fy, unsigned& shift) {
    u64* p = const_cast<u64*>(static_cast<const Impact*>(this)->overlay_word(page, fx, fy, shift));
    mark_written(engine_of(fy), p);  // an overlay page row is one main page row's words
    return p;
}

u32 Impact::overlay_pixel(unsigned page, int fx, int fy) const {
    unsigned shift;
    return static_cast<u32>(*overlay_word(page, fx, fy, shift) >> shift) & 0x1FF;
}

// The clip-ID match (PP1WINMODE bits 7:4, one bit per ID; none set: no
// check) and screen masks 1-4: WINMODE bit n-1 enables mask n (x and y
// ranges, min << 16 | max, in framebuffer coordinates) and bit n+3 keeps
// what is inside it rather than outside.
bool Impact::pixel_visible(int fx, int fy) const {
    const u32 match = (rss_[kPp1WinMode] >> 4) & 0xF;
    if (match && !((match >> cid_[std::size_t{static_cast<unsigned>(fy)} * kFbSize + static_cast<unsigned>(fx)]) & 1))
        return false;
    const u32 mode = rss_[kWinMode];
    for (unsigned n = 0; n < 4; ++n) {
        if (!(mode & (1u << n))) continue;
        const u32 mx = rss_[kScrMask1X + 2 * n], my = rss_[kScrMask1X + 2 * n + 1];
        const bool inside = fx >= half(mx >> 16) && fx <= half(mx) && fy >= half(my >> 16) && fy <= half(my);
        if (inside != ((mode & (0x10u << n)) != 0)) return false;
    }
    return true;
}

// Overlay drawing (draw field 0x4N) goes to the overlay buffer at
// DRBpointers' first page; the plane mask is ColorMaskMSBs' low 8 bits for
// GL's overlay (0x48), and all 8 planes for the X server's when it sets any.
void Impact::store_overlay(const PixelTarget& t, int fx, int fy, u32 value) {
    unsigned shift;
    u64* p = overlay_word(t.drb & 0x3FF, fx, fy, shift);
    const u64 old = (*p >> shift) & 0x1FF;
    const u64 mask = ((t.pp1 >> 14) & 0x7F) == 0x48 ? t.msbs & 0xFF : t.msbs ? 0xFF : 0;
    const u64 v = (t.pp1 & 4) ? logic_op(t.pp1 >> 26, value, old) : value;
    const u64 n = ((old & ~mask) | (v & mask)) & 0x1FF;
    *p = (*p & ~(u64{0x1FF} << shift)) | (n << shift);
}

Impact::PixelTarget Impact::rss_target() const {
    return {rss_[kPp1FillMode], rss_[kDrbPointers], rss_[kMaskLsbsA], rss_[kMaskLsbsB], rss_[kMaskMsbs]};
}

void Impact::store_pixel(int fx, int fy, u32 colour) {
    if (!pixel_visible(fx, fy)) return;
    if (((rss_[kPp1FillMode] >> 14) & 0x7F) == 0x50) {  // the clip-ID planes, under PP1WINMODE's write enables (11:10)
        u8& c = cid_[std::size_t{static_cast<unsigned>(fy)} * kFbSize + static_cast<unsigned>(fx)];
        const u8 m = static_cast<u8>((rss_[kPp1WinMode] >> 10) & 3);
        c = static_cast<u8>((c & ~m) | (colour & m));
        ++pixels_drawn_;
        return;
    }
    store_pixel_to(rss_target(), fx, fy, colour);
}

// The PP1s: the colour formatted for the pixel type, through the logic op
// and the plane masks into the buffers the draw field names.
Impact::StorePlan Impact::store_plan(const PixelTarget& t) const {
    StorePlan p;
    p.t = t;
    const u32 pp1 = t.pp1;
    const unsigned field = (pp1 >> 14) & 0x7F;
    p.type = (pp1 >> 8) & 7;
    p.overlay = (field & 0x70) == 0x40;
    p.unhandled = !p.overlay && field > 5;
    p.size36 = pp1 & (1u << 13);
    p.dither = pp1 & 8;
    p.logic = pp1 & 4;
    p.op = pp1 >> 26;
    // Buffer B is DRBpointers' second page (19:10), except where pixel
    // pairs and colour indices keep both buffers in one word (the plane
    // mask picks the half). Both buffers on one page are written once (an
    // XOR is not undone).
    const u32 drb = t.drb;
    const unsigned page_a = drb & 0x3FF;
    const bool same_word = p.type <= 1 || p.type >= 4;
    const unsigned page_b = same_word || !((drb >> 10) & 0x3FF) ? page_a : (drb >> 10) & 0x3FF;
    const u64 msbs = u64{t.msbs & 0xF} << 32;
    const u64 mask_a = t.lsb_a | msbs, mask_b = t.lsb_b | msbs;
    auto put = [&](unsigned page, u64 mask) {
        p.page[p.puts] = page;
        p.mask[p.puts++] = mask;
    };
    switch (field) {
        case 2:
        case 4: put(page_b, mask_b); break;
        case 3:
        case 5:
            if (page_b == page_a) put(page_a, mask_a | mask_b);
            else put(page_a, mask_a), put(page_b, mask_b);
            break;
        default: put(page_a, mask_a); break;
    }
    return p;
}

void Impact::store_planned(const StorePlan& p, int fx, int fy, u32 colour) {
    if (p.overlay) {
        store_overlay(p.t, fx, fy, colour & 0xFF);
        ++pixels_drawn_;
        return;
    }
    if (p.unhandled) {
        if (p.t.pp1 != unhandled_pp1_) unknown("draw field", kPp1FillMode, p.t.pp1);  // once per mode
        unhandled_pp1_ = p.t.pp1;
        return;
    }
    const u32 r = colour & 0xFF, g = (colour >> 8) & 0xFF, b = (colour >> 16) & 0xFF, a = colour >> 24;
    u64 w;
    switch (p.type) {
        case 0:
        case 1: {  // pixel pairs (4:4:4 or 5:5:5): the colour in both halves
            const unsigned bits = p.type == 0 ? 4 : 5;
            auto q = [&](u32 c) {
                if (!p.dither) return c >> (8 - bits);
                const u32 th = kBayer[fy & 3][fx & 3];
                const u32 v = (c * ((1u << bits) - 1) * 16 + th * 255) / (255 * 16);
                return std::min(v, (1u << bits) - 1);
            };
            const u64 v = q(r) | (q(g) << bits) | (q(b) << (2 * bits));
            if (p.type == 0) w = v | (v << 12) | (u64{a >> 4} << 24) | (u64{a >> 4} << 28);
            else w = v | (v << 15) | (u64{a >> 7} << 30) | (u64{a >> 7} << 31);
            break;
        }
        case 2: w = colour; break;  // RGBA 8888
        case 3:
            w = colour & 0xFFFFFF;
            if (p.size36) w |= u64{(r >> 4) | ((g >> 4) << 4) | ((b >> 4) << 8)} << 24;  // 12:12:12 low nibbles
            break;
        default: w = (colour & 0xFFF) | (u64{colour & 0xFFF} << 12); break;  // colour index, both buffers
    }
    for (unsigned k = 0; k < p.puts; ++k) {
        u64* q = word_at(p.page[k], fx, fy);
        const u64 v = p.logic ? logic_op(p.op, w, *q) : w;
        *q = ((*q & ~p.mask[k]) | (v & p.mask[k])) & kWord36;
    }
    ++pixels_drawn_;
}

// ---- scan-out -------------------------------------------------------------------

unsigned Impact::did_table_lines(u32 table) const {
    unsigned n = 0;
    while (n < 2048 && vc3_.sram[(table + n) & 0x7FFF] != 0xFFFF) ++n;
    return n;
}

// The window IDs of one displayed row: runs of (x << 5 | id) from the row's
// line table, each lasting until the next starts.
void Impact::did_row(u32 table, unsigned row, unsigned width, std::vector<u8>& dids) const {
    dids.assign(width, 0);
    u8* d = dids.data();
    const u16* sram = vc3_.sram.data();
    const u32 line = sram[(table + row) & 0x7FFF];
    for (unsigned k = 0; k < 256; ++k) {
        const u16 e = sram[(line + k) & 0x7FFF];
        const unsigned x = e >> 5;
        if (x >= 0x7FF) break;
        const u16 next = sram[(line + k + 1) & 0x7FFF];
        unsigned end = next >> 5;
        if (end <= x || end > width) end = width;
        if (x < end) std::fill(d + x, d + end, static_cast<u8>(e & 31));
    }
}

void Impact::display_size(unsigned& width, unsigned& height) const {
    width = 1280, height = 1024;
    const u32 table = vc3_.regs[kVc3MainDids];
    if (!table) return;
    const unsigned lines = did_table_lines(table);
    struct Size {
        unsigned h, w;
    };
    static constexpr Size kSizes[] = {{480, 640}, {492, 1280}, {576, 768}, {600, 800}, {768, 1024},
                                      {864, 1152}, {960, 1280}, {1024, 1280}, {1035, 1920}, {1200, 1600}};
    for (const Size& s : kSizes)
        if (s.h == lines) width = s.w, height = s.h;
}

Impact::Shown Impact::shown() const {
    Shown s;
    const u16 cc = vc3_.regs[kVc3CursorControl];
    s.cursor = (cc & 3) == 3;
    s.cursor_size = (cc & 8) ? 64 : 32;
    s.cursor_x = static_cast<int>(vc3_.regs[kVc3CursorX]) - 31;
    s.cursor_y = static_cast<int>(vc3_.regs[kVc3CursorY]) - 31;
    s.cursor_base = be_bytes(xmap_.config.data(), 4) & 0x7FF;
    s.two = engines() == 2 && !pixmem_[1].empty();
    s.tiles = std::max(1u, (rss_[kDrbSize] >> 2) & 15);
    s.overlay_tiles = std::max(1u, rss_[kDrbSize] & 3);
    display_size(s.width, s.height);
    return s;
}

// As render_full(), but converting only the rows that show a page row
// written since the last call, or that the cursor left or entered.
bool Impact::render(std::vector<u32>& argb, unsigned& width, unsigned& height) {
    const Shown now = shown();
    width = now.width, height = now.height;
    const std::size_t total = std::size_t{width} * height;
    if (!dac_.regs[4]) {  // pixel read mask 0: blank
        argb.assign(total, 0xFF000000u);
        full_redraw_ = true;
        return true;
    }
    const bool full = full_redraw_ || argb.size() != total || now.two != shown_.two || now.tiles != shown_.tiles ||
                      now.overlay_tiles != shown_.overlay_tiles || now.width != shown_.width ||
                      now.height != shown_.height;
    std::vector<u8> rows;
    if (full) {
        argb.resize(total);
    } else {
        rows.assign(height, 0);
        bool any = false;
        // The page rows a screen row shows: both buffers and the overlay.
        const u32 dib = be_bytes(xmap_.dib.data(), 4);
        const unsigned pages[] = {dib & 0x3FF, (dib >> 10) & 0x3FF};
        const unsigned overlay_page = (dib >> 20) & 0x3FF;
        const unsigned spans = (width + kTileWidth - 1) / kTileWidth, ospans = (width + kOverlayTileWidth - 1) / kOverlayTileWidth;
        for (unsigned row = 0; row < height; ++row) {
            const unsigned fy = height - 1 - row;
            const unsigned engine = now.two ? fy & 1 : 0, mrow = now.two ? fy >> 1 : fy;
            const u8* written = row_written_[engine].data();
            auto shows = [&](unsigned page, unsigned tiles, unsigned n) {
                for (unsigned k = 0; k < n; ++k)
                    if (written[((page + mrow / kTileRows * tiles + k) & (kPages - 1)) * kTileRows + mrow % kTileRows]) return true;
                return false;
            };
            if (shows(pages[0], now.tiles, spans) || shows(pages[1], now.tiles, spans) ||
                shows(overlay_page, now.overlay_tiles, ospans))
                rows[row] = 1, any = true;
        }
        if (!(now == shown_)) {  // the cursor moved, came or went, or changed colour
            for (const Shown* c : {static_cast<const Shown*>(&shown_), &now}) {
                if (!c->cursor) continue;
                for (int y = std::max(c->cursor_y, 0); y < std::min(c->cursor_y + c->cursor_size, static_cast<int>(height)); ++y)
                    rows[static_cast<unsigned>(y)] = 1, any = true;
            }
        }
        if (!any) rows.clear();  // nothing to convert
    }
    if (full || !rows.empty()) scan_out(argb, width, height, full ? nullptr : rows.data());
    for (auto& w : row_written_) std::fill(w.begin(), w.end(), u8{0});
    full_redraw_ = false;
    shown_ = now;
    return true;
}

bool Impact::render_full(std::vector<u32>& argb, unsigned& width, unsigned& height) const {
    display_size(width, height);
    argb.assign(std::size_t{width} * height, 0xFF000000u);
    if (!dac_.regs[4]) return true;  // pixel read mask 0: blank
    scan_out(argb, width, height, nullptr);
    return true;
}

void Impact::scan_out(std::vector<u32>& argb, unsigned width, unsigned height, const u8* rows) const {
    const u32 main_table = vc3_.regs[kVc3MainDids], overlay_table = vc3_.regs[kVc3OverlayDids];
    const u16 cc = vc3_.regs[kVc3CursorControl];
    const bool cursor = (cc & 3) == 3;
    const int csize = (cc & 8) ? 64 : 32, cwords = csize / 16;
    const int cx = static_cast<int>(vc3_.regs[kVc3CursorX]) - 31, cy = static_cast<int>(vc3_.regs[kVc3CursorY]) - 31;
    const u32 cbase = (be_bytes(xmap_.config.data(), 4) & 0x7FF) * 4;

    // What stays the same for the whole frame, worked out once: the XMAP's
    // modes for each window ID, the buffer pages, the memory layout and the
    // RAMDAC's gamma as ready ARGB parts.
    const u32 dib = be_bytes(xmap_.dib.data(), 4), buf_select = be_bytes(xmap_.buf_select.data(), 4);
    const unsigned page_a = dib & 0x3FF, page_b = (dib >> 10) & 0x3FF, overlay_page = (dib >> 20) & 0x3FF;
    std::array<u32, 32> modes{}, omodes{};
    for (unsigned d = 0; d < 32; ++d) {
        modes[d] = be_bytes(xmap_.main_mode.data() + 4 * d, 4);
        omodes[d] = be_bytes(xmap_.overlay_mode.data() + 4 * d, 4);
    }
    const u32* cmap = cmap_[0].entries.data();
    auto bgr_of = [](u32 c) { return ((c >> 16) & 0xFF) | (c & 0xFF00) | ((c & 0xFF) << 16); };
    std::array<u32, 256> gr{}, gg{}, gb{};
    for (unsigned i = 0; i < 256; ++i) {
        gr[i] = 0xFF000000u | (u32{dac_.gamma[i][0]} >> 2) << 16;
        gg[i] = (u32{dac_.gamma[i][1]} >> 2) << 8;
        gb[i] = u32{dac_.gamma[i][2]} >> 2;
    }
    const u32 *r_of = gr.data(), *g_of = gg.data(), *b_of = gb.data();
    auto argb_of = [=](u32 bgr) { return r_of[bgr & 0xFF] | g_of[(bgr >> 8) & 0xFF] | b_of[(bgr >> 16) & 0xFF]; };
    // The colour map as shown, through the gamma tables.
    std::vector<u32> shown(8192);
    for (unsigned i = 0; i < 8192; ++i) shown[i] = argb_of(bgr_of(cmap[i]));
    const u32* shown_cmap = shown.data();
    const bool two = engines() == 2 && !pixmem_[1].empty();
    const unsigned tiles = std::max(1u, (rss_[kDrbSize] >> 2) & 15), otiles = std::max(1u, rss_[kDrbSize] & 3);
    constexpr std::size_t kPageWords = std::size_t{kTileWidth} * kTileRows;

    // Rows are independent: several threads share them out (the machine is
    // stopped meanwhile).
    WorkerPool::shared().run(height, [&](unsigned first, unsigned last) {
        std::vector<u8> dids, odids;
        for (unsigned row = first; row < last; ++row) {
            if (rows && !rows[row]) continue;
            if (main_table) did_row(main_table, row, width, dids);
            else dids.assign(width, 0);
            if (overlay_table) did_row(overlay_table, row, width, odids);
            else odids.assign(width, 0);
            // This row's place in pixel memory (see word_at and overlay_word).
            const unsigned fy = height - 1 - row;
            const unsigned engine = two ? fy & 1 : 0, mrow = two ? fy >> 1 : fy;
            const u64* mem = pixmem_[engine].data();
            const unsigned row_pages = mrow / kTileRows * tiles, orow_pages = mrow / kTileRows * otiles;
            const std::size_t row_off = std::size_t{mrow % kTileRows} * kTileWidth;
            const unsigned orow_off = (mrow % kTileRows) * kOverlayTileWidth;
            u32* out = &argb[std::size_t{row} * width];

            // The main planes, a run of one window ID at a time, a page row
            // at a time within it.
            for (unsigned x0 = 0; x0 < width;) {
                const u8 did = dids[x0];
                unsigned x1 = x0 + 1;
                while (x1 < width && dids[x1] == did) ++x1;
                const u32 mode = modes[did], fmt = mode & 31;
                const bool buf_b = (buf_select >> did) & 1;
                const unsigned page = fmt != 5 && fmt != 7 && fmt >= 4 && buf_b ? page_b : page_a;
                auto each = [=](auto convert) {
                    for (unsigned x = x0; x < x1;) {
                        const unsigned tile = x / kTileWidth, end = std::min(x1, (tile + 1) * kTileWidth);
                        const u64* w = mem + ((page + row_pages + tile) & (kPages - 1)) * kPageWords + row_off + x % kTileWidth;
                        for (u32 *o = out + x, *e = out + end; o < e; ++o, ++w) *o = convert(*w);
                        x = end;
                    }
                };
                if (fmt < 4) {  // colour index
                    const unsigned shift = buf_b ? 12 : 0, keep = fmt == 1 ? 0xFF : 0xFFF, base = ((mode >> 5) & 31) * 256;
                    each([=](u64 w) { return shown_cmap[(base + (static_cast<u32>(w >> shift) & keep)) & 0x1FFF]; });
                } else if (fmt == 5) {  // 4:4:4 pairs
                    const unsigned shift = buf_b ? 12 : 0;
                    each([=](u64 w) {
                        const u32 h = static_cast<u32>(w >> shift) & 0xFFF;
                        return argb_of(((h & 15) * 17) | (((h >> 4) & 15) * 17) << 8 | (((h >> 8) & 15) * 17) << 16);
                    });
                } else if (fmt == 7) {  // 5:5:5 pairs
                    const unsigned shift = buf_b ? 15 : 0;
                    each([=](u64 w) {
                        const u32 h = static_cast<u32>(w >> shift) & 0x7FFF;
                        auto e = [](u32 c) { return (c << 3) | (c >> 2); };
                        return argb_of(e(h & 31) | e((h >> 5) & 31) << 8 | e((h >> 10) & 31) << 16);
                    });
                } else {  // 24-bit (and 12:12:12): 0x00BBGGRR
                    each([=](u64 w) { return argb_of(static_cast<u32>(w) & 0xFFFFFF); });
                }
                x0 = x1;
            }

            // The overlay, where its window's mode shows it and it is not 0.
            for (unsigned x0 = 0; x0 < width;) {
                const u8 odid = odids[x0];
                unsigned x1 = x0 + 1;
                while (x1 < width && odids[x1] == odid) ++x1;
                if (const u32 omode = omodes[odid]) {
                    // Four pixels to a word, most of them usually clear.
                    constexpr u64 kAll = 0xFFull | 0xFFull << 9 | 0xFFull << 18 | 0xFFull << 27;
                    const unsigned base = ((omode >> 3) & 31) * 256;
                    for (unsigned x = x0; x < x1;) {
                        const unsigned pg = (overlay_page + orow_pages + x / kOverlayTileWidth) & (kPages - 1);
                        const unsigned p = orow_off + x % kOverlayTileWidth;
                        const u64 word = mem[pg * kPageWords + p / 4];
                        const unsigned end = std::min(x1, x + 4 - p % 4);
                        if (word & kAll)
                            for (unsigned i = x, q = p % 4; i < end; ++i, ++q)
                                if (const u32 ov = static_cast<u32>(word >> (9 * q)) & 0xFF) out[i] = shown_cmap[(base + ov) & 0x1FFF];
                        x = end;
                    }
                }
                x0 = x1;
            }

            // The cursor on top.
            const int gy = static_cast<int>(row) - cy;
            if (cursor && gy >= 0 && gy < csize) {
                for (int gx = std::max(0, -cx); gx < csize && cx + gx < static_cast<int>(width); ++gx) {
                    const u32 at = vc3_.regs[kVc3CursorGlyph] + gy * cwords + gx / 16;
                    const unsigned bit = 15 - gx % 16;
                    const unsigned v = ((vc3_.sram[at & 0x7FFF] >> bit) & 1) |
                                       (((vc3_.sram[(at + csize * cwords) & 0x7FFF] >> bit) & 1) << 1);
                    if (v) out[cx + gx] = argb_of(bgr_of(cbase + v < 8192 ? cmap_[0].entries[cbase + v] : 0xFFFFFF));
                }
            }
        }
    });
}

bool Impact::cursor_position(int& x, int& y) const {
    if ((vc3_.regs[kVc3CursorControl] & 3) != 3) return false;
    x = static_cast<int>(vc3_.regs[kVc3CursorX]) - 31;
    y = static_cast<int>(vc3_.regs[kVc3CursorY]) - 31;
    return true;
}

}  // namespace sg::ip22
