// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// References: ANSI X3.131 (SCSI-2), SCSI Multimedia Commands for READ TOC.
#include "devices/scsi_disk.h"

#include <algorithm>
#include <cstring>
#include <span>

#include "core/log.h"

namespace sg {

namespace {

constexpr u8 kGood = 0x00;
constexpr u8 kCheckCondition = 0x02;

constexpr u8 kSenseNone = 0x0, kSenseNotReady = 0x2, kSenseMediumError = 0x3, kSenseIllegalRequest = 0x5;
constexpr u8 kSenseUnitAttention = 0x6, kSenseDataProtect = 0x7;
constexpr u8 kAscMediumChanged = 0x28, kAscPowerOnReset = 0x29, kAscMediumNotPresent = 0x3A, kAscRemovalPrevented = 0x53;
constexpr u8 kAscInvalidOpcode = 0x20, kAscLbaOutOfRange = 0x21, kAscWriteProtected = 0x27;
constexpr u8 kAscUnrecoveredRead = 0x11, kAscInvalidParameter = 0x26, kAscInvalidCdbField = 0x24;

u32 be32(const u8* p) { return (u32{p[0]} << 24) | (u32{p[1]} << 16) | (u32{p[2]} << 8) | p[3]; }
u16 be16(const u8* p) { return static_cast<u16>((p[0] << 8) | p[1]); }
void put32(u8* p, u32 v) {
    p[0] = static_cast<u8>(v >> 24);
    p[1] = static_cast<u8>(v >> 16);
    p[2] = static_cast<u8>(v >> 8);
    p[3] = static_cast<u8>(v);
}
void put_padded(u8* p, const char* s, std::size_t n) {
    std::memset(p, ' ', n);
    std::memcpy(p, s, std::min(n, std::strlen(s)));
}

}  // namespace

ScsiDisk::ScsiDisk(Kind kind, const std::string& path) : kind_(kind), read_only_(kind == Kind::CdRom) {
    block_size_ = default_block_size();
    open(path);
}

void ScsiDisk::reset() {
    block_size_ = default_block_size();
    prevent_removal_ = false;
    unit_attention_ = true;
    attention_asc_ = kAscPowerOnReset;
}

bool ScsiDisk::open(const std::string& path) {
    if (file_.is_open()) file_.close();
    file_.clear();
    bytes_ = 0;
    read_only_ = kind_ == Kind::CdRom;
    file_.open(path, std::ios::binary | std::ios::in | (read_only_ ? std::ios::openmode{} : std::ios::out));
    if (!file_.is_open() && !read_only_) {  // fall back to read-only access
        file_.open(path, std::ios::binary | std::ios::in);
        read_only_ = true;
    }
    if (file_.is_open()) {
        file_.seekg(0, std::ios::end);
        bytes_ = static_cast<u64>(file_.tellg()) / kDiskBlockSize * kDiskBlockSize;
    }
    return file_.is_open();
}

void ScsiDisk::eject() {
    if (!file_.is_open()) return;
    file_.close();
    bytes_ = 0;
    prevent_removal_ = false;
    unit_attention_ = true;
    attention_asc_ = kAscMediumChanged;
}

bool ScsiDisk::change_medium(const std::string& path) {
    prevent_removal_ = false;
    unit_attention_ = true;
    attention_asc_ = kAscMediumChanged;
    return open(path);
}

ScsiTarget::Result ScsiDisk::check_condition(u8 key, u8 asc, u8 ascq) {
    sense_key_ = key;
    sense_asc_ = asc;
    sense_ascq_ = ascq;
    Result r;
    r.status = kCheckCondition;
    return r;
}

bool ScsiDisk::read_blocks(u64 lba, u32 count, std::vector<u8>& out) {
    ++activity_;
    out.resize(static_cast<std::size_t>(count) * block_size_);
    file_.clear();
    file_.seekg(static_cast<std::streamoff>(lba * block_size_));
    return static_cast<bool>(file_.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(out.size())));
}

std::vector<u8> ScsiDisk::inquiry(unsigned allocation) const {
    std::vector<u8> d(36, 0);
    const bool cd = kind_ == Kind::CdRom;
    d[0] = cd ? 0x05 : 0x00;  // CD-ROM / direct access
    d[1] = cd ? 0x80 : 0x00;  // removable
    d[2] = 0x02;              // SCSI-2
    d[3] = 0x02;              // response data format
    d[4] = 31;                // additional length
    d[7] = 0x10;              // synchronous transfer supported
    put_padded(&d[8], cd ? "TOSHIBA" : "SGI", 8);
    put_padded(&d[16], cd ? "CD-ROM XM-3401TA" : "SEAGATE ST31200N", 16);
    put_padded(&d[32], cd ? "3593" : "8810", 4);
    d.resize(std::min<std::size_t>(d.size(), allocation));
    return d;
}

// The pages a drive of this kind has: the disk's error recovery, format,
// geometry and caching pages; the CD-ROM's error recovery, CD parameters and
// audio control pages. Page 0 (vendor) is the header alone; any other page
// is an illegal request (empty result).
std::vector<u8> ScsiDisk::mode_sense(const u8* cdb, bool ten_byte) const {
    const unsigned page = cdb[2] & 0x3F;
    const bool cd = kind_ == Kind::CdRom;
    static constexpr u8 kDiskPages[] = {0x00, 0x01, 0x03, 0x04, 0x08, 0x3F};
    static constexpr u8 kCdPages[] = {0x00, 0x01, 0x0D, 0x0E, 0x3F};
    bool known = false;
    for (u8 p : cd ? std::span<const u8>(kCdPages) : std::span<const u8>(kDiskPages)) known |= p == page;
    if (!known) return {};
    const bool no_block_descriptor = cdb[1] & 0x08;
    std::vector<u8> pages;
    auto add_page = [&](unsigned code, std::initializer_list<u8> body) {
        pages.push_back(static_cast<u8>(code));
        pages.push_back(static_cast<u8>(body.size()));
        pages.insert(pages.end(), body.begin(), body.end());
    };
    const u32 cylinders = static_cast<u32>(std::max<u64>(1, blocks() / (64 * 32)));
    if (page == 0x01 || page == 0x3F) add_page(0x01, {0, 8, 0, 0, 0, 0, 0, 0, 0, 0});  // error recovery
    if (!cd && (page == 0x03 || page == 0x3F))                                       // format: 64 sectors/track
        add_page(0x03, {0, 1, 0, 0, 0, 0, 0, 1, 0, 64, 2, 0, 0, 1, 0, 0, 0, 0, 0x40, 0, 0, 0});
    if (!cd && (page == 0x04 || page == 0x3F))  // geometry: 32 heads
        add_page(0x04, {static_cast<u8>(cylinders >> 16), static_cast<u8>(cylinders >> 8), static_cast<u8>(cylinders),
                        32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x15, 0x18, 0, 0});
    if (!cd && (page == 0x08 || page == 0x3F)) add_page(0x08, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0});  // caching
    // CD parameters: 60 seconds per minute, 75 frames per second
    if (cd && (page == 0x0D || page == 0x3F)) add_page(0x0D, {0, 0, 0, 60, 0, 75});
    // audio control: immediate; ports 0 and 1 play channels 0 and 1 at full volume
    if (cd && (page == 0x0E || page == 0x3F)) add_page(0x0E, {0x04, 0, 0, 0, 0, 0, 0x01, 0xFF, 0x02, 0xFF, 0, 0, 0, 0});

    std::vector<u8> block_descriptor;
    if (!no_block_descriptor) {
        block_descriptor.assign(8, 0);
        put32(&block_descriptor[0], static_cast<u32>(std::min<u64>(blocks(), 0xFFFFFF)));
        block_descriptor[0] = 0;  // density code
        put32(&block_descriptor[4], block_size_);
    }
    std::vector<u8> d;
    const u8 medium = 0x00;  // default medium type
    const u8 device_specific = read_only_ ? 0x80 : 0x00;
    if (ten_byte) {
        const std::size_t len = 8 + block_descriptor.size() + pages.size() - 2;
        d = {static_cast<u8>(len >> 8), static_cast<u8>(len), medium, device_specific, 0, 0, 0,
             static_cast<u8>(block_descriptor.size())};
    } else {
        const std::size_t len = 4 + block_descriptor.size() + pages.size() - 1;
        d = {static_cast<u8>(len), medium, device_specific, static_cast<u8>(block_descriptor.size())};
    }
    d.insert(d.end(), block_descriptor.begin(), block_descriptor.end());
    d.insert(d.end(), pages.begin(), pages.end());
    const unsigned allocation = ten_byte ? be16(&cdb[7]) : cdb[4];
    d.resize(std::min<std::size_t>(d.size(), allocation));
    return d;
}

ScsiTarget::Result ScsiDisk::command(const u8* cdb, unsigned length) {
    (void)length;
    Result r;
    const u8 op = cdb[0];
    if ((cdb[1] >> 5) != 0 && op != 0x03 && op != 0x12) return check_condition(kSenseIllegalRequest, 0x25);  // LUN
    if (op != 0x03 && op != 0x12) {
        if (unit_attention_) {
            unit_attention_ = false;
            return check_condition(kSenseUnitAttention, attention_asc_ ? attention_asc_ : kAscMediumChanged);
        }
        if (!file_.is_open() && op != 0x1B && op != 0x1E) return check_condition(kSenseNotReady, kAscMediumNotPresent);
    }
    switch (op) {
        case 0x00:  // TEST UNIT READY
        case 0x01:  // REZERO UNIT
        case 0x0B:  // SEEK(6)
        case 0x2B:  // SEEK(10)
        case 0x16:  // RESERVE
        case 0x17:  // RELEASE
        case 0x1D:  // SEND DIAGNOSTIC
        case 0x2F:  // VERIFY(10)
        case 0x35:  // SYNCHRONIZE CACHE
            break;
        case 0x1E:  // PREVENT ALLOW MEDIUM REMOVAL
            prevent_removal_ = cdb[4] & 1;
            break;
        case 0x1B:                                                   // START STOP UNIT
            if (kind_ == Kind::CdRom && (cdb[4] & 0x03) == 0x02) {  // LoEj, stop: eject
                if (prevent_removal_) return check_condition(kSenseIllegalRequest, kAscRemovalPrevented);
                file_.close();
                bytes_ = 0;
                if (eject_cb_) eject_cb_();
            }
            break;
        case 0x04:  // FORMAT UNIT
            if (read_only_) return check_condition(kSenseDataProtect, kAscWriteProtected);
            break;
        case 0x03: {  // REQUEST SENSE
            std::vector<u8> s(18, 0);
            s[0] = 0x70;
            s[2] = sense_key_;
            s[7] = 10;
            s[12] = sense_asc_;
            s[13] = sense_ascq_;
            s.resize(std::min<std::size_t>(s.size(), cdb[4] ? cdb[4] : 4));
            r.data_in = std::move(s);
            sense_key_ = kSenseNone;
            sense_asc_ = sense_ascq_ = 0;
            return r;
        }
        case 0x12:  // INQUIRY
            if ((cdb[1] >> 5) != 0) {  // no such LUN: peripheral qualifier 3
                r.data_in = inquiry(cdb[4]);
                if (!r.data_in.empty()) r.data_in[0] = 0x7F;
                return r;
            }
            r.data_in = inquiry(cdb[4]);
            return r;
        case 0x1A:  // MODE SENSE(6)
        case 0x5A:  // MODE SENSE(10)
            r.data_in = mode_sense(cdb, op == 0x5A);
            if (r.data_in.empty() && (op == 0x5A ? be16(&cdb[7]) : cdb[4]))
                return check_condition(kSenseIllegalRequest, kAscInvalidCdbField);  // no such page
            return r;
        case 0x15:  // MODE SELECT(6)
            r.data_out_length = cdb[4];
            return r;
        case 0x55:  // MODE SELECT(10)
            r.data_out_length = be16(&cdb[7]);
            return r;
        case 0x25: {  // READ CAPACITY(10)
            r.data_in.assign(8, 0);
            put32(&r.data_in[0], static_cast<u32>(std::min<u64>(blocks() ? blocks() - 1 : 0, 0xFFFFFFFF)));
            put32(&r.data_in[4], block_size_);
            return r;
        }
        case 0x08:    // READ(6)
        case 0x28: {  // READ(10)
            u64 lba;
            u32 count;
            if (op == 0x08) {
                lba = ((cdb[1] & 0x1F) << 16) | (cdb[2] << 8) | cdb[3];
                count = cdb[4] ? cdb[4] : 256;
            } else {
                lba = be32(&cdb[2]);
                count = be16(&cdb[7]);
            }
            if (lba + count > blocks()) return check_condition(kSenseIllegalRequest, kAscLbaOutOfRange);
            if (!read_blocks(lba, count, r.data_in)) return check_condition(kSenseMediumError, kAscUnrecoveredRead);
            return r;
        }
        case 0x0A:    // WRITE(6)
        case 0x2A: {  // WRITE(10)
            if (read_only_) return check_condition(kSenseDataProtect, kAscWriteProtected);
            const u32 count = op == 0x0A ? (cdb[4] ? cdb[4] : 256u) : be16(&cdb[7]);
            const u64 lba = op == 0x0A ? (((cdb[1] & 0x1F) << 16) | (cdb[2] << 8) | cdb[3]) : be32(&cdb[2]);
            if (lba + count > blocks()) return check_condition(kSenseIllegalRequest, kAscLbaOutOfRange);
            r.data_out_length = count * block_size_;
            return r;
        }
        case 0x43: {  // READ TOC: one data track, then the lead-out
            if (kind_ != Kind::CdRom) return check_condition(kSenseIllegalRequest, kAscInvalidOpcode);
            const bool msf = cdb[1] & 0x02;
            const unsigned format = cdb[9] >> 6, start = cdb[6];
            // An address as a block number, or as minutes, seconds and frames
            // (75 a second, two seconds of lead-in before block 0).
            auto address = [&](u8* at, u64 block) {
                if (!msf) {
                    put32(at, static_cast<u32>(block));
                    return;
                }
                const u64 frame = block * block_size_ / kCdBlockSize + 150;
                at[0] = 0;
                at[1] = static_cast<u8>(frame / (60 * 75));
                at[2] = static_cast<u8>(frame / 75 % 60);
                at[3] = static_cast<u8>(frame % 75);
            };
            std::vector<u8> t = {0, 0, 1, 1};  // length, first and last track (or session)
            if (format == 1) {  // the first session starts with track 1
                t.insert(t.end(), {0, 0x14, 1, 0, 0, 0, 0, 0});
                address(&t[8], 0);
            } else if (format == 0) {
                if (start > 1 && start != 0xAA) return check_condition(kSenseIllegalRequest, kAscInvalidCdbField);
                if (start <= 1) {
                    t.insert(t.end(), {0, 0x14, 1, 0, 0, 0, 0, 0});
                    address(&t[t.size() - 4], 0);
                }
                t.insert(t.end(), {0, 0x14, 0xAA, 0, 0, 0, 0, 0});
                address(&t[t.size() - 4], blocks());
            } else {
                return check_condition(kSenseIllegalRequest, kAscInvalidCdbField);
            }
            t[1] = static_cast<u8>(t.size() - 2);
            t.resize(std::min<std::size_t>(t.size(), be16(&cdb[7])));
            r.data_in = std::move(t);
            return r;
        }
        default:
            SG_LOG(Debug, "scsi", "unsupported command %02x", op);
            return check_condition(kSenseIllegalRequest, kAscInvalidOpcode);
    }
    return r;
}

u8 ScsiDisk::finish_data_out(const u8* cdb, const std::vector<u8>& data) {
    const u8 op = cdb[0];
    if (op == 0x0A || op == 0x2A) {
        const u64 lba = op == 0x0A ? (((cdb[1] & 0x1F) << 16) | (cdb[2] << 8) | cdb[3]) : be32(&cdb[2]);
        ++activity_;
        file_.clear();
        file_.seekp(static_cast<std::streamoff>(lba * block_size_));
        file_.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
        file_.flush();
        if (!file_) {
            check_condition(kSenseMediumError, 0x0C);  // write error
            return kCheckCondition;
        }
    }
    if (op == 0x15 || op == 0x55) {
        // The block descriptor's length selects the block size (0 keeps it):
        // 512 or 2048 on the CD-ROM, 512 on the disk. Pages are accepted
        // and ignored.
        const std::size_t header = op == 0x15 ? 4 : 8;
        const std::size_t descriptors = data.size() < header ? 0 : op == 0x15 ? data[3] : be16(&data[6]);
        if (descriptors >= 8 && data.size() >= header + 8) {
            const u32 size = (u32{data[header + 5]} << 16) | (u32{data[header + 6]} << 8) | data[header + 7];
            const bool ok = size == 0 || size == kDiskBlockSize || (kind_ == Kind::CdRom && size == kCdBlockSize);
            if (!ok) {
                check_condition(kSenseIllegalRequest, kAscInvalidParameter);
                return kCheckCondition;
            }
            if (size) block_size_ = size;
        }
    }
    return kGood;
}

}  // namespace sg
