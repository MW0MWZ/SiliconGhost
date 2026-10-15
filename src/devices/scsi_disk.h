// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// SCSI targets: the interface the WD33C93 talks to, and a direct-access disk /
// CD-ROM backed by an image file.
//
// IRIX media (CDs included) carry an SGI volume header and EFS/XFS file
// systems, and SGI's CD-ROM drives ran with 512-byte blocks, so a CD image
// is simply the raw block stream of the disc. The CD-ROM can be switched to
// 2048-byte blocks with MODE SELECT. Like most CD-ROM drives it powers on,
// and returns after a bus reset, with 2048-byte blocks: the PROM and IRIX
// select 512 before use, other systems (OpenBSD, NetBSD) keep 2048.
#pragma once

#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include "core/types.h"

namespace sg {

class ScsiTarget {
public:
    virtual ~ScsiTarget() = default;

    struct Result {
        u8 status = 0;            // SCSI status byte (0 = GOOD, 2 = CHECK CONDITION)
        std::vector<u8> data_in;  // data the target returns
        u32 data_out_length = 0;  // bytes the target expects from the initiator
    };

    // Runs a command. If data_out_length is non-zero, the initiator transfers
    // that many bytes and calls finish_data_out() for the final status.
    virtual Result command(const u8* cdb, unsigned length) = 0;
    virtual u8 finish_data_out(const u8* cdb, const std::vector<u8>& data) = 0;
    // SCSI bus reset (or BUS DEVICE RESET): back to power-on settings.
    virtual void bus_reset() {}
};

class ScsiDisk final : public ScsiTarget {
public:
    enum class Kind { Disk, CdRom };

    ScsiDisk(Kind kind, const std::string& path);
    bool ok() const { return file_.is_open(); }
    u64 blocks() const { return bytes_ / block_size_; }
    u32 block_size() const { return block_size_; }
    // Power-on or bus reset: mode parameters back to their defaults, the
    // medium unlocked, and UNIT ATTENTION (power on or reset) reported next.
    void reset();
    void bus_reset() override { reset(); }

    // Removable media: puts another image in the drive. The next command
    // (other than INQUIRY and REQUEST SENSE) reports UNIT ATTENTION, medium
    // may have changed, as a real drive does. False if it cannot be opened
    // (the drive is then empty).
    bool change_medium(const std::string& path);
    // Called when the initiator ejects the medium (START STOP UNIT, LoEj).
    void set_eject_callback(std::function<void()> cb) { eject_cb_ = std::move(cb); }
    // The initiator has locked the medium in (PREVENT MEDIUM REMOVAL).
    bool removal_prevented() const { return prevent_removal_; }
    // Ejects at once, as the drive's own button does; the callback is not called.
    void eject();
    bool loaded() const { return file_.is_open(); }
    // Counts reads and writes of the medium (for activity lights).
    u64 activity() const { return activity_; }

    Result command(const u8* cdb, unsigned length) override;
    u8 finish_data_out(const u8* cdb, const std::vector<u8>& data) override;

private:
    static constexpr u32 kDiskBlockSize = 512, kCdBlockSize = 2048;
    u32 default_block_size() const { return kind_ == Kind::CdRom ? kCdBlockSize : kDiskBlockSize; }

    Result check_condition(u8 key, u8 asc, u8 ascq = 0);
    bool read_blocks(u64 lba, u32 count, std::vector<u8>& out);
    std::vector<u8> inquiry(unsigned allocation) const;
    std::vector<u8> mode_sense(const u8* cdb, bool ten_byte) const;

    bool open(const std::string& path);

    Kind kind_;
    std::fstream file_;
    bool unit_attention_ = false;
    bool prevent_removal_ = false;
    u64 activity_ = 0;
    std::function<void()> eject_cb_;
    u64 bytes_ = 0;  // the medium's size
    u32 block_size_ = kDiskBlockSize;
    u8 attention_asc_ = 0;  // UNIT ATTENTION reason: 0x28 medium changed, 0x29 reset
    bool read_only_;
    u8 sense_key_ = 0, sense_asc_ = 0, sense_ascq_ = 0;
};

}  // namespace sg
