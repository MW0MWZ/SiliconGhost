// SPDX-License-Identifier: GPL-2.0-or-later
// SCSI targets: the interface the WD33C93 talks to, and a direct-access disk /
// CD-ROM backed by an image file of 512-byte blocks.
//
// IRIX media (CDs included) carry an SGI volume header and EFS/XFS file
// systems, and SGI's CD-ROM drives ran with 512-byte blocks, so a CD image
// is simply the raw block stream of the disc.
#pragma once

#include <fstream>
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
};

class ScsiDisk final : public ScsiTarget {
public:
    enum class Kind { Disk, CdRom };

    ScsiDisk(Kind kind, const std::string& path);
    bool ok() const { return file_.is_open(); }
    u64 blocks() const { return blocks_; }

    Result command(const u8* cdb, unsigned length) override;
    u8 finish_data_out(const u8* cdb, const std::vector<u8>& data) override;

private:
    static constexpr u32 kBlockSize = 512;

    Result check_condition(u8 key, u8 asc, u8 ascq = 0);
    bool read_blocks(u64 lba, u32 count, std::vector<u8>& out);
    std::vector<u8> inquiry(unsigned allocation) const;
    std::vector<u8> mode_sense(const u8* cdb, bool ten_byte) const;

    Kind kind_;
    std::fstream file_;
    u64 blocks_ = 0;
    bool read_only_;
    u8 sense_key_ = 0, sense_asc_ = 0, sense_ascq_ = 0;
};

}  // namespace sg
