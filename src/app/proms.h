// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// Boot PROM images: what each one is (from its CRC-32 and size, else from
// the version string inside it) and, from a folder of them, the one a
// machine configuration needs.
#pragma once

#include <string>
#include <vector>

#include "app/settings.h"

namespace sg::app {

struct PromImage {
    std::string path;
    std::string system;   // "ip24" (Indy), "ip22" (Indigo2), "ip20", "ip12", "ip26", "ip28"; empty: not a PROM we know
    std::string part;     // SGI part number, when the image is a known dump
    std::string version;  // the version string inside it, e.g. "5.3 Rev B10"
    int rank = 0;         // newer releases rank higher
    bool r4600 = false, r5000 = false;  // Indy: the processors beyond the R4000/R4400 it supports
    bool byte_swapped = false;          // dumped with its bytes swapped (can't be used as is)
};

// Identifies one image file; `system` is empty for anything else.
PromImage identify_prom(const std::string& path);
// Every file in `folder` that is a PROM image.
std::vector<PromImage> scan_proms(const std::string& folder);
// Whether `p` can start the configured machine; if not, why not.
bool prom_fits(const PromImage& p, const Settings& s, std::string& why);
// The folder searched for PROMs: the setting, else "roms" in the current
// folder if there is one, else the user's SiliconGhost roms folder.
std::string prom_folder(const Settings& s);
// The PROM for the configuration: the chosen file (`s.prom`) if any, else
// the newest suitable one in the PROM folder. False, with a message saying
// what is needed and where it was looked for, if there is none.
bool select_prom(const Settings& s, PromImage& out, std::string& error);

}  // namespace sg::app
