// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "app/proms.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace sg::app {

namespace {

// Known dumps. Part numbers and what each release added follow the PROM
// list on the Higher Intellect Vintage Wiki; CRCs are of the images.
struct KnownProm {
    u32 crc;
    u32 size;
    const char* system;
    const char* part;
    const char* version;
    int rank;
    bool r4600, r5000, byte_swapped;
};
constexpr KnownProm kKnown[] = {
    {0xBB2F32ABu, 262144, "ip12", "", "4.0.1 Rev C", 1, false, false, false},
    {0xEA4329EFu, 262144, "ip12", "070-8088-002", "4.0.1 Rev D", 2, false, false, true},
    {0xBFE2AA8Bu, 524288, "ip20", "070-8116-004", "4.0.5D Rev A", 1, false, false, false},
    {0x695355EFu, 524288, "ip20", "070-8116-005", "4.0.5G Rev B", 2, false, false, false},
    {0xAE5ECD08u, 524288, "ip22", "070-1367-002", "5.1 Rev B", 1, false, false, false},
    {0xF5E41008u, 524288, "ip24", "070-9101-005", "5.1.2 Rev B4", 1, false, false, false},
    {0x70D8D1B1u, 524288, "ip24", "070-9101-007", "5.0 Rev B6", 2, true, false, false},
    {0xEE0B55C4u, 524288, "ip24", "070-9101-008", "5.3 Rev B7", 3, true, false, false},
    {0x5E7F7E3Au, 524288, "ip24", "070-9101-011", "5.3 Rev B10", 4, true, true, false},
    {0x9CED01BEu, 524288, "ip26", "070-1371-003", "6.0.1 Rev A", 1, false, false, false},
    {0x58DD1DC2u, 524288, "ip26", "070-1371-004", "6.0.1 Rev A", 2, false, false, false},
    {0x72749952u, 524288, "ip26", "070-1371-007", "6.0.1 Rev B", 3, false, false, false},
    {0x78F06ACFu, 524288, "ip28", "070-1477-001", "6.2 Rev A", 1, false, false, false},
    {0xE55BBB60u, 524288, "ip28", "070-1477-002", "6.2 Rev A", 2, false, false, false},
};

u32 crc32(const std::vector<u8>& data) {
    static const std::array<u32, 256> table = [] {
        std::array<u32, 256> t{};
        for (u32 i = 0; i < 256; ++i) {
            u32 c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            t[i] = c;
        }
        return t;
    }();
    u32 c = 0xFFFFFFFFu;
    for (u8 b : data) c = table[(c ^ b) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

// "SGI Version 5.3 Rev B10 R4X00/R5000 IP24 Feb 12, 1996": the version and the system.
bool version_string(const std::vector<u8>& data, std::string& version, std::string& system, bool& r5000) {
    static const std::string kTag = "SGI Version ";
    auto it = std::search(data.begin(), data.end(), kTag.begin(), kTag.end());
    if (it == data.end()) return false;
    std::string text;
    for (auto p = it + static_cast<std::ptrdiff_t>(kTag.size()); p != data.end() && *p >= 0x20 && *p < 0x7F; ++p)
        text.push_back(static_cast<char>(*p));
    const auto ip = text.find(" IP");
    if (ip == std::string::npos || ip + 5 > text.size()) return false;
    system = "ip" + text.substr(ip + 3, 2);
    version = text.substr(0, text.find(" Rev ") != std::string::npos ? text.find(' ', text.find(" Rev ") + 5) : ip);
    r5000 = text.find("R5000") != std::string::npos;
    return true;
}

// Release order for an unknown dump, from its version number ("5.3" -> 53).
int rank_of(const std::string& version) {
    int major = 0, minor = 0;
    std::sscanf(version.c_str(), "%d.%d", &major, &minor);
    return major * 10 + minor;
}

const char* system_name(const std::string& system) {
    if (system == "ip24") return "Indy (IP24)";
    if (system == "ip22") return "Indigo2 (IP22)";
    if (system == "ip20") return "Indigo R4000 (IP20)";
    if (system == "ip12") return "Indigo R3000 (IP12)";
    if (system == "ip26") return "Indigo2 R8000 (IP26)";
    if (system == "ip28") return "Indigo2 R10000 (IP28)";
    return "unknown";
}

}  // namespace

PromImage identify_prom(const std::string& path) {
    PromImage p;
    p.path = path;
    std::ifstream f(path, std::ios::binary);
    if (!f) return p;
    std::vector<u8> data((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (data.empty() || data.size() > (1u << 20)) return p;
    const u32 crc = crc32(data);
    for (const KnownProm& k : kKnown) {
        if (k.crc != crc || k.size != data.size()) continue;
        p.system = k.system, p.part = k.part, p.version = k.version;
        p.rank = 100 + k.rank;  // a known dump ranks above an unknown one of its release
        p.r4600 = k.r4600, p.r5000 = k.r5000, p.byte_swapped = k.byte_swapped;
        return p;
    }
    // Not a dump we know: go by the version string inside it.
    bool r5000 = false;
    if (version_string(data, p.version, p.system, r5000)) {
        p.rank = rank_of(p.version);
        p.r5000 = r5000;
        p.r4600 = r5000 || p.rank >= 50;  // R4600 support arrived with 5.0 Rev B6
    }
    return p;
}

std::vector<PromImage> scan_proms(const std::string& folder) {
    std::vector<PromImage> found;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(folder, ec)) {
        if (!entry.is_regular_file(ec)) continue;
        PromImage p = identify_prom(entry.path().string());
        if (!p.system.empty()) found.push_back(std::move(p));
    }
    std::sort(found.begin(), found.end(), [](const PromImage& a, const PromImage& b) { return a.path < b.path; });
    return found;
}

bool prom_fits(const PromImage& p, const Settings& s, std::string& why) {
    const ip22::Board board = board_of(s);
    const std::string needed = board == ip22::Board::Indigo2 ? "ip22" : "ip24";
    const std::string name = std::filesystem::path(p.path).filename().string();
    if (p.system.empty()) {
        why = name + " is not an SGI PROM image this emulator knows";
        return false;
    }
    if (p.system != needed) {
        why = name + " is the " + system_name(p.system) + " PROM; the " + system_name(needed) + " needs its own";
        return false;
    }
    if (p.byte_swapped) {
        why = name + " was dumped with its bytes swapped";
        return false;
    }
    const CpuModel* cpu = find_cpu_model(s.cpu);
    if (cpu && cpu->core == mips::Core::R5000 && !p.r5000) {
        why = name + " (" + p.version + ") predates the R5000: it needs 5.3 Rev B10 (070-9101-011) or later";
        return false;
    }
    if (cpu && cpu->core == mips::Core::R4600 && !p.r4600) {
        why = name + " (" + p.version + ") predates the R4600: it needs 5.0 Rev B6 (070-9101-007) or later";
        return false;
    }
    return true;
}

std::string prom_folder(const Settings& s) {
    if (!s.prom_dir.empty()) return s.prom_dir;
    std::error_code ec;
    if (std::filesystem::is_directory("roms", ec)) return "roms";
    const std::string base = user_data_folder();
    return base.empty() ? std::string("roms") : (std::filesystem::path(base) / "roms").string();
}

bool select_prom(const Settings& s, PromImage& out, std::string& error) {
    std::string why;
    if (!s.prom.empty()) {  // a file chosen by hand: it must suit the machine
        out = identify_prom(s.prom);
        if (!std::ifstream(s.prom)) {
            error = "cannot open the boot PROM " + s.prom;
            return false;
        }
        if (!prom_fits(out, s, why)) {
            error = why;
            return false;
        }
        return true;
    }
    const std::string folder = prom_folder(s);
    const PromImage* best = nullptr;
    std::vector<std::string> unsuitable;
    const std::vector<PromImage> found = scan_proms(folder);
    for (const PromImage& p : found) {
        if (!prom_fits(p, s, why)) {
            if (p.system == (board_of(s) == ip22::Board::Indigo2 ? "ip22" : "ip24")) unsuitable.push_back(why);
            continue;
        }
        if (!best || p.rank > best->rank) best = &p;
    }
    if (best) {
        out = *best;
        return true;
    }
    const bool indigo2 = board_of(s) == ip22::Board::Indigo2;
    error = std::string("no ") + (indigo2 ? "Indigo2 (IP22)" : "Indy (IP24)") + " boot PROM for this configuration in " +
            folder + ": put a dump there, such as " +
            (indigo2 ? "ip22prom.070-1367-002.bin" : "ip24prom.070-9101-011.bin");
    for (const std::string& u : unsuitable) error += "\n  (" + u + ")";
    return false;
}

}  // namespace sg::app
