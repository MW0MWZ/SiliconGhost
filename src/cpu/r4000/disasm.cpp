// SPDX-License-Identifier: GPL-2.0-or-later
#include "cpu/r4000/disasm.h"

#include <cstdarg>
#include <cstdio>

namespace sg::mips {

namespace {

constexpr const char* kGpr[32] = {"zero", "at", "v0", "v1", "a0", "a1", "a2", "a3", "t0", "t1", "t2",
                                  "t3",   "t4", "t5", "t6", "t7", "s0", "s1", "s2", "s3", "s4", "s5",
                                  "s6",   "s7", "t8", "t9", "k0", "k1", "gp", "sp", "s8", "ra"};

constexpr const char* kCp0[32] = {"Index",    "Random",  "EntryLo0", "EntryLo1", "Context", "PageMask", "Wired",
                                  "$7",       "BadVAddr", "Count",   "EntryHi",  "Compare", "Status",   "Cause",
                                  "EPC",      "PRId",     "Config",  "LLAddr",   "WatchLo", "WatchHi",  "XContext",
                                  "$21",      "$22",      "$23",     "$24",      "$25",     "ECC",      "CacheErr",
                                  "TagLo",    "TagHi",    "ErrorEPC", "$31"};

std::string fmt(const char* f, ...) SG_PRINTF(1, 2);
std::string fmt(const char* f, ...) {
    char buf[128];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof(buf), f, ap);
    va_end(ap);
    return buf;
}

// Print addresses in the compact 32-bit form when they are sign-extended.
std::string addr(u64 a) {
    if (sext32(a) == a) return fmt("0x%08x", static_cast<u32>(a));
    return fmt("0x%016llx", static_cast<unsigned long long>(a));
}

std::string special(u32 i) {
    const char* rs = kGpr[(i >> 21) & 31];
    const char* rt = kGpr[(i >> 16) & 31];
    const char* rd = kGpr[(i >> 11) & 31];
    const unsigned sa = (i >> 6) & 31;
    switch (i & 63) {
        case 0x00: return i == 0 ? "nop" : fmt("sll %s, %s, %u", rd, rt, sa);
        case 0x02: return fmt("srl %s, %s, %u", rd, rt, sa);
        case 0x03: return fmt("sra %s, %s, %u", rd, rt, sa);
        case 0x04: return fmt("sllv %s, %s, %s", rd, rt, rs);
        case 0x06: return fmt("srlv %s, %s, %s", rd, rt, rs);
        case 0x07: return fmt("srav %s, %s, %s", rd, rt, rs);
        case 0x08: return fmt("jr %s", rs);
        case 0x09: return fmt("jalr %s, %s", rd, rs);
        case 0x0C: return "syscall";
        case 0x0D: return fmt("break 0x%x", (i >> 6) & 0xFFFFF);
        case 0x0F: return "sync";
        case 0x10: return fmt("mfhi %s", rd);
        case 0x11: return fmt("mthi %s", rs);
        case 0x12: return fmt("mflo %s", rd);
        case 0x13: return fmt("mtlo %s", rs);
        case 0x14: return fmt("dsllv %s, %s, %s", rd, rt, rs);
        case 0x16: return fmt("dsrlv %s, %s, %s", rd, rt, rs);
        case 0x17: return fmt("dsrav %s, %s, %s", rd, rt, rs);
        case 0x18: return fmt("mult %s, %s", rs, rt);
        case 0x19: return fmt("multu %s, %s", rs, rt);
        case 0x1A: return fmt("div %s, %s", rs, rt);
        case 0x1B: return fmt("divu %s, %s", rs, rt);
        case 0x1C: return fmt("dmult %s, %s", rs, rt);
        case 0x1D: return fmt("dmultu %s, %s", rs, rt);
        case 0x1E: return fmt("ddiv %s, %s", rs, rt);
        case 0x1F: return fmt("ddivu %s, %s", rs, rt);
        case 0x20: return fmt("add %s, %s, %s", rd, rs, rt);
        case 0x21: return ((i >> 16) & 31) == 0 ? fmt("move %s, %s", rd, rs) : fmt("addu %s, %s, %s", rd, rs, rt);
        case 0x22: return fmt("sub %s, %s, %s", rd, rs, rt);
        case 0x23: return fmt("subu %s, %s, %s", rd, rs, rt);
        case 0x24: return fmt("and %s, %s, %s", rd, rs, rt);
        case 0x25: return ((i >> 16) & 31) == 0 ? fmt("move %s, %s", rd, rs) : fmt("or %s, %s, %s", rd, rs, rt);
        case 0x26: return fmt("xor %s, %s, %s", rd, rs, rt);
        case 0x27: return fmt("nor %s, %s, %s", rd, rs, rt);
        case 0x2A: return fmt("slt %s, %s, %s", rd, rs, rt);
        case 0x2B: return fmt("sltu %s, %s, %s", rd, rs, rt);
        case 0x2C: return fmt("dadd %s, %s, %s", rd, rs, rt);
        case 0x2D: return fmt("daddu %s, %s, %s", rd, rs, rt);
        case 0x2E: return fmt("dsub %s, %s, %s", rd, rs, rt);
        case 0x2F: return fmt("dsubu %s, %s, %s", rd, rs, rt);
        case 0x30: return fmt("tge %s, %s", rs, rt);
        case 0x31: return fmt("tgeu %s, %s", rs, rt);
        case 0x32: return fmt("tlt %s, %s", rs, rt);
        case 0x33: return fmt("tltu %s, %s", rs, rt);
        case 0x34: return fmt("teq %s, %s", rs, rt);
        case 0x36: return fmt("tne %s, %s", rs, rt);
        case 0x38: return fmt("dsll %s, %s, %u", rd, rt, sa);
        case 0x3A: return fmt("dsrl %s, %s, %u", rd, rt, sa);
        case 0x3B: return fmt("dsra %s, %s, %u", rd, rt, sa);
        case 0x3C: return fmt("dsll32 %s, %s, %u", rd, rt, sa);
        case 0x3E: return fmt("dsrl32 %s, %s, %u", rd, rt, sa);
        case 0x3F: return fmt("dsra32 %s, %s, %u", rd, rt, sa);
    }
    return fmt(".word 0x%08x", i);
}

std::string regimm(u32 i, u64 pc) {
    const char* rs = kGpr[(i >> 21) & 31];
    const s16 imm = static_cast<s16>(i & 0xFFFF);
    const std::string target = addr(pc + 4 + (sext16(i & 0xFFFF) << 2));
    switch ((i >> 16) & 31) {
        case 0x00: return fmt("bltz %s, %s", rs, target.c_str());
        case 0x01: return fmt("bgez %s, %s", rs, target.c_str());
        case 0x02: return fmt("bltzl %s, %s", rs, target.c_str());
        case 0x03: return fmt("bgezl %s, %s", rs, target.c_str());
        case 0x08: return fmt("tgei %s, %d", rs, imm);
        case 0x09: return fmt("tgeiu %s, %d", rs, imm);
        case 0x0A: return fmt("tlti %s, %d", rs, imm);
        case 0x0B: return fmt("tltiu %s, %d", rs, imm);
        case 0x0C: return fmt("teqi %s, %d", rs, imm);
        case 0x0E: return fmt("tnei %s, %d", rs, imm);
        case 0x10: return fmt("bltzal %s, %s", rs, target.c_str());
        case 0x11: return ((i >> 21) & 31) == 0 ? fmt("bal %s", target.c_str()) : fmt("bgezal %s, %s", rs, target.c_str());
        case 0x12: return fmt("bltzall %s, %s", rs, target.c_str());
        case 0x13: return fmt("bgezall %s, %s", rs, target.c_str());
    }
    return fmt(".word 0x%08x", i);
}

std::string cop(u32 i, unsigned z) {
    const char* rt = kGpr[(i >> 16) & 31];
    const unsigned rd = (i >> 11) & 31;
    if (z == 0) {
        switch ((i >> 21) & 31) {
            case 0x00: return fmt("mfc0 %s, %s", rt, kCp0[rd]);
            case 0x01: return fmt("dmfc0 %s, %s", rt, kCp0[rd]);
            case 0x04: return fmt("mtc0 %s, %s", rt, kCp0[rd]);
            case 0x05: return fmt("dmtc0 %s, %s", rt, kCp0[rd]);
        }
        if (i & (1u << 25)) {
            switch (i & 63) {
                case 0x01: return "tlbr";
                case 0x02: return "tlbwi";
                case 0x06: return "tlbwr";
                case 0x08: return "tlbp";
                case 0x18: return "eret";
            }
        }
    } else if (z == 1) {
        switch ((i >> 21) & 31) {
            case 0x00: return fmt("mfc1 %s, $f%u", rt, rd);
            case 0x01: return fmt("dmfc1 %s, $f%u", rt, rd);
            case 0x02: return fmt("cfc1 %s, $%u", rt, rd);
            case 0x04: return fmt("mtc1 %s, $f%u", rt, rd);
            case 0x05: return fmt("dmtc1 %s, $f%u", rt, rd);
            case 0x06: return fmt("ctc1 %s, $%u", rt, rd);
        }
        return fmt("cop1 0x%07x", i & 0x1FFFFFF);
    }
    return fmt("cop%u 0x%07x", z, i & 0x1FFFFFF);
}

}  // namespace

const char* gpr_name(unsigned reg) { return kGpr[reg & 31]; }
const char* cp0_name(unsigned reg) { return kCp0[reg & 31]; }

std::string disassemble(u32 i, u64 pc) {
    const unsigned op = i >> 26;
    const char* rs = kGpr[(i >> 21) & 31];
    const char* rt = kGpr[(i >> 16) & 31];
    const s16 simm = static_cast<s16>(i & 0xFFFF);
    const unsigned zimm = i & 0xFFFF;
    const std::string btarget = addr(pc + 4 + (sext16(zimm) << 2));
    const std::string jtarget = addr(((pc + 4) & ~0x0FFFFFFFull) | (u64{i & 0x03FFFFFF} << 2));

    static constexpr const char* kMem[64] = {
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,  // 0x00
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,  // 0x08
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,  // 0x10
        nullptr, nullptr, "ldl",   "ldr",   nullptr, nullptr, nullptr, nullptr,  // 0x18
        "lb",    "lh",    "lwl",   "lw",    "lbu",   "lhu",   "lwr",   "lwu",    // 0x20
        "sb",    "sh",    "swl",   "sw",    "sdl",   "sdr",   "swr",   nullptr,  // 0x28
        "ll",    nullptr, nullptr, nullptr, "lld",   nullptr, nullptr, "ld",     // 0x30
        "sc",    nullptr, nullptr, nullptr, "scd",   nullptr, nullptr, "sd",     // 0x38
    };
    if (kMem[op]) return fmt("%s %s, %d(%s)", kMem[op], rt, simm, rs);

    switch (op) {
        case 0x00: return special(i);
        case 0x01: return regimm(i, pc);
        case 0x02: return fmt("j %s", jtarget.c_str());
        case 0x03: return fmt("jal %s", jtarget.c_str());
        case 0x04:
            if (((i >> 16) & 0x3FF) == 0) return fmt("b %s", btarget.c_str());
            return fmt("beq %s, %s, %s", rs, rt, btarget.c_str());
        case 0x05: return fmt("bne %s, %s, %s", rs, rt, btarget.c_str());
        case 0x06: return fmt("blez %s, %s", rs, btarget.c_str());
        case 0x07: return fmt("bgtz %s, %s", rs, btarget.c_str());
        case 0x08: return fmt("addi %s, %s, %d", rt, rs, simm);
        case 0x09:
            if (((i >> 21) & 31) == 0) return fmt("li %s, %d", rt, simm);
            return fmt("addiu %s, %s, %d", rt, rs, simm);
        case 0x0A: return fmt("slti %s, %s, %d", rt, rs, simm);
        case 0x0B: return fmt("sltiu %s, %s, %d", rt, rs, simm);
        case 0x0C: return fmt("andi %s, %s, 0x%x", rt, rs, zimm);
        case 0x0D: return fmt("ori %s, %s, 0x%x", rt, rs, zimm);
        case 0x0E: return fmt("xori %s, %s, 0x%x", rt, rs, zimm);
        case 0x0F: return fmt("lui %s, 0x%x", rt, zimm);
        case 0x10:
        case 0x11:
        case 0x12:
        case 0x13: return cop(i, op & 3);
        case 0x14: return fmt("beql %s, %s, %s", rs, rt, btarget.c_str());
        case 0x15: return fmt("bnel %s, %s, %s", rs, rt, btarget.c_str());
        case 0x16: return fmt("blezl %s, %s", rs, btarget.c_str());
        case 0x17: return fmt("bgtzl %s, %s", rs, btarget.c_str());
        case 0x18: return fmt("daddi %s, %s, %d", rt, rs, simm);
        case 0x19: return fmt("daddiu %s, %s, %d", rt, rs, simm);
        case 0x2F: return fmt("cache 0x%x, %d(%s)", (i >> 16) & 31, simm, rs);
        case 0x31: return fmt("lwc1 $f%u, %d(%s)", (i >> 16) & 31, simm, rs);
        case 0x35: return fmt("ldc1 $f%u, %d(%s)", (i >> 16) & 31, simm, rs);
        case 0x39: return fmt("swc1 $f%u, %d(%s)", (i >> 16) & 31, simm, rs);
        case 0x3D: return fmt("sdc1 $f%u, %d(%s)", (i >> 16) & 31, simm, rs);
    }
    return fmt(".word 0x%08x", i);
}

}  // namespace sg::mips
