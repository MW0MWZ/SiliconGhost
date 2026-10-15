// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// MIPS III disassembler (integer, COP0 and COP1 moves) for tracing and debugging.
#pragma once

#include <string>

#include "core/types.h"

namespace sg::mips {

std::string disassemble(u32 inst, u64 pc);
const char* gpr_name(unsigned reg);
const char* cp0_name(unsigned reg);

}  // namespace sg::mips
