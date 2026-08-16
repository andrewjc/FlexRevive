// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

// Crash reporting.
//
// On an unhandled exception, writes the faulting address and owning module, the registers and
// a symbolised stack into FlexRevive.log, and a minidump beside it.
//
// The handler chains: whatever filter was installed before, such as a dedicated crash logger,
// still runs afterwards on the same exception.
namespace f4kit::crash {

// Installs the handler, writing minidumps into `dumpDir` (with a trailing separator). Only
// the first call does anything.
void Install(const wchar_t* dumpDir);

}
