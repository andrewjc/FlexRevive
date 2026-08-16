// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

namespace f4kit::log {

// Opens a log file under the user's Documents folder, creating the directory if it is absent.
// `subPath` is relative to Documents, `fileName` names the file. Deny-write sharing, so the
// log can be tailed while the host runs.
bool Open(const wchar_t* subPath, const wchar_t* fileName);

// Always written.
void Write(const char* fmt, ...);

// Written only when Verbose is set in the INI. Per-call solver tracing, too noisy for
// normal play.
void Trace(const char* fmt, ...);

void SetVerbose(bool on);
bool Verbose();

const wchar_t* Path();

}
