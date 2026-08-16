// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

// Reading a plain Windows INI file, and clamping what comes back.
//
// A plugin's settings file is the one part of it a user edits by hand, so every value read is
// bounded here rather than trusted: a typo yields the default rather than nonsense behaviour.
namespace f4kit::ini {

// Points every later read at this file. Absolute path.
// Points every subsequent read at this file. Until it is called, reads return their fallbacks.
void SetFile(const wchar_t* path);

// Whether a file has been set. False means every read is answering with its built-in default.
bool HasFile();

int ReadInt(const wchar_t* section, const wchar_t* key, int fallback);
bool ReadBool(const wchar_t* section, const wchar_t* key, bool fallback);
float ReadFloat(const wchar_t* section, const wchar_t* key, float fallback);

float Clamp(float v, float lo, float hi);
int ClampI(int v, int lo, int hi);

}
