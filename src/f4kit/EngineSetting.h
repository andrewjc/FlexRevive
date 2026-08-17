// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

// Locates the engine's static Setting objects by name, so the plugin can read and write
// them at runtime.
//
// Setting layout: vptr @0x00, data @0x08, name @0x10. An object is found by locating its
// name string, then finding the static object whose name pointer references it; a candidate
// is accepted only if its vptr lands in .rdata.
namespace f4kit::engine {

union Data {
    uint32_t u32;
    int32_t s32;
    float f32;
    uint8_t u8; // bool
    char* s;
};

struct Binding {
    const char* fullName; // e.g. "bNVFlexEnable:NVFlex"
    Data* data;           // filled in by Resolve; null if not found
    // How many objects in the image carry this name. More than one means the image holds
    // duplicates and the first was taken, which may not be the one the engine reads.
    int candidates;
};

// Resolves every binding in one pass over the image. Returns how many were found.
int Resolve(Binding* bindings, int count);

// Reads `key` under `section` from one of the game's own ini files, returning `fallback` when
// the file or the key is absent.
//
// This, rather than the live Setting object, is what says how the game is configured while a
// plugin is loading. F4SE loads plugins before the game has parsed its ini files, so at that
// point every Setting still holds its compiled-in default: reading one says what the game was
// built with, and writing one is undone by the parse that follows. The file is what the game
// is about to read, and the only account of the user's intent that exists yet.
int ReadGameIni(const wchar_t* gameFolder, const wchar_t* fileName, const wchar_t* section,
                const wchar_t* key, int fallback);

// Writes `key=value` under `section` in one of the game's own ini files, by name, under
// Documents\My Games\<game>. Returns whether the file now holds that value.
//
// Some of the engine's settings are read once during startup and never consulted again, so
// changing the live Setting object has no effect on anything that already ran. The only way to
// alter those is to change what the game will read next time it starts.
//
// `gameFolder` is the folder under My Games, `fileName` the ini within it.
bool WriteGameIni(const wchar_t* gameFolder, const wchar_t* fileName, const wchar_t* section,
                  const wchar_t* key, int value);

bool GetBool(const Binding& b, bool& out);
bool SetBool(Binding& b, bool value);
bool GetInt(const Binding& b, int32_t& out);
bool SetInt(Binding& b, int32_t value);
bool GetFloat(const Binding& b, float& out);
bool SetFloat(Binding& b, float value);

}
