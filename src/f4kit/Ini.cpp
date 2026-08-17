// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "f4kit/Ini.h"

#include "f4kit/Log.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <cwchar>
#include <cstdlib>

namespace f4kit::ini {

namespace {
wchar_t s_path[MAX_PATH] = {};
bool s_warned = false;

// Reading with no file set returns every fallback, which looks exactly like a file full of
// default values. Say so once, or a whole session's settings are silently ignored.
void CheckFile()
{
    if (s_path[0] || s_warned)
        return;
    s_warned = true;
    log::Write("config: no ini file was set - every setting is its built-in default");
}
}

void SetFile(const wchar_t* path)
{
    if (!path)
        return;
    wcscpy_s(s_path, path);

    const DWORD attr = GetFileAttributesW(s_path);
    log::Write("config: reading %ls%s", s_path,
               attr == INVALID_FILE_ATTRIBUTES ? " (absent - using defaults)" : "");
}

bool HasFile()
{
    return s_path[0] != 0;
}

int ReadInt(const wchar_t* section, const wchar_t* key, int fallback)
{
    CheckFile();
    return int(GetPrivateProfileIntW(section, key, fallback, s_path));
}

bool ReadBool(const wchar_t* section, const wchar_t* key, bool fallback)
{
    return ReadInt(section, key, fallback ? 1 : 0) != 0;
}

int ReadEnum(const wchar_t* section, const wchar_t* key, const wchar_t* const* words, int count,
             int fallback)
{
    CheckFile();
    if (!words || count <= 0)
        return fallback;

    wchar_t buf[64] = {};
    GetPrivateProfileStringW(section, key, L"\x01", buf, 64, s_path);
    if (buf[0] == L'\x01' || !buf[0])
        return fallback;

    // Trim, since "gpu " and "gpu" are the same intent and an ini file collects trailing
    // spaces the moment anyone edits it by hand.
    wchar_t* begin = buf;
    while (*begin == L' ' || *begin == L'\t')
        ++begin;
    wchar_t* end = begin + wcslen(begin);
    while (end > begin && (end[-1] == L' ' || end[-1] == L'\t'))
        --end;
    *end = 0;

    for (int i = 0; i < count; ++i)
        if (words[i] && _wcsicmp(begin, words[i]) == 0)
            return i;

    log::Write("config: [%ls] %ls=\"%ls\" is not one of the accepted values - using \"%ls\"",
               section, key, begin,
               (fallback >= 0 && fallback < count && words[fallback]) ? words[fallback] : L"?");
    return fallback;
}

float ReadFloat(const wchar_t* section, const wchar_t* key, float fallback)
{
    CheckFile();

    wchar_t buf[64] = {};
    // A sentinel default distinguishes "absent" from "present but unparseable", so a typo in
    // the file falls back to the default instead of silently becoming zero.
    GetPrivateProfileStringW(section, key, L"\x01", buf, 64, s_path);
    if (buf[0] == L'\x01' || !buf[0])
        return fallback;

    wchar_t* end = nullptr;
    const float v = wcstof(buf, &end);
    if (end == buf) {
        log::Write("config: [%ls] %ls=\"%ls\" is not a number - using %.3f", section, key, buf,
                   fallback);
        return fallback;
    }
    return v;
}

float Clamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

int ClampI(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

}
