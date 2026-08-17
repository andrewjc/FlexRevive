// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "f4kit/EngineSetting.h"
#include "f4kit/Log.h"
#include "f4kit/PeImage.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <KnownFolders.h>
#include <ShlObj.h>

#include <cstdio>
#include <cstring>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace f4kit::engine {

namespace {

// Name strings are suffix-packed: one .rdata literal can serve several settings, each
// pointing partway into it. A match is therefore any position where the name begins and the
// string terminates, not only the start of a literal.
void FindNameAddresses(const char* name, std::vector<uintptr_t>& out)
{
    const size_t len = strlen(name);
    for (const pe::Section& sec : pe::Sections()) {
        if (sec.executable)
            continue;
        const uint8_t* p = sec.begin;
        const uint8_t* end = sec.begin + sec.size;
        if (size_t(end - p) < len + 1)
            continue;
        const uint8_t* last = end - (len + 1);
        for (; p <= last; ++p) {
            if (*p != uint8_t(name[0]))
                continue;
            if (memcmp(p, name, len) != 0 || p[len] != 0)
                continue;
            out.push_back(reinterpret_cast<uintptr_t>(p));
        }
    }
}

} // namespace

int Resolve(Binding* bindings, int count)
{
    if (!pe::Init())
        return 0;

    // Pass 1: locate each name string. A name may appear more than once.
    std::unordered_map<uintptr_t, int> addrToBinding;
    for (int i = 0; i < count; ++i) {
        bindings[i].data = nullptr;
        std::vector<uintptr_t> addrs;
        FindNameAddresses(bindings[i].fullName, addrs);
        for (uintptr_t a : addrs)
            addrToBinding.emplace(a, i);
    }
    if (addrToBinding.empty()) {
        log::Write("settings: none of the %d requested names exist in this executable", count);
        return 0;
    }

    // Pass 2: any 8-aligned qword equal to one of those addresses is a candidate name
    // pointer. The Setting object starts 0x10 earlier; require its vptr to land in .rdata.
    int found = 0;
    for (const pe::Section& sec : pe::Sections()) {
        if (sec.executable)
            continue;
        auto secBase = reinterpret_cast<uintptr_t>(sec.begin);
        uintptr_t aligned = (secBase + 7) & ~uintptr_t(7);
        auto p = reinterpret_cast<const uintptr_t*>(aligned);
        auto end = reinterpret_cast<const uintptr_t*>((secBase + sec.size) & ~uintptr_t(7));
        for (; p < end; ++p) {
            auto it = addrToBinding.find(*p);
            if (it == addrToBinding.end())
                continue;

            auto namePtrAddr = reinterpret_cast<uintptr_t>(p);
            if (namePtrAddr < secBase + 0x10)
                continue;
            uintptr_t settingBase = namePtrAddr - 0x10;
            uintptr_t vtbl = *reinterpret_cast<const uintptr_t*>(settingBase);
            if (!pe::PointsIntoReadOnlyData(vtbl))
                continue;

            Binding& b = bindings[it->second];
            ++b.candidates;
            if (!b.data) {
                b.data = reinterpret_cast<Data*>(settingBase + 0x08);
                ++found;
            }
        }
    }

    for (int i = 0; i < count; ++i)
        log::Write("  setting %s: %d candidate object(s)%s", bindings[i].fullName,
                   bindings[i].candidates, bindings[i].candidates > 1
                       ? "  <-- more than one, the first was taken" : "");
    log::Write("settings: resolved %d of %d engine settings", found, count);
    return found;
}

namespace {

// Documents\My Games\<game>\<file>, or false when the Documents folder cannot be found.
bool GameIniPath(const wchar_t* gameFolder, const wchar_t* fileName, wchar_t (&out)[MAX_PATH])
{
    PWSTR docs = nullptr;
    if (FAILED(SHGetKnownFolderPath(FOLDERID_Documents, 0, nullptr, &docs)))
        return false;
    swprintf_s(out, L"%s\\My Games\\%s\\%s", docs, gameFolder, fileName);
    CoTaskMemFree(docs);
    return true;
}

} // namespace

int ReadGameIni(const wchar_t* gameFolder, const wchar_t* fileName, const wchar_t* section,
                const wchar_t* key, int fallback)
{
    wchar_t path[MAX_PATH] = {};
    if (!gameFolder || !fileName || !section || !key ||
        !GameIniPath(gameFolder, fileName, path))
        return fallback;
    return int(GetPrivateProfileIntW(section, key, fallback, path));
}

bool WriteGameIni(const wchar_t* gameFolder, const wchar_t* fileName, const wchar_t* section,
                  const wchar_t* key, int value)
{
    wchar_t path[MAX_PATH] = {};
    if (!gameFolder || !fileName || !section || !key ||
        !GameIniPath(gameFolder, fileName, path))
        return false;

    // Only ever modify a file the game already keeps. Creating one would invent a settings
    // file in a folder that may belong to a different install than the one running.
    if (GetFileAttributesW(path) == INVALID_FILE_ATTRIBUTES) {
        log::Write("settings: %ls does not exist, so %ls cannot be written", path, key);
        return false;
    }

    wchar_t text[16] = {};
    swprintf_s(text, L"%d", value);
    if (!WritePrivateProfileStringW(section, key, text, path)) {
        log::Write("settings: could not write [%ls] %ls=%d to %ls (error %lu). The file may be "
                   "read-only, or owned by a mod manager.", section, key, value, path,
                   GetLastError());
        return false;
    }

    // Read back rather than trusting the write. A file redirected by a mod manager, or one the
    // game has open, can accept the call and keep the old contents.
    const int got = int(GetPrivateProfileIntW(section, key, ~value, path));
    if (got != value) {
        log::Write("settings: wrote [%ls] %ls=%d to %ls but it still reads %d", section, key,
                   value, path, got);
        return false;
    }

    log::Write("settings: wrote [%ls] %ls=%d to %ls", section, key, value, path);
    return true;
}

bool GetBool(const Binding& b, bool& out)
{
    if (!b.data)
        return false;
    out = b.data->u8 != 0;
    return true;
}

bool SetBool(Binding& b, bool value)
{
    if (!b.data)
        return false;

    // Setting objects live in writable data; this assumes nothing about it.
    DWORD oldProtect = 0;
    if (!VirtualProtect(b.data, sizeof(Data), PAGE_READWRITE, &oldProtect))
        return false;
    b.data->u8 = value ? 1 : 0;
    VirtualProtect(b.data, sizeof(Data), oldProtect, &oldProtect);
    return true;
}

bool GetInt(const Binding& b, int32_t& out)
{
    if (!b.data)
        return false;
    out = b.data->s32;
    return true;
}

bool SetInt(Binding& b, int32_t value)
{
    if (!b.data)
        return false;
    b.data->s32 = value;
    return true;
}

bool GetFloat(const Binding& b, float& out)
{
    if (!b.data)
        return false;
    out = b.data->f32;
    return true;
}

bool SetFloat(Binding& b, float value)
{
    if (!b.data)
        return false;
    b.data->f32 = value;
    return true;
}

}
