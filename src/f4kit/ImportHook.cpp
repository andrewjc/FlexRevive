// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "f4kit/ImportHook.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <cstdint>
#include <cstring>

namespace f4kit::imports {

bool Originals::Add(const char* name, void* fn)
{
    if (!name || !fn || m_count >= kMaxOriginals)
        return false;
    for (int i = 0; i < m_count; ++i)
        if (strcmp(m_entries[i].name, name) == 0)
            return false;
    m_entries[m_count].name = name;
    m_entries[m_count].fn = fn;
    ++m_count;
    return true;
}

void* Originals::For(const char* name) const
{
    if (!name)
        return nullptr;
    for (int i = 0; i < m_count; ++i)
        if (strcmp(m_entries[i].name, name) == 0)
            return m_entries[i].fn;
    return nullptr;
}

const Redirect* Find(const char* importName, const Redirect* table, int count,
                     const char* const* skip, int skipCount)
{
    if (!importName || !table || count <= 0)
        return nullptr;

    for (int i = 0; i < skipCount; ++i)
        if (skip && skip[i] && strcmp(skip[i], importName) == 0)
            return nullptr;

    for (int i = 0; i < count; ++i)
        if (table[i].name && strcmp(table[i].name, importName) == 0)
            return &table[i];
    return nullptr;
}

bool ModuleMatches(const char* moduleName, const char* prefix)
{
    if (!moduleName || !prefix || !*prefix)
        return false;
    return _strnicmp(moduleName, prefix, strlen(prefix)) == 0;
}

Report Patch(const char* prefix, const Redirect* table, int count, const char* const* skip,
             int skipCount, Originals& originals, Notice onLeftAlone, void* user)
{
    Report report;

    auto base = reinterpret_cast<uint8_t*>(GetModuleHandleW(nullptr));
    if (!base || !prefix || !table || count <= 0)
        return report;

    auto dos = reinterpret_cast<IMAGE_DOS_HEADER*>(base);
    auto nt = reinterpret_cast<IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
    const auto& dir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
    if (!dir.VirtualAddress)
        return report;

    for (auto desc = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR*>(base + dir.VirtualAddress);
         desc->Name; ++desc) {
        auto moduleName = reinterpret_cast<const char*>(base + desc->Name);
        if (!ModuleMatches(moduleName, prefix))
            continue;
        ++report.modules;

        auto thunk = reinterpret_cast<IMAGE_THUNK_DATA64*>(base + desc->FirstThunk);
        auto named = reinterpret_cast<IMAGE_THUNK_DATA64*>(
            base + (desc->OriginalFirstThunk ? desc->OriginalFirstThunk : desc->FirstThunk));

        for (; named->u1.AddressOfData; ++named, ++thunk) {
            if (named->u1.Ordinal & IMAGE_ORDINAL_FLAG64) {
                ++report.byOrdinal;
                if (onLeftAlone)
                    onLeftAlone(moduleName, nullptr, user);
                continue;
            }

            auto byName = reinterpret_cast<IMAGE_IMPORT_BY_NAME*>(base + named->u1.AddressOfData);
            const Redirect* redirect = Find(byName->Name, table, count, skip, skipCount);
            if (!redirect) {
                ++report.unmatched;
                if (onLeftAlone)
                    onLeftAlone(moduleName, byName->Name, user);
                continue;
            }

            DWORD oldProtect = 0;
            if (!VirtualProtect(thunk, sizeof(*thunk), PAGE_READWRITE, &oldProtect))
                continue;

            if (thunk->u1.Function)
                originals.Add(redirect->name,
                              reinterpret_cast<void*>(static_cast<uintptr_t>(thunk->u1.Function)));

            thunk->u1.Function = reinterpret_cast<ULONGLONG>(redirect->replacement);
            VirtualProtect(thunk, sizeof(*thunk), oldProtect, &oldProtect);
            ++report.patched;
        }
    }
    return report;
}

}
