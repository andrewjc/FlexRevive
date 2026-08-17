// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "f4kit/Archive.h"

#include "f4kit/Log.h"
#include "f4kit/PeImage.h"

// For the structured exception handling around the one call into engine code.
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <cstdint>
#include <cstring>
#include <vector>

namespace f4kit::archive {

namespace {

// lea rcx, [rip+disp32]. The archive name is the mount routine's first argument, so this is the
// instruction that names the archive at the call site, and the call follows it.
constexpr uint8_t kLeaRcx[3] = {0x48, 0x8D, 0x0D};
constexpr int kLeaLength = 7;

// How far past the lea to look for that call. Only the two remaining arguments are set up in
// between, which is a few bytes; the window is generous rather than exact.
constexpr int kCallWindow = 32;

// Every archive the game loads goes through this one routine, so it is called from several
// places. A candidate reached from only one is far more likely to be a mis-decode than the
// real thing.
constexpr int kMinCallSites = 3;

bool InExecutable(uintptr_t p)
{
    for (const pe::Section& sec : pe::Sections()) {
        if (!sec.executable)
            continue;
        const auto base = reinterpret_cast<uintptr_t>(sec.begin);
        if (p >= base && p < base + sec.size)
            return true;
    }
    return false;
}

// Where the rip-relative operand at `operand` points, given an instruction ending at `end`.
uintptr_t RipTarget(const uint8_t* operand, const uint8_t* end)
{
    int32_t disp = 0;
    memcpy(&disp, operand, sizeof(disp));
    return reinterpret_cast<uintptr_t>(end) + uintptr_t(intptr_t(disp));
}

// Every place `text` appears as a whole, NUL-terminated string in data. A name may be stored
// more than once, and the caller cannot tell which copy the call site references.
void FindLiteral(const char* text, std::vector<uintptr_t>& out)
{
    const size_t len = strlen(text);
    for (const pe::Section& sec : pe::Sections()) {
        if (sec.executable || sec.size < len + 1)
            continue;
        const uint8_t* p = sec.begin;
        const uint8_t* last = sec.begin + sec.size - (len + 1);
        for (; p <= last; ++p) {
            if (*p != uint8_t(text[0]))
                continue;
            if (memcmp(p, text, len) == 0 && p[len] == 0)
                out.push_back(reinterpret_cast<uintptr_t>(p));
        }
    }
}

// How many direct calls in the image reach `target`. This is what separates the mount routine
// from whatever else a stray 0xE8 byte might decode to.
int CountCallSites(uintptr_t target)
{
    int sites = 0;
    for (const pe::Section& sec : pe::Sections()) {
        if (!sec.executable || sec.size < 5)
            continue;
        const uint8_t* p = sec.begin;
        const uint8_t* last = sec.begin + sec.size - 5;
        for (; p <= last; ++p) {
            if (*p != 0xE8)
                continue;
            if (RipTarget(p + 1, p + 5) == target)
                ++sites;
        }
    }
    return sites;
}

struct Site {
    uintptr_t fn = 0;      // the mount routine
    const char* name = nullptr;  // the engine's own copy of the archive name
};

Site FindMountSite(const char* archiveName)
{
    std::vector<uintptr_t> names;
    FindLiteral(archiveName, names);
    if (names.empty()) {
        log::Write("archive: \"%s\" is not named anywhere in this executable", archiveName);
        return {};
    }

    for (const pe::Section& sec : pe::Sections()) {
        if (!sec.executable || sec.size < 5)
            continue;
        const uint8_t* const secLimit = sec.begin + sec.size - 5;
        const uint8_t* p = sec.begin;
        const uint8_t* const last = sec.begin + sec.size - kLeaLength;
        for (; p <= last; ++p) {
            if (memcmp(p, kLeaRcx, sizeof(kLeaRcx)) != 0)
                continue;

            const uintptr_t named = RipTarget(p + 3, p + kLeaLength);
            bool namesThisArchive = false;
            for (uintptr_t n : names)
                if (n == named) {
                    namesThisArchive = true;
                    break;
                }
            if (!namesThisArchive)
                continue;

            const uint8_t* stop = p + kLeaLength + kCallWindow;
            if (stop > secLimit)
                stop = secLimit;
            for (const uint8_t* q = p + kLeaLength; q <= stop; ++q) {
                // Past the end of this code path, so there is no call to find.
                if (*q == 0xC3 || *q == 0xCC)
                    break;
                if (*q != 0xE8)
                    continue;

                const uintptr_t target = RipTarget(q + 1, q + 5);
                if (!InExecutable(target))
                    continue;

                const int sites = CountCallSites(target);
                if (sites < kMinCallSites) {
                    log::Write("archive: the call after the \"%s\" reference reaches %p, which "
                               "only %d site(s) call; not the mount routine",
                               archiveName, reinterpret_cast<void*>(target), sites);
                    continue;
                }

                log::Write("archive: mount routine at %p, found from the call site that names "
                           "\"%s\", called from %d site(s) in the image",
                           reinterpret_cast<void*>(target), archiveName, sites);
                return {target, reinterpret_cast<const char*>(named)};
            }
        }
    }

    log::Write("archive: \"%s\" is named in this executable but nothing there mounts it",
               archiveName);
    return {};
}

} // namespace

bool Mount(const char* archiveName)
{
    if (!archiveName || !*archiveName)
        return false;
    if (!pe::Init())
        return false;

    const Site site = FindMountSite(archiveName);
    if (!site.fn)
        return false;

    // Guarded, because this is a call into a function identified by reading the game's own
    // code rather than one this plugin was compiled against. The identification can be right
    // and the call still be wrong: the routine reaches into engine state that has to have been
    // built first, and calling it too early dereferences a global that is still null.
    //
    // Without the guard that fault propagates out of F4SEPlugin_Load, and F4SE responds by
    // disabling the plugin for the whole session. So an optional extra costs the user weapon
    // debris entirely, which is precisely backwards. A failure here has to cost only itself.
    bool ok = false;
    __try {
        // The engine's own copy of the name is handed over rather than ours, so the routine
        // receives exactly what the call site it was found from would have given it.
        using MountFn = void(__fastcall*)(const char*, void*, void*);
        reinterpret_cast<MountFn>(site.fn)(site.name, nullptr, nullptr);
        ok = true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        log::Write("archive: mounting \"%s\" faulted, so the archive is not available. "
                   "Everything else is unaffected.", archiveName);
        return false;
    }

    if (ok)
        log::Write("archive: mounted \"%s\"", archiveName);
    return ok;
}

}
