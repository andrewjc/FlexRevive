// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

// Redirecting a running executable's imports to your own functions.
//
// A process reaches an imported function through a thunk holding its address. Rewriting that
// thunk redirects every later call without touching the target library at all, which means the
// replacement works whether or not that library has loaded, and works on any build of the host
// because the imports are matched by name rather than by address.
//
// The address a thunk held is kept, so a replacement that only wants to observe a call can
// record what it sees and hand the call straight on.
//
// Nothing here is specific to a particular host or library: give it a module prefix and a table
// of names, and it rewrites what it finds.
namespace f4kit::imports {

// One import to redirect: the exported name, and what to point it at instead.
struct Redirect {
    const char* name;
    void* replacement;
};

// How many original targets are remembered. Well beyond any single library's useful surface.
constexpr int kMaxOriginals = 64;

// What each thunk pointed at before it was rewritten, so a replacement can forward.
class Originals {
public:
    // Records `fn` under `name`. Refuses a null name or target, and refuses to overwrite a name
    // already recorded: the first value is the one the thunk genuinely held. Returns whether
    // anything was stored.
    bool Add(const char* name, void* fn);

    // The recorded target for `name`, or null.
    void* For(const char* name) const;

    int Count() const { return m_count; }
    void Clear() { m_count = 0; }

private:
    struct Entry {
        const char* name;
        void* fn;
    };
    Entry m_entries[kMaxOriginals] = {};
    int m_count = 0;
};

// The redirect for an imported name, or null when it is not in the table or appears in `skip`.
// Names match exactly and case-sensitively, as exported names do.
const Redirect* Find(const char* importName, const Redirect* table, int count,
                     const char* const* skip, int skipCount);

// Whether a module name begins with `prefix`, ignoring case. An empty prefix matches nothing,
// since it would otherwise claim every import in the process.
bool ModuleMatches(const char* moduleName, const char* prefix);

struct Report {
    int modules = 0;    // matching modules found in the import table
    int patched = 0;    // thunks rewritten
    int byOrdinal = 0;  // imported without a name, so nothing to match on
    int unmatched = 0;  // named, but not in the table or deliberately skipped
};

// Called for each import left pointing at its original, so a caller can name them. Imports by
// ordinal arrive with a null `importName`.
using Notice = void (*)(const char* moduleName, const char* importName, void* user);

// Rewrites the running executable's import thunks for every module matching `prefix`.
Report Patch(const char* prefix, const Redirect* table, int count, const char* const* skip,
             int skipCount, Originals& originals, Notice onLeftAlone, void* user);

}
