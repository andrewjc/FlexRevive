#include "f4kit/CrashLog.h"
#include "f4kit/Log.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <DbgHelp.h>

#include <cstdio>

#pragma comment(lib, "dbghelp.lib")

namespace f4kit::crash {

namespace {

LPTOP_LEVEL_EXCEPTION_FILTER s_previous = nullptr;
bool s_installed = false;
wchar_t s_dumpDir[MAX_PATH] = {};

const char* ExceptionName(DWORD code)
{
    switch (code) {
    case EXCEPTION_ACCESS_VIOLATION:      return "ACCESS_VIOLATION";
    case EXCEPTION_STACK_OVERFLOW:        return "STACK_OVERFLOW";
    case EXCEPTION_ILLEGAL_INSTRUCTION:   return "ILLEGAL_INSTRUCTION";
    case EXCEPTION_INT_DIVIDE_BY_ZERO:    return "INT_DIVIDE_BY_ZERO";
    case EXCEPTION_FLT_DIVIDE_BY_ZERO:    return "FLT_DIVIDE_BY_ZERO";
    case EXCEPTION_PRIV_INSTRUCTION:      return "PRIV_INSTRUCTION";
    case EXCEPTION_IN_PAGE_ERROR:         return "IN_PAGE_ERROR";
    case EXCEPTION_DATATYPE_MISALIGNMENT: return "DATATYPE_MISALIGNMENT";
    case EXCEPTION_ARRAY_BOUNDS_EXCEEDED: return "ARRAY_BOUNDS_EXCEEDED";
    default:                              return "unknown";
    }
}

// Names an address as module + offset. The offset is stable across runs, unlike the raw
// address, and is what a map file or disassembler takes.
void DescribeAddress(void* addr, char* out, size_t outSize)
{
    HMODULE mod = nullptr;
    if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                                GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                            static_cast<LPCSTR>(addr), &mod) ||
        !mod) {
        _snprintf_s(out, outSize, _TRUNCATE, "%p  <no module>", addr);
        return;
    }

    char path[MAX_PATH] = {};
    GetModuleFileNameA(mod, path, MAX_PATH);
    const char* leaf = strrchr(path, '\\');
    leaf = leaf ? leaf + 1 : path;

    const auto base = reinterpret_cast<uintptr_t>(mod);
    const auto here = reinterpret_cast<uintptr_t>(addr);
    _snprintf_s(out, outSize, _TRUNCATE, "%p  %s+0x%llX", addr, leaf,
                static_cast<unsigned long long>(here - base));
}

void WriteMiniDump(EXCEPTION_POINTERS* info)
{
    wchar_t path[MAX_PATH] = {};
    swprintf_s(path, L"%scrash.dmp", s_dumpDir);

    HANDLE file = CreateFileW(path, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
                              FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file == INVALID_HANDLE_VALUE) {
        log::Write("  (could not create the minidump file)");
        return;
    }

    MINIDUMP_EXCEPTION_INFORMATION mei = {};
    mei.ThreadId = GetCurrentThreadId();
    mei.ExceptionPointers = info;
    mei.ClientPointers = FALSE;

    const auto type = static_cast<MINIDUMP_TYPE>(MiniDumpWithIndirectlyReferencedMemory |
                                                 MiniDumpWithDataSegs |
                                                 MiniDumpWithThreadInfo);
    const BOOL ok = MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), file, type,
                                      &mei, nullptr, nullptr);
    CloseHandle(file);
    log::Write("  minidump: %ls (%s)", path, ok ? "written" : "FAILED");
}

LONG WINAPI OnUnhandledException(EXCEPTION_POINTERS* info)
{
    if (!info || !info->ExceptionRecord || !info->ContextRecord)
        return s_previous ? s_previous(info) : EXCEPTION_CONTINUE_SEARCH;

    const EXCEPTION_RECORD& er = *info->ExceptionRecord;
    const CONTEXT& ctx = *info->ContextRecord;

    char where[512] = {};
    DescribeAddress(er.ExceptionAddress, where, sizeof(where));

    log::Write("==================== UNHANDLED EXCEPTION ====================");
    log::Write("  %s (0x%08X) at %s", ExceptionName(er.ExceptionCode),
               static_cast<unsigned>(er.ExceptionCode), where);

    // For an access violation the record carries the direction and target address.
    if (er.ExceptionCode == EXCEPTION_ACCESS_VIOLATION && er.NumberParameters >= 2) {
        const ULONG_PTR op = er.ExceptionInformation[0];
        const auto target = reinterpret_cast<void*>(er.ExceptionInformation[1]);
        log::Write("  attempted to %s address %p",
                   op == 0 ? "READ" : (op == 1 ? "WRITE" : "EXECUTE"), target);
    }

    log::Write("  thread %lu", GetCurrentThreadId());
    log::Write("  RIP=%016llX RSP=%016llX RBP=%016llX EFL=%08lX", ctx.Rip, ctx.Rsp, ctx.Rbp,
               ctx.EFlags);
    log::Write("  RAX=%016llX RBX=%016llX RCX=%016llX RDX=%016llX", ctx.Rax, ctx.Rbx, ctx.Rcx,
               ctx.Rdx);
    log::Write("  RSI=%016llX RDI=%016llX R8 =%016llX R9 =%016llX", ctx.Rsi, ctx.Rdi, ctx.R8,
               ctx.R9);
    log::Write("  R10=%016llX R11=%016llX R12=%016llX R13=%016llX", ctx.R10, ctx.R11, ctx.R12,
               ctx.R13);
    log::Write("  R14=%016llX R15=%016llX", ctx.R14, ctx.R15);

    // StackWalk64 modifies the context, so it walks a copy.
    CONTEXT walk = ctx;
    STACKFRAME64 frame = {};
    frame.AddrPC.Offset = walk.Rip;
    frame.AddrPC.Mode = AddrModeFlat;
    frame.AddrFrame.Offset = walk.Rbp;
    frame.AddrFrame.Mode = AddrModeFlat;
    frame.AddrStack.Offset = walk.Rsp;
    frame.AddrStack.Mode = AddrModeFlat;

    const HANDLE proc = GetCurrentProcess();
    SymSetOptions(SYMOPT_DEFERRED_LOADS | SYMOPT_UNDNAME | SYMOPT_LOAD_LINES);
    SymInitialize(proc, nullptr, TRUE);

    log::Write("  ---- stack ----");
    for (int i = 0; i < 48; ++i) {
        if (!StackWalk64(IMAGE_FILE_MACHINE_AMD64, proc, GetCurrentThread(), &frame, &walk,
                         nullptr, SymFunctionTableAccess64, SymGetModuleBase64, nullptr))
            break;
        if (frame.AddrPC.Offset == 0)
            break;

        char frameWhere[512] = {};
        DescribeAddress(reinterpret_cast<void*>(frame.AddrPC.Offset), frameWhere,
                        sizeof(frameWhere));

        // Symbol name where one is available, which needs the build's PDB alongside the DLL.
        alignas(SYMBOL_INFO) char buffer[sizeof(SYMBOL_INFO) + 512] = {};
        auto* sym = reinterpret_cast<SYMBOL_INFO*>(buffer);
        sym->SizeOfStruct = sizeof(SYMBOL_INFO);
        sym->MaxNameLen = 511;
        DWORD64 displacement = 0;
        const bool named = SymFromAddr(proc, frame.AddrPC.Offset, &displacement, sym) != FALSE;

        DWORD lineDisp = 0;
        IMAGEHLP_LINE64 line = {};
        line.SizeOfStruct = sizeof(line);
        const bool haveLine =
            SymGetLineFromAddr64(proc, frame.AddrPC.Offset, &lineDisp, &line) != FALSE;

        if (named && haveLine)
            log::Write("  [%2d] %s  %s+0x%llX  (%s:%lu)", i, frameWhere, sym->Name,
                       static_cast<unsigned long long>(displacement), line.FileName,
                       line.LineNumber);
        else if (named)
            log::Write("  [%2d] %s  %s+0x%llX", i, frameWhere, sym->Name,
                       static_cast<unsigned long long>(displacement));
        else
            log::Write("  [%2d] %s", i, frameWhere);
    }

    WriteMiniDump(info);
    log::Write("============================================================");

    // Hand on to whatever filter was installed before, so it sees the same exception.
    return s_previous ? s_previous(info) : EXCEPTION_CONTINUE_SEARCH;
}

} // namespace

void Install(const wchar_t* dumpDir)
{
    if (s_installed)
        return;
    s_installed = true;
    if (dumpDir)
        wcscpy_s(s_dumpDir, dumpDir);
    s_previous = SetUnhandledExceptionFilter(OnUnhandledException);
    log::Write("crash reporting installed%s", s_previous ? " (chaining to the existing handler)"
                                                         : "");
}

}
