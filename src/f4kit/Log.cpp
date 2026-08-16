#include "f4kit/Log.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <ShlObj.h>

#include <cstdarg>
#include <cstdio>
#include <share.h>
#include <ctime>
#include <mutex>

namespace f4kit::log {

static FILE* s_file = nullptr;
static std::mutex s_mutex;
static wchar_t s_path[MAX_PATH] = {};
static bool s_verbose = false;

bool Open(const wchar_t* subPath, const wchar_t* fileName)
{
    std::lock_guard lock(s_mutex);
    if (s_file)
        return true;

    PWSTR docs = nullptr;
    if (FAILED(SHGetKnownFolderPath(FOLDERID_Documents, 0, nullptr, &docs)))
        return false;

    swprintf_s(s_path, L"%s\\%s", docs, subPath ? subPath : L"");
    CoTaskMemFree(docs);

    SHCreateDirectoryExW(nullptr, s_path, nullptr);
    wcscat_s(s_path, L"\\");
    wcscat_s(s_path, fileName ? fileName : L"plugin.log");

    // Deny-write sharing, so the log can be tailed while the game runs.
    s_file = _wfsopen(s_path, L"w", _SH_DENYWR);
    return s_file != nullptr;
}

static void WriteV(const char* fmt, va_list args)
{
    if (!s_file)
        return;

    time_t now = time(nullptr);
    tm local{};
    localtime_s(&local, &now);
    fprintf(s_file, "[%02d:%02d:%02d] ", local.tm_hour, local.tm_min, local.tm_sec);

    vfprintf(s_file, fmt, args);
    fputc('\n', s_file);
    fflush(s_file);
}

void Write(const char* fmt, ...)
{
    std::lock_guard lock(s_mutex);
    va_list args;
    va_start(args, fmt);
    WriteV(fmt, args);
    va_end(args);
}

void Trace(const char* fmt, ...)
{
    if (!s_verbose)
        return;
    std::lock_guard lock(s_mutex);
    va_list args;
    va_start(args, fmt);
    WriteV(fmt, args);
    va_end(args);
}

void SetVerbose(bool on)
{
    s_verbose = on;
}

bool Verbose()
{
    return s_verbose;
}

const wchar_t* Path()
{
    return s_path;
}

}
