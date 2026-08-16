// The plugin logs through f4kit::log. Under test there is no log file, so writes go to
// stdout and traces are dropped. Linked into any suite that pulls in plugin code.

#include <cstdarg>
#include <cstdio>

namespace f4kit::log {

void Write(const char* fmt, ...)
{
    va_list a;
    va_start(a, fmt);
    vprintf(fmt, a);
    va_end(a);
    printf("\n");
}

void Trace(const char*, ...) {}

}
