#pragma once

// A test harness with no dependencies, so the tests build anywhere the plugin does.
//
// Each suite is a translation unit with a main() that calls Suite() to name a group, the CHECK
// macros to assert, and Report() to return an exit code. A failure prints the expression, the
// values involved and the source line, then keeps going, so one run shows every break rather
// than only the first.

#include <cmath>
#include <cstdio>

namespace f4kit::test {

inline int g_checks = 0;
inline int g_failures = 0;
inline const char* g_suite = "";

inline void Suite(const char* name)
{
    g_suite = name;
    printf("  -- %s\n", name);
}

inline void Fail(const char* expr, const char* file, int line, const char* detail)
{
    ++g_failures;
    printf("  [FAIL] %s\n         %s\n         %s:%d\n", expr, detail, file, line);
}

inline bool Check(bool ok, const char* expr, const char* file, int line)
{
    ++g_checks;
    if (!ok)
        Fail(expr, file, line, "expected true");
    return ok;
}

inline bool CheckNear(double got, double want, double tol, const char* expr, const char* file,
                      int line)
{
    ++g_checks;
    const bool ok = std::isfinite(got) && std::fabs(got - want) <= tol;
    if (!ok) {
        char detail[192];
        snprintf(detail, sizeof(detail), "got %.9g, want %.9g (tolerance %.3g)", got, want, tol);
        Fail(expr, file, line, detail);
    }
    return ok;
}

inline bool CheckEq(long long got, long long want, const char* expr, const char* file, int line)
{
    ++g_checks;
    const bool ok = got == want;
    if (!ok) {
        char detail[192];
        snprintf(detail, sizeof(detail), "got %lld, want %lld", got, want);
        Fail(expr, file, line, detail);
    }
    return ok;
}

// Every component of a 3-vector within tolerance.
inline bool CheckVec3(const float* got, float x, float y, float z, double tol, const char* expr,
                      const char* file, int line)
{
    ++g_checks;
    const float want[3] = {x, y, z};
    for (int a = 0; a < 3; ++a) {
        if (!std::isfinite(got[a]) || std::fabs(double(got[a]) - want[a]) > tol) {
            char detail[192];
            snprintf(detail, sizeof(detail),
                     "got (%.6g, %.6g, %.6g), want (%.6g, %.6g, %.6g), tolerance %.3g",
                     got[0], got[1], got[2], want[0], want[1], want[2], tol);
            Fail(expr, file, line, detail);
            return false;
        }
    }
    return true;
}

inline int Report(const char* name)
{
    if (g_failures == 0)
        printf("  %s: ALL PASS (%d checks)\n", name, g_checks);
    else
        printf("  %s: %d FAILURES of %d checks\n", name, g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}

} // namespace f4kit::test

#define CHECK(expr)          ::f4kit::test::Check((expr), #expr, __FILE__, __LINE__)
#define CHECK_NEAR(g, w, t)  ::f4kit::test::CheckNear((g), (w), (t), #g, __FILE__, __LINE__)
#define CHECK_EQ(g, w)       ::f4kit::test::CheckEq((long long)(g), (long long)(w), #g, \
                                                         __FILE__, __LINE__)
#define CHECK_VEC3(g, x, y, z, t) \
    ::f4kit::test::CheckVec3((g), (x), (y), (z), (t), #g, __FILE__, __LINE__)
