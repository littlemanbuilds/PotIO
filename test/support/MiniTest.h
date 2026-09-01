#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace MiniTest
{
    static int tests = 0;
    static int assertions = 0;

    inline void fail(const char *expr, const char *file, int line)
    {
        std::fprintf(stderr, "FAIL: %s (%s:%d)\n", expr, file, line);
        std::exit(1);
    }

    inline void begin(const char *name)
    {
        ++tests;
        std::printf("[TEST] %s\n", name);
    }
}

#define CHECK(expr) do { ++MiniTest::assertions; if (!(expr)) MiniTest::fail(#expr, __FILE__, __LINE__); } while (0)
#define CHECK_NEAR(a,b,eps) do { ++MiniTest::assertions; const double _a=(a), _b=(b); if (std::fabs(_a-_b) > (eps)) MiniTest::fail(#a " ~= " #b, __FILE__, __LINE__); } while (0)
