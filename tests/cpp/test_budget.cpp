/**
 * test_budget.cpp - R5.1: compute::chunk_size
 */
#include "compute/budget.hpp"
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <stdexcept>

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::cerr << "FAIL line " << __LINE__ << ": " #c "\n"; ++failures; } } while (0)

static const std::int64_t MiB = 1024LL * 1024;
static const std::int64_t GiB = 1024LL * MiB;

template <class F> static bool throws_invalid(F f) {
    try { f(); } catch (const std::invalid_argument&) { return true; } catch (...) { return false; }
    return false;
}

int main() {
    using compute::chunk_size;
    // plenty of VRAM -> max chunk
    CHECK(chunk_size(8 * GiB, 8 * GiB, 0, 100) == 4194304);
    // tiny VRAM -> 0 (budget negative)
    CHECK(chunk_size(100 * MiB, 8 * GiB, 0, 100) == 0);
    // result below min -> 0
    CHECK(chunk_size(1 * GiB, 8 * GiB, 0, 4 * MiB) == 0);
    // exact formula, multiple of 256
    {
        std::int64_t free_b = 2 * GiB;
        double budget = 0.6 * double(free_b) - double(256 * MiB);
        std::int64_t expect = std::int64_t(budget / 2000.0);
        expect = std::min<std::int64_t>(expect, 4194304);
        expect -= expect % 256;
        std::int64_t c = chunk_size(free_b, 8 * GiB, 0, 1000);
        CHECK(c == expect);
        CHECK(c % 256 == 0);
    }
    // odd bytes_per_sample still rounds down to 256
    {
        std::int64_t c = chunk_size(1 * GiB, 8 * GiB, 0, 997);
        CHECK(c > 0 && c % 256 == 0);
    }
    // persistent bytes subtract
    {
        std::int64_t a = chunk_size(2 * GiB, 8 * GiB, 0, 1000);
        std::int64_t b = chunk_size(2 * GiB, 8 * GiB, 500 * MiB, 1000);
        CHECK(b < a && b > 0);
        CHECK(chunk_size(2 * GiB, 8 * GiB, 2 * GiB, 1000) == 0);
    }
    // user budget caps
    {
        CHECK(chunk_size(8 * GiB, 64 * MiB, 0, 1000) == 0);  // 33554 < min
        std::int64_t d = chunk_size(8 * GiB, 512 * MiB, 0, 1000);
        std::int64_t e = 512 * MiB / 2000;
        CHECK(d == e - e % 256);
    }
    // headroom respected
    CHECK(chunk_size(2 * GiB, 8 * GiB, 0, 1000, 0.5) < chunk_size(2 * GiB, 8 * GiB, 0, 1000, 0.6));
    // custom min/max
    CHECK(chunk_size(8 * GiB, 8 * GiB, 0, 100, 0.6, 0, 256, 1024) == 1024);
    // invalid inputs
    CHECK(throws_invalid([] { chunk_size(GiB, GiB, 0, 0); }));
    CHECK(throws_invalid([] { chunk_size(GiB, GiB, 0, -5); }));
    CHECK(throws_invalid([] { chunk_size(-1, GiB, 0, 10); }));
    CHECK(throws_invalid([] { chunk_size(GiB, 0, 0, 10); }));
    CHECK(throws_invalid([] { chunk_size(GiB, GiB, -1, 10); }));
    CHECK(throws_invalid([] { chunk_size(GiB, GiB, 0, 10, 0.0); }));
    CHECK(throws_invalid([] { chunk_size(GiB, GiB, 0, 10, 1.5); }));
    CHECK(throws_invalid([] { chunk_size(GiB, GiB, 0, 10, 0.6, -1); }));
    CHECK(throws_invalid([] { chunk_size(GiB, GiB, 0, 10, 0.6, 0, 100, 50); }));

    if (failures == 0) { std::cout << "All budget tests passed.\n"; return 0; }
    std::cerr << failures << " budget check(s) FAILED.\n";
    return 1;
}
