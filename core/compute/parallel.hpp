/**
 * parallel.hpp
 * R5.2: tiny parallel_for over [0, n). Header-only, no CUDA.
 */
#pragma once

#include <algorithm>
#include <cstddef>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

namespace compute {

inline int default_threads() {
    unsigned h = std::thread::hardware_concurrency();
    return h == 0 ? 1 : static_cast<int>(h);
}

/// Calls f(begin, end) on disjoint ranges covering [0, n). n_threads <= 0 means
/// hardware_concurrency. Each thread gets at least min_per_thread items, so a
/// small n runs on the calling thread. The first exception is rethrown.
template <class F>
void parallel_for(std::size_t n, int n_threads, std::size_t min_per_thread, F&& f) {
    if (n == 0) return;
    std::size_t nt = static_cast<std::size_t>(n_threads > 0 ? n_threads : default_threads());
    if (min_per_thread == 0) min_per_thread = 1;
    nt = std::min(nt, std::max<std::size_t>(1, n / min_per_thread));
    if (nt <= 1) {
        f(std::size_t(0), n);
        return;
    }
    const std::size_t per = (n + nt - 1) / nt;
    std::exception_ptr err;
    std::mutex mu;
    auto work = [&](std::size_t b, std::size_t e) {
        try {
            f(b, e);
        } catch (...) {
            std::lock_guard<std::mutex> lk(mu);
            if (!err) err = std::current_exception();
        }
    };
    std::vector<std::thread> th;
    th.reserve(nt - 1);
    for (std::size_t t = 1; t < nt; ++t) {
        const std::size_t b = t * per;
        if (b >= n) break;
        th.emplace_back(work, b, std::min(n, b + per));
    }
    work(0, std::min(n, per));
    for (auto& t : th) t.join();
    if (err) std::rethrow_exception(err);
}

}  // namespace compute
