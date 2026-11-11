/**
 * budget.hpp
 * R5.1: GPU chunk-size rule. Header-only, no CUDA.
 */
#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace compute {

/**
 * chunk_size - samples per GPU launch chunk.
 *   budget = min(user_budget, headroom*free - reserve) - persistent
 *   chunk  = floor(budget / (2 * bytes_per_sample))   (2 = double buffering)
 * Clamped to [min_chunk, max_chunk], rounded DOWN to a multiple of 256.
 * Returns 0 when budget <= 0 or the result is below min_chunk ("use CPU").
 * Throws std::invalid_argument on non-positive bytes_per_sample/user budget,
 * negative sizes, headroom outside (0,1], or bad min/max chunk.
 */
inline std::int64_t chunk_size(std::int64_t free_bytes,
                               std::int64_t user_budget_bytes,
                               std::int64_t persistent_bytes,
                               std::int64_t bytes_per_sample,
                               double headroom = 0.6,
                               std::int64_t reserve_bytes = 256LL * 1024 * 1024,
                               std::int64_t min_chunk = 65536,
                               std::int64_t max_chunk = 4194304) {
    if (free_bytes < 0 || user_budget_bytes <= 0 || persistent_bytes < 0 ||
        bytes_per_sample <= 0 || reserve_bytes < 0)
        throw std::invalid_argument("chunk_size: invalid byte argument");
    if (!(headroom > 0.0 && headroom <= 1.0))
        throw std::invalid_argument("chunk_size: headroom must be in (0, 1]");
    if (min_chunk < 256 || max_chunk < min_chunk)
        throw std::invalid_argument("chunk_size: invalid min/max chunk");

    const double avail = headroom * static_cast<double>(free_bytes) -
                         static_cast<double>(reserve_bytes);
    const double cap = std::min(static_cast<double>(user_budget_bytes), avail);
    const double budget = cap - static_cast<double>(persistent_bytes);
    if (budget <= 0.0) return 0;

    const double raw = budget / (2.0 * static_cast<double>(bytes_per_sample));
    std::int64_t chunk = raw >= static_cast<double>(max_chunk)
                             ? max_chunk
                             : static_cast<std::int64_t>(raw);
    if (chunk < min_chunk) return 0;
    chunk -= chunk % 256;
    if (chunk < min_chunk) return 0;
    return chunk;
}

}  // namespace compute
